#include <Arduino.h>
#include <ETH.h>
#include <NTPClient.h>
#include <WiFiUDP.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

//================================================================================
// ETHERNET AYARLARI (WT32-ETH01)
//================================================================================
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN   16
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        1
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18

// Watchdog konfigürasyonu
#define WDT_TIMEOUT_SECONDS 60
#define WDT_RESET_INTERVAL 30000
#define WDT_CRITICAL_OPERATIONS_TIMEOUT 10000

// Watchdog durumu
struct WatchdogManager {
    unsigned long lastResetTime = 0;
    bool isEnabled = false;
    uint16_t resetCount = 0;
    uint32_t lastRebootReason = 0;
} wdtManager;

//================================================================================
// SERI HABERLEŞME (dsPIC'e tarih/saat gönderimi)
//================================================================================
HardwareSerial picSerial(2);
#define PIC_RX_PIN 4
#define PIC_TX_PIN 14
#define PIC_BAUD_RATE 115200

//================================================================================
// UART İLETİŞİM (Birinci Kart ile - NTP bilgisi alımı)
//================================================================================
HardwareSerial masterSerial(1);
#define MASTER_RX_PIN 36
#define MASTER_TX_PIN 33
#define MASTER_BAUD 115200

// NTP komut buffer'ı
char masterBuffer[32];
uint8_t masterBufferIndex = 0;
bool ntpConfigReceived = false;
String receivedNtp1Part1 = "";
String receivedNtp1Part2 = "";
String receivedNtp2Part1 = "";
String receivedNtp2Part2 = "";

//================================================================================
// NTP AYARLARI
//================================================================================
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "0.0.0.0", 10800); // Başlangıçta boş, UTC+3

struct NTPServerManager {
    String ntp1;                    // Master karttan gelen NTP1
    String ntp2;                    // Master karttan gelen NTP2  
    bool usingNtp2;                // Şu anda NTP2 mi kullanılıyor
    uint8_t ntp1FailCount;         // NTP1 hata sayacı
    uint8_t ntp2FailCount;         // NTP2 hata sayacı
    bool hasValidConfig;           // Geçerli konfigürasyon var mı
    unsigned long lastSyncTime;    // Son başarılı senkronizasyon zamanı
    long timeOffset;               // Lokal saat düzeltme offseti (ms)
} ntpManager;

const uint8_t MAX_NTP_FAIL_COUNT = 5;      // Bir sunucudan diğerine geçmek için maksimum hata
const unsigned long NTP_RETRY_INTERVAL = 10000;
const unsigned long NTP_SYNC_INTERVAL = 30000;  // 30 saniyede bir senkronizasyon (daha sık)
unsigned long lastNtpFailTime = 0;

//================================================================================
// KALICI HAFIZA (PREFERENCES)
//================================================================================
Preferences preferences;
#define PREF_NTP_CONFIG_NAMESPACE "ntp-config"
#define PREF_NTP_SERVER1_KEY "ntpServer1"
#define PREF_NTP_SERVER2_KEY "ntpServer2"

//================================================================================
// GLOBAL DEĞİŞKENLER
//================================================================================
volatile bool ethConnected = false;
char dateBuffer[8];
char timeBuffer[8];

// YENİ: HASSAS ZAMAN YÖNETİMİ EKLE
struct PrecisionTimeManager {
    unsigned long lastNtpEpoch;
    unsigned long ntpCaptureMillis;
    bool isInitialized;
} timeSync;

#define TARGET_SEND_MS 50
#define SEND_TOLERANCE 5

//================================================================================
// FONKSIYON PROTOTİPLERİ
//================================================================================
void WiFiEvent(WiFiEvent_t event);
void initializeNTPServers();
void saveNtpServers(String ntp1, String ntp2);
void switchToNTP2();
void switchToNTP1();
uint8_t calculateChecksum(const char *str, uint8_t len);
void sendStatusToPic(char status);
void printNTPStatus();
void printNetworkInfo();
bool testDNSResolution();
void handleSerialCommands();

// Master kart iletişim fonksiyonları
void listenForMasterCommands();
void processMasterNTPCommand(const String& cmd);
void applyReceivedNTPConfig();
void testMasterConnection();
String parseIPPart(const String& part);

// Watchdog fonksiyonları
void checkRebootReason();
void initializeWatchdog();
void feedWatchdog();
void disableWatchdog();
void saveWatchdogStats();
void gracefulRestart();
void loadWatchdogStats();
void printWatchdogStatus();

// Hassas senkronizasyon fonksiyonları
unsigned long getPreciseEpochTime();
uint16_t getPreciseMillisecond();
bool updateTimeWithPrecision();
void syncedSendDateToPic();
void syncedSendTimeToPic();
void handleSyncedDsPICCommunication();
void setupPrecisionSync();
void printSyncStatus();

//================================================================================
// WATCHDOG FONKSİYONLARI
//================================================================================

void checkRebootReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    wdtManager.lastRebootReason = (uint32_t)reason;
    
    Serial.print("Son reboot nedeni: ");
    switch (reason) {
        case ESP_RST_POWERON:
            Serial.println("Normal acilis");
            break;
        case ESP_RST_SW:
            Serial.println("Yazilim restart");
            break;
        case ESP_RST_PANIC:
            Serial.println("Sistem panic");
            break;
        case ESP_RST_INT_WDT:
            Serial.println("Interrupt watchdog timeout");
            break;
        case ESP_RST_TASK_WDT:
            Serial.println("Task watchdog timeout - SISTEM DONMUSTU!");
            wdtManager.resetCount++;
            break;
        case ESP_RST_WDT:
            Serial.println("Watchdog timeout");
            wdtManager.resetCount++;
            break;
        case ESP_RST_BROWNOUT:
            Serial.println("Voltaj dusugu");
            break;
        default:
            Serial.printf("Bilinmeyen neden: %d\n", reason);
            break;
    }
}

void initializeWatchdog() {
    esp_err_t result;
    
    result = esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    
    if (result == ESP_OK) {
        result = esp_task_wdt_add(NULL);
        
        if (result == ESP_OK) {
            wdtManager.isEnabled = true;
            wdtManager.lastResetTime = millis();
            Serial.printf("Watchdog Timer baslatildi (%d saniye timeout)\n", WDT_TIMEOUT_SECONDS);
        } else {
            Serial.printf("HATA: Task watchdog'a eklenemedi: %d\n", result);
        }
    } else {
        Serial.printf("HATA: Watchdog baslatilamadi: %d\n", result);
    }
}

void feedWatchdog() {
    if (wdtManager.isEnabled) {
        esp_task_wdt_reset();
        wdtManager.lastResetTime = millis();
        
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 30000) {
            lastDebugPrint = millis();
            Serial.printf("[WDT] Watchdog resetlendi (Uptime: %lu sn)\n", millis() / 1000);
        }
    }
}

void disableWatchdog() {
    if (wdtManager.isEnabled) {
        esp_task_wdt_delete(NULL);
        esp_task_wdt_deinit();
        wdtManager.isEnabled = false;
        Serial.println("Watchdog devre disi birakildi");
    }
}

void saveWatchdogStats() {
    preferences.begin("wdt-stats", false);
    preferences.putUShort("resetCount", wdtManager.resetCount);
    preferences.putULong("lastReboot", wdtManager.lastRebootReason);
    preferences.putULong("uptime", millis());
    preferences.end();
    Serial.println("Watchdog istatistikleri kaydedildi");
}

void gracefulRestart() {
    Serial.println("Guvenli sistem restart baslatiliyor...");
    
    disableWatchdog();
    timeClient.end();
    saveWatchdogStats();
    picSerial.flush();
    masterSerial.flush();
    Serial.flush();
    
    Serial.println("3 saniye sonra restart...");
    delay(3000);
    ESP.restart();
}

void loadWatchdogStats() {
    preferences.begin("wdt-stats", true);
    wdtManager.resetCount = preferences.getUShort("resetCount", 0);
    uint32_t lastReboot = preferences.getULong("lastReboot", 0);
    uint32_t lastUptime = preferences.getULong("uptime", 0);
    preferences.end();
    
    if (wdtManager.resetCount > 0) {
        Serial.printf("Onceki watchdog reset sayisi: %d\n", wdtManager.resetCount);
        Serial.printf("Onceki uptime: %lu saniye\n", lastUptime / 1000);
    }
}

void printWatchdogStatus() {
    Serial.println("\n=== WATCHDOG DURUM ===");
    Serial.printf("Durum: %s\n", wdtManager.isEnabled ? "AKTIF" : "PASIF");
    Serial.printf("Timeout: %d saniye\n", WDT_TIMEOUT_SECONDS);
    Serial.printf("Son Reset: %lu ms once\n", millis() - wdtManager.lastResetTime);
    Serial.printf("Reset Sayisi: %d\n", wdtManager.resetCount);
    Serial.printf("Uptime: %lu saniye\n", millis() / 1000);
    Serial.println("=====================\n");
}

//================================================================================
// HASSAS ZAMAN SENKRONIZASYONU FONKSİYONLARI
//================================================================================

unsigned long getPreciseEpochTime() {
    if (!timeSync.isInitialized) {
        return timeClient.getEpochTime();
    }
    unsigned long elapsedMillis = millis() - timeSync.ntpCaptureMillis;
    return timeSync.lastNtpEpoch + (elapsedMillis / 1000);
}

uint16_t getPreciseMillisecond() {
    if (!timeSync.isInitialized) {
        return millis() % 1000;
    }
    unsigned long elapsedMillis = millis() - timeSync.ntpCaptureMillis;
    return elapsedMillis % 1000;
}

bool updateTimeWithPrecision() {
    if (!ntpManager.hasValidConfig) {
        Serial.println("[NTP] Hata: Gecerli konfigurasyon yok");
        return false;
    }
    
    if (!ethConnected) {
        Serial.println("[NTP] Hata: Ethernet baglantisi yok");
        return false;
    }
    
    unsigned long beforeMillis = millis();
    
    Serial.println("[NTP] ForceUpdate deneniyor...");
    if (timeClient.forceUpdate()) {
        unsigned long afterMillis = millis();
        unsigned long roundTripTime = afterMillis - beforeMillis;
        unsigned long estimatedMoment = beforeMillis + (roundTripTime / 2);
        
        timeSync.lastNtpEpoch = timeClient.getEpochTime();
        timeSync.ntpCaptureMillis = estimatedMoment;
        timeSync.isInitialized = true;
        ntpManager.lastSyncTime = millis();
        
        Serial.printf("[NTP] Sync OK | RTT: %lums | Epoch: %lu\n", 
                      roundTripTime, timeSync.lastNtpEpoch);
        return true;
    } else {
        Serial.println("[NTP] Hata: ForceUpdate basarisiz");
        return false;
    }
}

void syncedSendDateToPic() {
    time_t epochTime = getPreciseEpochTime();
    struct tm *timeinfo = localtime(&epochTime);
    
    snprintf(dateBuffer, 7, "%02u%02u%02u", 
             timeinfo->tm_mday, 
             timeinfo->tm_mon + 1, 
             timeinfo->tm_year % 100);
    
    uint8_t checksum = calculateChecksum(dateBuffer, 6);
    dateBuffer[6] = 'A' + checksum;
    dateBuffer[7] = '\0';
    
    picSerial.write((uint8_t*)dateBuffer, 7);
    
    uint16_t actualMs = getPreciseMillisecond();
    Serial.printf("[→dsPIC] Tarih: %s | Ms: %u\n", dateBuffer, actualMs);
}

void syncedSendTimeToPic() {
    snprintf(timeBuffer, 7, "%02u%02u%02u", 
             timeClient.getHours(), 
             timeClient.getMinutes(), 
             timeClient.getSeconds());
    
    uint8_t checksum = calculateChecksum(timeBuffer, 6);
    timeBuffer[6] = 'a' + checksum;
    timeBuffer[7] = '\0';
    
    picSerial.write((uint8_t*)timeBuffer, 7);
    
    uint16_t actualMs = getPreciseMillisecond();
    Serial.printf("[→dsPIC] Saat: %s | Ms: %u\n", timeBuffer, actualMs);
}

void handleSyncedDsPICCommunication() {
    static unsigned long lastSendEpoch = 0;
    static bool nextIsTarih = true;
    
    unsigned long currentEpoch = getPreciseEpochTime();
    uint16_t currentMs = getPreciseMillisecond();
    
    if (currentMs >= (TARGET_SEND_MS - SEND_TOLERANCE) && 
        currentMs <= (TARGET_SEND_MS + SEND_TOLERANCE)) {
        
        if (currentEpoch != lastSendEpoch) {
            lastSendEpoch = currentEpoch;
            
            if (nextIsTarih) {
                syncedSendDateToPic();
            } else {
                syncedSendTimeToPic();
            }
            
            nextIsTarih = !nextIsTarih;
            
            Serial.printf("[SYNC] Hedef: %dms | Gercek: %ums | Sapma: %dms\n", 
                          TARGET_SEND_MS, currentMs, 
                          (int)currentMs - (int)TARGET_SEND_MS);
        }
    }
}

void setupPrecisionSync() {
    timeSync.lastNtpEpoch = 0;
    timeSync.ntpCaptureMillis = 0;
    timeSync.isInitialized = false;
    
    Serial.println("\n=== HASSAS SENKRONIZASYON SISTEMI ===");
    Serial.printf("Hedef gonderim zamani: %dms\n", TARGET_SEND_MS);
    Serial.printf("Tolerans: ±%dms\n", SEND_TOLERANCE);
    Serial.println("=====================================\n");
    
    if (ethConnected && ntpManager.hasValidConfig) {
        Serial.println("Ilk hassas NTP senkronizasyonu yapiliyor...");
        
        for (int attempt = 0; attempt < 5; attempt++) {
            if (updateTimeWithPrecision()) {
                Serial.println("Hassas senkronizasyon basarili!");
                break;
            }
            Serial.printf("Deneme %d basarisiz...\n", attempt + 1);
            delay(1000);
            feedWatchdog();
        }
    }
}

void printSyncStatus() {
    Serial.println("\n=== SENKRONIZASYON DURUMU ===");
    Serial.printf("Hassas zaman: %s\n", timeSync.isInitialized ? "AKTIF" : "PASIF");
    Serial.printf("Epoch: %lu\n", getPreciseEpochTime());
    Serial.printf("Milisaniye: %u / 1000\n", getPreciseMillisecond());
    Serial.printf("Hedef gonderim: %dms (±%dms)\n", TARGET_SEND_MS, SEND_TOLERANCE);
    Serial.printf("Son NTP: %lu ms once\n", millis() - ntpManager.lastSyncTime);
    Serial.println("============================\n");
}

//================================================================================
// NTP FONKSİYONLARI
//================================================================================

void initializeNTPServers() {
    ntpManager.usingNtp2 = false;
    ntpManager.ntp1FailCount = 0;
    ntpManager.ntp2FailCount = 0;
    ntpManager.hasValidConfig = false;
    ntpManager.lastSyncTime = 0;
    ntpManager.timeOffset = 0;

    // Master karttan gelen kayıtlı sunucuları yükle
    preferences.begin(PREF_NTP_CONFIG_NAMESPACE, true);
    String savedNtp1 = preferences.getString(PREF_NTP_SERVER1_KEY, "");
    String savedNtp2 = preferences.getString(PREF_NTP_SERVER2_KEY, "");
    preferences.end();
    
    if (savedNtp1 != "" && savedNtp1.length() > 6) {
        ntpManager.ntp1 = savedNtp1;
        ntpManager.ntp2 = savedNtp2;  // Boş olabilir
        ntpManager.hasValidConfig = true;
        
        Serial.println("=== KAYITLI NTP KONFIGURASYONU YUKLENDI ===");
        Serial.print("NTP1: "); Serial.println(ntpManager.ntp1);
        Serial.print("NTP2: "); 
        Serial.println(ntpManager.ntp2.length() > 0 ? ntpManager.ntp2 : "Yok");
        Serial.println("==========================================");
    } else {
        Serial.println("!!! Kayitli NTP konfigurasyonu yok !!!");
        Serial.println("Master karttan konfigürasyon bekleniyor...");
    }
}

void saveNtpServers(String ntp1, String ntp2) {
    preferences.begin(PREF_NTP_CONFIG_NAMESPACE, false);
    preferences.putString(PREF_NTP_SERVER1_KEY, ntp1);
    preferences.putString(PREF_NTP_SERVER2_KEY, ntp2);
    preferences.end();
    Serial.println("Master NTP sunuculari kalici olarak kaydedildi.");
}

void switchToNTP2() {
    if (ntpManager.ntp2.length() > 6) {
        ntpManager.usingNtp2 = true;
        ntpManager.ntp2FailCount = 0;
        timeClient.setPoolServerName(ntpManager.ntp2.c_str());
        
        Serial.println("!!! NTP2'ye gecildi !!!");
        Serial.print("Yeni sunucu: ");
        Serial.println(ntpManager.ntp2);
    } else {
        Serial.println("UYARI: NTP2 adresi tanimli degil!");
        // NTP1'i tekrar dene
        ntpManager.ntp1FailCount = 0;
    }
}

void switchToNTP1() {
    if (ntpManager.ntp1.length() > 6) {
        ntpManager.usingNtp2 = false;
        ntpManager.ntp1FailCount = 0;
        timeClient.setPoolServerName(ntpManager.ntp1.c_str());
        
        Serial.println("!!! NTP1'e geri donuldu !!!");
        Serial.print("Yeni sunucu: ");
        Serial.println(ntpManager.ntp1);
    }
}

uint8_t calculateChecksum(const char *str, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += (str[i] - '0');
    }
    return sum % 10;
}


void sendStatusToPic(char status) {
    picSerial.write(status);
    if (status == 'Y') {
        Serial.println("dsPIC'e durum: Y (Ethernet yok)");
    } else if (status == 'X') {
        Serial.println("dsPIC'e durum: X (NTP yok)");
    }
}

void printNTPStatus() {
    Serial.println("\n=== NTP DURUM ===");
    
    if (!ntpManager.hasValidConfig) {
        Serial.println("DURUM: KONFIGURASYON YOK!");
        Serial.println("Master karttan NTP bilgisi bekleniyor...");
    } else {
        Serial.print("NTP1: "); 
        Serial.print(ntpManager.ntp1);
        Serial.print(" (Hata: ");
        Serial.print(ntpManager.ntp1FailCount);
        Serial.println(")");
        
        Serial.print("NTP2: ");
        if (ntpManager.ntp2.length() > 6) {
            Serial.print(ntpManager.ntp2);
            Serial.print(" (Hata: ");
            Serial.print(ntpManager.ntp2FailCount);
            Serial.println(")");
        } else {
            Serial.println("Tanimli degil");
        }
        
        Serial.print("AKTIF SUNUCU: ");
        if (ntpManager.usingNtp2) {
            Serial.print("NTP2 - ");
            Serial.println(ntpManager.ntp2);
        } else {
            Serial.print("NTP1 - ");
            Serial.println(ntpManager.ntp1);
        }
    }
    Serial.println("=================\n");
}

//================================================================================
// MASTER KART İLETİŞİM FONKSİYONLARI
//================================================================================

String parseIPPart(const String& part) {
    if (part.length() != 6) return "";
    
    String octet1 = part.substring(0, 3);
    String octet2 = part.substring(3, 6);
    
    int o1 = octet1.toInt();
    int o2 = octet2.toInt();
    
    return String(o1) + "." + String(o2);
}

void listenForMasterCommands() {
    while (masterSerial.available() > 0) {
        char receivedChar = masterSerial.read();
        
        if (receivedChar == 'u' || receivedChar == 'y' || 
            receivedChar == 'w' || receivedChar == 'x') {
            
            masterBuffer[masterBufferIndex] = '\0';
            String command = String(masterBuffer) + receivedChar;
            
            Serial.print("Master karttan komut: ");
            Serial.println(command);
            
            processMasterNTPCommand(command);
            
            masterBufferIndex = 0;
            memset(masterBuffer, 0, sizeof(masterBuffer));
            
        } else if ((receivedChar >= '0' && receivedChar <= '9') && 
                   masterBufferIndex < sizeof(masterBuffer) - 1) {
            masterBuffer[masterBufferIndex++] = receivedChar;
        }
    }
}

void processMasterNTPCommand(const String& cmd) {
    if (cmd.endsWith("u")) {
        receivedNtp1Part1 = cmd.substring(0, 6);
        Serial.print("NTP1 Part1 alindi: ");
        Serial.println(receivedNtp1Part1);
        masterSerial.println("ACK");
        masterSerial.flush();
        
    } else if (cmd.endsWith("y")) {
        receivedNtp1Part2 = cmd.substring(0, 6);
        Serial.print("NTP1 Part2 alindi: ");
        Serial.println(receivedNtp1Part2);
        
        if (receivedNtp1Part1.length() == 6 && receivedNtp1Part2.length() == 6) {
            String ntp1 = parseIPPart(receivedNtp1Part1) + "." + parseIPPart(receivedNtp1Part2);
            Serial.print("NTP1 IP adresi: ");
            Serial.println(ntp1);
            masterSerial.println("ACK");
            masterSerial.flush();
        }
        
    } else if (cmd.endsWith("w")) {
        receivedNtp2Part1 = cmd.substring(0, 6);
        Serial.print("NTP2 Part1 alindi: ");
        Serial.println(receivedNtp2Part1);
        masterSerial.println("ACK");
        masterSerial.flush();
        
    } else if (cmd.endsWith("x")) {
        receivedNtp2Part2 = cmd.substring(0, 6);
        Serial.print("NTP2 Part2 alindi: ");
        Serial.println(receivedNtp2Part2);
        
        if (receivedNtp2Part1.length() == 6 && receivedNtp2Part2.length() == 6) {
            String ntp2 = parseIPPart(receivedNtp2Part1) + "." + parseIPPart(receivedNtp2Part2);
            Serial.print("NTP2 IP adresi: ");
            Serial.println(ntp2);
            masterSerial.println("ACK");
            masterSerial.flush();
            
            applyReceivedNTPConfig();
        }
    }
}

void applyReceivedNTPConfig() {
    String ntp1 = parseIPPart(receivedNtp1Part1) + "." + parseIPPart(receivedNtp1Part2);
    String ntp2 = "";
    
    if (receivedNtp2Part1.length() == 6 && receivedNtp2Part2.length() == 6) {
        ntp2 = parseIPPart(receivedNtp2Part1) + "." + parseIPPart(receivedNtp2Part2);
    }
    
    Serial.println("\n=== MASTER KARTTAN NTP KONFIGURASYON ===");
    Serial.print("NTP1: "); Serial.println(ntp1);
    Serial.print("NTP2: "); Serial.println(ntp2.length() > 0 ? ntp2 : "Yok");
    
    if (ntp1.length() > 7) {
        ntpManager.ntp1 = ntp1;
        ntpManager.ntp2 = ntp2;
        ntpManager.hasValidConfig = true;
        ntpManager.usingNtp2 = false;
        ntpManager.ntp1FailCount = 0;
        ntpManager.ntp2FailCount = 0;
        
        saveNtpServers(ntp1, ntp2);

        // DÜZELTİLMİŞ KISIM: setPoolServerName kullan
        timeClient.setPoolServerName(ntp1.c_str());
        timeClient.setUpdateInterval(30000);
        
        // Eğer daha önce başlatılmamışsa başlat
        if (!ntpConfigReceived) {
            timeClient.begin();
            Serial.println("NTP istemcisi ilk kez baslatildi");
        }
        
        ntpConfigReceived = true;
        
        // Hassas senkronizasyonu başlat
        setupPrecisionSync();
        
        // Bufferları temizle
        receivedNtp1Part1 = "";
        receivedNtp1Part2 = "";
        receivedNtp2Part1 = "";
        receivedNtp2Part2 = "";
        
        Serial.println("Yeni NTP konfigürasyonu uygulandi ve hassas senkronizasyon baslatildi");
    }
}


void testMasterConnection() {
    Serial.println("Master kart baglantisi test ediliyor...");
    masterSerial.println("TEST");
    
    unsigned long startTime = millis();
    String response = "";
    
    while (millis() - startTime < 1000) {
        if (masterSerial.available()) {
            char c = masterSerial.read();
            if (c == '\n' || c == '\r') {
                if (response.length() > 0) {
                    Serial.print("Master kart yaniti: ");
                    Serial.println(response);
                    return;
                }
            } else {
                response += c;
            }
        }
        delay(1);
    }
    
    Serial.println("Master karttan yanit alinamadi");
}

//================================================================================
// NETWORK FONKSİYONLARI
//================================================================================

void printNetworkInfo() {
    Serial.println("\n=== AG BILGILERI ===");
    Serial.print("IP Adresi: "); Serial.println(ETH.localIP());
    Serial.print("Gateway: "); Serial.println(ETH.gatewayIP());
    Serial.print("Subnet Mask: "); Serial.println(ETH.subnetMask());
    Serial.print("DNS: "); Serial.println(ETH.dnsIP());
    Serial.print("MAC Adresi: "); Serial.println(ETH.macAddress());
    Serial.print("Link Durumu: "); Serial.println(ETH.linkUp() ? "Bagli" : "Bagli Degil");
    Serial.print("Hiz: "); Serial.print(ETH.linkSpeed()); Serial.println(" Mbps");
    Serial.print("Full Duplex: "); Serial.println(ETH.fullDuplex() ? "Evet" : "Hayir");
    Serial.println("==================\n");
}

bool testDNSResolution() {
    Serial.println("DNS cozumleme testi yapiliyor...");
    
    IPAddress testIP;
    bool dnsWorking = WiFi.hostByName("google.com", testIP);
    
    if (dnsWorking) {
        Serial.print("DNS calisyor - google.com: ");
        Serial.println(testIP);
        return true;
    } else {
        Serial.println("DNS sorunu tespit edildi!");
        return false;
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.length() > 50) {
            Serial.println("HATA: Komut cok uzun!");
            return;
        }
        
        if (command == "status") {
            printNTPStatus();
            printNetworkInfo();
            printWatchdogStatus();
            
        } else if (command == "reset") {
            gracefulRestart();
            
        } else if (command == "wdt") {
            printWatchdogStatus();
            
        } else if (command == "testmaster") {
            testMasterConnection();
    
        } else if (command == "masterinfo") {
            Serial.println("\n=== MASTER KART DURUMU ===");
            Serial.println("Baglanti: IO36(RX) <-> IO33(TX)");
            Serial.print("Baudrate: "); Serial.println(MASTER_BAUD);
            Serial.print("NTP konfig alindi: "); 
            Serial.println(ntpConfigReceived ? "EVET" : "HAYIR");
            if (receivedNtp1Part1.length() > 0) {
                Serial.print("NTP1 Part1: "); Serial.println(receivedNtp1Part1);
            }
            if (receivedNtp1Part2.length() > 0) {
                Serial.print("NTP1 Part2: "); Serial.println(receivedNtp1Part2);
            }
            Serial.println("========================\n");
            
            } else if (command == "sync") {
            printSyncStatus();
            
        } else if (command == "testsync") {
            Serial.println("10 saniye senkronizasyon testi...");
            for (int i = 0; i < 10; i++) {
                Serial.printf("T+%ds: Epoch=%lu, Ms=%u\n", 
                              i, getPreciseEpochTime(), getPreciseMillisecond());
                delay(1000);
            }

        } else if (command == "help") {
            Serial.println("\n=== KOMUTLAR ===");
            Serial.println("status     - Sistem durumu");
            Serial.println("reset      - Guvenli restart");
            Serial.println("wdt        - Watchdog durumu");
            Serial.println("testmaster - Master kart baglantisi test");
            Serial.println("masterinfo - Master kart bilgileri");
            Serial.println("help       - Bu yardim");
            Serial.println("\n=== PROTOKOL ===");
            Serial.println("Master kart: 192168u, 001002y, 192169w, 001001x");
            Serial.println("dsPIC'e: Tarih/Saat gonderimi");
            Serial.println("================\n");
        }
    }
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("ETH Baslatildi");
            ETH.setHostname("wt32-eth01-slave");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("ETH Baglandi");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.println("\n--- DHCP Bilgileri ---");
            Serial.print("  IP Adresi: "); Serial.println(ETH.localIP());
            Serial.print("  Ag Gecidi: "); Serial.println(ETH.gatewayIP());
            Serial.print("  DNS: "); Serial.println(ETH.dnsIP());
            Serial.println("----------------------");
            ethConnected = true;
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("ETH Baglanti Kesildi");
            ethConnected = false;
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("ETH Durduruldu");
            ethConnected = false;
            break;
        default:
            break;
    }
}

//================================================================================
// KURULUM (SETUP)
//================================================================================
void setup() {
    Serial.begin(115200);
    
    checkRebootReason();
    initializeWatchdog();
    
    Serial.println("WT32-ETH01 NTP Slave Kart baslatiliyor...");
    Serial.println("NTP bilgisi Master karttan alinacak");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("NVS flash temizleniyor...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret == ESP_OK) {
        Serial.println("NVS flash baslatildi.");
    }
    
    feedWatchdog();

    // Master kart ile iletişim başlat
    masterSerial.begin(MASTER_BAUD, SERIAL_8N1, MASTER_RX_PIN, MASTER_TX_PIN);
    Serial.println("Master kart iletisimi baslatildi (IO36-RX / IO33-TX)");
    Serial.printf("Baudrate: %d\n", MASTER_BAUD);

    // dsPIC'e tarih/saat göndermek için serial başlat
    picSerial.begin(PIC_BAUD_RATE, SERIAL_8N1, PIC_RX_PIN, PIC_TX_PIN);
    Serial.println("dsPIC iletisimi baslatildi (IO4-RX / IO14-TX)");

    WiFi.onEvent(WiFiEvent);
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

    feedWatchdog();

    Serial.print("Ethernet baglantisi bekleniyor...");
    unsigned long startTime = millis();
    while (!ethConnected && (millis() - startTime) < 30000) {
        delay(500);
        Serial.print(".");
        
        if ((millis() - wdtManager.lastResetTime) > 5000) {
            feedWatchdog();
        }
    }
    Serial.println();

    if (!ethConnected) {
        Serial.println("HATA: Ethernet baglantisi 30 saniyede kurulamadi!");
        return;
    }

    Serial.println("DNS sunuculari manuel olarak ayarlaniyor...");
    ETH.config(ETH.localIP(), ETH.gatewayIP(), ETH.subnetMask(), 
               IPAddress(8, 8, 8, 8), IPAddress(8, 8, 4, 4));
    
    delay(2000);
    feedWatchdog();
    
    printNetworkInfo();

    if (testDNSResolution()) {
        Serial.println("DNS cozumleme basarili");
    } else {
        Serial.println("DNS sorunu - IP adresleri kullanilacak");
    }
    
    feedWatchdog();
    
    // NTP sunucularını başlat
    initializeNTPServers();
    
    // Eğer kayıtlı konfigürasyon varsa NTP'yi başlat
    if (ntpManager.hasValidConfig) {
        String currentServer = ntpManager.usingNtp2 ? ntpManager.ntp2 : ntpManager.ntp1;
        timeClient.setPoolServerName(currentServer.c_str());
        
        // KRITIK: Daha sık güncelleme için interval'i düşür (30 saniye)
        timeClient.setUpdateInterval(30000);  // 30 saniye (120000'den düşürüldü)
        
        // NTP başlatmadan önce UDP socket'i optimize et
        ntpUDP.begin(123);  // NTP portu
        
        timeClient.begin();
        
        Serial.println("NTP istemcisi baslaniyor...");
        Serial.print("Baslangic NTP sunucusu: ");
        Serial.println(currentServer);
        Serial.print("Guncelleme araligi: 30 saniye\n");
        
        feedWatchdog();
        
        // İlk senkronizasyonda forceUpdate kullan
        Serial.println("Ilk NTP senkronizasyonu (forceUpdate) deneniyor...");
        bool syncSuccess = false;
        for (int attempt = 0; attempt < 5; attempt++) {
            // forceUpdate() kullanarak anında senkronize ol
            if (timeClient.forceUpdate()) {
                ntpManager.lastSyncTime = millis();
                Serial.printf("NTP senkronizasyonu basarili! (Deneme %d)\n", attempt + 1);
                Serial.printf("Epoch Time: %lu\n", timeClient.getEpochTime());
                syncSuccess = true;
                break;
            }
            Serial.printf("Deneme %d basarisiz, tekrar deneniyor...\n", attempt + 1);
            delay(1000);  // Denemeler arası kısa bekleme
            feedWatchdog();
        }

        if (syncSuccess) {
        setupPrecisionSync();
        }

        if (!syncSuccess) {
            Serial.println("UYARI: Ilk senkronizasyon basarisiz!");
        }
    } else {
        Serial.println("!!! UYARI: Master karttan NTP konfigurasyonu bekleniyor !!!");
        Serial.println("NTP istemcisi henuz baslatilmadi.");
    }
    
    printNTPStatus();
    printWatchdogStatus();

    // BURAYA YENİ SATIR EKLE:
    setupPrecisionSync();
    
    // Master kart bağlantısını test et
    testMasterConnection();
    
    Serial.println("\n=== SISTEM HAZIR ===");
    Serial.println("Master karttan NTP bilgisi bekleniyor...");
    Serial.println("Komutlar: 'status', 'reset', 'testmaster', 'masterinfo', 'help'");
}

//================================================================================
// ANA DÖNGÜ (LOOP)
//================================================================================
void loop() {
    feedWatchdog();
    listenForMasterCommands();
    handleSerialCommands();

    static unsigned long lastNtpUpdate = 0;
    static unsigned long lastNetworkCheck = 0;

    // Ağ durumu kontrol
    if (millis() - lastNetworkCheck >= 30000) {
        lastNetworkCheck = millis();
        
        bool linkStatus = ETH.linkUp();
        IPAddress currentIP = ETH.localIP();
        
        if (!linkStatus || currentIP.toString() == "0.0.0.0") {
            if (ethConnected) {
                Serial.println("Ethernet baglantisi kesildi!");
                ethConnected = false;
            }
        } else if (!ethConnected) {
            Serial.println("Ethernet yeniden kuruldu!");
            ethConnected = true;
        }
        feedWatchdog();
    }

    // NTP senkronizasyonu
    if (millis() - lastNtpUpdate >= 30000) {
        lastNtpUpdate = millis();
        if (ethConnected && ntpManager.hasValidConfig) {
            updateTimeWithPrecision();
        }
    }

    // Ethernet yok mu?
    if (!ethConnected) {
        static unsigned long lastStatusSend = 0;
        if (millis() - lastStatusSend >= 1000) {
            lastStatusSend = millis();
            sendStatusToPic('Y');
        }
        return;
    }

    // NTP config yok mu?
    if (!ntpManager.hasValidConfig || !timeSync.isInitialized) {
        static unsigned long lastStatusSend = 0;
        if (millis() - lastStatusSend >= 1000) {
            lastStatusSend = millis();
            sendStatusToPic('X');
        }
        return;
    }

    // Epoch geçerli mi?
    if (getPreciseEpochTime() < 100000) {
        static unsigned long lastStatusSend = 0;
        if (millis() - lastStatusSend >= 1000) {
            lastStatusSend = millis();
            sendStatusToPic('X');
        }
        return;
    }

    // SENKRON GÖNDERİM
    handleSyncedDsPICCommunication();
    
    delay(1);
}