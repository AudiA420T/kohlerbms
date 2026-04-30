// Complete ESP32-S3 CATL-to-Growatt BMS Bridge
// Reads CATL modules via CAN (ACAN_ESP32), translates to Growatt RS485 protocol
// Includes web dashboard, safety controls, temperature conversion (F->C)
// Enhanced with unknown message tracking and manual control features
// PROGRESSIVE LOADING VERSION - ESP32-S3 + ACAN_ESP32 adapted

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>
#include <FS.h>
#include <time.h>
#include <Preferences.h>
#include <ACAN_ESP32.h>

// WiFi credentials
const char* ssid = "NETWORK";
const char* password = "PASSWORD";

// Hardware pin definitions for ESP32-S3 with ACAN_ESP32
#define CAN_TX_PIN GPIO_NUM_15    // CAN TX (for Waveshare SN65HVD230)
#define CAN_RX_PIN GPIO_NUM_16    // CAN RX (for Waveshare SN65HVD230)

#define RS485_TX_PIN GPIO_NUM_17
#define RS485_RX_PIN GPIO_NUM_18
#define RS485_DE_PIN GPIO_NUM_4    // Driver Enable (DE/RE combined)

// Manual control pins (optional - for direct inverter control) - DISABLED FOR DEBUG
#define CHARGE_ENABLE_PIN 0     // DISABLED - was GPIO_NUM_19
#define DISCHARGE_ENABLE_PIN 0  // DISABLED - was GPIO_NUM_21

// RGB NeoPixel LED for ESP32-S3
#define LED_PIN 48
#define NUM_LEDS 1

// Create NeoPixel instance
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Use ACAN_ESP32 message structure
typedef CANMessage can_message_t;

// Growatt Protocol Constants
#define GROWATT_SLAVE_ADDR 0x01
#define GROWATT_BAUD_RATE 9600
#define GROWATT_TIMEOUT_MS 1000

// Growatt Register Addresses (from protocol document)
#define REG_PACK_VOLTAGE     0x0016  // 10mV units
#define REG_PACK_CURRENT     0x0017  // 10mA units (signed)
#define REG_SOC              0x0015  // Percentage 0-100
#define REG_STATUS           0x0013  // Status bits
#define REG_ERROR            0x0014  // Error code
#define REG_TEMPERATURES     0x0018  // Temperature -127~127°C
#define REG_MAX_CHARGE_CURR  0x0019  // Max charge/discharge current
#define REG_GAUGE_RM         0x001A  // Remaining capacity 10mAh
#define REG_GAUGE_FCC        0x001B  // Full charge capacity 10mAh
#define REG_CELL_VOLTAGES    0x0071  // Individual cell voltages (1mV units)

// Enhanced safety thresholds based on real-world testing
struct SafetyLimits {
  float maxCellVoltage = 3.60;      // Conservative limit
  float balanceThreshold = 3.363;   // Balance circuits activate above this
  float hardStopVoltage = 3.65;     // Absolute max before BMS shuts down
  float minCellVoltage = 3.00;      // Stop discharging at 3.0V
  float maxBalanceBoardTemp = 176.0; // 80°C in Fahrenheit
  float maxPackVoltage = 29.0;      // Stop charging at 29V pack (2S2P)
  float minPackVoltage = 24.0;      // Stop discharging at 24V pack (2S2P)
  float minBalanceCurrentPack = 0.4; // Minimum current for balancing (total pack)
  float balanceCurrentPerModule = 0.1; // ~100mA balance current per module
  bool manualOverride = false;      // Manual control override
} safetyLimits;

// CATL BMS Data Structure
struct CATLData {
  float cellVoltages[32];      // 4 modules x 8 cells
  float moduleVoltages[4];     // Total voltage per module
  float temperatures[8];       // 2 per module (converted to Celsius)
  float balanceBoardTemps[4];  // Balance board temps per module (converted to Celsius)
  float packVoltage;
  float maxTemp;
  float minTemp;
  float cellSpread;
  float systemCurrent;         // Current from master request
  uint8_t soc;                 // Calculated SOC 0-100%
  bool balancingActive;        // Track if balancing is happening
  bool nodeStatus[4];          // AA, AB, AC, AD connectivity
  bool prechargeActive;        // System status tracking
  bool contactorsEngaged;      // Contactor status
  unsigned long lastUpdate;
} catlData;

// Growatt BMS Response Structure
struct GrowattBMSResponse {
  uint16_t packVoltage;        // 10mV units
  int16_t packCurrent;         // 10mA units (signed)
  uint8_t soc;                 // 0-100%
  uint16_t statusBits;         // Status flags
  uint16_t errorBits;          // Error flags
  int16_t temperature;         // Temperature in Celsius
  uint16_t maxChargeCurrent;   // 10mA units
  uint16_t maxDischargeCurrent; // 10mA units
  uint16_t remainingCapacity;  // 10mAh units
  uint16_t fullChargeCapacity; // 10mAh units
  uint16_t cellVoltages[32];   // 1mV units for individual cells
  unsigned long lastGrowattRequest;
  uint32_t growattRequestCount;
  bool chargeEnabled = true;
  bool dischargeEnabled = true;
} growattResponse;

// Master Request Data Structure (enhanced for current measurement)
struct MasterRequestData {
  uint8_t targetModule;        // AA, AB, AC, AD
  uint8_t requestType;         // 12, 13, 14
  uint8_t chargingByte;        // 00=no charge, 01=charging >0.4A
  uint8_t counter;             // Incrementing counter
  float currentAmps;           // Decoded current value (more accurate)
  float powerWatts;            // Calculated power
  uint32_t count;              // Message count
  unsigned long lastSeen;      // Last seen timestamp
} masterRequest;

// Message tracking structures
struct MessageEntry {
  uint32_t id;
  uint32_t count;
  uint8_t length;
  uint8_t data[8];
  unsigned long lastSeen;
  char description[64];
  char potentialDescription[64];
};

// Enhanced message tracking arrays
MessageEntry unknownMessages[50];
MessageEntry knownMessages[50];
MessageEntry rs485Messages[20];  // Track RS485 Growatt messages
int unknownCount = 0;
int knownCount = 0;
int rs485Count = 0;

// Voltage history for graphing (circular buffer)
struct VoltageEntry {
  float voltage;
  unsigned long timestamp;
  bool valid;
};

struct VoltageHistory {
  VoltageEntry entries[60];  // Keep 60 readings (about 1 hour at 1min intervals)
  int currentIndex = 0;
  int totalEntries = 0;

  void addEntry(float voltage) {
    entries[currentIndex].voltage = voltage;
    entries[currentIndex].timestamp = millis();
    entries[currentIndex].valid = true;
    currentIndex = (currentIndex + 1) % 60;
    if (totalEntries < 60) totalEntries++;
  }

  void clear() {
    for (int i = 0; i < 60; i++) {
      entries[i].valid = false;
    }
    currentIndex = 0;
    totalEntries = 0;
  }
} voltageHistory;

// Time synchronization
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Preferences for persistent storage
Preferences preferences;

// Forward declarations
void setupCAN();
void setupRS485();
void setupWiFi();
void setupWebServer();
void processCATLMessage(can_message_t& msg);
void calculatePackStats();
void handleGrowattRequests();
void updateInverterControl();
void recordVoltage();
void loadVoltageHistory();
void saveVoltageHistory();
String getBMSDataJSON();
String getModulesJSON();
String getMessagesJSON();
String getVoltageHistoryJSON();
String getJavaScript();
String getCSS();
void broadcastToWebSocket(String data);
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// Growatt RS485 functions
void updateGrowattResponse();
void sendGrowattResponse(uint16_t startAddr, uint16_t regCount);
uint16_t getRegisterValue(uint16_t addr);
uint16_t calculateCRC(uint8_t* data, int length);
uint16_t calculateMaxChargeCurrent();
uint16_t calculateMaxDischargeCurrent();
uint16_t calculateStatusBits();
uint16_t calculateErrorBits();
float getMaxCellVoltage();
float getMinCellVoltage();
void captureRS485Message(uint8_t* data, int len, bool isRequest);

// Utility functions
float fahrenheitToCelsius(float fahrenheit) {
  return (fahrenheit - 32.0) * 5.0 / 9.0;
}

float celsiusToFahrenheit(float celsius) {
  return celsius * 9.0 / 5.0 + 32.0;
}

// Helper functions for CAN processing
int getModuleIndex(uint8_t moduleId) {
  switch(moduleId) {
    case 0xAA: return 0;  // Module 1
    case 0xAB: return 1;  // Module 2
    case 0xAC: return 2;  // Module 3
    case 0xAD: return 3;  // Module 4
    default: return -1;
  }
}

void trackKnownMessage(uint32_t id, const char* description, can_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < knownCount; i++) {
    if (knownMessages[i].id == id) {
      knownMessages[i].count++;
      knownMessages[i].lastSeen = millis();
      knownMessages[i].length = msg.len;
      memcpy(knownMessages[i].data, msg.data, 8);
      return;
    }
  }

  // Add new known message
  if (knownCount < 50) {
    knownMessages[knownCount].id = id;
    knownMessages[knownCount].count = 1;
    knownMessages[knownCount].lastSeen = millis();
    knownMessages[knownCount].length = msg.len;
    memcpy(knownMessages[knownCount].data, msg.data, 8);
    strncpy(knownMessages[knownCount].description, description, 63);
    knownMessages[knownCount].description[63] = '\0';
    knownCount++;
  }
}

void logUnknownMessage(can_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < unknownCount; i++) {
    if (unknownMessages[i].id == msg.id) {
      unknownMessages[i].count++;
      unknownMessages[i].lastSeen = millis();
      unknownMessages[i].length = msg.len;
      memcpy(unknownMessages[i].data, msg.data, 8);
      return;
    }
  }

  // Add new unknown message
  if (unknownCount < 50) {
    unknownMessages[unknownCount].id = msg.id;
    unknownMessages[unknownCount].count = 1;
    unknownMessages[unknownCount].lastSeen = millis();
    unknownMessages[unknownCount].length = msg.len;
    memcpy(unknownMessages[unknownCount].data, msg.data, 8);

    // Try to guess what this message might be
    uint32_t id = msg.id;
    if ((id & 0xFFFF0000) == 0x18000000) {
      strcpy(unknownMessages[unknownCount].potentialDescription, "Possible BMS Data");
    } else if ((id & 0xFF000000) == 0x1C000000) {
      strcpy(unknownMessages[unknownCount].potentialDescription, "Possible Control Message");
    } else {
      strcpy(unknownMessages[unknownCount].potentialDescription, "Unknown Pattern");
    }

    unknownCount++;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize RGB LED first for immediate visual feedback
  strip.begin();
  strip.setBrightness(50);  // Set to 50% brightness
  strip.setPixelColor(0, strip.Color(255, 0, 0));  // Red to show we're starting
  strip.show();

  // Initialize data structures
  memset(&catlData, 0, sizeof(catlData));
  memset(&masterRequest, 0, sizeof(masterRequest));
  memset(&growattResponse, 0, sizeof(growattResponse));
  memset(unknownMessages, 0, sizeof(unknownMessages));
  memset(knownMessages, 0, sizeof(knownMessages));
  memset(rs485Messages, 0, sizeof(rs485Messages));

  // Initialize optional control pins (only if they are defined and not 0)
  if (CHARGE_ENABLE_PIN != 0) {
    pinMode(CHARGE_ENABLE_PIN, OUTPUT);
    digitalWrite(CHARGE_ENABLE_PIN, HIGH);    // Start with charging enabled (active high)
  }
  if (DISCHARGE_ENABLE_PIN != 0) {
    pinMode(DISCHARGE_ENABLE_PIN, OUTPUT);
    digitalWrite(DISCHARGE_ENABLE_PIN, HIGH); // Start with discharging enabled
  }

  Serial.println("=== CATL-to-Growatt BMS Bridge (ESP32-S3) ===");
  Serial.println("[INIT] ESP32-S3 CAN <-> RS485 Protocol Bridge with Progressive Loading");

  if (CHARGE_ENABLE_PIN != 0 || DISCHARGE_ENABLE_PIN != 0) {
    Serial.println("[INIT] Optional inverter control pins initialized");
    if (CHARGE_ENABLE_PIN != 0) Serial.printf("   Charge Enable Pin: %d\n", CHARGE_ENABLE_PIN);
    if (DISCHARGE_ENABLE_PIN != 0) Serial.printf("   Discharge Enable Pin: %d\n", DISCHARGE_ENABLE_PIN);
  }

  // Green LED to indicate basic setup complete
  strip.setPixelColor(0, strip.Color(0, 255, 0));  // Green
  strip.show();

  // Initialize ACAN_ESP32 for CATL communication
  setupCAN();

  // Initialize RS485 for Growatt communication
  setupRS485();

  // Connect to WiFi
  setupWiFi();

  // Load voltage history from persistent storage first
  loadVoltageHistory();

  // Initialize time synchronization (non-blocking)
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[TIME] Starting NTP sync (non-blocking)");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }

  // Setup web server with progressive loading
  setupWebServer();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[READY] Bridge ready! Access dashboard at: http://" + WiFi.localIP().toString());
    // White LED for success
    strip.setPixelColor(0, strip.Color(255, 255, 255));
    strip.show();
  } else {
    Serial.println("[WARNING] WiFi not connected - check credentials and signal");
    // Orange LED for failure
    strip.setPixelColor(0, strip.Color(255, 100, 0));
    strip.show();
  }
  Serial.println("[INFO] CATL modules: CAN 250kbps on pins 15/16 (ACAN_ESP32)");
  Serial.println("[INFO] Growatt inverter: RS485 9600 baud on pins 17/18");
  Serial.println("[INFO] Progressive loading dashboard - memory optimized");
  Serial.printf("[MEMORY] Free heap after setup: %d bytes\n", ESP.getFreeHeap());

  // LED pattern: 3 blue flashes to indicate setup complete
  for (int i = 0; i < 3; i++) {
    strip.setPixelColor(0, strip.Color(0, 0, 255));
    strip.show();
    delay(200);
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
    delay(200);
  }
}

void setupCAN() {
  // Initialize ACAN_ESP32 for CATL communication (Waveshare SN65HVD230)
  ACAN_ESP32_Settings settings(250 * 1000); // 250 kbps for CATL BMS
  settings.mRxPin = CAN_RX_PIN; // CAN RX from Waveshare SN65HVD230
  settings.mTxPin = CAN_TX_PIN; // CAN TX to Waveshare SN65HVD230
  settings.mRequestedCANMode = ACAN_ESP32_Settings::ListenOnlyMode; // Listen only for BMS data

  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);

  if (0 == errorCode) {
    Serial.println("[OK] CAN started for CATL communication on pins 15/16");
    // Purple LED to indicate CAN is ready
    strip.setPixelColor(0, strip.Color(128, 0, 128));
    strip.show();
    delay(500);
  } else {
    Serial.print("[ERROR] ACAN_ESP32 CAN init failed, error code: 0x");
    Serial.println(errorCode, HEX);
    // Orange LED for CAN failure
    strip.setPixelColor(0, strip.Color(255, 100, 0));
    strip.show();
    delay(1000);
  }
}

void setupRS485() {
  // Configure RS485 pins
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);  // Start in receive mode

  // Initialize Serial2 for RS485 communication
  Serial2.begin(GROWATT_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println("[OK] RS485 Serial initialized for Growatt communication");
  Serial.printf("[INFO] RS485: TX=%d, RX=%d, DE=%d, Baud=%d\n",
                RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN, GROWATT_BAUD_RATE);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(1000);

  Serial.print("[WIFI] Connecting to ");
  Serial.println(ssid);

  // Apply ESP32 WiFi fix and connect
  boolean autoReconnectStatus = WiFi.getAutoReconnect();
  WiFi.setAutoReconnect(false);
  WiFi.begin(ssid, password);
  WiFi.setBandMode(WIFI_BAND_MODE_2G_ONLY);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setAutoReconnect(autoReconnectStatus);

  // Yellow LED during connection
  strip.setPixelColor(0, strip.Color(255, 255, 0));
  strip.show();

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("[OK] WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[ERROR] WiFi connection failed!");
  }
}

void loop() {
  // Check NTP sync status once
  static bool ntpChecked = false;
  if (!ntpChecked && WiFi.status() == WL_CONNECTED) {
    time_t now;
    time(&now);
    if (now > 1000000000) {
      Serial.println("[TIME] NTP sync successful");
    }
    ntpChecked = true;
  }

  // Read CATL data via ACAN_ESP32
  can_message_t rx_message;
  if (ACAN_ESP32::can.receive(rx_message)) {
    processCATLMessage(rx_message);
    // Blue flash to indicate CAN message received
    strip.setPixelColor(0, strip.Color(0, 0, 255));
    strip.show();

    // Debug: Print CAN message details
    static unsigned long lastCanDebug = 0;
    if (millis() - lastCanDebug > 2000) { // Print every 2 seconds
      Serial.printf("[CAN] Received ID: 0x%08X, Data: ", rx_message.id);
      for (int i = 0; i < rx_message.len; i++) {
        Serial.printf("%02X ", rx_message.data[i]);
      }
      Serial.println();
      lastCanDebug = millis();
    }
  }

  // Handle Growatt RS485 requests
  handleGrowattRequests();

  // Update inverter control based on safety conditions
  updateInverterControl();

  // Record voltage data for history graph (after pack stats are calculated)
  recordVoltage();

  // Send updates to web dashboard every 1 second
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    // Heartbeat LED blink (cyan)
    static bool ledState = false;
    ledState = !ledState;
    if (ledState) {
      strip.setPixelColor(0, strip.Color(0, 255, 255));  // Cyan
    } else {
      strip.setPixelColor(0, strip.Color(0, 0, 0));  // Off
    }
    strip.show();

    String data = getBMSDataJSON();
    broadcastToWebSocket(data);
    lastUpdate = millis();

    // Print status every 10 seconds
    static int statusCounter = 0;
    statusCounter++;
    if (statusCounter >= 10) {
      Serial.printf("[STATUS] Bridge Status: CATL=%s, Growatt=%d reqs, SOC=%d%%, Pack=%.1fV, Balance=%s, Known=%d, Unknown=%d, Free Heap=%d\n",
                    (millis() - catlData.lastUpdate < 5000) ? "OK" : "TIMEOUT",
                    growattResponse.growattRequestCount,
                    catlData.soc,
                    catlData.packVoltage,
                    catlData.balancingActive ? "YES" : "NO",
                    knownCount,
                    unknownCount,
                    ESP.getFreeHeap());

      // Additional CAN debug if no messages received
      if (knownCount == 0 && unknownCount == 0) {
        Serial.println("[CAN] No CAN messages received yet - Check BMS connection and wiring");
        Serial.println("[CAN] Expected: CATL BMS connected to pins 15(TX)/16(RX) via Waveshare SN65HVD230");
        Serial.println("[CAN] Verify: BMS is powered on, CAN bus terminated, 250kbps baud rate");
      }
      statusCounter = 0;
    }
  }

  // Clean up WebSocket clients
  ws.cleanupClients();

  // Watchdog
  delay(10);
}

// Main CAN message processing function (adapted for ACAN_ESP32)
void processCATLMessage(can_message_t& msg) {
  uint32_t id = msg.id;  // ACAN_ESP32 uses .id instead of .identifier
  uint8_t* data = msg.data;
  bool isKnownMessage = false;

  // Update last seen time
  catlData.lastUpdate = millis();

  // Parse voltage messages (cells 1-4)
  if ((id & 0xFFFFFF00) == 0x18001300) {
    isKnownMessage = true;
    trackKnownMessage(id, "Voltage Data (Cells 1-4)", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      catlData.nodeStatus[module] = true;
      for (int i = 0; i < 4; i++) {
        uint16_t voltage = (data[i*2+1] << 8) | data[i*2];
        catlData.cellVoltages[module*8 + i] = voltage / 1000.0;
      }
    }
  }

  // Parse voltage messages (cells 5-8)
  else if ((id & 0xFFFFFF00) == 0x18011300) {
    isKnownMessage = true;
    trackKnownMessage(id, "Voltage Data (Cells 5-8)", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      for (int i = 0; i < 4; i++) {
        uint16_t voltage = (data[i*2+1] << 8) | data[i*2];
        catlData.cellVoltages[module*8 + i + 4] = voltage / 1000.0;
      }
    }
  }

  // Parse temperature messages (convert F to C)
  else if ((id & 0xFFFFFF00) == 0x18001400) {
    isKnownMessage = true;
    trackKnownMessage(id, "Temperature Data", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      uint16_t temp1 = (data[1] << 8) | data[0];
      uint16_t temp2 = (data[3] << 8) | data[2];
      catlData.temperatures[module*2] = fahrenheitToCelsius(temp1/10.0);
      catlData.temperatures[module*2 + 1] = fahrenheitToCelsius(temp2/10.0);
    }
  }

  // Parse balance board temperature messages (convert F to C)
  else if ((id & 0xFFFFFF00) == 0x18031200) {
    isKnownMessage = true;
    trackKnownMessage(id, "Balance Board Temperature", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      uint16_t balanceTemp = (data[1] << 8) | data[0];
      catlData.balanceBoardTemps[module] = fahrenheitToCelsius(balanceTemp / 10.0);
    }
  }

  // Enhanced master request parsing
  else if (id == 0x180011F4) {
    isKnownMessage = true;
    trackKnownMessage(id, "Master Request (Enhanced)", msg);

    masterRequest.targetModule = data[0];  // AA, AB, AC, AD
    masterRequest.requestType = data[1];   // 12, 13, 14
    masterRequest.chargingByte = data[2];  // 00=no charge, 01=charging >0.4A
    masterRequest.counter = data[5];       // Incrementing counter
    masterRequest.lastSeen = millis();
    masterRequest.count++;

    // Decode current measurement (byte 7)
    uint8_t currentRaw = data[7];
    masterRequest.currentAmps = currentRaw * 2.0;  // Based on analysis
    catlData.systemCurrent = masterRequest.currentAmps;

    // Calculate power
    masterRequest.powerWatts = masterRequest.currentAmps * catlData.packVoltage;
  }

  // Track initialization sequence (0x1C04xxxx)
  else if ((id & 0xFFFF0000) == 0x1C040000) {
    isKnownMessage = true;
    trackKnownMessage(id, "Initialization Sequence (1C04)", msg);

    // Check for specific initialization markers
    if (data[0] == 0x01) {
      Serial.println("[BMS] BMS Initialization sequence detected");
      catlData.prechargeActive = false;  // Init complete
      catlData.contactorsEngaged = true;
    }
  }

  // If unknown, log it
  if (!isKnownMessage) {
    logUnknownMessage(msg);
  }

  // Calculate derived values
  calculatePackStats();
}

// Calculate pack statistics
void calculatePackStats() {
  float maxVoltage = 0;
  float minVoltage = 5.0;
  float maxTemp = -50;
  float minTemp = 100;

  // Calculate pack voltage for 2S2P configuration
  float series1Total = 0;  // First series string (modules AA+AB, cells 0-15)
  float series2Total = 0;  // Second series string (modules AC+AD, cells 16-31)

  // Sum first series string (16 cells)
  for (int i = 0; i < 16; i++) {
    if (catlData.cellVoltages[i] > 0) {
      series1Total += catlData.cellVoltages[i];
    }
  }

  // Sum second series string (16 cells)
  for (int i = 16; i < 32; i++) {
    if (catlData.cellVoltages[i] > 0) {
      series2Total += catlData.cellVoltages[i];
    }
  }

  // Pack voltage = average of both series strings
  catlData.packVoltage = (series1Total + series2Total) / 2.0;

  // Calculate cell spread across all active cells
  for (int i = 0; i < 32; i++) {
    if (catlData.cellVoltages[i] > 0) {
      if (catlData.cellVoltages[i] > maxVoltage) maxVoltage = catlData.cellVoltages[i];
      if (catlData.cellVoltages[i] < minVoltage) minVoltage = catlData.cellVoltages[i];
    }
  }

  // Calculate module voltages (8 cells each)
  for (int m = 0; m < 4; m++) {
    float moduleTotal = 0;
    for (int c = 0; c < 8; c++) {
      moduleTotal += catlData.cellVoltages[m*8 + c];
    }
    catlData.moduleVoltages[m] = moduleTotal;
  }

  // Calculate temperature range
  for (int i = 0; i < 8; i++) {
    if (catlData.temperatures[i] > maxTemp) maxTemp = catlData.temperatures[i];
    if (catlData.temperatures[i] < minTemp) minTemp = catlData.temperatures[i];
  }

  // Include balance board temperatures
  for (int i = 0; i < 4; i++) {
    if (catlData.balanceBoardTemps[i] > maxTemp) maxTemp = catlData.balanceBoardTemps[i];
    if (catlData.balanceBoardTemps[i] > 0 && catlData.balanceBoardTemps[i] < minTemp)
      minTemp = catlData.balanceBoardTemps[i];
  }

  catlData.cellSpread = (maxVoltage - minVoltage) * 1000; // mV
  catlData.maxTemp = maxTemp;
  catlData.minTemp = minTemp;

  // Calculate SOC based on pack voltage for 2S2P configuration
  float minPackV = 24.0;   // 16 cells * 1.5V (conservative low)
  float maxPackV = 28.8;   // 16 cells * 1.8V (conservative high)
  float currentV = catlData.packVoltage;

  if (currentV <= minPackV) {
    catlData.soc = 0;
  } else if (currentV >= maxPackV) {
    catlData.soc = 100;
  } else {
    catlData.soc = (uint8_t)(((currentV - minPackV) / (maxPackV - minPackV)) * 100);
  }

  // Check if balancing is active (any cell > balance threshold with current flowing)
  catlData.balancingActive = false;
  if (catlData.systemCurrent > safetyLimits.minBalanceCurrentPack) {
    for (int i = 0; i < 32; i++) {
      if (catlData.cellVoltages[i] > safetyLimits.balanceThreshold) {
        catlData.balancingActive = true;
        break;
      }
    }
  }
}

// RS485 Message tracking
void captureRS485Message(uint8_t* data, int len, bool isRequest) {
  if (rs485Count >= 20) rs485Count = 0; // Wrap around

  rs485Messages[rs485Count].id = isRequest ? 0x00000001 : 0x00000002;
  rs485Messages[rs485Count].count = 1;
  rs485Messages[rs485Count].length = len;
  rs485Messages[rs485Count].lastSeen = millis();
  memcpy(rs485Messages[rs485Count].data, data, min(len, 8));
  strncpy(rs485Messages[rs485Count].description, isRequest ? "Growatt Request" : "BMS Response", 63);
  rs485Count++;
}

void handleGrowattRequests() {
  uint8_t buffer[256];
  int len = 0;

  // Read from Serial2 (ESP32-S3 adaptation)
  if (Serial2.available()) {
    len = Serial2.readBytes(buffer, min((int)Serial2.available(), 256));
  }

  // Debug: Log ANY data received (even if not valid Modbus)
  if (len > 0) {
    Serial.printf("[RS485 DEBUG] Received %d bytes: ", len);
    for (int i = 0; i < len && i < 16; i++) {
      Serial.printf("%02X ", buffer[i]);
    }
    Serial.println();

    // Capture the request for monitoring
    captureRS485Message(buffer, len, true);

    // Parse Modbus request
    if (len >= 8 && buffer[0] == GROWATT_SLAVE_ADDR && buffer[1] == 0x03) {
      uint16_t startAddr = (buffer[2] << 8) | buffer[3];
      uint16_t regCount = (buffer[4] << 8) | buffer[5];

      Serial.printf("[GROWATT] Growatt request: addr=0x%04X, count=%d\n", startAddr, regCount);
      growattResponse.lastGrowattRequest = millis();
      growattResponse.growattRequestCount++;

      // Update Growatt response data first
      updateGrowattResponse();

      // Send appropriate response
      sendGrowattResponse(startAddr, regCount);
    }
  }
}

// Helper functions for Growatt response calculation
float getMaxCellVoltage() {
  float maxV = 0;
  for (int i = 0; i < 32; i++) {
    if (catlData.cellVoltages[i] > maxV) maxV = catlData.cellVoltages[i];
  }
  return maxV;
}

float getMinCellVoltage() {
  float minV = 5.0;
  for (int i = 0; i < 32; i++) {
    if (catlData.cellVoltages[i] > 0 && catlData.cellVoltages[i] < minV) minV = catlData.cellVoltages[i];
  }
  return minV;
}

void updateGrowattResponse() {
  // Convert CATL data to Growatt format
  growattResponse.packVoltage = (uint16_t)(catlData.packVoltage * 100);  // Convert to 10mV units
  growattResponse.packCurrent = (int16_t)(catlData.systemCurrent * 100); // Convert to 10mA units
  growattResponse.soc = catlData.soc;
  growattResponse.temperature = (int16_t)catlData.maxTemp; // Already in Celsius
  growattResponse.maxChargeCurrent = calculateMaxChargeCurrent();
  growattResponse.maxDischargeCurrent = calculateMaxDischargeCurrent();
  growattResponse.statusBits = calculateStatusBits();
  growattResponse.errorBits = calculateErrorBits();
  growattResponse.remainingCapacity = (uint16_t)(catlData.soc * 5000 / 100); // Estimate: 50Ah * SOC
  growattResponse.fullChargeCapacity = 5000; // 50Ah in 10mAh units

  // Copy individual cell voltages (convert to 1mV units)
  for (int i = 0; i < 32; i++) {
    growattResponse.cellVoltages[i] = (uint16_t)(catlData.cellVoltages[i] * 1000);
  }
}

uint16_t calculateMaxChargeCurrent() {
  // Adaptive charge current based on safety logic (return in 10mA units)
  float maxCellV = getMaxCellVoltage();
  float cellSpreadMv = catlData.cellSpread;

  if (maxCellV > safetyLimits.hardStopVoltage) return 0;      // Stop charging
  if (maxCellV > safetyLimits.maxCellVoltage) return 500;     // 5A
  if (maxCellV > 3.5 && cellSpreadMv > 100) return 1000;     // 10A - pause for balancing
  if (maxCellV > 3.5) return 1500;                           // 15A
  if (cellSpreadMv > 50) return 1500;                        // 15A
  return 2000; // 20A normal charging
}

uint16_t calculateMaxDischargeCurrent() {
  // Calculate max discharge current (return in 10mA units)
  float minCellV = getMinCellVoltage();

  if (minCellV < safetyLimits.minCellVoltage) return 0;       // Stop discharging
  if (minCellV < 3.1) return 1000;                           // 10A limited
  if (minCellV < 3.2) return 2000;                           // 20A limited
  return 3000; // 30A normal discharge
}

uint16_t calculateStatusBits() {
  uint16_t status = 0;

  // Bit 0-1: Operation status (00: soft_starting, 01: standby, 10: charging, 11: discharging)
  if (catlData.systemCurrent > 0.5) {
    status |= 0x02; // Charging
  } else if (catlData.systemCurrent < -0.5) {
    status |= 0x03; // Discharging
  } else {
    status |= 0x01; // Standby
  }

  // Bit 2: Error flag
  if (getMaxCellVoltage() > safetyLimits.hardStopVoltage ||
      getMinCellVoltage() < safetyLimits.minCellVoltage) {
    status |= 0x04; // Error bit
  }

  // Bit 3: Balancing active
  if (catlData.balancingActive) {
    status |= 0x08;
  }

  return status;
}

uint16_t calculateErrorBits() {
  uint16_t errors = 0;

  // Error bit definitions
  if (getMaxCellVoltage() > safetyLimits.hardStopVoltage) errors |= 0x01; // Overvoltage
  if (getMinCellVoltage() < safetyLimits.minCellVoltage) errors |= 0x02;  // Undervoltage
  if (catlData.maxTemp > safetyLimits.maxBalanceBoardTemp) errors |= 0x04; // Overtemp
  if (catlData.cellSpread > 200) errors |= 0x08; // Cell imbalance

  return errors;
}

void updateInverterControl() {
  // Enhanced safety control logic
  bool chargeEnabled = true;
  bool dischargeEnabled = true;

  // Check cell voltages for charge control
  float maxV = getMaxCellVoltage();
  if (maxV > safetyLimits.hardStopVoltage) {
    chargeEnabled = false;
    Serial.println("[SAFETY] Charge disabled - overvoltage detected");
  }

  // Check cell voltages for discharge control
  float minV = getMinCellVoltage();
  if (minV < safetyLimits.minCellVoltage) {
    dischargeEnabled = false;
    Serial.println("[SAFETY] Discharge disabled - undervoltage detected");
  }

  // Check temperature limits
  if (catlData.maxTemp > safetyLimits.maxBalanceBoardTemp) {
    chargeEnabled = false;
    dischargeEnabled = false;
    Serial.println("[SAFETY] All operations disabled - overtemperature");
  }

  // Update control pins if defined
  if (CHARGE_ENABLE_PIN != 0) {
    digitalWrite(CHARGE_ENABLE_PIN, chargeEnabled ? HIGH : LOW);
  }
  if (DISCHARGE_ENABLE_PIN != 0) {
    digitalWrite(DISCHARGE_ENABLE_PIN, dischargeEnabled ? HIGH : LOW);
  }

  // Update response status
  growattResponse.chargeEnabled = chargeEnabled;
  growattResponse.dischargeEnabled = dischargeEnabled;
}

void sendGrowattResponse(uint16_t startAddr, uint16_t regCount) {
  uint8_t response[256];
  int responseLen = 0;

  // Build Modbus response header
  response[0] = GROWATT_SLAVE_ADDR;  // Slave address
  response[1] = 0x03;                // Function code
  response[2] = regCount * 2;        // Byte count
  responseLen = 3;

  // Fill register data based on requested address
  for (uint16_t i = 0; i < regCount; i++) {
    uint16_t addr = startAddr + i;
    uint16_t value = getRegisterValue(addr);

    response[responseLen++] = (value >> 8) & 0xFF;  // High byte
    response[responseLen++] = value & 0xFF;         // Low byte
  }

  // Add CRC
  uint16_t crc = calculateCRC(response, responseLen);
  response[responseLen++] = crc & 0xFF;         // CRC low
  response[responseLen++] = (crc >> 8) & 0xFF;  // CRC high

  // Capture response for monitoring
  captureRS485Message(response, responseLen, false);

  // Send response via Serial2 (ESP32-S3 adaptation)
  digitalWrite(RS485_DE_PIN, HIGH); // Enable transmit
  delay(1); // Short delay for transceiver switching
  Serial2.write(response, responseLen);
  Serial2.flush(); // Wait for transmission complete
  digitalWrite(RS485_DE_PIN, LOW); // Back to receive

  Serial.printf("[GROWATT] Sent %d bytes to Growatt\n", responseLen);
}

uint16_t getRegisterValue(uint16_t addr) {
  switch (addr) {
    case REG_PACK_VOLTAGE:
      return growattResponse.packVoltage;
    case REG_PACK_CURRENT:
      return growattResponse.packCurrent;
    case REG_SOC:
      return growattResponse.soc;
    case REG_STATUS:
      return growattResponse.statusBits;
    case REG_ERROR:
      return growattResponse.errorBits;
    case REG_TEMPERATURES:
      return growattResponse.temperature;
    case REG_MAX_CHARGE_CURR:
      return growattResponse.maxChargeCurrent;
    case REG_GAUGE_RM:
      return growattResponse.remainingCapacity;
    case REG_GAUGE_FCC:
      return growattResponse.fullChargeCapacity;
    default:
      // Handle cell voltage registers (0x0071-0x0090)
      if (addr >= REG_CELL_VOLTAGES && addr < REG_CELL_VOLTAGES + 32) {
        int cellIndex = addr - REG_CELL_VOLTAGES;
        if (cellIndex < 32) {
          return growattResponse.cellVoltages[cellIndex];
        }
      }
      return 0; // Unknown register
  }
}

uint16_t calculateCRC(uint8_t* data, int length) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x01) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void recordVoltage() {
  static unsigned long lastRecord = 0;

  if (millis() - lastRecord > 60000) {  // Record every minute
    if (catlData.packVoltage >= 20.0) {  // Reasonable minimum for 2S2P
      voltageHistory.addEntry(catlData.packVoltage);
      saveVoltageHistory();
      lastRecord = millis();
    }
  }
}

void loadVoltageHistory() {
  preferences.begin("voltage", false);

  for (int i = 0; i < 60; i++) {
    String key = "v" + String(i);
    if (preferences.isKey(key.c_str())) {
      voltageHistory.entries[i].voltage = preferences.getFloat(key.c_str(), 0.0);
      voltageHistory.entries[i].valid = (voltageHistory.entries[i].voltage > 0);
      if (voltageHistory.entries[i].valid) {
        voltageHistory.totalEntries++;
      }
    }
  }

  preferences.end();
  Serial.printf("[STORAGE] Loaded %d voltage history entries\n", voltageHistory.totalEntries);
}

void saveVoltageHistory() {
  preferences.begin("voltage", false);

  // Save only valid entries
  int saved = 0;
  for (int i = 0; i < 60; i++) {
    String key = "v" + String(i);
    if (voltageHistory.entries[i].valid) {
      preferences.putFloat(key.c_str(), voltageHistory.entries[i].voltage);
      saved++;
    }
  }

  preferences.end();
}

String getBMSDataJSON() {
  StaticJsonDocument<1024> json;

  json["voltage"] = catlData.packVoltage;
  json["current"] = catlData.systemCurrent;
  json["soc"] = catlData.soc;
  json["temperature"] = catlData.maxTemp;
  json["connected"] = (millis() - catlData.lastUpdate < 5000);
  json["lastUpdate"] = catlData.lastUpdate;
  json["cellSpread"] = catlData.cellSpread;
  json["minTemp"] = catlData.minTemp;
  json["maxTemp"] = catlData.maxTemp;
  json["knownMessages"] = knownCount;
  json["unknownMessages"] = unknownCount;
  json["masterMessages"] = masterRequest.count;
  json["balancing"] = catlData.balancingActive;
  json["power"] = catlData.packVoltage * catlData.systemCurrent;

  String response;
  serializeJson(json, response);
  return response;
}

// WebSocket event handler
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] WebSocket client connected: %u from IP %s\n", client->id(), client->remoteIP().toString().c_str());
    // Send initial data
    String jsonData = getBMSDataJSON();
    if (jsonData.length() > 0) {
      client->text(jsonData);
    }
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] WebSocket client disconnected: %u\n", client->id());
  }
}

void broadcastToWebSocket(String data) {
  // Check if we have any WebSocket clients
  if (ws.count() == 0) {
    return; // No clients connected, skip broadcast
  }

  // Broadcast to all connected clients
  ws.textAll(data);
}

void setupWebServer() {
  // Ensure network is properly initialized
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ERROR] WiFi not connected - cannot start web server");
    return;
  }

  // Handle WebSocket connections
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Main dashboard (minimal HTML)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = getBasicHTML();
    request->send(200, "text/html", html);
  });

  // API endpoints
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String data = getBMSDataJSON();
    request->send(200, "application/json", data);
  });

  server.on("/api/modules", HTTP_GET, [](AsyncWebServerRequest *request) {
    String data = getModulesJSON();
    request->send(200, "application/json", data);
  });

  server.on("/api/messages", HTTP_GET, [](AsyncWebServerRequest *request) {
    String data = getMessagesJSON();
    request->send(200, "application/json", data);
  });

  server.on("/api/voltage-history", HTTP_GET, [](AsyncWebServerRequest *request) {
    String data = getVoltageHistoryJSON();
    request->send(200, "application/json", data);
  });

  // CSS and JavaScript
  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    String css = getCSS();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/css", css);
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    request->send(response);
  });

  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    String js = getJavaScript();
    AsyncWebServerResponse *response = request->beginResponse(200, "application/javascript", js);
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    request->send(response);
  });

  server.begin();
  Serial.println("[OK] Web server started with WebSocket support");
}

String getBasicHTML() {
  String html;
  html.reserve(2048);

  html = "<!DOCTYPE html><html><head><title>CATL-to-Growatt BMS Bridge</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<link rel='icon' href='data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTkuNSAxTDQgOUg3TDYuNSAxNUwxMiA3SDlMOS41IDFaIiBmaWxsPSIjRkZENzAwIiBzdHJva2U9IiNGRjg5MDAiIHN0cm9rZS13aWR0aD0iMC41Ii8+Cjwvc3ZnPgo=' type='image/svg+xml'>";
  html += "<link rel='stylesheet' href='/styles.css'>";
  html += "</head><body>";

  html += "<div id='connection-status' class='disconnected'>Connecting...</div>";
  html += "<button class='clear-button' onclick='clearAllData()'>Clear Data</button>";

  html += "<div class='header'><h1>&#128279; CATL-to-Growatt BMS Bridge</h1>";
  html += "<p>ESP32 Bridge: CAN &#8596; RS485 Protocol Translation with Enhanced Control</p></div>";

  html += "<div class='loading-indicator' id='loading'>Loading dashboard components...</div>";

  html += "<div class='bridge-status'>";
  html += "<h3>&#127760; Bridge Status</h3>";
  html += "<div style='display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px;'>";
  html += "<div><strong>Growatt Requests:</strong> <span id='growatt-requests'>---</span></div>";
  html += "<div><strong>Bridge Active:</strong> <span id='bridge-active'>---</span></div>";
  html += "<div><strong>Last Growatt Request:</strong> <span id='last-request'>---</span></div>";
  html += "<div><strong>RS485 Status:</strong> <span id='rs485-status'>Ready</span></div>";
  html += "</div></div>";

  html += "<div class='stats' id='main-stats'></div>";

  // Voltage history graph section (moved up after stats)
  html += "<div class='voltage-graph-section'>";
  html += "<h3>Pack Voltage History (Last 60 Readings)</h3>";
  html += "<p>Hover over data points for exact values and timestamps</p>";
  html += "<canvas id='voltageCanvas' width='800' height='400'></canvas>";
  html += "<div id='tooltipDiv'></div>";
  html += "</div>";

  html += "<div class='modules' id='modules'></div>";
  html += "<div class='control-panel' id='control-panel'></div>";
  html += "<div class='control-panel' id='master-request-panel'></div>";
  html += "<div class='message-section' id='unknown-messages'></div>";
  html += "<div class='message-section' id='known-messages'></div>";
  html += "<div class='message-section' id='rs485-monitor'></div>";

  html += "<script src='/script.js?v=7'></script>";
  html += "</body></html>";

  return html;
}

String getCSS() {
  String css;
  css.reserve(4096);

  css = "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }";
  css += ".header { background: #2c3e50; color: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; }";
  css += ".stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; margin-bottom: 20px; }";
  css += ".stat { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center; }";
  css += ".stat-value { font-size: 20px; font-weight: bold; color: #27ae60; }";
  css += ".stat-label { font-size: 12px; color: #7f8c8d; margin-bottom: 5px; }";
  css += ".bridge-status { background: #34495e; color: white; padding: 15px; border-radius: 8px; margin-bottom: 20px; }";
  css += ".modules { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin-bottom: 30px; }";
  css += ".module { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  css += ".module-header { font-size: 18px; font-weight: bold; margin-bottom: 10px; display: flex; justify-content: space-between; align-items: center; }";
  css += ".module-voltage { font-size: 16px; color: #2c3e50; font-weight: bold; margin-bottom: 8px; text-align: center; background: #ecf0f1; padding: 8px; border-radius: 4px; }";
  css += ".balance-temp { font-size: 12px; color: #e67e22; font-weight: bold; text-align: center; background: #fdf2e9; padding: 4px; border-radius: 3px; margin-bottom: 8px; }";
  css += ".status-dot { width: 12px; height: 12px; border-radius: 50%; background: #e74c3c; }";
  css += ".status-dot.connected { background: #27ae60; }";
  css += ".cells { display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; margin-bottom: 10px; }";
  css += ".cell { padding: 8px; text-align: center; border-radius: 4px; font-size: 12px; color: white; background: #95a5a6; }";
  css += ".cell.good { background: #27ae60; }";
  css += ".cell.warning { background: #f39c12; }";
  css += ".cell.danger { background: #e74c3c; }";
  css += ".temps { font-size: 14px; color: #7f8c8d; }";
  css += ".control-panel { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 20px; }";
  css += ".control-panel h3 { color: #8e44ad; margin-bottom: 15px; }";
  css += ".control-data { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 10px; margin-bottom: 15px; }";
  css += ".control-item { background: #f8f9fa; padding: 10px; border-radius: 4px; text-align: center; }";
  css += ".control-item .label { font-size: 11px; color: #6c757d; margin-bottom: 3px; }";
  css += ".control-item .value { font-size: 14px; font-weight: bold; color: #495057; font-family: monospace; }";
  css += ".message-section { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 20px; }";
  css += ".message-table { width: 100%; border-collapse: collapse; margin-top: 15px; }";
  css += ".message-table th, .message-table td { border: 1px solid #ddd; padding: 8px; text-align: left; font-size: 12px; }";
  css += ".message-table th { background-color: #f2f2f2; font-weight: bold; }";
  css += ".message-table tr:nth-child(even) { background-color: #f9f9f9; }";
  css += ".data-hex { font-family: monospace; font-size: 11px; }";
  css += "#connection-status { position: fixed; top: 10px; right: 10px; padding: 10px; border-radius: 4px; color: white; font-weight: bold; }";
  css += ".connected { background: #27ae60; }";
  css += ".disconnected { background: #e74c3c; }";
  css += ".clear-button { position: fixed; top: 60px; right: 10px; padding: 10px 15px; background: #e74c3c; color: white; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; }";
  css += ".clear-button:hover { background: #c0392b; }";
  css += ".loading-indicator { background: #3498db; color: white; padding: 15px; border-radius: 8px; text-align: center; margin-bottom: 20px; }";

  // Voltage graph styles
  css += ".voltage-graph-section { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 20px; text-align: center; width: 100%; box-sizing: border-box; }";
  css += ".voltage-graph-section h3 { color: #2c3e50; margin-bottom: 10px; }";
  css += ".voltage-graph-section p { color: #7f8c8d; font-size: 14px; margin-bottom: 15px; }";
  css += "#voltageCanvas { border: 1px solid #bdc3c7; border-radius: 4px; width: 100%; max-width: 100%; height: 400px; display: block; }";
  css += "#tooltipDiv { position: absolute; background: rgba(0,0,0,0.9); color: white; padding: 8px 12px; border-radius: 6px; font-size: 12px; pointer-events: none; display: none; z-index: 1000; white-space: nowrap; border: 1px solid rgba(255,255,255,0.2); }";

  return css;
}

String getJavaScript() {
  String js;
  js.reserve(6144);

  js = "console.log('CATL Dashboard JS loaded!');";
  js += "var ws; var reconnectTimer; var loadingSteps = 0; var totalSteps = 5; var voltageData = [];";

  js += "function updateLoadingProgress() {";
  js += "loadingSteps++; console.log('Loading progress:', loadingSteps, '/', totalSteps);";
  js += "if (loadingSteps >= totalSteps) { document.getElementById('loading').style.display = 'none'; }";
  js += "else { document.getElementById('loading').textContent = 'Loading... (' + loadingSteps + '/' + totalSteps + ')'; }";
  js += "}";

  js += "function connectWebSocket() {";
  js += "ws = new WebSocket('ws://' + window.location.host + '/ws');";
  js += "ws.onopen = function() { console.log('WebSocket connected'); document.getElementById('connection-status').textContent = 'Connected'; document.getElementById('connection-status').className = 'connected'; loadInitialData(); };";
  js += "ws.onmessage = function(event) { try { var data = JSON.parse(event.data); updateMainData(data); } catch (e) { console.error('Parse error:', e); } };";
  js += "ws.onclose = function() { console.log('WebSocket disconnected'); document.getElementById('connection-status').textContent = 'Disconnected'; document.getElementById('connection-status').className = 'disconnected'; reconnectTimer = setTimeout(connectWebSocket, 3000); };";
  js += "}";

  js += "function loadInitialData() {";
  js += "loadMainStats(); loadControlPanels(); setTimeout(loadModules, 200); setTimeout(loadMessages, 400); setTimeout(loadVoltageGraph, 600);";
  js += "}";

  js += "function loadMainStats() {";
  js += "var statsHtml = '';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">Pack Voltage</div><div class=\"stat-value\" id=\"pack-voltage\">---</div></div>';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">SOC</div><div class=\"stat-value\" id=\"soc\">---</div></div>';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">Cell Spread</div><div class=\"stat-value\" id=\"cell-spread\">---</div></div>';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">Temperature Range</div><div class=\"stat-value\" id=\"temp-range\">---</div></div>';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">System Current</div><div class=\"stat-value\" id=\"system-current\">---</div></div>';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">System Power</div><div class=\"stat-value\" id=\"system-power\">---</div></div>';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">Last Update</div><div class=\"stat-value\" id=\"last-update\">---</div></div>';";
  js += "statsHtml += '<div class=\"stat\"><div class=\"stat-label\">Balancing Status</div><div class=\"stat-value\" id=\"balance-status\">---</div></div>';";
  js += "document.getElementById('main-stats').innerHTML = statsHtml; updateLoadingProgress();";
  js += "}";

  js += "function loadControlPanels() {";
  js += "var controlHtml = '<h3>&#9889; Inverter Control Panel</h3>';";
  js += "controlHtml += '<div class=\"control-data\">';";
  js += "controlHtml += '<div class=\"control-item\"><div class=\"label\">Charge Status</div><div class=\"value\" id=\"charge-status\">---</div></div>';";
  js += "controlHtml += '<div class=\"control-item\"><div class=\"label\">Discharge Status</div><div class=\"value\" id=\"discharge-status\">---</div></div>';";
  js += "controlHtml += '<div class=\"control-item\"><div class=\"label\">Auto Mode</div><div class=\"value\" id=\"auto-mode\">---</div></div>';";
  js += "controlHtml += '<div class=\"control-item\"><div class=\"label\">Balancing Active</div><div class=\"value\" id=\"balancing-active\">---</div></div>';";
  js += "controlHtml += '<div class=\"control-item\"><div class=\"label\">Suggested Current</div><div class=\"value\" id=\"suggested-current\">---</div></div>';";
  js += "controlHtml += '<div class=\"control-item\"><div class=\"label\">Last Action</div><div class=\"value\" id=\"last-action\" style=\"font-size:10px;\">---</div></div>';";
  js += "controlHtml += '</div>';";
  js += "document.getElementById('control-panel').innerHTML = controlHtml;";
  js += "var masterHtml = '<h3>&#127919; Master Request Analysis</h3>';";
  js += "masterHtml += '<div class=\"control-data\">';";
  js += "masterHtml += '<div class=\"control-item\"><div class=\"label\">Target Module</div><div class=\"value\" id=\"master-module\">---</div></div>';";
  js += "masterHtml += '<div class=\"control-item\"><div class=\"label\">Request Type</div><div class=\"value\" id=\"master-type\">---</div></div>';";
  js += "masterHtml += '<div class=\"control-item\"><div class=\"label\">Counter</div><div class=\"value\" id=\"master-counter\">---</div></div>';";
  js += "masterHtml += '<div class=\"control-item\"><div class=\"label\">Current (A)</div><div class=\"value\" id=\"master-current\">---</div></div>';";
  js += "masterHtml += '<div class=\"control-item\"><div class=\"label\">Power (W)</div><div class=\"value\" id=\"master-power\">---</div></div>';";
  js += "masterHtml += '<div class=\"control-item\"><div class=\"label\">Message Count</div><div class=\"value\" id=\"master-count\">---</div></div>';";
  js += "masterHtml += '</div>';";
  js += "document.getElementById('master-request-panel').innerHTML = masterHtml; updateLoadingProgress();";
  js += "}";

  js += "function loadModules(skipProgressUpdate) {";
  js += "console.log('Loading modules...');";
  js += "fetch('/api/modules').then(response => {";
  js += "if (!response.ok) throw new Error('Modules API failed');";
  js += "return response.json();";
  js += "}).then(data => {";
  js += "console.log('Modules data loaded:', data);";
  js += "updateModules(data.modules); if (!skipProgressUpdate) updateLoadingProgress();";
  js += "}).catch(error => { console.error('Failed to load modules:', error); if (!skipProgressUpdate) updateLoadingProgress(); });";
  js += "}";

  js += "function loadMessages(skipProgressUpdate) {";
  js += "console.log('Loading messages...');";
  js += "fetch('/api/messages').then(response => {";
  js += "if (!response.ok) throw new Error('Messages API failed');";
  js += "return response.json();";
  js += "}).then(data => {";
  js += "console.log('Messages data loaded:', data);";
  js += "updateMessages(data); if (!skipProgressUpdate) updateLoadingProgress();";
  js += "}).catch(error => { console.error('Failed to load messages:', error); if (!skipProgressUpdate) updateLoadingProgress(); });";
  js += "}";

  js += "function updateModules(modules) {";
  js += "var modulesDiv = document.getElementById('modules'); modulesDiv.innerHTML = '';";
  js += "if (modules && modules.length) { modules.forEach(function(module, index) {";
  js += "var moduleDiv = document.createElement('div'); moduleDiv.className = 'module';";
  js += "var cellsHtml = ''; var cells = module.cells || [];";
  js += "for (var i = 0; i < cells.length; i++) {";
  js += "var voltage = cells[i]; var cellClass = 'good';";
  js += "if (voltage < 3.0 || voltage > 3.4) cellClass = 'danger';";
  js += "else if (voltage < 3.1 || voltage > 3.35) cellClass = 'warning';";
  js += "cellsHtml += '<div class=\"cell ' + cellClass + '\">Cell ' + (i + 1) + '<br>' + (voltage || 0).toFixed(3) + 'V</div>';";
  js += "}";
  js += "var moduleNames = ['AA', 'AB', 'AC', 'AD'];";
  js += "var balanceTemp = module.balanceTemp || 0;";
  js += "var balanceTempHtml = balanceTemp > 0 ? '<div class=\"balance-temp\">Balance: ' + balanceTemp.toFixed(1) + '&deg;C</div>' : '';";
  js += "var moduleHeader = '<div class=\"module-header\">Module ' + (index + 1) + ' (' + moduleNames[index] + ')<div class=\"status-dot ' + (module.connected ? 'connected' : '') + '\"></div></div>';";
  js += "var moduleVoltage = '<div class=\"module-voltage\">' + (module.voltage || 0).toFixed(3) + 'V</div>';";
  js += "var moduleCells = '<div class=\"cells\">' + cellsHtml + '</div>';";
  js += "var moduleTemps = '<div class=\"temps\">Temps: ' + module.temp1.toFixed(1) + '&deg;C, ' + module.temp2.toFixed(1) + '&deg;C</div>';";
  js += "moduleDiv.innerHTML = moduleHeader + moduleVoltage + balanceTempHtml + moduleCells + moduleTemps;";
  js += "modulesDiv.appendChild(moduleDiv); }); }";
  js += "}";

  js += "function updateMessages(data) {";
  js += "var unknownHtml = '<h2>Unknown CAN Messages</h2>';";
  js += "unknownHtml += '<p>Messages that do not match known protocol patterns:</p>';";
  js += "unknownHtml += '<table class=\"message-table\"><thead><tr>';";
  js += "unknownHtml += '<th>CAN ID</th><th>Count</th><th>Length</th><th>Data (Hex)</th><th>Potential Description</th><th>Last Seen</th>';";
  js += "unknownHtml += '</tr></thead><tbody>';";
  js += "if (data.unknownCount > 0) {";
  js += "unknownHtml += '<tr><td colspan=\"6\">Found ' + data.unknownCount + ' unknown message types</td></tr>';";
  js += "} else { unknownHtml += '<tr><td colspan=\"6\">No unknown messages detected</td></tr>'; }";
  js += "unknownHtml += '</tbody></table>';";
  js += "document.getElementById('unknown-messages').innerHTML = unknownHtml;";
  js += "var knownHtml = '<h2>Known CAN Messages</h2>';";
  js += "knownHtml += '<p>Messages we have identified and understand:</p>';";
  js += "knownHtml += '<table class=\"message-table\"><thead><tr>';";
  js += "knownHtml += '<th>CAN ID</th><th>Description</th><th>Count</th><th>Length</th><th>Data (Hex)</th><th>Last Seen</th>';";
  js += "knownHtml += '</tr></thead><tbody>';";
  js += "if (data.known && data.known.length > 0) {";
  js += "data.known.forEach(function(msg) { knownHtml += '<tr><td>0x' + msg.id + '</td><td>' + msg.desc + '</td><td>' + msg.count + '</td><td>8</td><td class=\"data-hex\">Live Data</td><td>Active</td></tr>'; });";
  js += "} else { knownHtml += '<tr><td colspan=\"6\">No known messages yet</td></tr>'; }";
  js += "knownHtml += '</tbody></table>';";
  js += "document.getElementById('known-messages').innerHTML = knownHtml;";
  js += "}";

  js += "function loadVoltageGraph(skipProgressUpdate) {";
  js += "console.log('Loading voltage graph...');";
  js += "fetch('/api/voltage-history').then(response => {";
  js += "if (!response.ok) throw new Error('Voltage history API failed');";
  js += "return response.json();";
  js += "}).then(data => {";
  js += "console.log('Voltage history loaded:', data);";
  js += "voltageData = data.entries || []; drawVoltageGraph(); setupCanvasMouseEvents(); if (!skipProgressUpdate) updateLoadingProgress();";
  js += "}).catch(error => { console.error('Failed to load voltage history:', error); if (!skipProgressUpdate) updateLoadingProgress(); });";
  js += "}";
  js += "function drawVoltageGraph() {";
  js += "var canvas = document.getElementById('voltageCanvas');";
  js += "if (!canvas) return;";
  js += "var ctx = canvas.getContext('2d');";
  js += "var container = canvas.parentElement;";
  js += "var containerWidth = container.clientWidth - 40;";
  js += "canvas.width = containerWidth;";
  js += "canvas.height = 400;";
  js += "canvas.style.width = containerWidth + 'px';";
  js += "canvas.style.height = '400px';";
  js += "ctx.clearRect(0, 0, canvas.width, canvas.height);";
  js += "var tooltip = document.getElementById('tooltipDiv'); if (tooltip) { tooltip.style.display = 'none'; }";
  js += "if (!Array.isArray(voltageData) || voltageData.length === 0) {";
  js += "ctx.fillStyle = '#7f8c8d'; ctx.font = '16px Arial'; ctx.textAlign = 'center';";
  js += "ctx.fillText('No voltage data available yet', canvas.width/2, canvas.height/2);";
  js += "canvas.packVoltagePoints = []; canvas.voltageMeta = null;";
  js += "return; }";
  js += "";
  js += "var margin = { top: 20, right: 50, bottom: 60, left: 60 };";
  js += "var chartWidth = canvas.width - margin.left - margin.right;";
  js += "var chartHeight = canvas.height - margin.top - margin.bottom;";
  js += "";
  js += "var rawEntries = voltageData.filter(function(d) { return d && typeof d.voltage === 'number' && d.voltage > 0; });";
  js += "if (rawEntries.length === 0) {";
  js += "ctx.fillStyle = '#7f8c8d'; ctx.font = '16px Arial'; ctx.textAlign = 'center';";
  js += "ctx.fillText('No valid voltage points yet', canvas.width/2, canvas.height/2);";
  js += "canvas.packVoltagePoints = []; canvas.voltageMeta = null;";
  js += "return; }";
  js += "";
  js += "function computeScaleFactor(v) { if (v <= 0) return 1; if (v < 5) return 16; if (v < 40) return 2; return 1; }";
  js += "var scalingSummary = {16: 0, 2: 0};";
  js += "var displayEntries = rawEntries.map(function(d) { var rawV = Number(d.voltage) || 0; var sf = computeScaleFactor(rawV); if (sf === 16) { scalingSummary[16]++; } else if (sf === 2) { scalingSummary[2]++; } var rawTs = Number(d.timestamp) || 0; var tsMs = rawTs < 946684800000 ? rawTs * 1000 : rawTs; return { timestamp: rawTs, timestampMs: tsMs, voltage: rawV * sf, scaleFactor: sf }; });";
  js += "var scaledCount = scalingSummary[16] + scalingSummary[2];";
  js += "if (scaledCount > 0) { console.log('Voltage history scaling applied', scalingSummary); }";
  js += "";
  js += "var DEFAULT_MIN = 44;";
  js += "var DEFAULT_MAX = 56;";
  js += "var voltages = displayEntries.map(function(d) { return d.voltage; });";
  js += "var minV = Math.min.apply(null, voltages);";
  js += "var maxV = Math.max.apply(null, voltages);";
  js += "if (!isFinite(minV) || !isFinite(maxV)) { minV = DEFAULT_MIN; maxV = DEFAULT_MAX; }";
  js += "minV = Math.min(minV, DEFAULT_MIN);";
  js += "maxV = Math.max(maxV, DEFAULT_MAX);";
  js += "var padding = 0.3;";
  js += "minV = Math.max(0, Math.floor((minV - padding) * 10) / 10);";
  js += "maxV = Math.ceil((maxV + padding) * 10) / 10;";
  js += "var range = Math.max(maxV - minV, 1);";
  js += "";
  js += "ctx.fillStyle = '#f8f9fa';";
  js += "ctx.fillRect(margin.left, margin.top, chartWidth, chartHeight);";
  js += "ctx.strokeStyle = '#dee2e6';";
  js += "ctx.lineWidth = 1;";
  js += "";
  js += "for (var i = 0; i <= 5; i++) {";
  js += "var y = margin.top + (chartHeight / 5) * i;";
  js += "ctx.beginPath(); ctx.moveTo(margin.left, y); ctx.lineTo(margin.left + chartWidth, y); ctx.stroke();";
  js += "var voltage = maxV - (range / 5) * i;";
  js += "ctx.fillStyle = '#6c757d'; ctx.font = '12px Arial';";
  js += "ctx.fillText(voltage.toFixed(1) + 'V', 10, y + 4);";
  js += "}";
  js += "";
  js += "var points = [];";
  js += "if (displayEntries.length > 0) {";
  js += "ctx.strokeStyle = '#007bff'; ctx.lineWidth = 2; ctx.beginPath();";
  js += "var denominator = Math.max(displayEntries.length - 1, 1);";
  js += "for (var i = 0; i < displayEntries.length; i++) {";
  js += "var entry = displayEntries[i];";
  js += "var x = margin.left + (chartWidth / denominator) * i;";
  js += "var y = margin.top + chartHeight - ((entry.voltage - minV) / range) * chartHeight;";
  js += "if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);";
  js += "points.push({ x: x, y: y, voltage: entry.voltage, timestampMs: entry.timestampMs });";
  js += "}";
  js += "ctx.stroke();";
  js += "ctx.fillStyle = '#007bff';";
  js += "for (var p = 0; p < points.length; p++) {";
  js += "ctx.beginPath(); ctx.arc(points[p].x, points[p].y, 4, 0, 2 * Math.PI); ctx.fill();";
  js += "}";
  js += "}";
  js += "";
  js += "canvas.packVoltagePoints = points;";
  js += "canvas.voltageMeta = { margin: margin, chartWidth: chartWidth, chartHeight: chartHeight, minV: minV, maxV: maxV, range: range };";
  js += "canvas.voltageScaleSummary = scalingSummary; canvas.voltageScaledCount = scaledCount;";
  js += "";
  js += "ctx.fillStyle = '#495057'; ctx.font = 'bold 14px Arial';";
  js += "ctx.fillText('Pack Voltage (V)', 10, canvas.height - 5);";
  js += "ctx.fillText('Time →', canvas.width - 60, canvas.height - 5);";
  js += "}";

  js += "function setupCanvasMouseEvents() {";
  js += "var canvas = document.getElementById('voltageCanvas');";
  js += "var tooltip = document.getElementById('tooltipDiv');";
  js += "if (!canvas || !tooltip) return;";
  js += "if (canvas.dataset.hoverBound === 'true') return;";
  js += "canvas.dataset.hoverBound = 'true';";
  js += "";
  js += "canvas.addEventListener('mousemove', function(e) {";
  js += "var points = canvas.packVoltagePoints || [];";
  js += "var meta = canvas.voltageMeta || null;";
  js += "if (points.length === 0 || !meta) { tooltip.style.display = 'none'; return; }";
  js += "var rect = canvas.getBoundingClientRect();";
  js += "var scaleX = canvas.width / rect.width;";
  js += "var scaleY = canvas.height / rect.height;";
  js += "var x = (e.clientX - rect.left) * scaleX;";
  js += "var y = (e.clientY - rect.top) * scaleY;";
  js += "if (x < meta.margin.left || x > meta.margin.left + meta.chartWidth || y < meta.margin.top || y > meta.margin.top + meta.chartHeight) { tooltip.style.display = 'none'; return; }";
  js += "";
  js += "var closestPoint = null;";
  js += "var minDistance = Infinity;";
  js += "for (var i = 0; i < points.length; i++) {";
  js += "var point = points[i];";
  js += "var dx = point.x - x;";
  js += "var dy = point.y - y;";
  js += "var dist = Math.sqrt(dx * dx + dy * dy);";
  js += "if (dist < minDistance) { minDistance = dist; closestPoint = point; }";
  js += "}";
  js += "";
  js += "if (!closestPoint || minDistance > 40) { tooltip.style.display = 'none'; return; }";
  js += "";
  js += "var timestampMs = Number(closestPoint.timestampMs) || 0;";
  js += "var tooltipLines = [];";
  js += "if (timestampMs >= 946684800000) { tooltipLines.push(new Date(timestampMs).toLocaleString()); } else { tooltipLines.push('Time sync pending'); }";
  js += "tooltipLines.push('Voltage: ' + closestPoint.voltage.toFixed(2) + 'V');";
  js += "";
  js += "tooltip.innerHTML = tooltipLines.join('<br>');";
  js += "tooltip.style.display = 'block';";
  js += "var tooltipWidth = tooltip.offsetWidth;";
  js += "var tooltipHeight = tooltip.offsetHeight;";
  js += "var displayX = rect.left + (closestPoint.x / scaleX);";
  js += "var displayY = rect.top + (closestPoint.y / scaleY);";
  js += "var left = displayX - tooltipWidth / 2;";
  js += "var top = displayY - tooltipHeight - 12;";
  js += "if (top < window.scrollY + 10) { top = window.scrollY + 10; }";
  js += "if (left < 10) { left = 10; }";
  js += "tooltip.style.left = left + 'px';";
  js += "tooltip.style.top = top + 'px';";
  js += "});";
  js += "";
  js += "canvas.addEventListener('mouseleave', function() { tooltip.style.display = 'none'; });";
  js += "}";

  js += "function updateMainData(data) {";
  js += "console.log('WebSocket data received:', data);";
  js += "try {";
  js += "if (data.voltage !== undefined && document.getElementById('pack-voltage')) document.getElementById('pack-voltage').textContent = data.voltage.toFixed(2) + 'V';";
  js += "if (data.soc !== undefined && document.getElementById('soc')) document.getElementById('soc').textContent = data.soc + '%';";
  js += "if (data.cellSpread !== undefined && document.getElementById('cell-spread')) document.getElementById('cell-spread').textContent = data.cellSpread.toFixed(0) + 'mV';";
  js += "if (data.minTemp !== undefined && data.maxTemp !== undefined && document.getElementById('temp-range')) document.getElementById('temp-range').innerHTML = data.minTemp.toFixed(1) + '-' + data.maxTemp.toFixed(1) + '&deg;C';";
  js += "if (data.current !== undefined && document.getElementById('system-current')) document.getElementById('system-current').textContent = data.current.toFixed(2) + 'A';";
  js += "if (data.power !== undefined && document.getElementById('system-power')) document.getElementById('system-power').textContent = data.power.toFixed(0) + 'W';";
  js += "if (data.lastUpdate !== undefined && document.getElementById('last-update')) { var ago = Math.floor((Date.now() - Date.now()) / 1000); if (data.connected) { document.getElementById('last-update').textContent = 'Just now'; } else { document.getElementById('last-update').textContent = 'No data'; } }";
  js += "if (data.balancing !== undefined && document.getElementById('balance-status')) document.getElementById('balance-status').textContent = data.balancing ? 'Active' : 'Inactive';";
  js += "if (data.knownMessages !== undefined && document.getElementById('master-count')) document.getElementById('master-count').textContent = data.masterMessages;";
  js += "if (data.current !== undefined && document.getElementById('master-current')) document.getElementById('master-current').textContent = data.current.toFixed(2) + 'A';";
  js += "if (data.power !== undefined && document.getElementById('master-power')) document.getElementById('master-power').textContent = data.power.toFixed(0) + 'W';";
  js += "loadModules(true); loadMessages(true);";
  js += "} catch (error) { console.error('Update error:', error); }";
  js += "}";

  js += "function clearAllData() {";
  js += "if (confirm('Are you sure you want to clear all message tracking data?')) {";
  js += "var xhr = new XMLHttpRequest();";
  js += "xhr.open('POST', '/api/clear', true);";
  js += "xhr.setRequestHeader('Content-Type', 'application/json');";
  js += "xhr.onreadystatechange = function() {";
  js += "if (xhr.readyState === 4 && xhr.status === 200) { loadMessages(); }";
  js += "};";
  js += "xhr.send();";
  js += "}";
  js += "}";

  js += "connectWebSocket();";
  js += "setTimeout(function() { if (loadingSteps < totalSteps) { console.log('Fallback: forcing component load'); loadInitialData(); } }, 2000);";
  js += "setInterval(function() { if (loadingSteps >= totalSteps) { loadModules(true); loadMessages(true); loadVoltageGraph(true); } }, 10000);";
  js += "window.addEventListener('resize', function() { if (voltageData.length > 0) { setTimeout(drawVoltageGraph, 100); } });";

  return js;
}

String getModulesJSON() {
  StaticJsonDocument<2048> json;
  JsonArray modules = json.createNestedArray("modules");

  for (int m = 0; m < 4; m++) {
    JsonObject module = modules.createNestedObject();
    module["id"] = m;
    module["connected"] = catlData.nodeStatus[m];
    module["voltage"] = catlData.moduleVoltages[m];
    module["temp1"] = catlData.temperatures[m*2];
    module["temp2"] = catlData.temperatures[m*2 + 1];
    module["balanceTemp"] = catlData.balanceBoardTemps[m];

    JsonArray cells = module.createNestedArray("cells");
    for (int j = 0; j < 8; j++) {
      cells.add(catlData.cellVoltages[m*8 + j]);
    }
  }

  String response;
  serializeJson(json, response);
  return response;
}

String getMessagesJSON() {
  StaticJsonDocument<2048> doc;

  // Unknown messages
  JsonArray unknown = doc.createNestedArray("unknownMessages");
  for (int i = 0; i < unknownCount && i < 20; i++) {
    JsonObject msg = unknown.createNestedObject();
    msg["id"] = String(unknownMessages[i].id, HEX);
    msg["count"] = unknownMessages[i].count;
    msg["length"] = unknownMessages[i].length;
    msg["lastSeen"] = unknownMessages[i].lastSeen;
    msg["potentialDescription"] = strlen(unknownMessages[i].potentialDescription) > 0 ? unknownMessages[i].potentialDescription : "Unknown Pattern";

    JsonArray data = msg.createNestedArray("data");
    for (int j = 0; j < unknownMessages[i].length && j < 8; j++) {
      data.add(unknownMessages[i].data[j]);
    }
  }

  // Known messages
  JsonArray known = doc.createNestedArray("knownMessages");
  for (int i = 0; i < knownCount && i < 30; i++) {
    JsonObject msg = known.createNestedObject();
    msg["id"] = String(knownMessages[i].id, HEX);
    msg["description"] = strlen(knownMessages[i].description) > 0 ? knownMessages[i].description : "Unknown";
    msg["count"] = knownMessages[i].count;
    msg["length"] = knownMessages[i].length;
    msg["lastSeen"] = knownMessages[i].lastSeen;

    JsonArray data = msg.createNestedArray("data");
    for (int j = 0; j < knownMessages[i].length && j < 8; j++) {
      data.add(knownMessages[i].data[j]);
    }
  }

  String output;
  output.reserve(2048);
  serializeJson(doc, output);

  Serial.printf("[DEBUG] Messages JSON length: %d, unknown: %d, known: %d\n", output.length(), unknownCount, knownCount);

  return output;
}

String getVoltageHistoryJSON() {
  StaticJsonDocument<2048> doc;
  JsonArray entries = doc.createNestedArray("entries");

  int count = 0;
  for (int i = 0; i < voltageHistory.totalEntries && i < 60; i++) {
    int index = (voltageHistory.currentIndex - voltageHistory.totalEntries + i + 60) % 60;
    if (voltageHistory.entries[index].valid) {
      JsonObject entry = entries.createNestedObject();
      entry["voltage"] = voltageHistory.entries[index].voltage;
      entry["timestamp"] = voltageHistory.entries[index].timestamp;
      count++;
    }
  }

  doc["count"] = count;
  doc["maxEntries"] = 60;

  String jsonData;
  jsonData.reserve(2048);
  serializeJson(doc, jsonData);

  Serial.printf("[API] Voltage history endpoint called, response size: %d, entries: %d\n", jsonData.length(), count);

  if (jsonData.length() > 0) {
    return jsonData;
  } else {
    return "{\"entries\":[], \"count\":0, \"maxEntries\":60}";
  }
}