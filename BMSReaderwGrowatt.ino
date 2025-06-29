// Complete ESP32 CATL-to-Growatt BMS Bridge
// Reads CATL modules via CAN, translates to Growatt RS485 protocol
// Includes web dashboard, safety controls, temperature conversion (F->C)
// Enhanced with unknown message tracking and manual control features
// PROGRESSIVE LOADING VERSION - Fixes memory issues

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <FS.h>
#include "driver/can.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// WiFi credentials - UPDATE THESE!
const char* ssid = "Boddah";
const char* password = "RipVanWinkle";

// Hardware pin definitions
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16

#define RS485_TX_PIN GPIO_NUM_21
#define RS485_RX_PIN GPIO_NUM_22
#define RS485_DE_PIN GPIO_NUM_18    // Driver Enable (DE/RE combined)
#define RS485_UART_NUM UART_NUM_2

// Manual control pins (optional - for direct inverter control)
#define CHARGE_ENABLE_PIN GPIO_NUM_19     // Optional direct control
#define DISCHARGE_ENABLE_PIN GPIO_NUM_23  // Optional direct control

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
  float maxPackVoltage = 110.0;     // Stop charging at 110V pack
  float minPackVoltage = 96.0;      // Stop discharging at 96V pack
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
} growattResponse;

// Master Request Data Structure
struct MasterRequestData {
  uint8_t targetModule;        // AA, AB, AC, AD
  uint8_t requestType;         // 12, 13, 14, etc.
  uint8_t counter;             // Incrementing counter
  float currentAmps;           // Decoded current measurement
  float powerWatts;            // Calculated power
  unsigned long lastSeen;
  uint32_t count;
} masterRequest;

// Enhanced control state
struct InverterControl {
  bool chargeEnabled = true;
  bool dischargeEnabled = true;
  bool autoMode = true;
  bool balancingActive = false;
  float suggestedChargeCurrent = 2.0; // Adaptive charge current suggestion
  unsigned long lastControlUpdate = 0;
  String lastAction = "Bridge Started";
  String balanceStatus = "Unknown";
} inverterControl;

// Enhanced message tracking structures
struct SimpleUnknownMessage {
  uint32_t id;
  uint8_t data[8];
  uint8_t length;
  uint32_t count;
  unsigned long lastSeen;
  const char* potentialDescription;
};

struct SimpleKnownMessage {
  uint32_t id;
  const char* description;
  uint32_t count;
  unsigned long lastSeen;
  uint8_t data[8];
  uint8_t length;
};

SimpleUnknownMessage unknownMessages[20];
int unknownCount = 0;

SimpleKnownMessage knownMessages[30];
int knownCount = 0;

// Track which cells belong to which module
const char* moduleNames[] = {"AA", "AB", "AC", "AD"};

// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Utility functions
float fahrenheitToCelsius(float fahrenheit) {
  return (fahrenheit - 32.0) * 5.0 / 9.0;
}

int getModuleIndex(uint8_t nodeId) {
  switch(nodeId) {
    case 0xAA: return 0;
    case 0xAB: return 1;
    case 0xAC: return 2;
    case 0xAD: return 3;
    default: return -1;
  }
}

void trackKnownMessage(uint32_t id, const char* description, can_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < knownCount; i++) {
    if (knownMessages[i].id == id) {
      knownMessages[i].count++;
      knownMessages[i].lastSeen = millis();
      memcpy(knownMessages[i].data, msg.data, msg.data_length_code);
      knownMessages[i].length = msg.data_length_code;
      return;
    }
  }
  
  // Add new known message if we have space
  if (knownCount < 30) {
    knownMessages[knownCount].id = id;
    knownMessages[knownCount].description = description;
    knownMessages[knownCount].count = 1;
    knownMessages[knownCount].lastSeen = millis();
    memcpy(knownMessages[knownCount].data, msg.data, msg.data_length_code);
    knownMessages[knownCount].length = msg.data_length_code;
    knownCount++;
  }
}

void logUnknownMessage(can_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < unknownCount; i++) {
    if (unknownMessages[i].id == msg.identifier) {
      memcpy(unknownMessages[i].data, msg.data, msg.data_length_code);
      unknownMessages[i].length = msg.data_length_code;
      unknownMessages[i].count++;
      unknownMessages[i].lastSeen = millis();
      return;
    }
  }
  
  // Add new message if we have space
  if (unknownCount < 20) {
    unknownMessages[unknownCount].id = msg.identifier;
    memcpy(unknownMessages[unknownCount].data, msg.data, msg.data_length_code);
    unknownMessages[unknownCount].length = msg.data_length_code;
    unknownMessages[unknownCount].count = 1;
    unknownMessages[unknownCount].lastSeen = millis();
    
    // Enhanced potential descriptions based on analysis
    uint32_t id = msg.identifier;
    if ((id & 0xFFFFFF00) == 0x18001200) {
      unknownMessages[unknownCount].potentialDescription = "Basic ACK Messages";
    }
    else if ((id & 0xFFFFFF00) == 0x18011200) {
      unknownMessages[unknownCount].potentialDescription = "Node Heartbeats (All Zeros)";
    }
    else if ((id & 0xFFFFFF00) == 0x18021200) {
      unknownMessages[unknownCount].potentialDescription = "Secondary Heartbeats";
    }
    else if ((id & 0xFFFFFF00) == 0x18041200) {
      unknownMessages[unknownCount].potentialDescription = "System Configuration (Identical Data)";
    }
    else if ((id & 0xFFFF0000) == 0x1C050000) {
      unknownMessages[unknownCount].potentialDescription = "J1939 Diagnostic (1C05)";
    }
    else if ((id & 0xFFFF0000) == 0x1C060000) {
      unknownMessages[unknownCount].potentialDescription = "J1939 Diagnostic (1C06)";
    }
    else if ((id & 0xFFFF0000) == 0x1C070000) {
      unknownMessages[unknownCount].potentialDescription = "J1939 Diagnostic (1C07)";
    }
    else {
      unknownMessages[unknownCount].potentialDescription = "Unknown Pattern";
    }
    
    unknownCount++;
    Serial.printf("NEW Unknown CAN ID: 0x%08X\n", msg.identifier);
  }
}

void processCATLMessage(can_message_t& msg) {
  uint32_t id = msg.identifier;
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
  
  // Parse temperature messages (keep in Fahrenheit as received from BMS)
  else if ((id & 0xFFFFFF00) == 0x18001400) {
    isKnownMessage = true;
    trackKnownMessage(id, "Temperature Data", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      uint16_t temp1 = (data[1] << 8) | data[0];
      uint16_t temp2 = (data[3] << 8) | data[2];
      catlData.temperatures[module*2] = temp1/10.0;  // Keep in Fahrenheit
      catlData.temperatures[module*2 + 1] = temp2/10.0;  // Keep in Fahrenheit
    }
  }
  
  // Parse balance board temperature messages (keep in Fahrenheit as received from BMS)
  else if ((id & 0xFFFFFF00) == 0x18031200) {
    isKnownMessage = true;
    trackKnownMessage(id, "Balance Board Temperature", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      uint16_t balanceTemp = (data[1] << 8) | data[0];
      catlData.balanceBoardTemps[module] = balanceTemp / 10.0;  // Keep in Fahrenheit
    }
  }
  
  // Enhanced master request parsing with charging detection
  else if (id == 0x180011F4) {
    isKnownMessage = true;
    trackKnownMessage(id, "Master Request (Enhanced)", msg);
    
    masterRequest.targetModule = data[0];  // AA, AB, AC, AD
    masterRequest.requestType = data[1];   // 12, 13, 14
    uint8_t chargingByte = data[2];        // 00=no charge, 01=charging >0.4A
    masterRequest.counter = data[5];       // Incrementing counter
    masterRequest.lastSeen = millis();
    masterRequest.count++;
    
    // Decode current measurement (byte 7) and charging status
    uint8_t currentRaw = data[7];
    masterRequest.currentAmps = currentRaw * 2.0;  // Based on analysis
    catlData.systemCurrent = masterRequest.currentAmps;
    
    // Determine if balancing is active
    bool chargingDetected = (chargingByte == 0x01);
    float avgCellVoltage = catlData.packVoltage / 32.0;
    
    inverterControl.balancingActive = chargingDetected && 
                                     (avgCellVoltage > safetyLimits.balanceThreshold) &&
                                     (masterRequest.currentAmps > safetyLimits.minBalanceCurrentPack);
    
    if (inverterControl.balancingActive) {
      inverterControl.balanceStatus = "ACTIVE - Balancing circuits working";
    } else if (chargingDetected && avgCellVoltage <= safetyLimits.balanceThreshold) {
      inverterControl.balanceStatus = "INACTIVE - Below 3.363V threshold";
    } else if (chargingDetected && masterRequest.currentAmps <= safetyLimits.minBalanceCurrentPack) {
      inverterControl.balanceStatus = "INACTIVE - Current too low (<0.4A)";
    } else {
      inverterControl.balanceStatus = "INACTIVE - Not charging";
    }
    
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
  
  // If unknown, log it with enhanced descriptions
  if (!isKnownMessage) {
    logUnknownMessage(msg);
  }
  
  // Calculate derived values
  calculatePackStats();
  updateGrowattResponse();
}

void calculatePackStats() {
  float total = 0;
  float maxVoltage = 0;
  float minVoltage = 5.0;
  float maxTemp = -50;
  float minTemp = 100;
  
  // Calculate pack voltage and cell spread
  for (int i = 0; i < 32; i++) {
    if (catlData.cellVoltages[i] > 0) {
      total += catlData.cellVoltages[i];
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
  
  // Calculate temperature range (temperatures are now in Fahrenheit)
  for (int i = 0; i < 8; i++) {
    if (catlData.temperatures[i] > maxTemp) maxTemp = catlData.temperatures[i];
    if (catlData.temperatures[i] < minTemp) minTemp = catlData.temperatures[i];
  }
  
  // Include balance board temperatures in range (also in Fahrenheit)
  for (int i = 0; i < 4; i++) {
    if (catlData.balanceBoardTemps[i] > maxTemp) maxTemp = catlData.balanceBoardTemps[i];
    if (catlData.balanceBoardTemps[i] > 0 && catlData.balanceBoardTemps[i] < minTemp) 
      minTemp = catlData.balanceBoardTemps[i];
  }
  
  catlData.packVoltage = total;
  catlData.cellSpread = (maxVoltage - minVoltage) * 1000; // mV
  catlData.maxTemp = maxTemp;
  catlData.minTemp = minTemp;
  
  // Calculate SOC based on pack voltage (simple linear approximation)
  float minPackV = 96.0;  // 32 cells * 3.0V
  float maxPackV = 115.2; // 32 cells * 3.6V
  float currentV = catlData.packVoltage;
  
  if (currentV <= minPackV) {
    catlData.soc = 0;
  } else if (currentV >= maxPackV) {
    catlData.soc = 100;
  } else {
    catlData.soc = (uint8_t)(((currentV - minPackV) / (maxPackV - minPackV)) * 100);
  }
}

void updateGrowattResponse() {
  // Convert CATL data to Growatt format
  growattResponse.packVoltage = (uint16_t)(catlData.packVoltage * 100);  // Convert to 10mV units
  growattResponse.packCurrent = (int16_t)(catlData.systemCurrent * 100); // Convert to 10mA units
  growattResponse.soc = catlData.soc;
  growattResponse.temperature = (int16_t)fahrenheitToCelsius(catlData.maxTemp); // Convert to Celsius for Growatt
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
  
  // Bit 3: Cell balance status
  if (catlData.balancingActive) {
    status |= 0x08; // Balancing active
  }
  
  // Bit 5: Output discharge status
  if (inverterControl.dischargeEnabled) {
    status |= 0x20;
  }
  
  // Bit 6: Output charge status  
  if (inverterControl.chargeEnabled) {
    status |= 0x40;
  }
  
  return status;
}

uint16_t calculateErrorBits() {
  uint16_t error = 0;
  
  float maxCellV = getMaxCellVoltage();
  float minCellV = getMinCellVoltage();
  
  // Bit 2: Over voltage protection
  if (maxCellV > safetyLimits.hardStopVoltage) {
    error |= 0x04;
  }
  
  // Bit 3: Under voltage protection
  if (minCellV < safetyLimits.minCellVoltage) {
    error |= 0x08;
  }
  
  // Bit 5: Over temperature charge protection (convert F to C for comparison)
  if (fahrenheitToCelsius(catlData.maxTemp) > fahrenheitToCelsius(safetyLimits.maxBalanceBoardTemp)) {
    error |= 0x20;
  }
  
  // Bit 10: Cell unbalance (convert to bit position)
  if (catlData.cellSpread > 100) { // >100mV spread
    error |= 0x400;
  }
  
  return error;
}

float getMaxCellVoltage() {
  float maxV = 0;
  for (int i = 0; i < 32; i++) {
    if (catlData.cellVoltages[i] > maxV) {
      maxV = catlData.cellVoltages[i];
    }
  }
  return maxV;
}

float getMinCellVoltage() {
  float minV = 5.0;
  for (int i = 0; i < 32; i++) {
    if (catlData.cellVoltages[i] > 0 && catlData.cellVoltages[i] < minV) {
      minV = catlData.cellVoltages[i];
    }
  }
  return minV;
}

void handleGrowattRequests() {
  uint8_t buffer[256];
  int len = uart_read_bytes(RS485_UART_NUM, buffer, sizeof(buffer), 10 / portTICK_PERIOD_MS);
  
  if (len > 0) {
    // Parse Modbus request
    if (len >= 8 && buffer[0] == GROWATT_SLAVE_ADDR && buffer[1] == 0x03) {
      uint16_t startAddr = (buffer[2] << 8) | buffer[3];
      uint16_t regCount = (buffer[4] << 8) | buffer[5];
      
      Serial.printf("[GROWATT] Growatt request: addr=0x%04X, count=%d\n", startAddr, regCount);
      growattResponse.lastGrowattRequest = millis();
      growattResponse.growattRequestCount++;
      
      // Send appropriate response
      sendGrowattResponse(startAddr, regCount);
    }
  }
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
  
  // Send response
  gpio_set_level(RS485_DE_PIN, 1); // Enable transmit
  uart_write_bytes(RS485_UART_NUM, response, responseLen);
  uart_wait_tx_done(RS485_UART_NUM, 100 / portTICK_PERIOD_MS);
  gpio_set_level(RS485_DE_PIN, 0); // Back to receive
  
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
        return growattResponse.cellVoltages[cellIndex];
      }
      return 0; // Unknown register
  }
}

uint16_t calculateCRC(uint8_t* data, int length) {
  // Standard Modbus CRC-16 calculation
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Enhanced inverter control logic with balancing awareness
void updateInverterControl() {
  if (!inverterControl.autoMode || safetyLimits.manualOverride) return; // Skip if manual mode
  
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 2000) return; // Check every 2 seconds
  lastCheck = millis();
  
  bool shouldDisableCharge = false;
  String reason = "";
  
  // Check cell voltages with bounds checking
  float maxCellV = getMaxCellVoltage();
  float minCellV = getMinCellVoltage();
  float cellSpreadMv = catlData.cellSpread;
  
  // Simple adaptive charge current
  if (maxCellV > 3.6) {
    inverterControl.suggestedChargeCurrent = 5.0;
  } else if (maxCellV > 3.5) {
    inverterControl.suggestedChargeCurrent = 20.0;
  } else if (cellSpreadMv > 50) {
    inverterControl.suggestedChargeCurrent = 40.0;
  } else {
    inverterControl.suggestedChargeCurrent = 80.0;
  }
  
  // Safety checks
  if (maxCellV > safetyLimits.hardStopVoltage) {
    shouldDisableCharge = true;
    reason = "Cell overvoltage";
  }
  else if (maxCellV > safetyLimits.maxCellVoltage) {
    shouldDisableCharge = true;
    reason = "Cell approaching limit";
  }
  else if (maxCellV > 3.5 && cellSpreadMv > 100) {
    shouldDisableCharge = true;
    reason = "Pausing for balance";
  }
  
  // Update charge control
  if (shouldDisableCharge && inverterControl.chargeEnabled) {
    inverterControl.chargeEnabled = false;
    if (CHARGE_ENABLE_PIN != 0) digitalWrite(CHARGE_ENABLE_PIN, LOW);
    inverterControl.lastAction = reason;
    Serial.println("Charging disabled: " + reason);
    inverterControl.lastControlUpdate = millis();
  }
  else if (!shouldDisableCharge && !inverterControl.chargeEnabled) {
    if (maxCellV < (safetyLimits.maxCellVoltage - 0.1) && cellSpreadMv < 80) {
      inverterControl.chargeEnabled = true;
      if (CHARGE_ENABLE_PIN != 0) digitalWrite(CHARGE_ENABLE_PIN, HIGH);
      inverterControl.lastAction = "Charge re-enabled";
      Serial.println("Charging re-enabled");
      inverterControl.lastControlUpdate = millis();
    }
  }
}

// PROGRESSIVE LOADING: Separate JSON functions for smaller memory usage
String getBMSDataJSON() {
  // Use smaller static allocation for main data
  StaticJsonDocument<1536> doc; // Reduced from 3072
  
  doc["packVoltage"] = catlData.packVoltage;
  doc["cellSpread"] = catlData.cellSpread;
  doc["maxTemp"] = catlData.maxTemp;
  doc["minTemp"] = catlData.minTemp;
  doc["systemCurrent"] = catlData.systemCurrent;
  doc["systemPower"] = masterRequest.powerWatts;
  doc["soc"] = catlData.soc;
  doc["lastUpdate"] = catlData.lastUpdate;
  
  // Bridge status
  JsonObject bridge = doc.createNestedObject("bridgeStatus");
  bridge["growattRequests"] = growattResponse.growattRequestCount;
  bridge["lastGrowattRequest"] = growattResponse.lastGrowattRequest;
  bridge["bridgeActive"] = (millis() - growattResponse.lastGrowattRequest) < 30000;
  
  // Inverter control data
  JsonObject control = doc.createNestedObject("inverterControl");
  control["chargeEnabled"] = inverterControl.chargeEnabled;
  control["dischargeEnabled"] = inverterControl.dischargeEnabled;
  control["autoMode"] = inverterControl.autoMode;
  control["balancingActive"] = inverterControl.balancingActive;
  control["balanceStatus"] = inverterControl.balanceStatus;
  control["lastAction"] = inverterControl.lastAction;
  control["suggestedChargeCurrent"] = inverterControl.suggestedChargeCurrent;
  
  // Master request data
  JsonObject masterReq = doc.createNestedObject("masterRequest");
  masterReq["targetModule"] = masterRequest.targetModule;
  masterReq["requestType"] = masterRequest.requestType;
  masterReq["counter"] = masterRequest.counter;
  masterReq["currentAmps"] = masterRequest.currentAmps;
  masterReq["powerWatts"] = masterRequest.powerWatts;
  masterReq["count"] = masterRequest.count;
  
  String output;
  output.reserve(1536);
  serializeJson(doc, output);
  return output;
}

// PROGRESSIVE LOADING: Separate modules JSON
String getModulesJSON() {
  StaticJsonDocument<2048> doc;
  JsonArray modules = doc.to<JsonArray>();
  
  for (int m = 0; m < 4; m++) {
    JsonObject module = modules.createNestedObject();
    module["name"] = moduleNames[m];
    module["connected"] = catlData.nodeStatus[m];
    module["voltage"] = catlData.moduleVoltages[m];
    module["balanceBoardTemp"] = catlData.balanceBoardTemps[m];
    
    JsonArray cells = module.createNestedArray("cells");
    for (int c = 0; c < 8; c++) {
      float cellVoltage = catlData.cellVoltages[m*8 + c];
      cells.add(cellVoltage > 0 ? cellVoltage : 0.0); // Ensure we don't send negative or invalid values
    }
    
    JsonArray temps = module.createNestedArray("temperatures");
    temps.add(catlData.temperatures[m*2]);
    temps.add(catlData.temperatures[m*2 + 1]);
  }
  
  String output;
  output.reserve(2048);
  serializeJson(doc, output);
  
  // Debug output
  Serial.printf("[DEBUG] Modules JSON length: %d\n", output.length());
  
  return output;
}

// PROGRESSIVE LOADING: Separate messages JSON
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
    msg["potentialDescription"] = unknownMessages[i].potentialDescription ? unknownMessages[i].potentialDescription : "Unknown Pattern";
    
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
    msg["description"] = knownMessages[i].description ? knownMessages[i].description : "Unknown";
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
  
  // Debug output
  Serial.printf("[DEBUG] Messages JSON length: %d, unknown: %d, known: %d\n", output.length(), unknownCount, knownCount);
  
  return output;
}

void broadcastUpdate() {
  // Check available memory before creating JSON
  if (ESP.getFreeHeap() < 4096) {
    Serial.println("[MEMORY] Low memory, skipping broadcast");
    return;
  }
  
  // Check if we have any WebSocket clients
  if (ws.count() == 0) {
    return; // No clients connected, skip broadcast
  }
  
  // Additional safety check - limit broadcast frequency
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast < 1000) {
    return;
  }
  lastBroadcast = millis();
  
  String jsonData = getBMSDataJSON();
  if (jsonData.length() > 0 && jsonData.length() < 2048) {
    ws.textAll(jsonData);
    Serial.printf("[WS] Broadcast sent to %d clients (%d bytes)\n", ws.count(), jsonData.length());
  } else {
    Serial.printf("[MEMORY] JSON data invalid size: %d bytes, skipping broadcast\n", jsonData.length());
  }
}

void setupCAN() {
  can_timing_config_t t_config = CAN_TIMING_CONFIG_250KBITS();
  can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
  can_general_config_t g_config = {
    .mode = CAN_MODE_LISTEN_ONLY,
    .tx_io = CAN_TX_PIN,
    .rx_io = CAN_RX_PIN,
    .clkout_io = CAN_IO_UNUSED,
    .bus_off_io = CAN_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 30,
    .alerts_enabled = CAN_ALERT_NONE,
    .clkout_divider = 0
  };
  
  esp_err_t result = can_driver_install(&g_config, &t_config, &f_config);
  if (result == ESP_OK) {
    Serial.println("[OK] CAN driver installed");
    can_start();
    Serial.println("[OK] CAN started for CATL communication");
  } else {
    Serial.printf("[ERROR] CAN failed: %s\n", esp_err_to_name(result));
  }
}

void setupRS485() {
  // Configure UART for RS485 communication with Growatt
  uart_config_t uart_config = {
    .baud_rate = GROWATT_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  
  ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN, 
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, 1024, 1024, 0, NULL, 0));
  
  // Configure DE pin for RS485 direction control
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << RS485_DE_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf);
  gpio_set_level(RS485_DE_PIN, 0); // Start in receive mode
  
  Serial.println("[OK] RS485 initialized for Growatt communication");
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("[OK] Connected to %s\n", ssid);
    Serial.printf("[INFO] IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[INFO] Dashboard URL: http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[ERROR] WiFi connection failed!");
  }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] WebSocket client connected: %u from IP %s\n", client->id(), client->remoteIP().toString().c_str());
    // Send basic data first, modules and messages will be loaded separately
    String jsonData = getBMSDataJSON();
    if (jsonData.length() > 0) {
      client->text(jsonData);
      Serial.printf("[WS] Sent initial data to client %u (%d bytes)\n", client->id(), jsonData.length());
    }
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] WebSocket client disconnected: %u\n", client->id());
  } else if (type == WS_EVT_ERROR) {
    Serial.printf("[WS] WebSocket error from client %u\n", client->id());
  } else if (type == WS_EVT_DATA) {
    Serial.printf("[WS] WebSocket data received from client %u\n", client->id());
  }
}

void setupWebServer() {
  // Handle WebSocket connections
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // PROGRESSIVE LOADING: Main dashboard (minimal HTML)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = getBasicHTML();
    request->send(200, "text/html", html);
  });
  
  // PROGRESSIVE LOADING: CSS styles
  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request){
    String css = getCSS();
    request->send(200, "text/css", css);
  });
  
  // PROGRESSIVE LOADING: JavaScript
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
    String js = getJavaScript();
    request->send(200, "application/javascript", js);
  });
  
  // API endpoint for current data with enhanced memory checks
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
    if (ESP.getFreeHeap() < 4096) {
      request->send(503, "text/plain", "Low memory - try again later");
      return;
    }
    
    String jsonData = getBMSDataJSON();
    Serial.printf("[API] Data endpoint called, response size: %d\n", jsonData.length());
    
    if (jsonData.length() > 0 && jsonData.length() < 4096) {
      request->send(200, "application/json", jsonData);
    } else {
      request->send(500, "text/plain", "Data generation failed");
    }
  });

  // Test endpoint to verify API is working
  server.on("/api/test", HTTP_GET, [](AsyncWebServerRequest *request){
    String testJson = "{\"test\":\"ok\",\"heap\":" + String(ESP.getFreeHeap()) + ",\"time\":" + String(millis()) + "}";
    Serial.printf("[API] Test endpoint called\n");
    request->send(200, "application/json", testJson);
  });

  // API endpoint for modules data (separate from main data)
  server.on("/api/modules", HTTP_GET, [](AsyncWebServerRequest *request){
    if (ESP.getFreeHeap() < 4096) {
      request->send(503, "text/plain", "Low memory - try again later");
      return;
    }
    
    String jsonData = getModulesJSON();
    Serial.printf("[API] Modules endpoint called, response size: %d\n", jsonData.length());
    
    if (jsonData.length() > 0) {
      request->send(200, "application/json", jsonData);
    } else {
      request->send(500, "text/plain", "Modules data generation failed");
    }
  });

  // API endpoint for message tracking data
  server.on("/api/messages", HTTP_GET, [](AsyncWebServerRequest *request){
    if (ESP.getFreeHeap() < 4096) {
      request->send(503, "text/plain", "Low memory - try again later");
      return;
    }
    
    String jsonData = getMessagesJSON();
    Serial.printf("[API] Messages endpoint called, response size: %d\n", jsonData.length());
    
    if (jsonData.length() > 0) {
      request->send(200, "application/json", jsonData);
    } else {
      request->send(500, "text/plain", "Messages data generation failed");
    }
  });

  // API endpoint to clear all data
  server.on("/api/clear", HTTP_POST, [](AsyncWebServerRequest *request){
    // Clear unknown messages
    memset(unknownMessages, 0, sizeof(unknownMessages));
    unknownCount = 0;
    
    // Clear known messages
    memset(knownMessages, 0, sizeof(knownMessages));
    knownCount = 0;
    
    // Reset some BMS data but keep current readings
    masterRequest.count = 0;
    growattResponse.growattRequestCount = 0;
    
    Serial.println("[CLEAR] Message tracking data cleared via web request");
    request->send(200, "text/plain", "Data cleared successfully");
  });

  // Inverter control endpoints
  server.on("/api/control/charge", HTTP_POST, [](AsyncWebServerRequest *request){
    String action = "unknown";
    if (request->hasParam("action", true)) {
      action = request->getParam("action", true)->value();
      if (action == "enable") {
        inverterControl.chargeEnabled = true;
        if (CHARGE_ENABLE_PIN != 0) digitalWrite(CHARGE_ENABLE_PIN, HIGH);
        inverterControl.lastAction = "Charge ENABLED manually";
        Serial.println("[CONTROL] Charging ENABLED via web request");
      } else if (action == "disable") {
        inverterControl.chargeEnabled = false;
        if (CHARGE_ENABLE_PIN != 0) digitalWrite(CHARGE_ENABLE_PIN, LOW);
        inverterControl.lastAction = "Charge DISABLED manually";
        Serial.println("[CONTROL] Charging DISABLED via web request");
      }
      inverterControl.lastControlUpdate = millis();
      safetyLimits.manualOverride = true; // Mark as manual override
    }
    request->send(200, "application/json", "{\"status\":\"ok\",\"chargeEnabled\":" + String(inverterControl.chargeEnabled) + "}");
  });
  
  server.on("/api/control/discharge", HTTP_POST, [](AsyncWebServerRequest *request){
    String action = "unknown";
    if (request->hasParam("action", true)) {
      action = request->getParam("action", true)->value();
      if (action == "enable") {
        inverterControl.dischargeEnabled = true;
        if (DISCHARGE_ENABLE_PIN != 0) digitalWrite(DISCHARGE_ENABLE_PIN, HIGH);
        inverterControl.lastAction = "Discharge ENABLED manually";
        Serial.println("[CONTROL] Discharging ENABLED via web request");
      } else if (action == "disable") {
        inverterControl.dischargeEnabled = false;
        if (DISCHARGE_ENABLE_PIN != 0) digitalWrite(DISCHARGE_ENABLE_PIN, LOW);
        inverterControl.lastAction = "Discharge DISABLED manually";
        Serial.println("[CONTROL] Discharging DISABLED via web request");
      }
      inverterControl.lastControlUpdate = millis();
      safetyLimits.manualOverride = true; // Mark as manual override
    }
    request->send(200, "application/json", "{\"status\":\"ok\",\"dischargeEnabled\":" + String(inverterControl.dischargeEnabled) + "}");
  });
  
  server.on("/api/control/auto", HTTP_POST, [](AsyncWebServerRequest *request){
    String mode = "unknown";
    if (request->hasParam("mode", true)) {
      mode = request->getParam("mode", true)->value();
      inverterControl.autoMode = (mode == "true");
      safetyLimits.manualOverride = !inverterControl.autoMode;
      inverterControl.lastAction = inverterControl.autoMode ? "Auto mode ENABLED" : "Auto mode DISABLED";
      Serial.printf("[CONTROL] Auto mode %s via web request\n", inverterControl.autoMode ? "ENABLED" : "DISABLED");
      inverterControl.lastControlUpdate = millis();
    }
    request->send(200, "application/json", "{\"status\":\"ok\",\"autoMode\":" + String(inverterControl.autoMode) + "}");
  });
  
  // Handle favicon request
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(204);
  });
  
  // Start server
  server.begin();
  Serial.println("[OK] Web server started with progressive loading");
}

// PROGRESSIVE LOADING: Basic HTML structure only
String getBasicHTML() {
  String html;
  html.reserve(2048); // Much smaller allocation
  
  html = "<!DOCTYPE html><html><head><title>CATL-to-Growatt BMS Bridge</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
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
  html += "<div class='control-panel' id='control-panel'></div>";
  html += "<div class='control-panel' id='master-request-panel'></div>";
  html += "<div class='modules' id='modules'></div>";
  html += "<div class='message-section' id='unknown-messages'></div>";
  html += "<div class='message-section' id='known-messages'></div>";
  
  html += "<script src='/script.js'></script>";
  html += "</body></html>";
  
  return html;
}

// PROGRESSIVE LOADING: CSS styles
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
  
  return css;
}

// PROGRESSIVE LOADING: JavaScript
String getJavaScript() {
  String js;
  js.reserve(8192);
  
  js = "var ws; var reconnectTimer; var startTime = Date.now();";
  js += "var loadingSteps = 0; var totalSteps = 4;";
  
  js += "function updateLoadingProgress() {";
  js += "loadingSteps++; console.log('Loading progress:', loadingSteps, '/', totalSteps);";
  js += "if (loadingSteps >= totalSteps) {";
  js += "document.getElementById('loading').style.display = 'none';";
  js += "console.log('All components loaded successfully');";
  js += "} else { document.getElementById('loading').textContent = 'Loading... (' + loadingSteps + '/' + totalSteps + ')'; }";
  js += "}";
  
  js += "function connectWebSocket() {";
  js += "try {";
  js += "console.log('Attempting WebSocket connection to:', 'ws://' + window.location.host + '/ws');";
  js += "ws = new WebSocket('ws://' + window.location.host + '/ws');";
  js += "ws.onopen = function() { console.log('WebSocket connected'); document.getElementById('connection-status').textContent = 'Connected'; document.getElementById('connection-status').className = 'connected'; setTimeout(loadInitialData, 100); };";
  js += "ws.onmessage = function(event) { try { console.log('WebSocket message received:', event.data.length + ' bytes'); var data = JSON.parse(event.data); updateMainData(data); } catch (e) { console.error('Failed to parse WebSocket data:', e); } };";
  js += "ws.onclose = function(event) { console.log('WebSocket closed, code:', event.code, 'reason:', event.reason); document.getElementById('connection-status').textContent = 'Disconnected'; document.getElementById('connection-status').className = 'disconnected'; reconnectTimer = setTimeout(connectWebSocket, 3000); };";
  js += "ws.onerror = function(error) { console.error('WebSocket error:', error); };";
  js += "} catch (e) { console.error('WebSocket connection failed:', e); }";
  js += "}";
  
  js += "function loadInitialData() {";
  js += "console.log('Starting initial data load...');";
  js += "loadMainStats(); loadControlPanels(); setTimeout(function() { loadModules(); }, 200); setTimeout(function() { loadMessages(); }, 400);";
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
  js += "controlHtml += '<div style=\"text-align:center;\">';";
  js += "controlHtml += '<button onclick=\"toggleCharge()\" id=\"charge-btn\" style=\"margin:5px; padding:10px 15px; border:none; border-radius:4px; font-weight:bold; cursor:pointer;\">Toggle Charge</button>';";
  js += "controlHtml += '<button onclick=\"toggleDischarge()\" id=\"discharge-btn\" style=\"margin:5px; padding:10px 15px; border:none; border-radius:4px; font-weight:bold; cursor:pointer;\">Toggle Discharge</button>';";
  js += "controlHtml += '<button onclick=\"toggleAuto()\" id=\"auto-btn\" style=\"margin:5px; padding:10px 15px; border:none; border-radius:4px; font-weight:bold; cursor:pointer;\">Toggle Auto</button>';";
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
  
  js += "function loadModules() {";
  js += "console.log('Loading modules...');";
  js += "fetch('/api/modules').then(response => {";
  js += "if (!response.ok) throw new Error('Modules API failed');";
  js += "return response.json();";
  js += "}).then(data => {";
  js += "console.log('Modules loaded:', data);";
  js += "updateModules(data); updateLoadingProgress();";
  js += "}).catch(error => { console.error('Failed to load modules:', error); updateLoadingProgress(); });";
  js += "}";
  
  js += "function loadMessages() {";
  js += "console.log('Loading messages...');";
  js += "fetch('/api/messages').then(response => {";
  js += "if (!response.ok) throw new Error('Messages API failed');";
  js += "return response.json();";
  js += "}).then(data => {";
  js += "console.log('Messages loaded:', data);";
  js += "updateMessages(data); updateLoadingProgress();";
  js += "}).catch(error => { console.error('Failed to load messages:', error); updateLoadingProgress(); });";
  js += "}";
  
  js += "function updateMainData(data) {";
  js += "try {";
  js += "document.getElementById('pack-voltage').textContent = (data.packVoltage || 0).toFixed(2) + 'V';";
  js += "document.getElementById('soc').textContent = (data.soc || 0) + '%';";
  js += "document.getElementById('cell-spread').textContent = (data.cellSpread || 0).toFixed(0) + 'mV';";
  js += "document.getElementById('temp-range').textContent = (data.minTemp || 0).toFixed(1) + '\\u00B0F - ' + (data.maxTemp || 0).toFixed(1) + '\\u00B0F';";
  js += "document.getElementById('system-current').textContent = (data.systemCurrent || 0).toFixed(1) + 'A';";
  js += "document.getElementById('system-power').textContent = (data.systemPower || 0).toFixed(0) + 'W';";
  
  js += "if (data.lastUpdate && data.lastUpdate > 0) {";
  js += "var timeSinceUpdate = Math.floor((Date.now() - startTime - data.lastUpdate) / 1000);";
  js += "if (timeSinceUpdate < 0) timeSinceUpdate = 0;";
  js += "if (timeSinceUpdate < 5) { document.getElementById('last-update').textContent = 'Just now'; }";
  js += "else if (timeSinceUpdate < 60) { document.getElementById('last-update').textContent = timeSinceUpdate + 's ago'; }";
  js += "else if (timeSinceUpdate < 3600) { document.getElementById('last-update').textContent = Math.floor(timeSinceUpdate/60) + 'm ago'; }";
  js += "else { document.getElementById('last-update').textContent = Math.floor(timeSinceUpdate/3600) + 'h ago'; }";
  js += "} else { document.getElementById('last-update').textContent = '---'; }";
  
  js += "if (data.inverterControl) {";
  js += "document.getElementById('balance-status').textContent = data.inverterControl.balanceStatus || 'Unknown';";
  js += "var balanceEl = document.getElementById('balance-status');";
  js += "if (data.inverterControl.balancingActive) { balanceEl.style.color = '#27ae60'; }";
  js += "else { balanceEl.style.color = '#e67e22'; }";
  js += "}";
  
  js += "if (data.bridgeStatus) {";
  js += "document.getElementById('growatt-requests').textContent = data.bridgeStatus.growattRequests || 0;";
  js += "document.getElementById('bridge-active').textContent = data.bridgeStatus.bridgeActive ? 'YES' : 'NO';";
  js += "document.getElementById('bridge-active').style.color = data.bridgeStatus.bridgeActive ? '#27ae60' : '#e74c3c';";
  js += "if (data.bridgeStatus.lastGrowattRequest > 0) {";
  js += "var timeSinceRequest = Math.floor((Date.now() - startTime - data.bridgeStatus.lastGrowattRequest) / 1000);";
  js += "if (timeSinceRequest < 0) timeSinceRequest = 0;";
  js += "document.getElementById('last-request').textContent = timeSinceRequest + 's ago';";
  js += "} else { document.getElementById('last-request').textContent = 'Never'; }";
  js += "}";
  
  js += "if (data.masterRequest) {";
  js += "document.getElementById('master-module').textContent = '0x' + (data.masterRequest.targetModule || '').toString(16).toUpperCase();";
  js += "document.getElementById('master-type').textContent = '0x' + (data.masterRequest.requestType || 0).toString(16).toUpperCase();";
  js += "document.getElementById('master-counter').textContent = '0x' + (data.masterRequest.counter || 0).toString(16).padStart(2, '0').toUpperCase();";
  js += "document.getElementById('master-current').textContent = (data.masterRequest.currentAmps || 0).toFixed(1);";
  js += "document.getElementById('master-power').textContent = (data.masterRequest.powerWatts || 0).toFixed(0);";
  js += "document.getElementById('master-count').textContent = (data.masterRequest.count || 0);";
  js += "}";
  
  js += "if (data.inverterControl) {";
  js += "var chargeBtn = document.getElementById('charge-btn');";
  js += "var dischargeBtn = document.getElementById('discharge-btn');";
  js += "var autoBtn = document.getElementById('auto-btn');";
  js += "if (chargeBtn) {";
  js += "document.getElementById('charge-status').textContent = data.inverterControl.chargeEnabled ? 'ENABLED' : 'DISABLED';";
  js += "document.getElementById('discharge-status').textContent = data.inverterControl.dischargeEnabled ? 'ENABLED' : 'DISABLED';";
  js += "document.getElementById('auto-mode').textContent = data.inverterControl.autoMode ? 'ON' : 'OFF';";
  js += "document.getElementById('balancing-active').textContent = data.inverterControl.balancingActive ? 'YES' : 'NO';";
  js += "document.getElementById('suggested-current').textContent = (data.inverterControl.suggestedChargeCurrent || 0).toFixed(1) + 'A';";
  js += "document.getElementById('last-action').textContent = data.inverterControl.lastAction || '---';";
  js += "chargeBtn.style.backgroundColor = data.inverterControl.chargeEnabled ? '#27ae60' : '#e74c3c';";
  js += "chargeBtn.style.color = 'white';";
  js += "dischargeBtn.style.backgroundColor = data.inverterControl.dischargeEnabled ? '#27ae60' : '#e74c3c';";
  js += "dischargeBtn.style.color = 'white';";
  js += "autoBtn.style.backgroundColor = data.inverterControl.autoMode ? '#3498db' : '#95a5a6';";
  js += "autoBtn.style.color = 'white';";
  js += "}";
  js += "}";
  js += "} catch (error) { console.error('Error updating main data:', error); }";
  js += "}";
  
  js += "function updateModules(modules) {";
  js += "var modulesDiv = document.getElementById('modules'); modulesDiv.innerHTML = '';";
  js += "if (modules && modules.length) { modules.forEach(function(module) {";
  js += "var moduleDiv = document.createElement('div'); moduleDiv.className = 'module';";
  js += "var cellsHtml = ''; var cells = module.cells || [];";
  js += "for (var i = 0; i < cells.length; i++) {";
  js += "var voltage = cells[i]; var cellClass = 'good';";
  js += "if (voltage < 3.0 || voltage > 3.4) cellClass = 'danger';";
  js += "else if (voltage < 3.1 || voltage > 3.35) cellClass = 'warning';";
  js += "cellsHtml += '<div class=\"cell ' + cellClass + '\">Cell ' + (i + 1) + '<br>' + (voltage || 0).toFixed(3) + 'V</div>';";
  js += "}";
  js += "var temps = module.temperatures || [0, 0];";
  js += "var balanceTemp = module.balanceBoardTemp || 0;";
  js += "var balanceTempHtml = balanceTemp > 0 ? '<div class=\"balance-temp\">Balance Board: ' + balanceTemp.toFixed(1) + '\\u00B0F</div>' : '';";
  js += "moduleDiv.innerHTML = '<div class=\"module-header\">Module ' + (module.name || 'Unknown') + '<div class=\"status-dot ' + (module.connected ? 'connected' : '') + '\"></div></div>' + '<div class=\"module-voltage\">Pack Voltage: ' + (module.voltage || 0).toFixed(2) + 'V</div>' + balanceTempHtml + '<div class=\"cells\">' + cellsHtml + '</div>' + '<div class=\"temps\">Sensor Temps: ' + temps[0].toFixed(1) + '\\u00B0F, ' + temps[1].toFixed(1) + '\\u00B0F</div>';";
  js += "modulesDiv.appendChild(moduleDiv); }); }";
  js += "}";
  
  js += "function updateMessages(data) {";
  js += "var unknownHtml = '<h2>Unknown CAN Messages</h2>';";
  js += "unknownHtml += '<p>Messages that do not match known protocol patterns:</p>';";
  js += "unknownHtml += '<table class=\"message-table\"><thead><tr>';";
  js += "unknownHtml += '<th>CAN ID</th><th>Count</th><th>Length</th><th>Data (Hex)</th><th>Potential Description</th><th>Last Seen</th>';";
  js += "unknownHtml += '</tr></thead><tbody>';";
  
  js += "if (data.unknownMessages && data.unknownMessages.length > 0) {";
  js += "var sortedUnknown = data.unknownMessages.sort(function(a, b) { var descA = a.potentialDescription || 'ZZZ_Unknown Pattern'; var descB = b.potentialDescription || 'ZZZ_Unknown Pattern'; if (descA !== descB) return descA.localeCompare(descB); return parseInt(a.id, 16) - parseInt(b.id, 16); });";
  js += "unknownHtml += sortedUnknown.map(function(msg) {";
  js += "var dataHex = (msg.data || []).map(function(b) { return b.toString(16).padStart(2, '0').toUpperCase(); }).join(' ');";
  js += "var timeSince = Math.floor((Date.now() - startTime - (msg.lastSeen || 0)) / 1000);";
  js += "if (timeSince < 0) timeSince = 0;";
  js += "var timeText = 'Just now'; if (timeSince >= 5) { if (timeSince < 60) timeText = timeSince + 's ago'; else if (timeSince < 3600) timeText = Math.floor(timeSince/60) + 'm ago'; else timeText = Math.floor(timeSince/3600) + 'h ago'; }";
  js += "return '<tr><td>0x' + (msg.id || '').toString(16).toUpperCase() + '</td><td>' + (msg.count || 0) + '</td><td>' + (msg.length || 0) + '</td><td class=\"data-hex\">' + dataHex + '</td><td>' + (msg.potentialDescription || 'Unknown Pattern') + '</td><td>' + timeText + '</td></tr>';";
  js += "}).join('');";
  js += "} else { unknownHtml += '<tr><td colspan=\"6\">No unknown messages detected</td></tr>'; }";
  js += "unknownHtml += '</tbody></table>';";
  js += "document.getElementById('unknown-messages').innerHTML = unknownHtml;";
  
  js += "var knownHtml = '<h2>Known CAN Messages</h2>';";
  js += "knownHtml += '<p>Messages we have identified and understand:</p>';";
  js += "knownHtml += '<table class=\"message-table\"><thead><tr>';";
  js += "knownHtml += '<th>CAN ID</th><th>Description</th><th>Count</th><th>Length</th><th>Data (Hex)</th><th>Last Seen</th>';";
  js += "knownHtml += '</tr></thead><tbody>';";
  
  js += "if (data.knownMessages && data.knownMessages.length > 0) {";
  js += "var sortedKnown = data.knownMessages.sort(function(a, b) { return parseInt(a.id, 16) - parseInt(b.id, 16); });";
  js += "knownHtml += sortedKnown.map(function(msg) {";
  js += "var timeSince = Math.floor((Date.now() - startTime - (msg.lastSeen || 0)) / 1000);";
  js += "if (timeSince < 0) timeSince = 0;";
  js += "var timeText = 'Just now'; if (timeSince >= 5) { if (timeSince < 60) timeText = timeSince + 's ago'; else if (timeSince < 3600) timeText = Math.floor(timeSince/60) + 'm ago'; else timeText = Math.floor(timeSince/3600) + 'h ago'; }";
  js += "var dataHex = (msg.data || []).map(function(b) { return b.toString(16).padStart(2, '0').toUpperCase(); }).join(' ');";
  js += "return '<tr><td>0x' + (msg.id || '').toString(16).toUpperCase() + '</td><td>' + (msg.description || 'Unknown') + '</td><td>' + (msg.count || 0) + '</td><td>' + (msg.length || 0) + '</td><td class=\"data-hex\">' + dataHex + '</td><td>' + timeText + '</td></tr>';";
  js += "}).join('');";
  js += "} else { knownHtml += '<tr><td colspan=\"6\">No known messages tracked yet</td></tr>'; }";
  js += "knownHtml += '</tbody></table>';";
  js += "document.getElementById('known-messages').innerHTML = knownHtml;";
  js += "}";
  
  js += "function toggleCharge() {";
  js += "var chargeEnabled = document.getElementById('charge-status').textContent === 'ENABLED';";
  js += "var action = chargeEnabled ? 'disable' : 'enable';";
  js += "var xhr = new XMLHttpRequest();";
  js += "xhr.open('POST', '/api/control/charge', true);";
  js += "xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');";
  js += "xhr.send('action=' + action);";
  js += "}";
  
  js += "function toggleDischarge() {";
  js += "var dischargeEnabled = document.getElementById('discharge-status').textContent === 'ENABLED';";
  js += "var action = dischargeEnabled ? 'disable' : 'enable';";
  js += "var xhr = new XMLHttpRequest();";
  js += "xhr.open('POST', '/api/control/discharge', true);";
  js += "xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');";
  js += "xhr.send('action=' + action);";
  js += "}";
  
  js += "function toggleAuto() {";
  js += "var autoMode = document.getElementById('auto-mode').textContent === 'ON';";
  js += "var mode = autoMode ? 'false' : 'true';";
  js += "var xhr = new XMLHttpRequest();";
  js += "xhr.open('POST', '/api/control/auto', true);";
  js += "xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');";
  js += "xhr.send('mode=' + mode);";
  js += "}";
  
  js += "function clearAllData() {";
  js += "if (confirm('Are you sure you want to clear all message tracking data?')) {";
  js += "var xhr = new XMLHttpRequest(); xhr.open('POST', '/api/clear', true);";
  js += "xhr.setRequestHeader('Content-Type', 'application/json');";
  js += "xhr.onreadystatechange = function() {";
  js += "if (xhr.readyState === 4 && xhr.status === 200) { loadMessages(); }";
  js += "}; xhr.send(); }";
  js += "}";
  
  js += "connectWebSocket();";
  js += "setTimeout(function() { if (loadingSteps < totalSteps) { console.log('Fallback: forcing component load'); loadInitialData(); } }, 2000);";
  js += "setInterval(function() { if (loadingSteps >= totalSteps) { loadModules(); loadMessages(); } }, 10000);";
  
  return js;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Print memory info for debugging
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
  
  // Initialize data structures
  memset(&catlData, 0, sizeof(catlData));
  memset(&masterRequest, 0, sizeof(masterRequest));
  memset(&growattResponse, 0, sizeof(growattResponse));
  memset(unknownMessages, 0, sizeof(unknownMessages));
  memset(knownMessages, 0, sizeof(knownMessages));
  
  // Initialize optional control pins (only if they are defined and not 0)
  if (CHARGE_ENABLE_PIN != 0) {
    pinMode(CHARGE_ENABLE_PIN, OUTPUT);
    digitalWrite(CHARGE_ENABLE_PIN, HIGH);    // Start with charging enabled (active high)
  }
  if (DISCHARGE_ENABLE_PIN != 0) {
    pinMode(DISCHARGE_ENABLE_PIN, OUTPUT);
    digitalWrite(DISCHARGE_ENABLE_PIN, HIGH); // Start with discharging enabled
  }
  
  Serial.println("=== CATL-to-Growatt BMS Bridge ===");
  Serial.println("[INIT] ESP32 CAN <-> RS485 Protocol Bridge with Progressive Loading");
  
  if (CHARGE_ENABLE_PIN != 0 || DISCHARGE_ENABLE_PIN != 0) {
    Serial.println("[INIT] Optional inverter control pins initialized");
    if (CHARGE_ENABLE_PIN != 0) Serial.printf("   Charge Enable Pin: %d\n", CHARGE_ENABLE_PIN);
    if (DISCHARGE_ENABLE_PIN != 0) Serial.printf("   Discharge Enable Pin: %d\n", DISCHARGE_ENABLE_PIN);
  }
  
  // Initialize CAN for CATL communication
  setupCAN();
  
  // Initialize RS485 for Growatt communication
  setupRS485();
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup web server with progressive loading
  setupWebServer();
  
  Serial.println("[READY] Bridge ready! Access dashboard at: http://" + WiFi.localIP().toString());
  Serial.println("[INFO] CATL modules: CAN 250kbps on pins 16/17");
  Serial.println("[INFO] Growatt inverter: RS485 9600 baud on pins 21/22");
  Serial.println("[INFO] Progressive loading dashboard - memory optimized");
}

void loop() {
  // Read CATL data via CAN
  can_message_t rx_message;
  if (can_receive(&rx_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    processCATLMessage(rx_message);
  }
  
  // Handle Growatt RS485 requests
  handleGrowattRequests();
  
  // Update inverter control based on safety conditions
  updateInverterControl();
  
  // Send updates to web dashboard every 1 second
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    broadcastUpdate();
    lastUpdate = millis();
  }
  
  // Clean up WebSocket clients
  ws.cleanupClients();
  
  // Print bridge status every 30 seconds
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 30000) {
    Serial.printf("[STATUS] Bridge Status: CATL=%s, Growatt=%d reqs, SOC=%d%%, Pack=%.1fV, Balance=%s, Free Heap=%d\n",
                  (millis() - catlData.lastUpdate < 5000) ? "OK" : "TIMEOUT",
                  growattResponse.growattRequestCount,
                  catlData.soc,
                  catlData.packVoltage,
                  inverterControl.balancingActive ? "ACTIVE" : "INACTIVE",
                  ESP.getFreeHeap());
    lastStatus = millis();
  }
}