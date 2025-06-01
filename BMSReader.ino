#include "driver/can.h"
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "AsyncWebSocket.h"
#include "ArduinoJson.h"

// WiFi credentials - UPDATE THESE!
const char* ssid = "YourNetworkName";
const char* password = "YourNetworkPassword";

// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Simple unknown message tracking
struct SimpleUnknownMessage {
  uint32_t id;
  uint8_t data[8];
  uint8_t length;
  uint32_t count;
  unsigned long lastSeen;
};

// Simple known message tracking - just count what we see
struct SimpleKnownMessage {
  uint32_t id;
  const char* description;
  uint32_t count;
  unsigned long lastSeen;
  uint8_t data[8];      // Store last seen data
  uint8_t length;       // Store data length
};

SimpleUnknownMessage unknownMessages[20];  // Track last 20 unknown messages
int unknownCount = 0;

SimpleKnownMessage knownMessages[30];  // Track known message types
int knownCount = 0;

// BMS Data Structure
struct BMSData {
  float cellVoltages[32];  // 4 modules x 8 cells
  float moduleVoltages[4]; // Total voltage per module
  float temperatures[8];   // 2 per module
  float packVoltage;
  float maxTemp;
  float minTemp;
  float cellSpread;
  unsigned long lastUpdate;
  bool nodeStatus[4];      // AA, AB, AC, AD connectivity
} bmsData;

// Track which cells belong to which module
const char* moduleNames[] = {"AA", "AB", "AC", "AD"};

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Initialize BMS data
  memset(&bmsData, 0, sizeof(bmsData));
  memset(unknownMessages, 0, sizeof(unknownMessages));
  memset(knownMessages, 0, sizeof(knownMessages));
  
  Serial.println("=== BMS Network Dashboard ===");
  
  // Initialize CAN (your existing code)
  setupCAN();
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup web server
  setupWebServer();
  
  Serial.println("Dashboard ready!");
  Serial.printf("Access dashboard at: http://%s\n", WiFi.localIP().toString().c_str());
}

void setupCAN() {
  can_timing_config_t t_config = CAN_TIMING_CONFIG_250KBITS();
  can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
  can_general_config_t g_config = {
    .mode = CAN_MODE_LISTEN_ONLY,
    .tx_io = GPIO_NUM_17,
    .rx_io = GPIO_NUM_16,
    .clkout_io = CAN_IO_UNUSED,
    .bus_off_io = CAN_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 30,
    .alerts_enabled = CAN_ALERT_NONE,
    .clkout_divider = 0
  };
  
  esp_err_t result = can_driver_install(&g_config, &t_config, &f_config);
  if (result == ESP_OK) {
    Serial.println("✅ CAN driver installed");
    can_start();
    Serial.println("✅ CAN started");
  } else {
    Serial.printf("❌ CAN failed: %s\n", esp_err_to_name(result));
  }
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
    Serial.printf("✅ Connected to %s\n", ssid);
    Serial.printf("📡 IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("🌐 Dashboard URL: http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ WiFi connection failed!");
  }
}

void setupWebServer() {
  // Handle WebSocket connections
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // Serve the dashboard HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getDashboardHTML());
  });
  
  // API endpoint for current data
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getBMSDataJSON());
  });
  
  // Start server
  server.begin();
  Serial.println("✅ Web server started");
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client connected: %u\n", client->id());
    // Send current data to new client
    client->text(getBMSDataJSON());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client disconnected: %u\n", client->id());
  }
}

void loop() {
  can_message_t rx_message;
  
  // Read CAN messages
  if (can_receive(&rx_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    procesCANMessage(rx_message);
  }
  
  // Send updates to connected clients every 1 second
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    broadcastUpdate();
    lastUpdate = millis();
  }
  
  // Clean up WebSocket clients
  ws.cleanupClients();
}

// Simple helper to track known messages without disrupting existing logic
void trackKnownMessage(uint32_t id, const char* description, can_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < knownCount; i++) {
    if (knownMessages[i].id == id) {
      knownMessages[i].count++;
      knownMessages[i].lastSeen = millis();
      // Update with latest data
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

void procesCANMessage(can_message_t& msg) {
  uint32_t id = msg.identifier;
  uint8_t* data = msg.data;
  bool isKnownMessage = false;
  
  // Update last seen time
  bmsData.lastUpdate = millis();
  
  // Parse voltage messages (cells 1-4) - ORIGINAL CODE UNCHANGED
  if ((id & 0xFFFFFF00) == 0x18001300) {
    isKnownMessage = true;
    trackKnownMessage(id, "Voltage Data (Cells 1-4)", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      bmsData.nodeStatus[module] = true;
      for (int i = 0; i < 4; i++) {
        uint16_t voltage = (data[i*2+1] << 8) | data[i*2];
        bmsData.cellVoltages[module*8 + i] = voltage / 1000.0;
      }
    }
  }
  
  // Parse voltage messages (cells 5-8) - ORIGINAL CODE UNCHANGED
  else if ((id & 0xFFFFFF00) == 0x18011300) {
    isKnownMessage = true;
    trackKnownMessage(id, "Voltage Data (Cells 5-8)", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      for (int i = 0; i < 4; i++) {
        uint16_t voltage = (data[i*2+1] << 8) | data[i*2];
        bmsData.cellVoltages[module*8 + i + 4] = voltage / 1000.0;
      }
    }
  }
  
  // Parse temperature messages - ORIGINAL CODE UNCHANGED
  else if ((id & 0xFFFFFF00) == 0x18001400) {
    isKnownMessage = true;
    trackKnownMessage(id, "Temperature Data", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      uint16_t temp1 = (data[1] << 8) | data[0];
      uint16_t temp2 = (data[3] << 8) | data[2];
      bmsData.temperatures[module*2] = temp1/10.0;     // Keep in Fahrenheit
      bmsData.temperatures[module*2 + 1] = temp2/10.0;
    }
  }
  
  // Check for the new message types we discovered - ORIGINAL CODE UNCHANGED
  else if (id == 0x180011F4) {
    isKnownMessage = true;
    trackKnownMessage(id, "Master Request", msg);  // Pass msg for data
    Serial.printf("Master Request: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
  else if ((id & 0xFFFFFF00) == 0x18001200) {
    isKnownMessage = true;  // Basic ACK messages
    trackKnownMessage(id, "Basic ACK Messages", msg);  // Pass msg for data
  }
  else if ((id & 0xFFFFFF00) == 0x18021200) {
    isKnownMessage = true;  // Heartbeat messages
    trackKnownMessage(id, "Heartbeat Messages", msg);  // Pass msg for data
  }
  else if ((id & 0xFFFFFF00) == 0x18031200) {
    isKnownMessage = true;  // Node health messages - ORIGINAL CODE UNCHANGED
    trackKnownMessage(id, "Node Health/Status", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      Serial.printf("Node %s Health: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                   moduleNames[module], data[0], data[1], data[2], data[3], 
                   data[4], data[5], data[6], data[7]);
    }
  }
  else if ((id & 0xFFFFFF00) == 0x18041200) {
    isKnownMessage = true;  // System config messages - ORIGINAL CODE UNCHANGED
    trackKnownMessage(id, "System Configuration", msg);  // Pass msg for data
    Serial.printf("System Config: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
  
  // If unknown, log it - ORIGINAL CODE UNCHANGED
  if (!isKnownMessage) {
    logUnknownMessage(msg);
  }
  
  // Calculate derived values - ORIGINAL CODE UNCHANGED
  calculatePackStats();
}

void logUnknownMessage(can_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < unknownCount; i++) {
    if (unknownMessages[i].id == msg.identifier) {
      // Update existing entry
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
    unknownCount++;
    
    Serial.printf("NEW Unknown CAN ID: 0x%08X\n", msg.identifier);
  }
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

void calculatePackStats() {
  float total = 0;
  float maxVoltage = 0;
  float minVoltage = 5.0;
  float maxTemp = -50;
  float minTemp = 100;
  
  // Calculate pack voltage and cell spread
  for (int i = 0; i < 32; i++) {
    if (bmsData.cellVoltages[i] > 0) {
      total += bmsData.cellVoltages[i];
      if (bmsData.cellVoltages[i] > maxVoltage) maxVoltage = bmsData.cellVoltages[i];
      if (bmsData.cellVoltages[i] < minVoltage) minVoltage = bmsData.cellVoltages[i];
    }
  }
  
  // Calculate module voltages (8 cells each)
  for (int m = 0; m < 4; m++) {
    float moduleTotal = 0;
    for (int c = 0; c < 8; c++) {
      moduleTotal += bmsData.cellVoltages[m*8 + c];
    }
    bmsData.moduleVoltages[m] = moduleTotal;
  }
  
  // Calculate temperature range
  for (int i = 0; i < 8; i++) {
    if (bmsData.temperatures[i] > maxTemp) maxTemp = bmsData.temperatures[i];
    if (bmsData.temperatures[i] < minTemp) minTemp = bmsData.temperatures[i];
  }
  
  bmsData.packVoltage = total;
  bmsData.cellSpread = (maxVoltage - minVoltage) * 1000; // mV
  bmsData.maxTemp = maxTemp;
  bmsData.minTemp = minTemp;
}

String getBMSDataJSON() {
  DynamicJsonDocument doc(3072);
  
  doc["packVoltage"] = bmsData.packVoltage;
  doc["cellSpread"] = bmsData.cellSpread;
  doc["maxTemp"] = bmsData.maxTemp;
  doc["minTemp"] = bmsData.minTemp;
  doc["lastUpdate"] = bmsData.lastUpdate;
  
  // Cell voltages by module
  JsonArray modules = doc.createNestedArray("modules");
  for (int m = 0; m < 4; m++) {
    JsonObject module = modules.createNestedObject();
    module["name"] = moduleNames[m];
    module["connected"] = bmsData.nodeStatus[m];
    module["voltage"] = bmsData.moduleVoltages[m];
    
    JsonArray cells = module.createNestedArray("cells");
    for (int c = 0; c < 8; c++) {
      cells.add(bmsData.cellVoltages[m*8 + c]);
    }
    
    JsonArray temps = module.createNestedArray("temperatures");
    temps.add(bmsData.temperatures[m*2]);
    temps.add(bmsData.temperatures[m*2 + 1]);
  }
  
  // Add unknown messages
  JsonArray unknowns = doc.createNestedArray("unknownMessages");
  for (int i = 0; i < unknownCount; i++) {
    JsonObject unknown = unknowns.createNestedObject();
    unknown["id"] = String(unknownMessages[i].id, HEX);
    unknown["count"] = unknownMessages[i].count;
    unknown["length"] = unknownMessages[i].length;
    
    JsonArray dataArray = unknown.createNestedArray("data");
    for (int j = 0; j < unknownMessages[i].length; j++) {
      dataArray.add(unknownMessages[i].data[j]);
    }
  }
  
  // Add known messages - NEW
  JsonArray knowns = doc.createNestedArray("knownMessages");
  for (int i = 0; i < knownCount; i++) {
    JsonObject known = knowns.createNestedObject();
    known["id"] = String(knownMessages[i].id, HEX);
    known["description"] = knownMessages[i].description;
    known["count"] = knownMessages[i].count;
    known["lastSeen"] = knownMessages[i].lastSeen;
    known["length"] = knownMessages[i].length;
    
    JsonArray dataArray = known.createNestedArray("data");
    for (int j = 0; j < knownMessages[i].length; j++) {
      dataArray.add(knownMessages[i].data[j]);
    }
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

void broadcastUpdate() {
  String jsonData = getBMSDataJSON();
  ws.textAll(jsonData);
}

String getDashboardHTML() {
  return R"(
<!DOCTYPE html>
<html>
<head>
    <title>BMS Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background: #f0f0f0;
        }
        .header {
            background: #2c3e50;
            color: white;
            padding: 20px;
            border-radius: 8px;
            margin-bottom: 20px;
        }
        .stats {
            display: flex;
            justify-content: space-around;
            margin-bottom: 20px;
        }
        .stat {
            background: white;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            text-align: center;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            color: #27ae60;
        }
        .modules {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        .module {
            background: white;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .module-header {
            font-size: 18px;
            font-weight: bold;
            margin-bottom: 10px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .module-voltage {
            font-size: 16px;
            color: #2c3e50;
            font-weight: bold;
            margin-bottom: 8px;
            text-align: center;
            background: #ecf0f1;
            padding: 8px;
            border-radius: 4px;
        }
        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #e74c3c;
        }
        .status-dot.connected {
            background: #27ae60;
        }
        .cells {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 5px;
            margin-bottom: 10px;
        }
        .cell {
            padding: 8px;
            text-align: center;
            border-radius: 4px;
            font-size: 12px;
            color: white;
            background: #95a5a6;
        }
        .cell.good { background: #27ae60; }
        .cell.warning { background: #f39c12; }
        .cell.danger { background: #e74c3c; }
        .temps {
            font-size: 14px;
            color: #7f8c8d;
        }
        .unknown-section {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        .unknown-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }
        .unknown-table th,
        .unknown-table td {
            border: 1px solid #ddd;
            padding: 8px;
            text-align: left;
        }
        .unknown-table th {
            background-color: #f2f2f2;
            font-weight: bold;
        }
        .unknown-table tr:nth-child(even) {
            background-color: #f9f9f9;
        }
        .data-hex {
            font-family: monospace;
            font-size: 12px;
        }
        #connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 10px;
            border-radius: 4px;
            color: white;
            font-weight: bold;
        }
        .connected { background: #27ae60; }
        .disconnected { background: #e74c3c; }
    </style>
</head>
<body>
    <div id="connection-status" class="disconnected">Connecting...</div>
    
    <div class="header">
        <h1>BMS Dashboard with Message Tracking</h1>
        <p>Real-time battery monitoring and protocol analysis</p>
    </div>
    
    <div class="stats">
        <div class="stat">
            <div>Pack Voltage</div>
            <div class="stat-value" id="pack-voltage">---</div>
        </div>
        <div class="stat">
            <div>Cell Spread</div>
            <div class="stat-value" id="cell-spread">---</div>
        </div>
        <div class="stat">
            <div>Temperature Range</div>
            <div class="stat-value" id="temp-range">---</div>
        </div>
        <div class="stat">
            <div>Last Update</div>
            <div class="stat-value" id="last-update">---</div>
        </div>
    </div>
    
    <div class="modules" id="modules">
        <!-- Modules will be populated by JavaScript -->
    </div>

    <div class="unknown-section">
        <h2>Unknown CAN Messages</h2>
        <p>Messages that don't match known protocol patterns:</p>
        <table class="unknown-table">
            <thead>
                <tr>
                    <th>CAN ID</th>
                    <th>Count</th>
                    <th>Length</th>
                    <th>Data (Hex)</th>
                </tr>
            </thead>
            <tbody id="unknown-table-body">
                <tr><td colspan="4">No unknown messages detected</td></tr>
            </tbody>
        </table>
    </div>

    <div class="unknown-section">
        <h2>Known CAN Messages</h2>
        <p>Messages we've identified and understand:</p>
        <table class="unknown-table">
            <thead>
                <tr>
                    <th>CAN ID</th>
                    <th>Description</th>
                    <th>Count</th>
                    <th>Length</th>
                    <th>Data (Hex)</th>
                    <th>Last Seen</th>
                </tr>
            </thead>
            <tbody id="known-table-body">
                <tr><td colspan="6">No known messages tracked yet</td></tr>
            </tbody>
        </table>
    </div>

    <script>
        let ws;
        let reconnectTimer;
        
        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.host + '/ws');
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                document.getElementById('connection-status').textContent = 'Connected';
                document.getElementById('connection-status').className = 'connected';
                clearInterval(reconnectTimer);
            };
            
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    updateDashboard(data);
                } catch (e) {
                    console.error('Failed to parse data:', e);
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                document.getElementById('connection-status').textContent = 'Disconnected';
                document.getElementById('connection-status').className = 'disconnected';
                
                // Reconnect after 3 seconds
                reconnectTimer = setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }
        
        function updateDashboard(data) {
            // Update summary stats
            document.getElementById('pack-voltage').textContent = data.packVoltage.toFixed(2) + 'V';
            document.getElementById('cell-spread').textContent = data.cellSpread.toFixed(0) + 'mV';
            document.getElementById('temp-range').textContent = 
                data.minTemp.toFixed(1) + 'F - ' + data.maxTemp.toFixed(1) + 'F';
            document.getElementById('last-update').textContent = 
                new Date(data.lastUpdate + Date.now() - performance.now()).toLocaleTimeString();
            
            // Update modules
            const modulesDiv = document.getElementById('modules');
            modulesDiv.innerHTML = '';
            
            data.modules.forEach(module => {
                const moduleDiv = document.createElement('div');
                moduleDiv.className = 'module';
                
                moduleDiv.innerHTML = 
                    '<div class="module-header">Module ' + module.name + 
                    '<div class="status-dot ' + (module.connected ? 'connected' : '') + '"></div></div>' +
                    '<div class="module-voltage">Pack Voltage: ' + module.voltage.toFixed(2) + 'V</div>' +
                    '<div class="cells">' +
                    module.cells.map((voltage, index) => {
                        let cellClass = 'good';
                        if (voltage < 3.0 || voltage > 3.4) cellClass = 'danger';
                        else if (voltage < 3.1 || voltage > 3.35) cellClass = 'warning';
                        
                        return '<div class="cell ' + cellClass + '">Cell ' + (index + 1) + '<br>' + voltage.toFixed(3) + 'V</div>';
                    }).join('') +
                    '</div>' +
                    '<div class="temps">Temperatures: ' + module.temperatures[0].toFixed(1) + 'F, ' + module.temperatures[1].toFixed(1) + 'F</div>';
                
                modulesDiv.appendChild(moduleDiv);
            });
            
            // Update unknown messages table
            const tableBody = document.getElementById('unknown-table-body');
            if (data.unknownMessages && data.unknownMessages.length > 0) {
                tableBody.innerHTML = data.unknownMessages.map(msg => {
                    const dataHex = msg.data.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
                    return '<tr>' +
                        '<td>0x' + msg.id.toUpperCase() + '</td>' +
                        '<td>' + msg.count + '</td>' +
                        '<td>' + msg.length + '</td>' +
                        '<td class="data-hex">' + dataHex + '</td>' +
                        '</tr>';
                }).join('');
            } else {
                tableBody.innerHTML = '<tr><td colspan="4">No unknown messages detected</td></tr>';
            }
            
            // Update known messages table
            const knownTableBody = document.getElementById('known-table-body');
            if (data.knownMessages && data.knownMessages.length > 0) {
                // Sort by CAN ID to prevent jumping around
                const sortedKnown = data.knownMessages.sort((a, b) => {
                    const idA = parseInt(a.id, 16);
                    const idB = parseInt(b.id, 16);
                    return idA - idB;
                });
                knownTableBody.innerHTML = sortedKnown.map(msg => {
                    const timeSince = Math.floor((Date.now() - (msg.lastSeen + Date.now() - performance.now())) / 1000);
                    let timeText;
                    if (timeSince < 0 || timeSince > 86400) {
                        timeText = 'Just now';
                    } else if (timeSince < 60) {
                        timeText = timeSince + 's ago';
                    } else if (timeSince < 3600) {
                        timeText = Math.floor(timeSince/60) + 'm ago';
                    } else {
                        timeText = Math.floor(timeSince/3600) + 'h ago';
                    }
                    
                    const dataHex = msg.data.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
                    
                    return '<tr>' +
                        '<td>0x' + msg.id.toUpperCase() + '</td>' +
                        '<td>' + msg.description + '</td>' +
                        '<td>' + msg.count + '</td>' +
                        '<td>' + msg.length + '</td>' +
                        '<td class="data-hex">' + dataHex + '</td>' +
                        '<td>' + timeText + '</td>' +
                        '</tr>';
                }).join('');
            } else {
                knownTableBody.innerHTML = '<tr><td colspan="6">No known messages tracked yet</td></tr>';
            }
        }
        
        // Start connection
        connectWebSocket();
        
        // Refresh page if no data for 30 seconds
        let lastDataTime = Date.now();
        setInterval(() => {
            if (Date.now() - lastDataTime > 30000) {
                location.reload();
            }
        }, 5000);
    </script>
</body>
</html>
)";
}