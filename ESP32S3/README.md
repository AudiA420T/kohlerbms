# CATL BMS to Growatt Inverter Bridge (ESP32-S3)

Complete ESP32-S3 based bridge that translates CATL BMS CAN bus data to Growatt solar inverter RS485/Modbus protocol.

## Features

- **Real-time CAN to RS485 Protocol Translation**
- **Web Dashboard** with live voltage graphs and system monitoring
- **Advanced Safety Controls** with overvoltage/undervoltage protection
- **Cell Balancing Detection** and thermal management
- **Complete BMS Data Visualization** via WebSocket interface
- **Message Tracking & Debugging** for both CAN and RS485 protocols

## Hardware Requirements

### ESP32-S3 Development Board
- ESP32-S3 with built-in RGB LED (pin 48)
- Minimum 4MB flash, 8MB PSRAM recommended

### CAN Bus Interface
- **Waveshare SN65HVD230 CAN Module**
  - VCC → 3.3V
  - GND → GND
  - CTX → GPIO 15
  - CRX → GPIO 16

### RS485 Interface
- **RS485 to TTL Module** (e.g., MAX485)
  - A/B → Growatt inverter RS485 terminals
  - RO → GPIO 18 (RX)
  - DI → GPIO 17 (TX)
  - DE/RE → GPIO 8

### Optional Control Outputs
- Charge Enable → GPIO 19 (disabled by default)
- Discharge Enable → GPIO 20 (disabled by default)

## Software Installation

### 1. Arduino IDE Setup
- Install ESP32 boards package (v3.0+)
- Select "ESP32S3 Dev Module" as board

### 2. Library Installation
The project includes all required libraries in the `libraries/` folder:

- **ESPAsyncWebServer** (Modified for ESP32-S3)
- **AsyncTCP** (Modified with TCPIP core fixes)
- **ACAN_ESP32** (CAN bus communication)
- **Adafruit_NeoPixel** (RGB LED support)

### 3. Upload Firmware
1. Update WiFi credentials in `BMSReaderS3.ino`:
   ```cpp
   const char* ssid = "Your_WiFi_Network";
   const char* password = "Your_WiFi_Password";
   ```

2. Compile and upload `BMSReaderS3.ino`

3. Open Serial Monitor (115200 baud) to verify operation

## Usage

### Access Web Dashboard
- Connect to your WiFi network
- Navigate to the ESP32-S3's IP address (shown in Serial Monitor)
- Dashboard shows real-time BMS data, voltage graphs, and system status

### Monitor Operation
- **RGB LED Status Indicators:**
  - Red: Starting up
  - Green: Basic setup complete
  - Purple: CAN bus ready
  - Yellow: WiFi connecting
  - White: All systems ready
  - Cyan Heartbeat: Normal operation

### RS485 Communication
- Bridge automatically responds to Growatt inverter Modbus requests
- Translates all BMS parameters (voltage, current, SOC, cell voltages, etc.)
- Implements safety limits and adaptive charge/discharge control

## Configuration

### Safety Limits
Located in `BMSReaderS3.ino` - modify as needed for your battery chemistry:

```cpp
struct SafetyLimits {
  float maxCellVoltage = 3.60;      // V - charge reduction threshold
  float hardStopVoltage = 3.65;     // V - emergency stop
  float minCellVoltage = 3.00;      // V - discharge cutoff
  float maxPackVoltage = 29.0;      // V - pack level limits
  float minPackVoltage = 24.0;      // V
} safetyLimits;
```

### Pin Assignments
All pins can be modified in the header section of `BMSReaderS3.ino`

## Troubleshooting

### No CAN Messages
- Verify CATL BMS is powered and transmitting
- Check CAN bus termination (120Ω resistors)
- Confirm wiring: TX→15, RX→16, 250kbps baud rate

### RS485 Communication Issues
- Verify Growatt inverter RS485 settings match (9600 baud, 8N1)
- Check A/B polarity and termination
- Monitor Serial output for Modbus request/response debugging

### Web Dashboard Not Loading
- Verify WiFi connection and IP address
- Check browser console for JavaScript errors
- Try hard refresh (Ctrl+F5) to clear cache

## API Endpoints

- `http://[ip]/` - Main dashboard
- `http://[ip]/api/data` - JSON BMS data
- `http://[ip]/api/modules` - Cell voltage data
- `http://[ip]/api/messages` - CAN message tracking
- `http://[ip]/api/voltage-history` - Voltage graph data

## License

This project is open source under the MIT License.

## Acknowledgments

- ACAN_ESP32 library by Pierre Molinaro
- ESPAsyncWebServer by Hristo Gochkov
- Adafruit NeoPixel library
- CATL BMS protocol reverse engineering community