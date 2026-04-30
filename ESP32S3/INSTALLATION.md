# Installation Guide

## Quick Start

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/CATL-BMS-ESP32S3-Bridge.git
cd CATL-BMS-ESP32S3-Bridge
```

### 2. Install Libraries
**Option A: Use bundled libraries (recommended)**
- Copy the `libraries/` folder contents to your Arduino libraries directory:
  - **Windows**: `Documents\Arduino\libraries\`
  - **macOS**: `~/Documents/Arduino/libraries/`
  - **Linux**: `~/Arduino/libraries/`

**Option B: Library Manager (may need manual fixes)**
```
Tools → Manage Libraries → Search and install:
- ACAN_ESP32 by Pierre Molinaro
- ESPAsyncWebServer by lacamera
- AsyncTCP by dvarrel
- Adafruit NeoPixel
```

### 3. Arduino IDE Configuration
1. File → Preferences → Additional Board Manager URLs:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

2. Tools → Board → Boards Manager → Search "ESP32" → Install

3. Select Board: **ESP32S3 Dev Module**

4. Configure Board Settings:
   - USB CDC On Boot: Enabled
   - CPU Frequency: 240MHz
   - Flash Mode: QIO
   - Flash Size: 4MB (or larger)
   - PSRAM: OPI PSRAM

### 4. Upload Firmware
1. Update WiFi credentials in `BMSReaderS3.ino` lines 19-20
2. Connect ESP32-S3 via USB
3. Select correct COM port in Tools → Port
4. Click Upload button
5. Monitor Serial output for IP address

### 5. Hardware Connections

#### CAN Bus (Waveshare SN65HVD230)
```
ESP32-S3    SN65HVD230    CATL BMS
GPIO 15  →  CTX
GPIO 16  →  CRX
3.3V     →  VCC
GND      →  GND
            CANH   →  CAN High
            CANL   →  CAN Low
```

#### RS485 (MAX485 Module)
```
ESP32-S3    MAX485       Growatt
GPIO 17  →  DI
GPIO 18  →  RO
GPIO 8   →  DE & RE
5V       →  VCC
GND      →  GND
            A      →  A (RS485+)
            B      →  B (RS485-)
```

## Testing

### 1. Serial Monitor Check
- Open Serial Monitor (115200 baud)
- Look for successful startup messages:
  ```
  [OK] CAN started for CATL communication
  [OK] RS485 Serial initialized
  [OK] WiFi connected! IP: xxx.xxx.xxx.xxx
  ```

### 2. Web Dashboard
- Navigate to displayed IP address
- Verify dashboard loads with live data
- Check voltage graph updates

### 3. CAN Communication
- Monitor for CAN message reception:
  ```
  [CAN] Received ID: 0x18001300, Data: A1 B2 C3...
  ```

### 4. RS485 Communication
- Connect Growatt inverter
- Watch for Modbus requests/responses:
  ```
  [GROWATT] Growatt request: addr=0x0016, count=10
  [GROWATT] Sent 25 bytes to Growatt
  ```

## Troubleshooting

### Compilation Errors
- Ensure all libraries are installed correctly
- Use bundled modified libraries for ESP32-S3 compatibility
- Verify ESP32 board package version (v3.0+)

### Upload Failures
- Press and hold BOOT button while connecting USB
- Try different USB cable/port
- Select correct board and port settings

### No Network Connection
- Verify WiFi credentials are correct
- Check 2.4GHz network compatibility
- Try WiFi network without special characters

For more help, see full troubleshooting guide in README.md