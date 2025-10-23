# VCU - Vehicle Control Unit with Web Interface

A professional FreeRTOS-based vehicle control unit for ESP32-S3 with a modern web interface for real-time monitoring and configuration.

## Features

### Core Functionality
- **FreeRTOS Multi-tasking**: Three dedicated tasks for CAN communication, control logic, and web server
- **Advanced Pedal Control**: Three-zone system (Regen/Coast/Accel) with smooth torque transitions
- **CAN Bus Integration**: Manages BMS, DMC, BSC, and NLG subsystems
- **Persistent Configuration**: JSON-based flash storage with validation
- **Real-time Monitoring**: Error logging, performance metrics, and system health checks

### Web Interface
- **Always-On Web Server**: Configure and monitor from any device on the network
- **Modern Dashboard**: Real-time charts for speed and power, live vehicle status
- **Tabbed Interface**:
  - **Dashboard**: Live monitoring with gauges and graphs
  - **Charging**: Configure charging current and target SOC
  - **Configuration**: Driving modes, pedal zones, torque transitions
  - **CAN Monitor**: Live CAN bus message viewer
  - **Task Monitor**: FreeRTOS task statistics
  - **System**: WiFi settings, debug mode, firmware info
- **WebSocket Updates**: 10Hz real-time data streaming
- **Dark Theme**: Professional automotive UI with purple/blue accents

## Hardware

### Microcontroller
- **ESP32-S3 DevKit C1**
- Dual-core ARM @ 240 MHz
- 16 MB Flash, 8 MB SRAM

### Communication
- MCP2515 CAN Controller (SPI)
- ADS1115 16-bit ADC (I2C)
- WiFi 802.11 b/g/n

### I/O
- Dual throttle pedal inputs
- HV contactor control
- Cooling pump PWM
- Charger/inverter enable outputs
- Connector unlock interrupt

## Quick Start

### 1. Build and Upload Firmware

```bash
# Install PlatformIO
pip install platformio

# Build project
pio run

# Upload firmware
pio run --target upload

# Upload filesystem (web interface)
pio run --target uploadfs
```

### 2. First-Time Setup

1. Power on the VCU
2. Look for WiFi network: `VCU_XXXXX` (XXXXX = chip ID)
3. Connect with password: `vcu12345`
4. Open browser to: `http://192.168.4.1`

### 3. Configure WiFi

1. Navigate to **System** tab
2. Enter your WiFi SSID and password
3. VCU will connect to your network
4. Note the new IP address displayed

## Web Interface

### Dashboard Tab
- **Live Data** (updates every 100ms):
  - Vehicle state, gear, speed, torque
  - Battery SOC, voltage, current
  - Motor/inverter/battery temperatures
  - Throttle, regen, brake inputs
- **Real-time Charts**:
  - Speed chart (last 100 samples)
  - Power chart (positive = discharge, negative = charge)

### Charging Tab
- Set maximum charging current (6-32A)
- Configure target SOC (50-100%)
- Monitor charger state and current

### Configuration Tab

#### Driving Mode
- **Legacy**: Traditional throttle response
- **Regen**: Regenerative braking enabled
- **OPD**: One Pedal Drive mode

#### Pedal Zones
- **Regen Zone End**: 10-50% (default 16%)
- **Coast Zone End**: 15-60% (default 17%)
- **Regen Progression**: 1.0-3.0 (curve aggressiveness)
- **Accel Progression**: 1.0-3.0 (curve aggressiveness)

#### Torque Transitions
- **Regen Engage**: Time for 0 → negative torque
- **Regen Release**: Time for negative → 0 torque
- **Power Engage**: Time for 0 → positive torque
- **Power Release**: Time for positive → 0 torque
- **Crossover**: Time for regen ↔ power transitions

### CAN Monitor Tab
- Live view of CAN messages from:
  - BMS (0x0F1) - Battery data
  - DMC (0x280) - Motor controller
  - BSC (0x26A) - DC-DC converter
  - NLG (0x728) - Charger

### Task Monitor Tab
- FreeRTOS task information
- Heap memory usage
- CPU frequency
- System performance metrics

### System Tab
- **WiFi Configuration**: Change network settings
- **Debug Mode**: Enable/disable (prevents deep sleep)
- **Firmware Info**: Chip model, version, uptime
- **Configuration Management**:
  - Export configuration to JSON file
  - Import configuration from JSON file
  - Factory reset

## REST API

All configuration is accessible via REST API:

### Status Endpoints
```
GET /api/status              - System status
GET /api/status/live         - Real-time vehicle data
GET /api/status/errors       - Error log
```

### Configuration Endpoints
```
GET  /api/config/driving     - Get driving config
POST /api/config/driving     - Set driving config
GET  /api/config/charging    - Get charging config
POST /api/config/charging    - Set charging config
GET  /api/config/pedal       - Get pedal zones
POST /api/config/pedal       - Set pedal zones
GET  /api/config/transition  - Get torque transitions
POST /api/config/transition  - Set torque transitions
GET  /api/config/limits      - Get speed/power limits
POST /api/config/limits      - Set limits
```

### System Endpoints
```
POST /api/system/save        - Save config to flash
POST /api/system/reset       - Factory reset
GET  /api/system/export      - Download config JSON
POST /api/system/import      - Upload config JSON
GET  /api/system/info        - System information
POST /api/system/debug       - Enable/disable debug mode
```

### CAN Monitor
```
GET /api/can/messages        - Get CAN message data
```

### WebSocket
```
ws://[IP_ADDRESS]/ws         - Real-time data stream (10Hz)
```

## Debug Mode

Debug mode prevents the VCU from entering deep sleep, useful for:
- Development and testing
- Keeping WiFi active during extended stops
- Debugging CAN communication issues

Enable via:
- Web interface: System tab → Enable Debug Mode
- API: `POST /api/system/debug {"enabled": true}`

## FreeRTOS Task Structure

### Core 0 (CAN Communication)
- **CAN Task** (Priority 1, 12KB stack)
  - CAN message RX/TX
  - Motor speed updates
  - Torque calculation
  - Temperature monitoring
  - 1ms cycle time

### Core 1 (Control & Web)
- **Control Task** (Priority 1, 24KB stack)
  - State machine updates
  - Interrupt handling
  - Performance monitoring
  - 50ms cycle time

- **Web Server Task** (Priority 1, 16KB stack)
  - WiFi management
  - HTTP request handling
  - WebSocket broadcasts
  - 10ms cycle time

## Safety Features

- Input validation on all configuration changes
- Automatic fallback to safe defaults
- Error monitoring with severity levels
- System health checks
- CAN timeout detection
- Temperature monitoring
- Voltage/current limits
- Emergency stop capability

## Configuration Backup

### Export Configuration
1. Open web interface
2. Navigate to System tab
3. Click "Export Configuration"
4. Save `vcu_config.json` file

### Import Configuration
1. Open web interface
2. Navigate to System tab
3. Click "Import Configuration"
4. Select previously saved JSON file

## Network Configuration

### Access Point Mode (Default)
- SSID: `VCU_XXXXX` (XXXXX = chip ID)
- Password: `vcu12345`
- IP Address: `192.168.4.1`

### Station Mode
- Configure via web interface System tab
- Automatically falls back to AP if connection fails
- Reconnects automatically if WiFi drops

## Troubleshooting

### Can't Connect to WiFi
1. Check that AP mode is active (look for `VCU_` network)
2. Default password is `vcu12345`
3. Try factory reset if needed

### Web Interface Not Loading
1. Check IP address in serial monitor
2. Ensure device is on same network
3. Try clearing browser cache
4. Use incognito/private browsing mode

### Configuration Not Saving
1. Click "Save Configuration" button after changes
2. Check API response in browser console
3. Verify flash storage isn't full

### Real-time Data Not Updating
1. Check WebSocket connection status (indicator in header)
2. Refresh page to reconnect
3. Ensure CAN bus is connected and operational

## Development

### File Structure
```
VCU/
├── src/
│   ├── main.cpp                  - Main application
│   ├── state_manager.cpp         - State machine
│   ├── vehicle_control.cpp       - Pedal control
│   ├── can_manager.cpp           - CAN communication
│   ├── configuration.cpp         - Config management
│   ├── wifi_manager.cpp          - WiFi management
│   └── web_server.cpp            - Web server
├── include/
│   └── [corresponding headers]
├── data/
│   ├── index.html                - Main web page
│   ├── styles.css                - Styling
│   ├── app.js                    - API/config logic
│   └── dashboard.js              - Charts/live data
├── platformio.ini                - Build configuration
└── min_spiffs.csv                - Partition table
```

### Building from Source
```bash
# Clone repository
git clone [your-repo-url]
cd VCU

# Install dependencies (automatic via PlatformIO)
pio lib install

# Build
pio run

# Upload firmware
pio run --target upload

# Upload web interface
pio run --target uploadfs

# Monitor serial output
pio device monitor
```

## License

[Your License Here]

## Credits

Built with:
- PlatformIO
- ESP32 Arduino Framework
- ESPAsyncWebServer
- ArduinoJson
- FreeRTOS

## Support

For issues, questions, or contributions, please open an issue on GitHub.
