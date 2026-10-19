# VCU - Vehicle Control Unit

A professional FreeRTOS-based vehicle control unit for ESP32-S3 with a JSON/WebSocket API. [LIVI](../../LIVI) is the user interface; the VCU serves no web page of its own.

## Features

### Core Functionality
- **FreeRTOS Multi-tasking**: Three dedicated tasks for CAN communication, control logic, and web server
- **Advanced Pedal Control**: Three-zone system (Regen/Coast/Accel) with smooth torque transitions
- **CAN Bus Integration**: Manages BMS, DMC, BSC, and NLG subsystems
- **Persistent Configuration**: JSON-based flash storage with validation
- **Real-time Monitoring**: Error logging, performance metrics, and system health checks

### API
- **Always-On Web Server**: REST + WebSocket on port 80, used by LIVI
- **WebSocket Updates**: 10Hz live data streaming
- **BMS cells and faults**: see the end of this file

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
```

### 2. First-Time Setup

1. Power on the VCU
2. Look for WiFi network: `VCU_XXXXX` (XXXXX = chip ID)
3. Connect with password: `vcu12345`
4. Point LIVI at `192.168.4.1` (Settings → Vehicle → Status)

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
- API: `POST /api/system/debug {"enabled": true}`

## FreeRTOS Task Structure

### Core 0 (CAN Communication)
- **CAN Task** (Priority 1, 12KB stack)
  - CAN message RX/TX
  - Motor speed updates
  - Torque calculation
  - Temperature monitoring
  - 1ms cycle time

### Core 1 (Control & API)
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
`GET /api/system/export` returns the configuration as JSON; save it as `vcu_config.json`.

### Import Configuration
`POST /api/system/import` with the saved JSON, then `POST /api/system/save`.

## Network Configuration

### Access Point Mode (Default)
- SSID: `VCU_XXXXX` (XXXXX = chip ID)
- Password: `vcu12345`
- IP Address: `192.168.4.1`

### Station Mode
- The VCU has no API for station credentials; see `wifi_manager`
- Automatically falls back to AP if connection fails
- Reconnects automatically if WiFi drops

## Troubleshooting

### Can't Connect to WiFi
1. Check that AP mode is active (look for `VCU_` network)
2. Default password is `vcu12345`
3. Try factory reset if needed

### LIVI Can't Reach the VCU
1. Check the VCU's IP address in the serial monitor
2. Ensure LIVI's host is on the VCU's network
3. Check `http://<ip>/api/system/info` answers

### Configuration Not Saving
1. Writes are saved by the VCU itself; check the API response of `POST /api/system/save`
2. Look at the error LIVI shows under Adjustments
3. Verify flash storage isn't full

### Real-time Data Not Updating
1. Check that `/ws` accepts a connection (LIVI reconnects with backoff)
2. Confirm the link state under Settings → Vehicle → Status
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


## BMS cells and faults API (for LIVI)

- `GET /api/bms/cells` → `{ "cellsMv": [...], "balancing": [idx...], "temps": [...], "ageMs": n }`.
  Cells come from the Orion *Battery Cell Broadcast* frame (see `include/bms_cells.h`).
  `cellsMv` is empty until a valid frame has been decoded.
- `GET|POST /api/config/bms` → `{ cellId, tempId, bigEndian, checkChecksum, cellCount, tempCount }`.
  `cellId`/`tempId` are the CAN IDs set in the Orion BMS Utility; `0` turns decoding off (default).
  Stored in its own NVS namespace, applied immediately.
- `GET /api/status/faults` → `{ "faults": [{ code, name, severity, active, count, lastSeenS }] }`.
  Aggregated from `ErrorMonitor` entries at WARNING or above. `active` is edge-triggered: set on raise, cleared by `FaultHandler` once the value is back inside the hysteresis band (no time-based expiry). `count` is the number of inactive→active transitions.


## Fault handling

`FaultHandler` (`include/fault_handler.h`, `src/fault_handler.cpp`) runs every CAN-task cycle in `main.cpp`.
It is the only place that raises/clears the fault codes below and the only place that maps faults to reactions.
`ErrorMonitor` just stores them. Raising an already active fault does not log or count again.

### Faults and thresholds

Values marked *existing* come from `include/vehicle_parameters.h` and are unchanged (`INV_HIGH`/`MOT_HIGH` = 45 °C, `INV_LOW`/`MOT_LOW` = 30 °C and `FAN_ON` = 65 °C are the cooling thresholds in `StateManager`, the pump on above 45 and off below 30, the fan on above 65, and are not used for derating); the others are new and live in `FaultLimits` (`fault_handler.h`) so the safety header was not touched.

| Code | Severity | Raised when | Cleared when |
|---|---|---|---|
| `BMS_TIMEOUT` | ERROR | no BMS frame (0x010) for > 1500 ms (*existing* `Timing::BMS_TIMEOUT_MS`) | a BMS frame is fresh again |
| `DMC_TIMEOUT` | CRITICAL | in RUN, no DMC frame (0x258/0x259/0x458) for > 500 ms (new) | frame fresh again, or leaving RUN |
| `BSC_TIMEOUT` | WARNING | no BSC status frame for > 1500 ms (new) | frame fresh again |
| `NLG_TIMEOUT` | WARNING | in CHARGING, no NLG frame for > 1500 ms (new) | frame fresh again, or leaving CHARGING |
| `CHARGER_OVERTEMP` | INFO | in CHARGING, charger cooling plate > 85 °C (new; the charger itself is good to 95 °C) | plate <= 80 °C, or leaving CHARGING |
| `BATTERY_UNDERVOLTAGE` | ERROR | pack < 333 V (*existing* `Battery::MIN_VOLTAGE`), BMS data fresh | pack >= 338 V (+5 V, new) |
| `BATTERY_OVERVOLTAGE` | CRITICAL | pack > 437 V (*existing* `Battery::MAX_VOLTAGE`), BMS data fresh | pack <= 432 V (-5 V, new) |
| `INVERTER_OVERTEMP` | ERROR | inverter > 100 °C (new `TEMP_DERATE_C`); CRITICAL above 105 °C (new `TEMP_CUTOFF_C`) | inverter < 95 °C (-5 °C, new) |
| `MOTOR_OVERTEMP` | ERROR | motor > 100 °C (new `TEMP_DERATE_C`); CRITICAL above 105 °C (new `TEMP_CUTOFF_C`) | motor < 95 °C (-5 °C, new) |

Nodes that never sent anything are timed from boot (BMS, BSC) or from entering the state that needs them (DMC, NLG), so an unpowered node in standby does not raise a fault. Temperature faults are evaluated only after the first DMC frame, voltage faults only while BMS data is fresh.
Not wired yet (no source signal in the firmware): `BATTERY_OVERCURRENT`, `DMC_ERROR`, `SOC_OUT_OF_RANGE`, precharge and charger codes.

### Reactions

Applied in `canTask` (`main.cpp`) in RUN state; the fault reaction is the minimum over all active faults.

| Active fault | Reaction |
|---|---|
| `INVERTER_OVERTEMP` / `MOTOR_OVERTEMP` | torque request x 0.5 (drive and regen); x 0 above 105 °C |
| `BATTERY_UNDERVOLTAGE` | torque request x 0.5 |
| `BATTERY_OVERVOLTAGE` | torque request x 0 |
| `BMS_TIMEOUT` | torque request x 0 |
| any active CRITICAL fault (`DMC_TIMEOUT`, `BATTERY_OVERVOLTAGE`, overtemp above 105 °C) | torque x 0 and DMC enable bit forced off |
| `BSC_TIMEOUT`, `NLG_TIMEOUT`, `CHARGER_OVERTEMP` | log/report only (the charger reduces its own power) |

Reactions are not latched: they release automatically once the fault clears. `isSystemHealthy()` now reports unhealthy while a CRITICAL fault is active (instead of whenever any critical line was ever logged). `/api/status/faults` output shape is unchanged. CAN IDs and task timing are unchanged.

- `GET /api/system/limits` → the limits LIVI uses for gauges and slider ranges: `maxRpm`
  (`VehicleParams::Motor::MAX_RPM`, the one applied in `CANManager`), `maxTorque`, `maxSoc`,
  `chargeCurrent` (min/max), `motorTemp`/`inverterTemp` (`derate`, `cutoff`),
  `packVoltage` (min/max), `chargerTempMax`.
