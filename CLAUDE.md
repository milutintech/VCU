# VCU firmware: working notes

ESP32-S3, PlatformIO (`pio run`), FreeRTOS. LIVI (`/Users/cmo/LIVI`) is the only UI: the VCU serves
no web page and no ESP-NOW. Wire contract and plan: `/Users/cmo/LIVI/docs/VCU.md`.

## API used by LIVI

`GET /api/system/info`, `/api/system/limits`, `/api/status/live` (also `/ws`, includes `motor`,
`vehicle`, `inputs`, `charging`, `dcdc`), `/api/status/faults`, `/api/bms/cells`,
`GET|POST /api/config/{driving,charging,pedal,transition,throttle,bms}`, `/api/throttle/raw`,
`POST /api/system/save`. Root `/` returns one line of text.

## Added in this work

- `bms_cells.{h,cpp}`: Orion cell broadcast decode (cell index, 0.1 mV voltage, shunt flag in
  byte 3 bit 7, checksum), IDs/byte order/counts configurable, off until `cellId` is set; own NVS
  namespace `vcu_bms`.
- `fault_handler.{h,cpp}`: edge-triggered faults with hysteresis, CAN timeouts (BMS/DMC/BSC/NLG),
  torque reactions (derate x0.5 at 100 °C, cutoff 105 °C, ...). Thresholds in `FaultLimits` and the
  README tables; `vehicle_parameters.h` is untouched and must not be changed without asking.
- `ErrorMonitor` fault list behind `/api/status/faults`; repeated raises are rate-limited.
- Throttle POST applies min/max in a safe order (min must stay below max).

## Rules

- No PlatformIO in Claude's shell: the user compiles. Say when something was not built.
- Do not change CAN IDs, control task timing or safety thresholds without asking.

## Open items

- `MIN_LVVOLTAGE = 14.4` is an `int` so the BSC gets 14.0 V; `Motor::MAX_RPM` 6000 (used) vs
  `Limits::MAX_MOTOR_RPM` 8000 (unused); BSC `modeAct` values unnamed (need the DBC for 0x26A).
- Station WiFi credentials have no API (needed if the VCU should join the car's network).
- Orion frame details to confirm with a capture; thermistor frame layout is an assumption.
