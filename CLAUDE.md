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

## Decisions on the constants (all in `vehicle_parameters.h` / `fault_handler.h`)

Walked through one by one with the owner; the pack is **104 cells in series**.

- LV charge voltage 14.4 V (`float`, rounded when sent). Max rpm 6000 (the unused 8000 is gone).
- Pack volts 333 / 382 / 437 V (3.2 / 3.67 / 4.2 V x 104): faults, charger target (437 V), DMC
  voltage limits, BSC minimum.
- DMC current: min(450 A drive, 420 A regen, BMS limit); 0 A without fresh BMS data.
- Torque: pedal full scale = `maxTorque` (forward) or `maxReverseTorque` (reverse, default 450), both
  100 to 850 Nm and settable in LIVI. `Motor::MAX_TRQ` 850 stays a hard cap. Before this the
  configured value was ignored (hard-coded 850).
- Charging 80 A DC / 32 A AC / 20 A precharge, BSC 100 A, temperatures 100 / 105 / 5 (derate,
  cutoff, hysteresis), timeouts BMS 1500, DMC 500, BSC 1500, NLG 1500 ms, pedal and shift values
  unchanged.
- Cooling: pump on above 45 C, off below 30 C, fan on above 65 C (`FAN_ON`). Charger limit 95 C;
  `CHARGER_OVERTEMP` is an INFO fault above 85 C (clears at 80), no reaction.
- Unused constants were deleted (speed limits, MAX_REQ_TRQ, DMC_DC_MOT, NOM_VOLTAGE, ...).
- None of this has been compiled or flashed yet (last known good build: before these edits).

## WiFi behaviour (wifi_manager.cpp)

Saved station credentials (`/api/config/wifi`, applied on the next boot) make the VCU join that
network (meant: the infotainment Pi's own WPA2 network) and switch its AP off. At boot it tries for
10 s, falls back to its AP (`VCU_xxxx`), then retries every 30 s from the AP (skipped while a client is
on the AP) and drops the AP once joined. If the station is gone for 60 s the AP comes back. The
Pi is powered through the VCU, so it boots later than the VCU: the retry is what makes that work.
The HTTP API has no authentication: only put the VCU on a network that is yours alone.

## Pi power and shutdown (pi_power_logic.h, pi_power.{h,cpp})

IO21 (`Pins::LWP6`) switches the Pi's supply (HIGH = on, assumed active-high: check the hardware).
The pin is LOW after every reset. Sequence: Pi powered once the ignition has been on 1 s (and power
was off 3 s); ignition off for 3 s = `REQUESTED` (`GET /api/pi` and the live JSON `pi` block say
`shutdownRequested`); the Pi POSTs `/api/pi/ack`; power is cut 20 s after the ack, 15 s without an
ack, 60 s at the very latest. A request always runs to the end, also if the key comes back, and
the Pi is powered again after the off time (quick key cycle = clean Pi restart, car unaffected:
it runs in the web task, never in the control task). Deep sleep waits until the Pi has no power
(`StateManager::handleStandbyState`), IO21 is held low through it. The Pi side is
`scripts/pi/vcu-power-watch.py` in the LIVI repo (own systemd service, not LIVI). Charging with the
ignition off does NOT power the Pi. Host test of the logic:
`c++ -std=c++17 -I include tools/host-tests/pi_power_logic_test.cpp && ./a.out`.
Not compiled for the ESP32 yet.

## Open items

- Station WiFi credentials: API `GET|POST /api/config/wifi` exists (applies on next boot).
- Orion frame details to confirm with a capture (cell frame, thermistor layout, charge limit byte
  `x2` decode); BSC `modeAct` values unnamed (need the DBC for 0x26A).
- AP IP in `startAP()` is 192.168.4.3, LIVI's default host is 192.168.4.1.
