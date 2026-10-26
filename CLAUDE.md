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

Walked through one by one with the owner; the pack is **100 cells in series** (was 104 until 2026-10-22).

- LV charge voltage 14.4 V (`float`, rounded when sent). Max rpm 6000 (the unused 8000 is gone).
- Pack volts (100S): min 300 V (undervoltage fault, DMC motor limit, BSC minimum), max 420 V
  (charger CV target, DMC regen limit 424 V), overvoltage fault above 422 V (`Battery::OVERVOLTAGE`).
- DMC current: min(450 A drive, 420 A regen, BMS limit); 0 A without fresh BMS data.
- Torque: pedal full scale = `maxTorque` (forward) or `maxReverseTorque` (reverse, default 450), both
  100 to 850 Nm and settable in LIVI. `Motor::MAX_TRQ` 850 stays a hard cap. Before this the
  configured value was ignored (hard-coded 850).
- Charging 80 A DC / 32 A AC / 20 A precharge, BSC 100 A, temperatures 100 / 105 / 5 (derate,
  cutoff, hysteresis), timeouts BMS 1600 (owner raised it from 1500), DMC 500, BSC 1500, NLG 1500 ms, pedal and shift values
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

## BMS frames (Lada.dbc, Orion)

- 0x010 `BMS_PACK` (248 ms): SOC 0.5 %, voltage 0.1 V, current int16 0.1 A (negative = charging;
  stored raw, the API divides by 10), DCL 1 A, CCL 2 A. Matches the DBC.
- 0x001 `BMS_MINMAX` (632 ms, 7 bytes): temps int8, low/high cell 0.1 mV, byte 6 bit 0 charge
  allowed, bit 1 discharge allowed. Seen in a capture (2026-10-23): full pack (SOC 100 %, CCL 0) sends
  both flags 0, and the BMS starts sending a moment after the VCU boots (flags count as
  "not allowed" until then).
  Discharge flag 0 while plugged in is expected: Orion charge/drive interlock (owner).
- Allow flags are safety critical: missing frame = not allowed. Override `ignoreAllowFlags` in
  `/api/config/bms`, runtime only (never saved), DCL/CCL still apply.
- Max SOC 100 % = no SOC check, charge until unplugged (BMS CCL and charge flag decide).

## CAN task scheduling (owner approved 2026-10-23)

The CAN task was priority 1 on core 0 next to WiFi/lwIP/AsyncTCP and got starved: the MCP2515
(2 RX buffers, bus bursts of 3 to 5 frames within 1 ms) overflowed, BMS frames were lost and
0x210 went late (DMC `CanCtrlMsgLost`). Now: CAN task core 1, priority 5; AsyncTCP pinned to
core 0 (`platformio.ini`); MCP2515 INT on IO37 wakes the task to read frames at once (send
periods and the 1-tick loop unchanged). Self-test 3 s after boot logs whether INT fires;
`GET /api/system/info` `can { interrupts, frames, rxOverflows, intPinLow }`, overflows logged
every 10 s. Serial has a 4 KB TX buffer so logging does not block.

Second cause, found with the overflow counter: the ADS1115 reads (pedal x4, gear switches A2/A3)
ran inside the CAN task at 128 SPS, ~47 ms blocked per loop in RUN. Now an ADC task (core 1,
priority 2) owns I2C, 860 SPS, sleeps during conversions, full set every ~7 ms; the CAN task uses
the cached values. Older than 100 ms (`INPUT_STALE_MS`) = 0 % torque and neutral (owner approved).

## Open items

- Station WiFi credentials: API `GET|POST /api/config/wifi` exists (applies on next boot).
- Orion frame details to confirm with a capture (cell frame, thermistor layout, charge limit byte
  `x2` decode); BSC `modeAct` values unnamed (need the DBC for 0x26A).
- AP IP in `startAP()` is 192.168.4.3, LIVI's default host is 192.168.4.1.
