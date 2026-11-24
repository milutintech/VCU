# VCU firmware: working notes

ESP32-S3, PlatformIO (`pio run`), FreeRTOS. LIVI (`/Users/cmo/LIVI`) is the only UI: the VCU serves
no web page and no ESP-NOW. Wire contract and plan: `/Users/cmo/LIVI/docs/VCU.md`.

## API used by LIVI

`GET /api/system/info`, `/api/system/limits`, `/api/status/live` (also `/ws`, includes `motor`,
`vehicle`, `inputs`, `charging`, `dcdc`), `/api/status/faults`, `/api/bms/cells`,
`GET|POST /api/config/{driving,charging,pedal,transition,throttle,bms}`, `/api/throttle/raw`,
`POST /api/system/save`, `GET /api/log/events` (RTC event log). Serial commands: `service on|off`, `net`, `events`, `canids`. Root `/` returns one line of text.

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
- DMC current: drive 450 A cap with DCL pulses (see "Acceleration pulses"), regen min(420 A, CCL);
  0 A without fresh BMS data.
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

## CAN RX task and per-ID counts (owner approved 2026-11-19)

The 10 MHz log still showed overflows: INT->read ~1 ms already without Ethernet (CAN task busy in its
loop, up to ~3 ms) plus SPI waits of 0.5-1.4 ms behind the W5500. Now a `CAN_RX` task (core 1, priority 6,
above the CAN task) is woken by the INT ISR (1-tick poll fallback), reads frames into a 64-frame queue,
checks/clears the overflow flags and notifies the CAN task, which processes the queue in `update()` as
before (send timing unchanged). Loop work no longer delays reading; the W5500 wait remains (separate SPI
bus planned in the new hardware). Per-ID frame counts (`used` = processed by the VCU) in `/api/system/info`
`can.ids`, `can.queueDrops`, and the serial command `canids` (with rates), to choose MCP2515 filters.
Not compiled.

## Ethernet (ethernet_link.{h,cpp}, added 2026-10-26)

W5500 on the MCP2515's SPI bus (SCK IO4, MOSI IO6, MISO IO5), CS IO15 (`EXT_CS`), INT IO3,
RST IO20. Direct cable to the Pi, fixed IP, no DHCP: VCU 192.168.5.1/24, Pi 192.168.5.2 (set
on the Pi, LIVI host 192.168.5.1). Bus sharing: both drivers lock through SPIClass transactions
on `CANManager::spiBus()` (started in setup before ETH; the old `SPI.begin()` in the CAN task
is gone, it took the pins from that bus). W5500 at 20 MHz, its receive task lowered from 15 to
4 (below the CAN task). Watch `can.rxOverflows` with Ethernet traffic. Overflow diagnostics (2026-11-19): max per 10 s of INT->read latency,
SPI bus wait before a read, `queueFrame` time, CAN loop time and TX fails, in the overflow log line and
`/api/system/info` `can.timingMaxUs`. Not compiled. Same day: all periodic MCP2515 access is our own code in
`LockedMcp2515` at 10 MHz (`SPI_HZ`, was the library's 4 MHz): `readFrame`, `queueFrame` (READ STATUS +
LOAD TX BUFFER + RTS in one transaction, was ~10 library transactions per frame), `readClearOverflow`
(was `checkError`). Only init uses the library. If frames come in garbled, try 8 MHz. Live JSON on `/ws`
5 Hz (`WS_BROADCAST_INTERVAL` 200 ms, was 100), skipped while a client's queue is full. Not compiled. The Seeed library reads received
frames (`mcp2515_read_canMsg`) without the SPI lock, which broke CAN once the W5500 shared the
bus (2026-10-28: garbage/no frames). `LockedMcp2515::readFrame()` (can_manager) reads status and
frame under one `beginTransaction()`; do not go back to `readMsgBuf()`. RST held low in deep
sleep. mDNS `vcu.local` starts with Ethernet (the Pi's shutdown watcher uses it). `GET /api/system/info` has `eth { present, link, ip, mac, speedMbps }`.
IO20 is USB D+ and IO3 a strapping pin: fine while the board programs over UART0.
Builds (2026-10-26, user's `pio run`); not flashed or tested on hardware yet.

## WiFi = service mode (wifi_manager.cpp)

WiFi is off at boot. Serial (UART0, 115200) commands, read in the web task: `service on` starts
WiFi as before (saved station from `/api/config/wifi`, else AP `VCU_xxxx` at 192.168.4.3, with
the retry/fallback logic below), `service off` stops it, `net` prints status. Runtime only,
a reboot ends service mode. The old `EnhancedSerialConsole` is still never created.
Inside service mode: tries the saved station 10 s, falls back to the AP, retries every 30 s from
the AP (skipped while a client is on the AP), AP back after 60 s without the station.
The HTTP API has no authentication: only put the VCU on a network that is yours alone.

## Pi power and shutdown (pi_power_logic.h, pi_power.{h,cpp})

IO21 (`Pins::LWP6`) switches the Pi's supply (HIGH = on, assumed active-high: check the hardware).
The pin is LOW after every reset. Sequence: Pi powered once the ignition has been on 1 s (and power
was off 3 s); ignition off for 3 s = `REQUESTED` (`GET /api/pi` and the live JSON `pi` block say
`shutdownRequested`); the Pi POSTs `/api/pi/ack`; power is cut 50 s after the ack (owner, 2026-10-29), 15 s
without an ack, 70 s at the very latest. A request always runs to the end, also if the key comes back, and
the Pi is powered again after the off time (quick key cycle = clean Pi restart, car unaffected:
it runs in the web task, never in the control task). Deep sleep waits until the Pi has no power
(`StateManager::handleStandbyState`), IO21 is held low through it. The Pi side is
`scripts/pi/vcu-power-watch.py` in the LIVI repo (own systemd service, not LIVI). Ignition only:
charging does not power the Pi (owner, 2026-11-20, screen off while charging; from 2026-11-02 it did).
Key off during a charge = normal Pi shutdown, the charge continues; key on while charging powers it. 12 V from the BSC (fresh frame, >= 5 V) below 11.5 V counts like ignition off, so
the normal shutdown runs; the Pi may start again from 12.5 V (`pi_power.cpp`, 12.5 V is a VCU choice
against a boot loop). Live JSON `pi.lvLow`. Host test of the logic:
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
- Regen (RUN) needs fresh 0x010, CCL > 0 and the charge flag (`chargeAllowed()`, so
  `ignoreAllowFlags` applies). Owner put the flag back 2026-11-02 after trying CCL only.
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

## Precharge (state_manager.cpp `armBattery`, owner approved 2026-10-28)

- Starts (and counts) only with fresh BMS data (`BMS_TIMEOUT_MS`) and a pack voltage within
  `Battery::MIN_VOLTAGE..MAX_VOLTAGE`; otherwise BSC off, "Precharge waiting" every 5 s.
- RUN only: no precharge and no BSC (boost or buck) while the BMS does not allow discharging
  (`dischargeAllowed()`, so `ignoreAllowFlags` applies). Flag drops after closing: BSC off,
  contactor stays closed, BSC back 2 s after it returns. CHARGING ignores the flag (Orion
  interlock clears it while plugged in, and the DC-DC must run). Owner, 2026-10-29.
- Max 3 timeouts, then `PRECHARGE_FAILED` (ERROR) and no more tries until `transitionToStandby()`
  (ignition off / charge session over), which also clears the fault. Before: retried forever.
- BSC HV target (0x260 byte 2, offset 220 V) is clamped to 220..475 V; it used to wrap
  (100 V pack -> 356 V target).
- DMC KL15 (IO13) is HIGH for all of RUN, on purpose: the same line powers the Orion BMS
  (owner, 2026-10-28), and precharge needs its pack voltage. Gating it on precharge deadlocked.
  Driving stays gated: 0x210 enable needs `isPreCharged()`. Not HIGH in CHARGING, not needed:
  the BMS has a separate supply there (owner, 2026-11-02).

## Acceleration pulses above the DCL (dcl_pulse_logic.h, owner 2026-11-02)

The Orion DCL is conservative, so the DMC drive limit may exceed it on purpose: min(450 A DMC cap,
200 % of DCL) for 60 s of time above the DCL, the last 5 s a linear taper down to the DCL. Budget
returns 1:1 only below 90 % of the DCL. DCL 0 / discharge not allowed / stale BMS = 0 A, never
pulsed over. 200 % and 60 s mirror the Orion profile; 5 s and 90 % are VCU choices. The DCL in
0x010 does not include the Orion's pulse (owner), so the 200 % does not stack. Live JSON
`battery.dmcDriveLimit`, `battery.pulseRemainingS`. Host test
`tools/host-tests/dcl_pulse_logic_test.cpp`. Not compiled for the ESP32, not tested on hardware.

## Charger current ramp (can_manager.cpp `sendNLG`, 2026-10-30)

Orion raised P0A08 "charge limit exceeded" at a full pack: CCL dropped to 0 (Orion voltage derate timer was
0.1 s) while the NLG still delivered ~10 A. NLG DC current command = min(80 A, CCL), 0 without the charge
flag, now with a ramp: falling follows the limit at once, rising at most `NLG_RAMP_UP_A_PER_S` (5 A/s,
`config.h`), and it restarts from 0 A whenever the demand is not CHARGE or `sendNLG` paused longer than
`NLG_RAMP_RESTART_MS`. No charge-end logic on purpose (balancing needs recharge). A log line
("CCL dropped to 0 +0/1/2s: sent, charger, pack") is written at every CCL drop to 0. Orion side (owner):
SOC derate on while charging, base 200 A, Voltage Derate CCL To Zero Timer 3-5 s, Restore ~5 s.
NLG current scale is 0.1 A with -102.4 A offset (`dcHvCurrentAct` is the raw value; the web API divides
by 100, which is wrong). Not compiled, not tested on hardware: check that the NLG still starts charging
with the 0 A start value.

Charger SLEEP/STANDBY ping-pong (2026-10-30): after the unlock button (`nlgCharged`) and a quick replug the
VCU missed the unplug (500 ms poll), then the STANDBY branch restarted the 4 s / 15 s unlock timeout on every
pass, and `connectorLocked` was only refreshed by the unlock interrupt, so `handlePersistentUnlock` never
reached standby. Fixed in `chargeManage` / `handlePersistentUnlock`; not compiled or tested. The local
statics in `chargeManage` that shadowed `waitingForUnlockComplete` / `connectorUnlockStartTime` are gone
(2026-11-19, see below).

Charger state debounce (2026-11-17): while charging, the demand went CHARGE -> STANDBY, the contactor opened
and precharge ran again, with CAN RX overflows in the log. `chargeManage` acted on every single 0x728 frame
(SLEEP with plug high = demand STANDBY, an unknown state = `armBattery(false)`). `nlgData.stateAct` is now
debounced in `processNLGMessage`: a new state counts after `NLG_STATE_DEBOUNCE_FRAMES` (3, `config.h`)
consecutive frames. `stateActRaw` is the latest frame (live JSON `charging.stateRaw`). Every raw change is
logged with the full frame ("NLG stateAct raw a -> b ..."), so a garbage read and a real charger state can be
told apart. Not compiled or tested yet.

## Long charging sessions, replug, event log (2026-11-19, owner)

Balancing needs the NLG powered for hours; the owner came back to a sleeping VCU (Pi off). Causes and fixes:
- Unplug was one LOW reading of `NLG_HW_Wakeup` (500 ms poll, plus a `delay(500)` in the control task). Now
  `updatePlugState()`: unplugged after 2 s LOW (`NLG_UNPLUG_DEBOUNCE_MS`), shorter drops are logged
  (`PLUG_GLITCH`). The NLG SLEEP case no longer starts the unlock wait on a raw LOW reading.
- Wake sources: `add/removePinFromWakeSources` built the EXT1 mask from `esp_sleep_get_ext1_wakeup_status()`
  (the pin that caused the last wake, not the configured mask), so the plug line was often not a wake
  source after a session. Replaced by `configureWakeSources()` right before deep sleep (IGNITION EXT0;
  NLG wake, IGNITION, UNLCKCON EXT1). No deep sleep while the plug line is HIGH (it would wake at once):
  STANDBY stays awake until unplugged (`WAIT_UNPLUG`).
- Battery armed for the whole CHARGING session (owner: do not re-arm during charging). It used to be
  disarmed on any charger state other than READY2CHARGE/CHARGE (and nothing on SLEEP/STANDBY), which opened
  the contactor, re-ran precharge and stopped the DC-DC. Disarmed only by `transitionToStandby()`.
- Replug without an ignition cycle (owner): unplugging clears `unlockPersist` (set by the unlock button,
  SOC limit, `nlgCharged`). The unlock button now always sets `unlockPersist`, so it ends the session
  until the plug is removed. `waitingForUnlockComplete`, `connectorLockDelay`, `delayingConnectorLock` are
  members, reset in `transitionToCharging()` (they were statics that carried over into the next session).
- `event_log.{h,cpp}`: 64 entries in `RTC_NOINIT` memory (survives deep sleep and software reset, not power
  loss), time = system clock seconds (keeps running in deep sleep). Boot (reset reason, wake cause), state
  changes with a stop reason, deep sleep, plug glitches, precharge done/timeout/failed, battery off, charger
  state (debounced) and demand, NLG timeout, SOC limit, unlock button; pack V and SOC with each.
  `GET /api/log/events` (JSON, `nowS` = current clock), serial command `events`.
- Not compiled, not tested on hardware.

Timestamp race (2026-11-19): code took `now = millis()` first and read a CAN frame time (`lastNLGMs`,
`lastBMSMs`, ...) later; the CAN task (priority 5, same core) can store a newer frame in between, and
`now - last` wrapped to ~4e9 ms = false timeout. Seen as demand CHARGE -> STANDBY while the NLG was in
CHARGE (chargeManage's NLG timeout); FaultHandler::ageMs (BMS/BSC/DMC/NLG timeouts) and `bmsPackFresh`
had the same race, and so did `ErrorMonitor::isSystemHealthy` ("System health check failed" then
"passed" right after). All use `msSince(last, now)` (`config.h`, newer than now = 0). The NLG CHARGE case
now also restores the CHARGE demand (unless unlock/SOC stop), so a one-cycle STANDBY no longer sticks.
Not compiled.

Charger restart (2026-11-20, owner): the NLG ended a charge by itself ~95 s after boot (CHARGE -> SHUTDOWN ->
STANDBY, 411 V, SOC 100 %) and the session stalled for good: the VCU kept the CHARGE demand, but the NLG only
goes STANDBY -> READY2CHARGE while the demand is STANDBY (seen at session start). The old code got out by
accident (disarm on SHUTDOWN = new precharge and sequence). Now the NLG_ACT_STANDBY case sends STANDBY unless
the session was ended on purpose (`nlgCharged`, `unlockPersist`, unlock request); READY2CHARGE then asks for
CHARGE. No hold-off (owner: the Orion now allows charging at 100 %, the CCL decides). Not compiled; if it does
not restart, record a trace.

Unlock button noise (2026-11-20): right after "Closing contactor" the unlock interrupt fired (RISING edge,
no level check), the connector was not locked yet, so `handleConnectorUnlock` ended the session
(BUTTON_UNLOCKED, `unlockPersist`). Now a press counts only if UNLCKCON is still HIGH 30 ms after the edge
(`checkAndHandleInterrupt`, control task); shorter pulses are logged ("short pulse ignored"). Not compiled.

## Diagnostic interface for the scanner app (uds_server.{h,cpp}, isotp.h, 2026-11-24)

UDS on ISO-TP over the vehicle CAN, on request only (same functions as LIVI: read values, read/clear faults,
change parameters). Request 0x7E0 (0x7DF functional, single frames), response 0x7E8, big endian, fixed
scaling. Services: 0x10 (default / extended 0x03, S3 5 s), 0x3E, 0x22 (up to 16 DIDs), 0x2E (extended
session), 0x19 (01, 02, 04 snapshot, 06 counter + severity, 0A), 0x14 (extended session). ISO-TP sends FC with
BS 0 / STmin 2 ms and never sends consecutive frames faster than 2 ms. Table of DIDs, scaling and DTC texts for
the app: `docs/diag/vcu_diag_table.json` (hand-kept, update it with the DID table in `uds_server.cpp`).
  Readable protocol doc: `docs/diag/VCU_DIAG_PROTOCOL.md`, generated from the JSON with `python3 tools/diag/gen_protocol_doc.py`.
- Everything the DMC/BSC/NLG send is reachable: decoded DIDs (0x03xx, 0x04xx, 0x05xx, BMS 0x02xx) plus raw
  DID `0x4000 + CAN ID` = last frame of any ID the VCU received (age, length, 8 bytes), ID list in 0x3F00.
  The raw cache is in `CANManager::IdStat` (`rawFrame()`).
- DTC = `0xE00000 + ErrorCode`, status 0x09 active / 0x08 stored. `UdsServer::syncFaults()` (CAN task, 100 ms)
  compares the ErrorMonitor list with a store of 32 DTCs (count, severity, 13 byte freeze frame at the
  occurrence), kept in NVS namespace `vcu_dtc`; flash is written by `housekeeping()` in the web task (max
  every 5 s), never in the CAN task. 0x14 = `clearAllFaults()` + forget inactive + wipe the store; faults whose
  cause is still there come back at once.
- Config writes use the same setters and limits as LIVI, then `config.save()` ~1 s after the last write (web
  task, status in DID 0x0F10). `ignoreAllowFlags` (0x2050) stays runtime only.
- Runs in the CAN task: `UdsServer::tick()` after `faultHandler->update`, requests arrive through
  `CANManager::setDiagRx` (frames 0x7E0/0x7DF skip the normal decoding). Sending uses `CANManager::sendFrame`
  (CAN task only). Request while a multi-frame answer is still going out is dropped.
- New CAN IDs on the bus (owner confirmed 2026-11-24): TX 0x7E8 (answers only) and 0x6E0 (status frame, 1 Hz: fault flags,
  counts, max severity, state, rolling counter). The scanner dev said 0x700-0x739/0x799 are "the BMS"; in this
  firmware those are the NLG charger frames (0x711, 0x728, 0x739, 0x799), 0x7E0/0x7E8 do not clash.
- No SecurityAccess (0x27): any node on the CAN can write parameters in the extended session. Same trust level
  as the unauthenticated HTTP API; add 0x27 if the bus ever gets other nodes.
- Host test of the transport: `c++ -std=c++17 -I include tools/host-tests/isotp_test.cpp && ./a.out` (passes).
  Builds (2026-11-24, user's `pio run`, no warnings from our files); not flashed or tested on the bus yet.

## Open items

- Station WiFi credentials: API `GET|POST /api/config/wifi` exists (used by the next `service on`).
- LIVI/Pi side of the Ethernet link (static 192.168.5.2 on the Pi, LIVI host 192.168.5.1).
- Orion frame details to confirm with a capture (cell frame, thermistor layout, charge limit byte
  `x2` decode); BSC `modeAct` values unnamed (need the DBC for 0x26A).
- AP IP in `startAP()` is 192.168.4.3 (service mode only).
