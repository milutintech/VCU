# VCU Diagnose-Protokoll (UDS über ISO-TP)

Generiert aus `docs/diag/vcu_diag_table.json` (`python3 tools/diag/gen_protocol_doc.py`). Quelle im Code:
`include/uds_server.h`, `src/uds_server.cpp`.

## Transport

| | |
|---|---|
| Bus | CAN 500 kbit/s, 11 bit |
| Anfrage | 0x7E0 (physical) / 0x7DF (functional, single frames only) |
| Antwort | 0x7E8 |
| Padding | 0xAA, all frames 8 bytes |
| Flusssteuerung VCU empfängt | BS 0, STmin 2 ms |
| Flusssteuerung VCU sendet | BS and STmin of the tester, but never below 2 ms between consecutive frames |
| Timeout | 1000 ms |
| Max. Nachricht | 512 Byte |
| Byte-Reihenfolge | big endian |

Die VCU sendet nur auf Anfrage, ausser dem Status-Frame (1 Hz, siehe unten). Kommt eine Anfrage, während noch
eine mehrteilige Antwort läuft, wird sie verworfen: erst die Antwort abwarten.

## Services

| SID | Funktion |
|---|---|
| `0x10` | session: 01 default, 03 extended; response carries P2 50 ms / P2* 5000 ms |
| `0x3E` | tester present (00, or 80 = no response) |
| `0x22` | read 1..16 DIDs per request, response = 62 { DID data }... |
| `0x2E` | write one DID (extended session only); saved to flash ~1 s after the last write (DID 0x0F10 = status) |
| `0x19` | 01 count by mask, 02 list by mask, 04 snapshot (DTC + record 01 or FF), 06 extended data (record 01 occurrences, 02 severity, FF both), 0A all supported DTCs |
| `0x14` | clear: 14 FF FF FF = all, 14 E0 xx xx = one DTC (extended session only). Faults whose cause is still present come back within a cycle. |

**Session:** Extended session ends after 5 s without any request (S3): send 0x3E 00 about once per second.

Negative Antworten (`7F <SID> <NRC>`):

| NRC | Bedeutung |
|---|---|
| `0x11` | service not supported |
| `0x12` | sub-function not supported |
| `0x13` | wrong length |
| `0x14` | response too long |
| `0x31` | unknown DID / DTC / value outside the allowed range |
| `0x7F` | needs the extended session |

Bei funktionaler Adressierung (0x7DF) antwortet die VCU nicht mit 0x11 / 0x12 / 0x31 / 0x7F.

## Beispiele

Alle Bytes hex, vor ISO-TP (die VCU packt sie in Single-/First-/Consecutive-Frames).

| Anfrage | Antwort | Bedeutung |
|---|---|---|
| `10 03` | `50 03 00 32 01 F4` | Extended Session, P2 50 ms, P2* 5000 ms |
| `3E 00` | `7E 00` | Tester present |
| `22 F1 86` | `62 F1 86 01` | aktive Session (1 = default) |
| `22 02 00` | `62 02 00 <9 Byte>` | BMS: SOC, Spannung, Strom, DCL, CCL |
| `22 02 00 03 00 04 00` | `62 02 00 .. 03 00 .. 04 00 ..` | mehrere DIDs in einer Anfrage |
| `22 40 28` | `62 40 28 <age u16> <len> <8 Byte>` | letzter Rohframe der CAN-ID 0x028 |
| `19 01 09` | `59 01 09 01 00 02` | 2 DTCs mit Statusmaske 0x09 |
| `19 02 09` | `59 02 09 E0 00 CA 09 ...` | DTC-Liste (3 Byte DTC + 1 Byte Status) |
| `19 04 E0 00 CA 01` | `59 04 E0 00 CA 09 01 01 0F 00 <13 Byte>` | Snapshot zu BMS_TIMEOUT |
| `19 06 E0 00 CA FF` | `59 06 E0 00 CA 09 01 <n> 02 <sev>` | Zähler und Schwere |
| `19 0A` | `59 0A 09 { E0 hi lo status }...` | alle 36 DTCs, Status 0 = nie aufgetreten |
| `10 03`, dann `14 FF FF FF` | `54` | alle Fehler löschen |
| `10 03`, dann `2E 20 02 02 58` | `6E 20 02` | maxTorque = 600 Nm (Speichern nach ca. 1 s) |

## Live-Werte (nur lesen)

### `0xF186` Active session
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | session | u8 | 1 |  | 1 default, 3 extended |

### `0xF189` Software version
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | text | ascii | 1 |  | 'VCU <build date> <build time>', variable length |

### `0x0100` VCU state
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | state | u8 | 1 |  | 0 STANDBY, 1 RUN, 2 CHARGING |
| 1 | gear | u8 | 1 |  | 0 N, 1 D, 2 R |
| 2 | driveMode | u8 | 1 |  | 0 LEGACY, 1 REGEN, 2 OPD |
| 3 | flags | u8 | 1 |  | bit0 batteryArmed, bit1 preCharged, bit2 BSC enabled, bit3 BSC boost, bit4 BMS allow flags ignored |
| 4 | uptime | u32 | 1 | s |  |

### `0x0101` Limits sent
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | dmcDriveLimit | u16 | 1 | A | discharge limit sent to the DMC (incl. DCL pulse) |
| 1 | pulseRemaining | u16 | 1 | s |  |
| 2 | bscHvTarget | u16 | 1 | V |  |

### `0x0102` CAN receive statistics
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | rxFrames | u32 | 1 |  |  |
| 1 | rxOverflows | u32 | 1 |  | MCP2515 receive overflow passes (frames lost) |
| 2 | queueDrops | u16 | 1 |  | saturates |
| 3 | otherIdFrames | u16 | 1 |  | frames of IDs beyond the 48 entry table |

### `0x0103` Frame age per node
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | bmsPack | u16 | 1 | ms | 0xFFFF = never or older than 65 s |
| 1 | bmsFlags | u16 | 1 | ms |  |
| 2 | dmc | u16 | 1 | ms |  |
| 3 | bsc | u16 | 1 | ms |  |
| 4 | nlg | u16 | 1 | ms |  |

### `0x0104` CAN timing maxima (last 10 s window)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | intLatency | u16 | 1 | us |  |
| 1 | spiWait | u16 | 1 | us |  |
| 2 | txWait | u16 | 1 | us |  |
| 3 | loop | u16 | 1 | us |  |
| 4 | txFails | u16 | 1 |  |  |

### `0x0200` BMS pack (0x010)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | soc | u8 | 1 | % |  |
| 1 | packVoltage | u16 | 1 | V | whole volts (the VCU stores V, not 0.1 V) |
| 2 | packCurrent | i16 | x 0.1 | A | negative = charging |
| 3 | dcl | u16 | 1 | A | discharge current limit |
| 4 | ccl | u16 | 1 | A | charge current limit |

### `0x0201` BMS min/max (0x001)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | tempHigh | i8 | 1 | degC |  |
| 1 | tempLow | i8 | 1 | degC |  |
| 2 | cellLow | u16 | x 0.1 | mV |  |
| 3 | cellHigh | u16 | x 0.1 | mV |  |
| 4 | flags | u8 | 1 |  | bit0 charge relay, bit1 discharge relay (raw flags); bit2 charge allowed, bit3 discharge allowed (as the VCU uses them, incl. override); bit4 0x010 fresh, bit5 0x001 fresh |

### `0x0300` DMC status (0x258)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | flags | u8 | 1 |  | bit0 ready, bit1 running |
| 1 | torqueAvailable | i16 | x 0.1 | Nm |  |
| 2 | torqueActual | i16 | x 0.1 | Nm |  |
| 3 | speed | i16 | 1 | rpm |  |

### `0x0301` DMC power (0x259)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | dcVoltage | u16 | x 0.1 | V |  |
| 1 | dcCurrent | i16 | x 0.1 | A |  |
| 2 | acCurrent | u16 | x 0.1 | A |  |
| 3 | mechPower | i32 | 1 | W |  |

### `0x0302` DMC temperatures (0x458)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | inverter | i16 | x 0.1 | degC |  |
| 1 | motor | i16 | x 0.1 | degC |  |
| 2 | system | i8 | 1 | degC |  |

### `0x0400` BSC DC-DC (0x26A)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | hvVoltage | u16 | x 0.1 | V |  |
| 1 | lvVoltage | u16 | x 0.1 | V |  |
| 2 | hvCurrent | i16 | x 0.1 | A |  |
| 3 | lvCurrent | i16 | 1 | A |  |
| 4 | mode | u8 | 1 |  | 0 buck, 1 boost (as decoded) |
| 5 | flags | u8 | 1 |  | bit0 enabled (VCU command), bit1 boost (VCU command) |

### `0x0500` NLG charger (0x728)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | state | u8 | 1 |  | debounced: 0 SLEEP, 1 WAKEUP, 2 STANDBY, 3 READY2CHARGE, 4 CHARGE, 5 SHUTDOWN |
| 1 | stateRaw | u8 | 1 |  | latest frame, not debounced |
| 2 | controlPilot | u8 | 1 |  |  |
| 3 | flags | u8 | 1 |  | bit0 connector locked, bit1 unlock request |
| 4 | dcHvVoltage | u16 | 1 |  | RAW value of the 13 bit field, scale to be confirmed |
| 5 | dcHvCurrent | u16 | x 0.1 -102.4 | A | RAW 11 bit field; physical = raw*0.1 - 102.4 |
| 6 | demandState | u8 | 1 |  | 0 STANDBY, 1 CHARGE, 6 SLEEP |
| 7 | ledDemand | u8 | 1 |  |  |

### `0x0501` NLG cooling (0x739)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | coolingRequest | u8 | 1 |  |  |
| 1 | coolPlate | i16 | x 0.1 | degC |  |

### `0x0800` Fault summary
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | active | u8 | 1 |  | active DTCs |
| 1 | stored | u8 | 1 |  | stored DTCs |
| 2 | maxActiveSeverity | u8 | 1 |  | 0 info, 1 warning, 2 error, 3 critical, 0xFF none |

### `0x0F10` Config save status
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | status | u8 | 1 |  | 0 nothing written, 1 pending (saved ~1 s after the last write), 2 saved, 3 failed, 4 saving |

### `0x3F00` CAN IDs seen on the bus
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | count | u8 | 1 |  |  |
| 1 | entries | array | 1 |  | count x { id u16, frames u32 } since boot |

## Parameter (lesen + schreiben, Extended Session)

### `0x2001` Drive mode  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | driveMode | u8 | 1 |  | 0 LEGACY, 1 REGEN, 2 OPD |

### `0x2002` Max torque forward  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | maxTorque | u16 | 1 | Nm | 100..850 |

### `0x2003` Max torque reverse  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | maxReverseTorque | u16 | 1 | Nm | 100..850 |

### `0x2004` Max SOC  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | maxSoc | u8 | 1 | % | 50..100 |

### `0x2005` Max charging current (AC)  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | maxChargingCurrent | u8 | 1 | A | 6..32 |

### `0x2010` Regen zone end  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | regenZoneEnd | u16 | x 0.1 | % | 10..50, must stay below coast zone end |

### `0x2011` Coast zone end  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | coastZoneEnd | u16 | x 0.1 | % | 15..60, above regen zone end |

### `0x2012` Regen progression  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | regenProgression | u16 | x 0.01 |  | 1.00..3.00 |

### `0x2013` Accel progression  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | accelProgression | u16 | x 0.01 |  | 1.00..3.00 |

### `0x2020` Regen engage time  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | time | u16 | 1 | ms | 0..1000 |

### `0x2021` Regen release time  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | time | u16 | 1 | ms | 0..1000 |

### `0x2022` Power engage time  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | time | u16 | 1 | ms | 0..1000 |

### `0x2023` Power release time  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | time | u16 | 1 | ms | 0..1000 |

### `0x2024` Crossover time  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | time | u16 | 1 | ms | 0..1000 |

### `0x2030` Throttle min ADC  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | adc | u16 | 1 |  | 0..32767, below max |

### `0x2031` Throttle max ADC  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | adc | u16 | 1 |  | 0..32767, above min |

### `0x2040` Base speed  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | baseSpeed | u16 | 1 | rpm | 500..5000 |

### `0x2041` Delta speed  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | deltaSpeed | u16 | 1 | rpm | 100..2000 |

### `0x2042` Nominal power  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | nominalPower | u16 | x 0.1 | % | 50..100 |

### `0x2043` Drive power limits (5 zones)  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | zone0..4 | u16[5] | x 0.1 | % | 10..120 each; all or nothing |

### `0x2044` Regen power limits (5 zones)  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | zone0..4 | u16[5] | x 0.1 | % | 10..100 each; all or nothing |

### `0x2050` BMS allow flags ignored  (lesen + schreiben)
| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | ignore | u8 | 1 |  | 0/1. Runtime only, never saved, reset by reboot. Safety critical. |

## Rohframes

### `0x4000 + CAN ID (0x4000..0x47FF)` Last raw frame of a CAN ID
Any ID the VCU has received, also those it does not decode (all of DMC / BSC / NLG). NRC 0x31 if the ID was never seen. Use 0x3F00 for the list.

| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | age | u16 | 1 | ms | 0xFFFF = 65 s or older |
| 1 | length | u8 | 1 |  |  |
| 2 | data | u8[8] | 1 |  | as on the bus, unused bytes 0 |

## Fehler (DTC)

Format: E0 hi lo. Status: 0x09 = active (testFailed + confirmed), 0x08 = stored, not active; availability mask 0x09.
DTC = 0xE00000 + VCU error code. A new occurrence is counted when a fault becomes active; the store (max 32) is kept in flash. Snapshot DID 0x0F00 (13 bytes) is taken at the occurrence.

### Snapshot (Record 01, DID `0x0F00`)

| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | packVoltage | u16 | 1 | V |  |
| 1 | packCurrent | i16 | x 0.1 | A |  |
| 2 | soc | u8 | 1 | % |  |
| 3 | speed | i16 | 1 | rpm |  |
| 4 | inverterTemp | i8 | 1 | degC |  |
| 5 | motorTemp | i8 | 1 | degC |  |
| 6 | lvVoltage | u16 | x 0.1 | V |  |
| 7 | state | u8 | 1 |  | 0 STANDBY, 1 RUN, 2 CHARGING |
| 8 | gear | u8 | 1 |  | 0 N, 1 D, 2 R |

### Fehlercodes

| DTC | Code | Name | Schwere | Beschreibung |
|---|---|---|---|---|
| `0xE00064` | 100 | System boot |  |  |
| `0xE00065` | 101 | System shutdown |  |  |
| `0xE00066` | 102 | Watchdog reset |  |  |
| `0xE00067` | 103 | Memory error |  |  |
| `0xE00068` | 104 | 12V battery low | warning | 12-V-Spannung unter 11.5 V (Warnung, kommt erst ab 11.8 V zurueck) |
| `0xE000C8` | 200 | CAN init failed |  |  |
| `0xE000C9` | 201 | CAN bus off |  |  |
| `0xE000CA` | 202 | BMS CAN timeout | critical | Keine BMS-Frames (0x010) innerhalb 1600 ms. Antrieb gesperrt. |
| `0xE000CB` | 203 | DMC CAN timeout | critical | Keine DMC-Frames innerhalb 500 ms im Fahrbetrieb. Antrieb gesperrt. |
| `0xE000CC` | 204 | BSC CAN timeout | warning | Keine BSC-Frames innerhalb 1500 ms. |
| `0xE000CD` | 205 | NLG CAN timeout | warning | Keine Lader-Frames innerhalb 1500 ms beim Laden. |
| `0xE0012C` | 300 | Battery undervoltage | error | Packspannung unter 300 V. Drehmoment x0.5. |
| `0xE0012D` | 301 | Battery overvoltage | critical | Packspannung ueber 422 V. Drehmoment 0. |
| `0xE0012E` | 302 | Battery overcurrent |  |  |
| `0xE0012F` | 303 | Precharge failed | error | Vorladung 3x gescheitert, keine weiteren Versuche bis Zuendung aus / Ladeende. |
| `0xE00130` | 304 | Precharge timeout |  |  |
| `0xE00131` | 305 | SOC out of range |  |  |
| `0xE00132` | 306 | BMS does not allow discharging | critical | BMS erlaubt kein Entladen (oder Flag-Frame 0x001 fehlt). Antrieb gesperrt. |
| `0xE00133` | 307 | BMS does not allow charging | info | BMS erlaubt kein Laden (am Ende der Ladung normal). |
| `0xE00134` | 308 | BMS allow flags ignored | warning | Override der BMS-Freigaben aktiv (bis Neustart). |
| `0xE00190` | 400 | Motor overtemperature | error (critical above cutoff) | Motor ueber 120 C: Drehmoment x0.5, ueber 125 C: Antrieb aus. |
| `0xE00191` | 401 | Inverter overtemperature | error (critical above cutoff) | Umrichter ueber 120 C: Drehmoment x0.5, ueber 125 C: Antrieb aus. |
| `0xE00192` | 402 | DMC error |  |  |
| `0xE00193` | 403 | Torque limit exceeded |  |  |
| `0xE00194` | 404 | Speed limit exceeded |  |  |
| `0xE001F4` | 500 | Charger error |  |  |
| `0xE001F5` | 501 | Charger running hot | info | Ladegeraet Kuehlplatte ueber 85 C, Ladeleistung reduziert (nur Info). |
| `0xE001F6` | 502 | Connector error |  |  |
| `0xE001F7` | 503 | Charge current error |  |  |
| `0xE00258` | 600 | Configuration invalid |  |  |
| `0xE00259` | 601 | Configuration save failed |  |  |
| `0xE0025A` | 602 | Configuration load failed |  |  |
| `0xE0025B` | 603 | Calibration error |  |  |
| `0xE002BC` | 700 | Gear shift error |  |  |
| `0xE002BD` | 701 | Pedal error |  |  |
| `0xE002BE` | 702 | User abort |  |  |

## Status-Frame (ohne Anfrage)

ID `0x6E0`, 1 Hz, 8 Byte.

| # | Feld | Typ | Skalierung | Einheit | Bemerkung |
|---|---|---|---|---|---|
| 0 | flags | u8 | 1 |  | bit0 any fault active, bit1 critical active, bit2 BMS flags ignored, bit3 extended session, bit4 any DTC stored |
| 1 | activeCount | u8 | 1 |  |  |
| 2 | storedCount | u8 | 1 |  |  |
| 3 | maxActiveSeverity | u8 | 1 |  | 0xFF none |
| 4 | state | u8 | 1 |  |  |
| 5 | reserved | u8 | 1 |  |  |
| 6 | counter | u16 | 1 |  | rolling, to see a stale frame |
