#!/usr/bin/env python3
"""Generates docs/diag/VCU_DIAG_PROTOCOL.md from docs/diag/vcu_diag_table.json.
Run from the repo root: python3 tools/diag/gen_protocol_doc.py"""
import json

d = json.load(open("docs/diag/vcu_diag_table.json"))

def scale(f):
    s = f.get("scale", 1)
    out = "" if s in (1, "") else f"x {s}"
    if "offset" in f:
        out += f" {f['offset']:+}"
    return out or "1"

def table(fields):
    rows = ["| # | Feld | Typ | Skalierung | Einheit | Bemerkung |", "|---|---|---|---|---|---|"]
    for i, f in enumerate(fields):
        rows.append(f"| {i} | {f['name']} | {f['type']} | {scale(f)} | {f.get('unit','')} | {f.get('note','')} |")
    return "\n".join(rows)

def group(title, pred):
    out = [f"## {title}\n"]
    for x in d["dids"]:
        if pred(x["did"]):
            out.append(f"### `{x['did']}` {x['name']}" + ("  (lesen + schreiben)" if x["access"] == "rw" else ""))
            if x.get("note"): out.append(x["note"] + "\n")
            out.append(table(x["fields"]) + "\n")
    return "\n".join(out)

def num(h):
    try: return int(h, 16)
    except ValueError: return -1

t = d["transport"]
md = f"""# VCU Diagnose-Protokoll (UDS über ISO-TP)

Generiert aus `docs/diag/vcu_diag_table.json` (`python3 tools/diag/gen_protocol_doc.py`). Quelle im Code:
`include/uds_server.h`, `src/uds_server.cpp`.

## Transport

| | |
|---|---|
| Bus | {t['bus']} |
| Anfrage | {t['request']} |
| Antwort | {t['response']} |
| Padding | {t['padding']} |
| Flusssteuerung VCU empfängt | {t['flowControlSentByVcu']} |
| Flusssteuerung VCU sendet | {t['flowControlHonoredByVcu']} |
| Timeout | {t['timeoutMs']} ms |
| Max. Nachricht | {t['maxMessage']} Byte |
| Byte-Reihenfolge | {t['byteOrder']} |

Die VCU sendet nur auf Anfrage, ausser dem Status-Frame (1 Hz, siehe unten). Kommt eine Anfrage, während noch
eine mehrteilige Antwort läuft, wird sie verworfen: erst die Antwort abwarten.

## Services

| SID | Funktion |
|---|---|
""" + "\n".join(f"| `{k}` | {v} |" for k, v in d["services"].items()) + f"""

**Session:** {d['session']}

Negative Antworten (`7F <SID> <NRC>`):

| NRC | Bedeutung |
|---|---|
""" + "\n".join(f"| `{k}` | {v} |" for k, v in d["negativeResponses"].items()) + """

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

""" + group("Live-Werte (nur lesen)", lambda h: 0 <= num(h) < 0x2000 or num(h) in (0x3F00, 0xF186, 0xF189)) \
  + "\n" + group("Parameter (lesen + schreiben, Extended Session)", lambda h: 0x2000 <= num(h) < 0x3000) \
  + "\n## Rohframes\n\n" + "\n".join(
      f"### `{x['did']}` {x['name']}\n{x['note']}\n\n" + table(x["fields"]) + "\n"
      for x in d["dids"] if num(x["did"]) < 0 ) + f"""
## Fehler (DTC)

Format: {d['dtcFormat']['bytes']}. Status: {d['dtcFormat']['status']}.
{d['dtcFormat']['note']}

### Snapshot (Record 01, DID `0x0F00`)

""" + table(d["snapshotDid_0x0F00"]) + """

### Fehlercodes

| DTC | Code | Name | Schwere | Beschreibung |
|---|---|---|---|---|
""" + "\n".join(f"| `{x['dtc']}` | {x['errorCode']} | {x['name']} | {x.get('severity','')} | {x.get('text','')} |" for x in d["dtcs"]) + f"""

## Status-Frame (ohne Anfrage)

ID `{d['statusFrame']['id']}`, {d['statusFrame']['rateHz']} Hz, 8 Byte.

""" + table(d["statusFrame"]["bytes"]) + "\n"
open("docs/diag/VCU_DIAG_PROTOCOL.md", "w").write(md)
print(len(md.splitlines()), "lines")
