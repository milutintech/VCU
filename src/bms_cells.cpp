#include "bms_cells.h"

static const char* NS = "vcu_bms";  // own namespace: Configuration::begin() may clear its own

void BmsCells::begin() {
    Preferences p;
    if (!p.begin(NS, true)) return;
    BmsCellMap m;
    m.cellId = p.getUInt("cell_id", m.cellId);
    m.tempId = p.getUInt("temp_id", m.tempId);
    m.bigEndian = p.getBool("big_end", m.bigEndian);
    m.checkChecksum = p.getBool("chk", m.checkChecksum);
    m.cellCount = p.getUChar("cells", m.cellCount);
    m.tempCount = p.getUChar("temps", m.tempCount);
    p.end();
    if (m.cellCount >= 1 && m.cellCount <= MAX_CELLS && m.tempCount <= MAX_TEMPS) map = m;
}

void BmsCells::save() {
    Preferences p;
    if (!p.begin(NS, false)) return;
    p.putUInt("cell_id", map.cellId);
    p.putUInt("temp_id", map.tempId);
    p.putBool("big_end", map.bigEndian);
    p.putBool("chk", map.checkChecksum);
    p.putUChar("cells", map.cellCount);
    p.putUChar("temps", map.tempCount);
    p.end();
}

bool BmsCells::setMap(const BmsCellMap& next) {
    if (next.cellCount < 1 || next.cellCount > MAX_CELLS) return false;
    if (next.tempCount > MAX_TEMPS) return false;
    if (next.cellId > 0x1FFFFFFF || next.tempId > 0x1FFFFFFF) return false;
    map = next;
    // Stale readings from the old mapping must not linger.
    memset(cellDeciMv, 0, sizeof(cellDeciMv));
    memset(shunting, 0, sizeof(shunting));
    memset(tempSeen, 0, sizeof(tempSeen));
    save();
    return true;
}

bool BmsCells::handleFrame(uint32_t id, const uint8_t* buf, uint8_t len) {
    if (!buf) return false;

    if (map.cellId != 0 && id == map.cellId) {
        if (len < 8) return true;
        if (map.checkChecksum) {
            uint16_t sum = id + 8;
            for (int i = 0; i < 7; i++) sum += buf[i];
            if ((sum & 0xFF) != buf[7]) return true;  // corrupt, drop
        }
        uint8_t idx = buf[0];
        if (idx >= map.cellCount) return true;
        uint16_t v = map.bigEndian ? ((buf[1] << 8) | buf[2]) : ((buf[2] << 8) | buf[1]);
        cellDeciMv[idx] = v;
        shunting[idx] = (buf[3] & 0x80) != 0;
        lastFrameMs = millis();
        return true;
    }

    if (map.tempId != 0 && id == map.tempId) {
        if (len < 2) return true;
        uint8_t idx = buf[0];
        if (idx >= map.tempCount) return true;
        temps[idx] = (int8_t)buf[1];
        tempSeen[idx] = true;
        lastFrameMs = millis();
        return true;
    }

    return false;
}

String BmsCells::cellsJSON() const {
    JsonDocument doc;
    JsonArray mv = doc["cellsMv"].to<JsonArray>();
    JsonArray bal = doc["balancing"].to<JsonArray>();
    JsonArray t = doc["temps"].to<JsonArray>();

    // Nothing decoded yet: an empty list tells LIVI there is no data, not a pack of 0 V cells.
    bool any = false;
    for (uint8_t i = 0; i < map.cellCount; i++) {
        if (cellDeciMv[i] != 0) { any = true; break; }
    }
    if (any) {
        for (uint8_t i = 0; i < map.cellCount; i++) {
            mv.add((cellDeciMv[i] + 5) / 10);
            if (shunting[i]) bal.add(i);
        }
    }
    for (uint8_t i = 0; i < map.tempCount; i++) {
        if (tempSeen[i]) t.add(temps[i]);
    }
    doc["ageMs"] = lastFrameMs ? (long)(millis() - lastFrameMs) : -1L;

    String out;
    serializeJson(doc, out);
    return out;
}

String BmsCells::mapJSON() const {
    JsonDocument doc;
    doc["cellId"] = map.cellId;
    doc["tempId"] = map.tempId;
    doc["bigEndian"] = map.bigEndian;
    doc["checkChecksum"] = map.checkChecksum;
    doc["cellCount"] = map.cellCount;
    doc["tempCount"] = map.tempCount;
    String out;
    serializeJson(doc, out);
    return out;
}

bool BmsCells::applyMapJSON(const JsonObject& obj) {
    BmsCellMap next = map;
    if (!obj["cellId"].isNull()) next.cellId = obj["cellId"].as<uint32_t>();
    if (!obj["tempId"].isNull()) next.tempId = obj["tempId"].as<uint32_t>();
    if (!obj["bigEndian"].isNull()) next.bigEndian = obj["bigEndian"].as<bool>();
    if (!obj["checkChecksum"].isNull()) next.checkChecksum = obj["checkChecksum"].as<bool>();
    if (!obj["cellCount"].isNull()) next.cellCount = obj["cellCount"].as<uint8_t>();
    if (!obj["tempCount"].isNull()) next.tempCount = obj["tempCount"].as<uint8_t>();
    return setMap(next);
}
