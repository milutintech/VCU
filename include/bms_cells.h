#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

/**
 * @brief Where the Orion BMS cell data sits on the bus. All of it is configurable
 *        because the broadcast ID is chosen in the Orion BMS Utility.
 *
 * Cell frame (Orion "Battery Cell Broadcast", 8 bytes):
 *   0 cell index (from 0) | 1-2 instant voltage, 0.1 mV | 3-4 internal resistance
 *   0.01 mOhm, top bit of byte 3 = shunting | 5-6 open voltage, 0.1 mV |
 *   7 checksum = (id + 8 + bytes 0..6) & 0xFF
 *
 * Thermistor frame (optional, layout assumed, verify with a capture):
 *   0 sensor index | 1 temperature, int8 degC
 */
struct BmsCellMap {
    uint32_t cellId = 0;      ///< 0 = cell decoding off
    uint32_t tempId = 0;      ///< 0 = thermistor decoding off
    bool bigEndian = true;    ///< byte order of the 16-bit fields (verify with a capture)
    bool checkChecksum = true;
    uint8_t cellCount = 100;
    uint8_t tempCount = 8;
};

class BmsCells {
public:
    static constexpr uint8_t MAX_CELLS = 128;
    static constexpr uint8_t MAX_TEMPS = 16;

    /** Load the mapping from flash. */
    void begin();

    /** Feed every received frame; returns true when it was a cell or thermistor frame. */
    bool handleFrame(uint32_t id, const uint8_t* buf, uint8_t len);

    const BmsCellMap& getMap() const { return map; }
    bool setMap(const BmsCellMap& next);

    /** `{ "cellsMv": [...], "balancing": [idx...], "temps": [...], "ageMs": n }` */
    String cellsJSON() const;
    String mapJSON() const;

    /** Applies the JSON fields present in `obj`; false when a value is out of range. */
    bool applyMapJSON(const JsonObject& obj);

private:
    BmsCellMap map;
    uint16_t cellDeciMv[MAX_CELLS] = {0};   ///< 0.1 mV, 0 = never seen
    bool shunting[MAX_CELLS] = {false};
    int8_t temps[MAX_TEMPS] = {0};
    bool tempSeen[MAX_TEMPS] = {false};
    unsigned long lastFrameMs = 0;

    void save();
};
