/**
 * @file configuration.cpp - ENHANCED with Torque Transition Configuration
 * @brief Implementation of enhanced configuration management with transition timing
 */

#include "configuration.h"
#include "vehicle_parameters.h"

// Static members initialization
const char* Configuration::NAMESPACE = "vcu_config";

// Storage keys - Basic
const char* Configuration::KEY_DRIVE_MODE = "drive_mode";
const char* Configuration::KEY_MAX_TORQUE = "max_torque";
const char* Configuration::KEY_MAX_SOC = "max_soc";
const char* Configuration::KEY_MAX_CHARGING_CURRENT = "max_ac_curr";

// Storage keys - Curtis
const char* Configuration::KEY_BASE_SPEED = "curtis_base_speed";
const char* Configuration::KEY_DELTA_SPEED = "curtis_delta_speed";
const char* Configuration::KEY_NOMINAL_POWER = "curtis_nom_power";
const char* Configuration::KEY_DRIVE_LIMITS = "curtis_drive_limits";
const char* Configuration::KEY_REGEN_LIMITS = "curtis_regen_limits";

// Storage keys - Pedal zones
const char* Configuration::KEY_REGEN_ZONE_END = "pedal_regen_end";
const char* Configuration::KEY_COAST_ZONE_END = "pedal_coast_end";
const char* Configuration::KEY_REGEN_PROGRESSION = "pedal_regen_prog";
const char* Configuration::KEY_ACCEL_PROGRESSION = "pedal_accel_prog";

// NEW: Storage keys - Transition timing
const char* Configuration::KEY_REGEN_ENGAGE_TIME = "trans_regen_engage";
const char* Configuration::KEY_REGEN_RELEASE_TIME = "trans_regen_release";
const char* Configuration::KEY_POWER_ENGAGE_TIME = "trans_power_engage";
const char* Configuration::KEY_POWER_RELEASE_TIME = "trans_power_release";
const char* Configuration::KEY_CROSSOVER_TIME = "trans_crossover";

// Global configuration instance
Configuration config;

/**
 * @brief Initialize configuration system
 */
void Configuration::begin() {
    // Set defaults first
    resetToDefaults();
    
    // Try to load stored settings
    load();
}

/**
 * @brief Reset all settings to default values
 */
void Configuration::resetToDefaults() {
    // Basic defaults
    driveMode = DriveMode::REGEN;
    maxTorque = VehicleParams::Motor::MAX_TRQ;
    maxSOC = 100;
    maxChargingCurrent = VehicleParams::Power::NLG_MAX_AC;
    
    // Curtis defaults (keep for power limiting)
    resetCurtisDefaults();
    
    // Pedal zone defaults
    resetPedalDefaults();
    
    // NEW: Transition timing defaults
    resetTransitionDefaults();
}

/**
 * @brief Reset Curtis power limits to defaults
 */
void Configuration::resetCurtisDefaults() {
    baseSpeed = 1000.0f;  // ✓ Already correct
    deltaSpeed = 500.0f;  // ✓ Already correct
    nominalPower = 85.0f; // ✓ Keep as is
    
    // Drive curve - all 100% (already correct)
    drivePowerLimits[0] = 100.0f;  // ✓ Already correct
    drivePowerLimits[1] = 100.0f;  // ✓ Already correct
    drivePowerLimits[2] = 100.0f;  // ✓ Already correct
    drivePowerLimits[3] = 100.0f;  // ✓ Already correct
    drivePowerLimits[4] = 100.0f;  // ✓ Already correct
    
    // Update regen curve to match HTML
    regenPowerLimits[0] = 20.0f;   // ✓ Already correct
    regenPowerLimits[1] = 30.0f;   // Change from 25.0f to 30.0f
    regenPowerLimits[2] = 30.0f;   // Change from 35.0f to 30.0f
    regenPowerLimits[3] = 40.0f;   // Change from 50.0f to 40.0f
    regenPowerLimits[4] = 50.0f;   // ✓ Already correct
}

/**
 * @brief Reset pedal zone settings to defaults
 */
void Configuration::resetPedalDefaults() {
    regenZoneEnd = VehicleParams::Pedal::DEFAULT_REGEN_ZONE_END;
    coastZoneEnd = VehicleParams::Pedal::DEFAULT_COAST_ZONE_END;
    regenProgression = VehicleParams::Pedal::DEFAULT_REGEN_PROGRESSION;
    accelProgression = VehicleParams::Pedal::DEFAULT_ACCEL_PROGRESSION;
}

/**
 * @brief NEW: Reset torque transition timing to defaults
 */
void Configuration::resetTransitionDefaults() {
    regenEngageTime = 300.0f;   // 300ms - smooth regen engagement (lift-off)
    regenReleaseTime = 150.0f;  // 150ms - quick regen release (back to coast)
    powerEngageTime = 200.0f;   // 200ms - gentle power engagement (acceleration)
    powerReleaseTime = 100.0f;  // 100ms - quick power release (lift throttle)
    crossoverTime = 400.0f;     // 400ms - slow regen<->power crossover for comfort
}

/**
 * @brief Save current settings to flash
 */
bool Configuration::save() {
    bool success = true;
    
    preferences.begin(NAMESPACE, false);
    
    // Save basic parameters
    success &= preferences.putUChar(KEY_DRIVE_MODE, static_cast<uint8_t>(driveMode));
    success &= preferences.putInt(KEY_MAX_TORQUE, maxTorque);
    success &= preferences.putUChar(KEY_MAX_SOC, maxSOC);
    success &= preferences.putUChar(KEY_MAX_CHARGING_CURRENT, maxChargingCurrent);
    
    // Save Curtis parameters
    success &= preferences.putFloat(KEY_BASE_SPEED, baseSpeed);
    success &= preferences.putFloat(KEY_DELTA_SPEED, deltaSpeed);
    success &= preferences.putFloat(KEY_NOMINAL_POWER, nominalPower);
    success &= preferences.putBytes(KEY_DRIVE_LIMITS, drivePowerLimits, sizeof(drivePowerLimits));
    success &= preferences.putBytes(KEY_REGEN_LIMITS, regenPowerLimits, sizeof(regenPowerLimits));
    
    // Save pedal zone parameters
    success &= preferences.putFloat(KEY_REGEN_ZONE_END, regenZoneEnd);
    success &= preferences.putFloat(KEY_COAST_ZONE_END, coastZoneEnd);
    success &= preferences.putFloat(KEY_REGEN_PROGRESSION, regenProgression);
    success &= preferences.putFloat(KEY_ACCEL_PROGRESSION, accelProgression);
    
    // NEW: Save transition timing parameters
    success &= preferences.putFloat(KEY_REGEN_ENGAGE_TIME, regenEngageTime);
    success &= preferences.putFloat(KEY_REGEN_RELEASE_TIME, regenReleaseTime);
    success &= preferences.putFloat(KEY_POWER_ENGAGE_TIME, powerEngageTime);
    success &= preferences.putFloat(KEY_POWER_RELEASE_TIME, powerReleaseTime);
    success &= preferences.putFloat(KEY_CROSSOVER_TIME, crossoverTime);
    
    preferences.end();
    return success;
}

/**
 * @brief Load settings from flash
 */
bool Configuration::load() {
    bool success = true;
    
    preferences.begin(NAMESPACE, true);
    
    // Load basic parameters
    if (preferences.isKey(KEY_DRIVE_MODE)) {
        uint8_t mode = preferences.getUChar(KEY_DRIVE_MODE, static_cast<uint8_t>(driveMode));
        if (mode <= static_cast<uint8_t>(DriveMode::OPD)) {
            driveMode = static_cast<DriveMode>(mode);
        }
    }
    
    if (preferences.isKey(KEY_MAX_TORQUE)) {
        int torque = preferences.getInt(KEY_MAX_TORQUE, maxTorque);
        if (torque >= MIN_TORQUE_LIMIT && torque <= MAX_TORQUE_LIMIT) {
            maxTorque = torque;
        }
    }
    
    if (preferences.isKey(KEY_MAX_SOC)) {
        uint8_t soc = preferences.getUChar(KEY_MAX_SOC, maxSOC);
        if (soc >= MIN_SOC_LIMIT && soc <= MAX_SOC_LIMIT) {
            maxSOC = soc;
        }
    }
    
    if (preferences.isKey(KEY_MAX_CHARGING_CURRENT)) {
        uint8_t current = preferences.getUChar(KEY_MAX_CHARGING_CURRENT, maxChargingCurrent);
        if (current >= MIN_CHARGING_CURRENT && current <= MAX_CHARGING_CURRENT) {
            maxChargingCurrent = current;
        }
    }
    
    // Load Curtis parameters
    if (preferences.isKey(KEY_BASE_SPEED)) {
        float speed = preferences.getFloat(KEY_BASE_SPEED, baseSpeed);
        if (speed >= MIN_BASE_SPEED && speed <= MAX_BASE_SPEED) {
            baseSpeed = speed;
        }
    }
    
    if (preferences.isKey(KEY_DELTA_SPEED)) {
        float speed = preferences.getFloat(KEY_DELTA_SPEED, deltaSpeed);
        if (speed >= MIN_DELTA_SPEED && speed <= MAX_DELTA_SPEED) {
            deltaSpeed = speed;
        }
    }
    
    if (preferences.isKey(KEY_NOMINAL_POWER)) {
        float power = preferences.getFloat(KEY_NOMINAL_POWER, nominalPower);
        if (power >= MIN_NOMINAL_POWER && power <= MAX_NOMINAL_POWER) {
            nominalPower = power;
        }
    }
    
    // Load power limit arrays
    if (preferences.isKey(KEY_DRIVE_LIMITS)) {
        size_t len = preferences.getBytesLength(KEY_DRIVE_LIMITS);
        if (len == sizeof(drivePowerLimits)) {
            preferences.getBytes(KEY_DRIVE_LIMITS, drivePowerLimits, len);
        }
    }
    
    if (preferences.isKey(KEY_REGEN_LIMITS)) {
        size_t len = preferences.getBytesLength(KEY_REGEN_LIMITS);
        if (len == sizeof(regenPowerLimits)) {
            preferences.getBytes(KEY_REGEN_LIMITS, regenPowerLimits, len);
        }
    }
    
    // Load pedal zone parameters
    if (preferences.isKey(KEY_REGEN_ZONE_END)) {
        float value = preferences.getFloat(KEY_REGEN_ZONE_END, regenZoneEnd);
        if (value >= VehicleParams::Pedal::MIN_REGEN_ZONE_END && 
            value <= VehicleParams::Pedal::MAX_REGEN_ZONE_END) {
            regenZoneEnd = value;
        }
    }
    
    if (preferences.isKey(KEY_COAST_ZONE_END)) {
        float value = preferences.getFloat(KEY_COAST_ZONE_END, coastZoneEnd);
        if (value >= VehicleParams::Pedal::MIN_COAST_ZONE_END && 
            value <= VehicleParams::Pedal::MAX_COAST_ZONE_END &&
            value > regenZoneEnd) {  // Must be greater than regen zone end
            coastZoneEnd = value;
        }
    }
    
    if (preferences.isKey(KEY_REGEN_PROGRESSION)) {
        float value = preferences.getFloat(KEY_REGEN_PROGRESSION, regenProgression);
        if (value >= VehicleParams::Pedal::MIN_PROGRESSION && 
            value <= VehicleParams::Pedal::MAX_PROGRESSION) {
            regenProgression = value;
        }
    }
    
    if (preferences.isKey(KEY_ACCEL_PROGRESSION)) {
        float value = preferences.getFloat(KEY_ACCEL_PROGRESSION, accelProgression);
        if (value >= VehicleParams::Pedal::MIN_PROGRESSION && 
            value <= VehicleParams::Pedal::MAX_PROGRESSION) {
            accelProgression = value;
        }
    }
    
    // NEW: Load transition timing parameters
    if (preferences.isKey(KEY_REGEN_ENGAGE_TIME)) {
        float time = preferences.getFloat(KEY_REGEN_ENGAGE_TIME, regenEngageTime);
        if (time >= MIN_TRANSITION_TIME && time <= MAX_TRANSITION_TIME) {
            regenEngageTime = time;
        }
    }
    
    if (preferences.isKey(KEY_REGEN_RELEASE_TIME)) {
        float time = preferences.getFloat(KEY_REGEN_RELEASE_TIME, regenReleaseTime);
        if (time >= MIN_TRANSITION_TIME && time <= MAX_TRANSITION_TIME) {
            regenReleaseTime = time;
        }
    }
    
    if (preferences.isKey(KEY_POWER_ENGAGE_TIME)) {
        float time = preferences.getFloat(KEY_POWER_ENGAGE_TIME, powerEngageTime);
        if (time >= MIN_TRANSITION_TIME && time <= MAX_TRANSITION_TIME) {
            powerEngageTime = time;
        }
    }
    
    if (preferences.isKey(KEY_POWER_RELEASE_TIME)) {
        float time = preferences.getFloat(KEY_POWER_RELEASE_TIME, powerReleaseTime);
        if (time >= MIN_TRANSITION_TIME && time <= MAX_TRANSITION_TIME) {
            powerReleaseTime = time;
        }
    }
    
    if (preferences.isKey(KEY_CROSSOVER_TIME)) {
        float time = preferences.getFloat(KEY_CROSSOVER_TIME, crossoverTime);
        if (time >= MIN_TRANSITION_TIME && time <= MAX_TRANSITION_TIME) {
            crossoverTime = time;
        }
    }
    
    preferences.end();
    return success;
}

// === TRANSITION TIMING SETTERS (NEW) ===
bool Configuration::setRegenEngageTime(float timeMs) {
    if (timeMs >= MIN_TRANSITION_TIME && timeMs <= MAX_TRANSITION_TIME) {
        regenEngageTime = timeMs;
        return true;
    }
    return false;
}

bool Configuration::setRegenReleaseTime(float timeMs) {
    if (timeMs >= MIN_TRANSITION_TIME && timeMs <= MAX_TRANSITION_TIME) {
        regenReleaseTime = timeMs;
        return true;
    }
    return false;
}

bool Configuration::setPowerEngageTime(float timeMs) {
    if (timeMs >= MIN_TRANSITION_TIME && timeMs <= MAX_TRANSITION_TIME) {
        powerEngageTime = timeMs;
        return true;
    }
    return false;
}

bool Configuration::setPowerReleaseTime(float timeMs) {
    if (timeMs >= MIN_TRANSITION_TIME && timeMs <= MAX_TRANSITION_TIME) {
        powerReleaseTime = timeMs;
        return true;
    }
    return false;
}

bool Configuration::setCrossoverTime(float timeMs) {
    if (timeMs >= MIN_TRANSITION_TIME && timeMs <= MAX_TRANSITION_TIME) {
        crossoverTime = timeMs;
        return true;
    }
    return false;
}

// === PEDAL ZONE SETTERS (UNCHANGED) ===
bool Configuration::setRegenZoneEnd(float value) {
    if (value >= VehicleParams::Pedal::MIN_REGEN_ZONE_END && 
        value <= VehicleParams::Pedal::MAX_REGEN_ZONE_END &&
        value < coastZoneEnd) {  // Must be less than coast zone end
        regenZoneEnd = value;
        return true;
    }
    return false;
}

bool Configuration::setCoastZoneEnd(float value) {
    if (value >= VehicleParams::Pedal::MIN_COAST_ZONE_END && 
        value <= VehicleParams::Pedal::MAX_COAST_ZONE_END &&
        value > regenZoneEnd) {  // Must be greater than regen zone end
        coastZoneEnd = value;
        return true;
    }
    return false;
}

bool Configuration::setRegenProgression(float value) {
    if (value >= VehicleParams::Pedal::MIN_PROGRESSION && 
        value <= VehicleParams::Pedal::MAX_PROGRESSION) {
        regenProgression = value;
        return true;
    }
    return false;
}

bool Configuration::setAccelProgression(float value) {
    if (value >= VehicleParams::Pedal::MIN_PROGRESSION && 
        value <= VehicleParams::Pedal::MAX_PROGRESSION) {
        accelProgression = value;
        return true;
    }
    return false;
}

// === EXISTING SETTERS (UNCHANGED) ===
bool Configuration::setDriveModeFromByte(uint8_t modeByte) {
    if (modeByte <= static_cast<uint8_t>(DriveMode::OPD)) {
        driveMode = static_cast<DriveMode>(modeByte);
        return true;
    }
    return false;
}

bool Configuration::setDriveMode(DriveMode mode) {
    driveMode = mode;
    return true;
}

bool Configuration::setDriveMode(const String& modeStr) {
    String mode = modeStr;
    mode.toLowerCase();
    
    if (mode == "legacy") {
        driveMode = DriveMode::LEGACY;
        return true;
    } else if (mode == "regen") {
        driveMode = DriveMode::REGEN;
        return true;
    } else if (mode == "opd") {
        driveMode = DriveMode::OPD;
        return true;
    }
    
    return false;
}

bool Configuration::setMaxTorque(int torque) {
    if (torque >= MIN_TORQUE_LIMIT && torque <= MAX_TORQUE_LIMIT) {
        maxTorque = torque;
        return true;
    }
    return false;
}

bool Configuration::setMaxSOC(uint8_t soc) {
    if (soc >= MIN_SOC_LIMIT && soc <= MAX_SOC_LIMIT) {
        maxSOC = soc;
        return true;
    }
    return false;
}

bool Configuration::setMaxChargingCurrent(uint8_t current) {
    if (current >= MIN_CHARGING_CURRENT && current <= MAX_CHARGING_CURRENT) {
        maxChargingCurrent = current;
        return true;
    }
    return false;
}

String Configuration::getDriveModeString() const {
    switch(driveMode) {
        case DriveMode::LEGACY:
            return "LEGACY";
        case DriveMode::REGEN:
            return "REGEN";
        case DriveMode::OPD:
            return "OPD";
        default:
            return "UNKNOWN";
    }
}

// === CURTIS SETTERS (UNCHANGED) ===
bool Configuration::setBaseSpeed(float speed) {
    if (speed >= MIN_BASE_SPEED && speed <= MAX_BASE_SPEED) {
        baseSpeed = speed;
        return true;
    }
    return false;
}

bool Configuration::setDeltaSpeed(float speed) {
    if (speed >= MIN_DELTA_SPEED && speed <= MAX_DELTA_SPEED) {
        deltaSpeed = speed;
        return true;
    }
    return false;
}

bool Configuration::setNominalPower(float power) {
    if (power >= MIN_NOMINAL_POWER && power <= MAX_NOMINAL_POWER) {
        nominalPower = power;
        return true;
    }
    return false;
}

bool Configuration::setDrivePowerLimit(int zone, float power) {
    if (zone >= 0 && zone < 5 && power >= MIN_POWER_LIMIT && power <= MAX_DRIVE_POWER_LIMIT) {
        drivePowerLimits[zone] = power;
        return true;
    }
    return false;
}

bool Configuration::setRegenPowerLimit(int zone, float power) {
    if (zone >= 0 && zone < 5 && power >= MIN_POWER_LIMIT && power <= MAX_REGEN_POWER_LIMIT) {
        regenPowerLimits[zone] = power;
        return true;
    }
    return false;
}

// === JSON INTERFACE (UPDATED) ===
String Configuration::toJSON() {
    JsonDocument doc;
    
    // Basic config
    JsonObject driving = doc["driving"].to<JsonObject>();
    driving["mode"] = getDriveModeString();
    driving["maxTorque"] = maxTorque;
    
    JsonObject battery = doc["battery"].to<JsonObject>();
    battery["maxSOC"] = maxSOC;
    battery["maxChargingCurrentAC"] = maxChargingCurrent;
    
    // Curtis config
    JsonObject curtis = doc["curtis"].to<JsonObject>();
    curtis["baseSpeed"] = baseSpeed;
    curtis["deltaSpeed"] = deltaSpeed;
    curtis["nominalPower"] = nominalPower;
    
    JsonArray driveArray = curtis["drivePowerLimits"].to<JsonArray>();
    JsonArray regenArray = curtis["regenPowerLimits"].to<JsonArray>();
    for (int i = 0; i < 5; i++) {
        driveArray.add(drivePowerLimits[i]);
        regenArray.add(regenPowerLimits[i]);
    }
    
    // Pedal config
    JsonObject pedal = doc["pedal"].to<JsonObject>();
    pedal["regenZoneEnd"] = regenZoneEnd;
    pedal["coastZoneEnd"] = coastZoneEnd;
    pedal["regenProgression"] = regenProgression;
    pedal["accelProgression"] = accelProgression;
    
    // NEW: Transition config
    JsonObject transitions = doc["transitions"].to<JsonObject>();
    transitions["regenEngageTime"] = regenEngageTime;
    transitions["regenReleaseTime"] = regenReleaseTime;
    transitions["powerEngageTime"] = powerEngageTime;
    transitions["powerReleaseTime"] = powerReleaseTime;
    transitions["crossoverTime"] = crossoverTime;
    
    String result;
    serializeJson(doc, result);
    return result;
}

bool Configuration::fromJSON(const String& json) {
    JsonDocument doc;  // Increase from default
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        return false;
    }
    
    // Parse and apply configuration
    if (doc["driving"].is<JsonObject>()) {
        parseDrivingJSON(doc["driving"]);
    }
    
    if (doc["curtis"].is<JsonObject>()) {
        parseCurtisJSON(doc["curtis"]);
    }
    
    if (doc["pedal"].is<JsonObject>()) {
        parsePedalJSON(doc["pedal"]);
    }
    
    // NEW: Parse transition configuration
    if (doc["transitions"].is<JsonObject>()) {
        parseTransitionJSON(doc["transitions"]);
    }
    
    return true;
}

String Configuration::getCategoryJSON(const String& category) {
    if (category == "driving") {
        JsonDocument doc = createDrivingJSON();
        String result;
        serializeJson(doc, result);
        return result;
    } else if (category == "curtis") {
        JsonDocument doc = createCurtisJSON();
        String result;
        serializeJson(doc, result);
        return result;
    } else if (category == "pedal") {
        JsonDocument doc = createPedalJSON();
        String result;
        serializeJson(doc, result);
        return result;
    } else if (category == "transitions") {  // NEW
        JsonDocument doc = createTransitionJSON();
        String result;
        serializeJson(doc, result);
        return result;
    }
    
    return "{}";
}

bool Configuration::setCategoryJSON(const String& category, const String& json) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        return false;
    }
    
    if (category == "driving") {
        return parseDrivingJSON(doc.as<JsonObject>());
    } else if (category == "curtis") {
        return parseCurtisJSON(doc.as<JsonObject>());
    } else if (category == "pedal") {
        return parsePedalJSON(doc.as<JsonObject>());
    } else if (category == "transitions") {  // NEW
        return parseTransitionJSON(doc.as<JsonObject>());
    }
    
    return false;
}

// === HELPER METHODS ===
JsonDocument Configuration::createDrivingJSON() {
    JsonDocument doc;
    doc["mode"] = getDriveModeString();
    doc["maxTorque"] = maxTorque;
    return doc;
}

JsonDocument Configuration::createCurtisJSON() {
    JsonDocument doc;
    doc["baseSpeed"] = baseSpeed;
    doc["deltaSpeed"] = deltaSpeed;
    doc["nominalPower"] = nominalPower;
    
    JsonArray driveArray = doc["drivePowerLimits"].to<JsonArray>();
    JsonArray regenArray = doc["regenPowerLimits"].to<JsonArray>();
    for (int i = 0; i < 5; i++) {
        driveArray.add(drivePowerLimits[i]);
        regenArray.add(regenPowerLimits[i]);
    }
    
    return doc;
}

JsonDocument Configuration::createPedalJSON() {
    JsonDocument doc;
    doc["regenZoneEnd"] = regenZoneEnd;
    doc["coastZoneEnd"] = coastZoneEnd;
    doc["regenProgression"] = regenProgression;
    doc["accelProgression"] = accelProgression;
    return doc;
}

// NEW: Transition JSON helper
JsonDocument Configuration::createTransitionJSON() {
    JsonDocument doc;
    doc["regenEngageTime"] = regenEngageTime;
    doc["regenReleaseTime"] = regenReleaseTime;
    doc["powerEngageTime"] = powerEngageTime;
    doc["powerReleaseTime"] = powerReleaseTime;
    doc["crossoverTime"] = crossoverTime;
    return doc;
}

bool Configuration::parseDrivingJSON(const JsonObject& obj) {
    if (obj["mode"].is<const char*>()) {
        setDriveMode(String(obj["mode"].as<const char*>()));
    }
    
    if (obj["maxTorque"].is<int>()) {
        setMaxTorque(obj["maxTorque"]);
    }
    
    return true;
}

bool Configuration::parseCurtisJSON(const JsonObject& obj) {
    if (obj["baseSpeed"].is<float>()) {
        setBaseSpeed(obj["baseSpeed"]);
    }
    
    if (obj["deltaSpeed"].is<float>()) {
        setDeltaSpeed(obj["deltaSpeed"]);
    }
    
    if (obj["nominalPower"].is<float>()) {
        setNominalPower(obj["nominalPower"]);
    }
    
    if (obj["drivePowerLimits"].is<JsonArray>()) {
        JsonArray arr = obj["drivePowerLimits"];
        for (int i = 0; i < 5 && i < arr.size(); i++) {
            if (arr[i].is<float>()) {
                setDrivePowerLimit(i, arr[i]);
            }
        }
    }
    
    if (obj["regenPowerLimits"].is<JsonArray>()) {
        JsonArray arr = obj["regenPowerLimits"];
        for (int i = 0; i < 5 && i < arr.size(); i++) {
            if (arr[i].is<float>()) {
                setRegenPowerLimit(i, arr[i]);
            }
        }
    }
    
    return true;
}

bool Configuration::parsePedalJSON(const JsonObject& obj) {
    if (obj["regenZoneEnd"].is<float>()) {
        setRegenZoneEnd(obj["regenZoneEnd"]);
    }
    
    if (obj["coastZoneEnd"].is<float>()) {
        setCoastZoneEnd(obj["coastZoneEnd"]);
    }
    
    if (obj["regenProgression"].is<float>()) {
        setRegenProgression(obj["regenProgression"]);
    }
    
    if (obj["accelProgression"].is<float>()) {
        setAccelProgression(obj["accelProgression"]);
    }
    
    return true;
}

// NEW: Parse transition JSON
bool Configuration::parseTransitionJSON(const JsonObject& obj) {
    if (obj["regenEngageTime"].is<float>()) {
        setRegenEngageTime(obj["regenEngageTime"]);
    }
    
    if (obj["regenReleaseTime"].is<float>()) {
        setRegenReleaseTime(obj["regenReleaseTime"]);
    }
    
    if (obj["powerEngageTime"].is<float>()) {
        setPowerEngageTime(obj["powerEngageTime"]);
    }
    
    if (obj["powerReleaseTime"].is<float>()) {
        setPowerReleaseTime(obj["powerReleaseTime"]);
    }
    
    if (obj["crossoverTime"].is<float>()) {
        setCrossoverTime(obj["crossoverTime"]);
    }
    
    return true;
}