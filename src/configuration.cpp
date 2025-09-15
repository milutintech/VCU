/**
 * @file configuration.cpp
 * @brief Implementation of runtime configuration management with Curtis Power Limiting
 */

#include "configuration.h"
#include "vehicle_parameters.h"

// Static members initialization
const char* Configuration::NAMESPACE = "vcu_config";

// Existing parameter keys
const char* Configuration::KEY_DRIVE_MODE = "drive_mode";
const char* Configuration::KEY_MAX_TORQUE = "max_torque";
const char* Configuration::KEY_MAX_SOC = "max_soc";
const char* Configuration::KEY_MAX_CHARGING_CURRENT = "max_ac_curr";

// NEW: Curtis parameter keys
const char* Configuration::KEY_BASE_SPEED = "curtis_base_speed";
const char* Configuration::KEY_DELTA_SPEED = "curtis_delta_speed";
const char* Configuration::KEY_NOMINAL_POWER = "curtis_nom_power";
const char* Configuration::KEY_DRIVE_LIMITS = "curtis_drive_limits";
const char* Configuration::KEY_REGEN_LIMITS = "curtis_regen_limits";
const char* Configuration::KEY_BASELINE_UPDATE_RATE = "neutral_update_rate";
const char* Configuration::KEY_BASELINE_DECAY_RATE = "neutral_decay_rate";
const char* Configuration::KEY_REGEN_MULTIPLIER = "regen_multiplier";

// Global configuration instance
Configuration config;

/**
 * @brief Initialize configuration system
 */
void Configuration::begin() {
    // Default values - will be overwritten if valid settings exist in flash
    resetToDefaults();
    
    // Try to load stored settings
    load();
}

/**
 * @brief Reset all settings to default values
 */
void Configuration::resetToDefaults() {
    // Existing defaults
    driveMode = DriveMode::REGEN;
    maxTorque = VehicleParams::Motor::MAX_TRQ;
    maxSOC = 100;
    maxChargingCurrent = VehicleParams::Power::NLG_MAX_AC;
    
    // Curtis defaults
    resetCurtisDefaults();
    
    // Save defaults to flash
    save();
}

/**
 * @brief Reset Curtis power limits to motor-optimized defaults
 */
void Configuration::resetCurtisDefaults() {
    // Default Curtis power curve configuration
    baseSpeed = 2000.0f;         // Start limiting at 2000 RPM
    deltaSpeed = 500.0f;         // 500 RPM increments
    nominalPower = 85.0f;        // 85% nominal power
    
    // Default drive curve - typical AC motor characteristics
    // Zone 0: 0 to 2500 RPM (baseSpeed + 1×deltaSpeed)
    drivePowerLimits[0] = 85.0f;   // Nominal power - full torque available
    
    // Zone 1: 2500 to 3000 RPM (baseSpeed + 2×deltaSpeed) 
    drivePowerLimits[1] = 95.0f;   // Slight increase for peak power
    
    // Zone 2: 3000 to 4000 RPM (baseSpeed + 4×deltaSpeed)
    drivePowerLimits[2] = 100.0f;  // Peak power zone
    
    // Zone 3: 4000 to 6000 RPM (baseSpeed + 8×deltaSpeed)
    drivePowerLimits[3] = 70.0f;   // Power drops due to back-EMF
    
    // Zone 4: 6000+ RPM 
    drivePowerLimits[4] = 35.0f;   // Limited high-speed power
    
    // Default regen curve - more conservative for battery protection
    regenPowerLimits[0] = 80.0f;   // Strong regen at low speeds
    regenPowerLimits[1] = 85.0f;   // Peak regen zone
    regenPowerLimits[2] = 90.0f;   // Maximum regen capability
    regenPowerLimits[3] = 60.0f;   // Reduced regen at high speeds
    regenPowerLimits[4] = 25.0f;   // Minimal regen at very high speeds
    
    // Neutral braking defaults - balanced for good feel
    baselineUpdateRate = 0.08f;    // Moderate baseline tracking
    baselineDecayRate = 0.995f;    // Slow decay for natural feel  
    regenMultiplier = 1.3f;        // Slightly stronger regen than accel
}

/**
 * @brief Save current settings to flash
 * @return true if save was successful
 */
bool Configuration::save() {
    bool success = true;
    
    preferences.begin(NAMESPACE, false);  // Open in RW mode
    
    // Save existing parameters
    success &= preferences.putUChar(KEY_DRIVE_MODE, static_cast<uint8_t>(driveMode));
    success &= preferences.putInt(KEY_MAX_TORQUE, maxTorque);
    success &= preferences.putUChar(KEY_MAX_SOC, maxSOC);
    success &= preferences.putUChar(KEY_MAX_CHARGING_CURRENT, maxChargingCurrent);
    
    // Save Curtis power limiting parameters
    success &= preferences.putFloat(KEY_BASE_SPEED, baseSpeed);
    success &= preferences.putFloat(KEY_DELTA_SPEED, deltaSpeed);
    success &= preferences.putFloat(KEY_NOMINAL_POWER, nominalPower);
    success &= preferences.putBytes(KEY_DRIVE_LIMITS, drivePowerLimits, sizeof(drivePowerLimits));
    success &= preferences.putBytes(KEY_REGEN_LIMITS, regenPowerLimits, sizeof(regenPowerLimits));
    
    // Save neutral braking parameters
    success &= preferences.putFloat(KEY_BASELINE_UPDATE_RATE, baselineUpdateRate);
    success &= preferences.putFloat(KEY_BASELINE_DECAY_RATE, baselineDecayRate);
    success &= preferences.putFloat(KEY_REGEN_MULTIPLIER, regenMultiplier);
    
    preferences.end();
    return success;
}

/**
 * @brief Load settings from flash
 * @return true if load was successful
 */
bool Configuration::load() {
    bool success = true;
    
    preferences.begin(NAMESPACE, true);  // Open in read-only mode
    
    // Load existing parameters
    if (preferences.isKey(KEY_DRIVE_MODE)) {
        uint8_t mode = preferences.getUChar(KEY_DRIVE_MODE, static_cast<uint8_t>(driveMode));
        if (mode <= static_cast<uint8_t>(DriveMode::OPD)) {
            driveMode = static_cast<DriveMode>(mode);
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_MAX_TORQUE)) {
        int torque = preferences.getInt(KEY_MAX_TORQUE, maxTorque);
        if (torque >= MIN_TORQUE_LIMIT && torque <= MAX_TORQUE_LIMIT) {
            maxTorque = torque;
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_MAX_SOC)) {
        uint8_t soc = preferences.getUChar(KEY_MAX_SOC, maxSOC);
        if (soc >= MIN_SOC_LIMIT && soc <= MAX_SOC_LIMIT) {
            maxSOC = soc;
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_MAX_CHARGING_CURRENT)) {
        uint8_t current = preferences.getUChar(KEY_MAX_CHARGING_CURRENT, maxChargingCurrent);
        if (current >= MIN_CHARGING_CURRENT && current <= MAX_CHARGING_CURRENT) {
            maxChargingCurrent = current;
        } else {
            success = false;
        }
    }
    
    // Load Curtis parameters
    if (preferences.isKey(KEY_BASE_SPEED)) {
        float speed = preferences.getFloat(KEY_BASE_SPEED, baseSpeed);
        if (speed >= MIN_BASE_SPEED && speed <= MAX_BASE_SPEED) {
            baseSpeed = speed;
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_DELTA_SPEED)) {
        float speed = preferences.getFloat(KEY_DELTA_SPEED, deltaSpeed);
        if (speed >= MIN_DELTA_SPEED && speed <= MAX_DELTA_SPEED) {
            deltaSpeed = speed;
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_NOMINAL_POWER)) {
        float power = preferences.getFloat(KEY_NOMINAL_POWER, nominalPower);
        if (power >= MIN_NOMINAL_POWER && power <= MAX_NOMINAL_POWER) {
            nominalPower = power;
        } else {
            success = false;
        }
    }
    
    // Load power limit arrays
    if (preferences.isKey(KEY_DRIVE_LIMITS)) {
        size_t len = preferences.getBytesLength(KEY_DRIVE_LIMITS);
        if (len == sizeof(drivePowerLimits)) {
            preferences.getBytes(KEY_DRIVE_LIMITS, drivePowerLimits, len);
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_REGEN_LIMITS)) {
        size_t len = preferences.getBytesLength(KEY_REGEN_LIMITS);
        if (len == sizeof(regenPowerLimits)) {
            preferences.getBytes(KEY_REGEN_LIMITS, regenPowerLimits, len);
        } else {
            success = false;
        }
    }
    
    // Load neutral braking parameters
    if (preferences.isKey(KEY_BASELINE_UPDATE_RATE)) {
        float rate = preferences.getFloat(KEY_BASELINE_UPDATE_RATE, baselineUpdateRate);
        if (rate >= MIN_BASELINE_UPDATE_RATE && rate <= MAX_BASELINE_UPDATE_RATE) {
            baselineUpdateRate = rate;
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_BASELINE_DECAY_RATE)) {
        float rate = preferences.getFloat(KEY_BASELINE_DECAY_RATE, baselineDecayRate);
        if (rate >= MIN_BASELINE_DECAY_RATE && rate <= MAX_BASELINE_DECAY_RATE) {
            baselineDecayRate = rate;
        } else {
            success = false;
        }
    }
    
    if (preferences.isKey(KEY_REGEN_MULTIPLIER)) {
        float multiplier = preferences.getFloat(KEY_REGEN_MULTIPLIER, regenMultiplier);
        if (multiplier >= MIN_REGEN_MULTIPLIER && multiplier <= MAX_REGEN_MULTIPLIER) {
            regenMultiplier = multiplier;
        } else {
            success = false;
        }
    }
    
    preferences.end();
    return success;
}

// Existing parameter setters
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

// NEW: Curtis parameter setters
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

bool Configuration::setBaselineUpdateRate(float rate) {
    if (rate >= MIN_BASELINE_UPDATE_RATE && rate <= MAX_BASELINE_UPDATE_RATE) {
        baselineUpdateRate = rate;
        return true;
    }
    return false;
}

bool Configuration::setBaselineDecayRate(float rate) {
    if (rate >= MIN_BASELINE_DECAY_RATE && rate <= MAX_BASELINE_DECAY_RATE) {
        baselineDecayRate = rate;
        return true;
    }
    return false;
}

bool Configuration::setRegenMultiplier(float multiplier) {
    if (multiplier >= MIN_REGEN_MULTIPLIER && multiplier <= MAX_REGEN_MULTIPLIER) {
        regenMultiplier = multiplier;
        return true;
    }
    return false;
}