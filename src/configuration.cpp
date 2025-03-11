/**
 * @file configuration.cpp
 * @brief Implementation of runtime configuration management
 * 
 * Handles saving and loading configuration parameters to/from flash
 * using the ESP32 Preferences library.
 */

#include "configuration.h"

// Static members initialization
const char* Configuration::NAMESPACE = "vcu_config";
const char* Configuration::KEY_DRIVE_MODE = "drive_mode";
const char* Configuration::KEY_MAX_TORQUE = "max_torque";
const char* Configuration::KEY_MAX_SOC = "max_soc";
const char* Configuration::KEY_MAX_CHARGING_CURRENT = "max_ac_curr";

// Global configuration instance
Configuration config;

/**
 * @brief Initialize configuration system
 * 
 * Loads settings from flash if available or initializes
 * with default values if no stored settings exist.
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
    driveMode = DriveMode::REGEN;  // Default to regen braking mode
    maxTorque = VehicleParams::Motor::MAX_TRQ;
    maxSOC = 100;  // Default to 100% maximum SOC
    maxChargingCurrent = VehicleParams::Power::NLG_MAX_AC;
    
    // Save defaults to flash
    save();
}

/**
 * @brief Save current settings to flash
 * @return true if save was successful
 */
bool Configuration::save() {
    bool success = true;
    
    preferences.begin(NAMESPACE, false);  // Open in RW mode
    
    // Save all parameters
    success &= preferences.putUChar(KEY_DRIVE_MODE, static_cast<uint8_t>(driveMode));
    success &= preferences.putInt(KEY_MAX_TORQUE, maxTorque);
    success &= preferences.putUChar(KEY_MAX_SOC, maxSOC);
    success &= preferences.putUChar(KEY_MAX_CHARGING_CURRENT, maxChargingCurrent);
    
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
    
    // Only update values if keys exist
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
    
    preferences.end();
    return success;
}

/**
 * @brief Set drive mode
 * @param mode New drive mode to use
 * @return true if the value was valid and set
 */
bool Configuration::setDriveModeFromByte(uint8_t modeByte) {
    if (modeByte <= static_cast<uint8_t>(DriveMode::OPD)) {
        driveMode = static_cast<DriveMode>(modeByte);
        return true;
    }
    return false;
}

/**
 * @brief Set drive mode from string
 * @param modeStr Drive mode as string ("legacy", "regen", "opd")
 * @return true if the value was valid and set
 */
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

/**
 * @brief Set maximum motor torque limit
 * @param torque New maximum torque in Nm
 * @return true if the value was valid and set
 */
bool Configuration::setMaxTorque(int torque) {
    if (torque >= MIN_TORQUE_LIMIT && torque <= MAX_TORQUE_LIMIT) {
        maxTorque = torque;
        return true;
    }
    return false;
}

/**
 * @brief Set maximum allowed state of charge
 * @param soc New maximum SOC percentage (0-100)
 * @return true if the value was valid and set
 */
bool Configuration::setMaxSOC(uint8_t soc) {
    if (soc >= MIN_SOC_LIMIT && soc <= MAX_SOC_LIMIT) {
        maxSOC = soc;
        return true;
    }
    return false;
}


/**
 * @brief Set maximum AC charging current
 * @param current New maximum charging current in Amperes
 * @return true if the value was valid and set
 */
bool Configuration::setMaxChargingCurrent(uint8_t current) {
    if (current >= MIN_CHARGING_CURRENT && current <= MAX_CHARGING_CURRENT) {
        maxChargingCurrent = current;
        return true;
    }
    return false;
}

/**
 * @brief Get string representation of drive mode
 * @return Drive mode as string ("LEGACY", "REGEN", "OPD")
 */
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