/**
 * @file configuration.h
 * @brief Runtime Configuration and Persistent Storage Management
 * 
 * Manages user-configurable parameters that persist across reboots:
 * - Driving mode selection
 * - Maximum torque limit
 * - Maximum state of charge (SOC) limit
 * - Maximum AC charging current
 * 
 * Uses ESP32 Preferences library for non-volatile storage with wear leveling.
 */

#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

/**
 * @brief Configuration management system
 * 
 * Handles runtime configurable parameters and persistent storage to ESP32 flash.
 * Provides validation, default values, and serialization for all parameters.
 * 
 * Configuration values are applied immediately but only saved to flash
 * when explicitly requested to avoid excessive write cycles.
 */
class Configuration {
public:
    /**
     * @brief Initialize configuration system
     * 
     * Loads stored settings from flash if available
     * Initializes default values if no stored settings exist
     */
    void begin();

    /**
     * @brief Save current settings to flash
     * Persists all configuration parameters to non-volatile storage
     * 
     * @return true if save was successful
     */
    bool save();

    /**
     * @brief Load settings from flash
     * Loads all configuration parameters from non-volatile storage
     * 
     * @return true if load was successful
     */
    bool load();

    /**
     * @brief Reset all settings to default values
     */
    void resetToDefaults();

    /**
     * @brief Get current drive mode
     * @return Currently selected drive mode
     */
    DriveMode getDriveMode() const { return driveMode; }

    /**
     * @brief Set drive mode
     * @param mode New drive mode to use
     * @return true if the value was valid and set
     */
    bool setDriveMode(DriveMode mode);
    
    /**
     * @brief Set drive mode from string
     * @param modeStr Drive mode as string ("legacy", "regen", "opd")
     * @return true if the value was valid and set
     */
    bool setDriveMode(const String& modeStr);

    /**
     * @brief Get maximum motor torque limit
     * @return Maximum torque value in Nm
     */
    int getMaxTorque() const { return maxTorque; }

    /**
     * @brief Set maximum motor torque limit
     * @param torque New maximum torque in Nm
     * @return true if the value was valid and set
     */
    bool setMaxTorque(int torque);

    /**
     * @brief Get maximum allowed state of charge
     * @return Maximum state of charge percentage (0-100)
     */
    uint8_t getMaxSOC() const { return maxSOC; }

    /**
     * @brief Set maximum allowed state of charge
     * @param soc New maximum SOC percentage (0-100)
     * @return true if the value was valid and set
     */
    bool setMaxSOC(uint8_t soc);

    /**
     * @brief Get maximum AC charging current
     * @return Maximum AC charging current in Amperes
     */
    uint8_t getMaxChargingCurrent() const { return maxChargingCurrent; }

    /**
     * @brief Set maximum AC charging current
     * @param current New maximum charging current in Amperes
     * @return true if the value was valid and set
     */
    bool setMaxChargingCurrent(uint8_t current);

    /**
     * @brief Get string representation of drive mode
     * @return Drive mode as string ("LEGACY", "REGEN", "OPD")
     */
    String getDriveModeString() const;

private:
    Preferences preferences;         ///< ESP32 preferences handle
    static const char* NAMESPACE;    ///< Preferences namespace
    
    // Configuration parameters
    DriveMode driveMode;             ///< Current driving mode
    int maxTorque;                   ///< Maximum motor torque (Nm)
    uint8_t maxSOC;                  ///< Maximum state of charge (%)
    uint8_t maxChargingCurrent;      ///< Maximum AC charging current (A)
    
    // Parameter keys for storage
    static const char* KEY_DRIVE_MODE;
    static const char* KEY_MAX_TORQUE;
    static const char* KEY_MAX_SOC;
    static const char* KEY_MAX_CHARGING_CURRENT;
    
    // Validation limits
    static constexpr int MIN_TORQUE_LIMIT = 100;
    static constexpr int MAX_TORQUE_LIMIT = 850;
    static constexpr int MIN_SOC_LIMIT = 50;
    static constexpr int MAX_SOC_LIMIT = 100;
    static constexpr int MIN_CHARGING_CURRENT = 6;
    static constexpr int MAX_CHARGING_CURRENT = 32;
};

// Global configuration instance
extern Configuration config;

// Utility functions for command parsing
inline String driveModeToString(DriveMode mode) {
    switch(mode) {
        case DriveMode::LEGACY: return "LEGACY";
        case DriveMode::REGEN: return "REGEN";
        case DriveMode::OPD: return "OPD";
        default: return "UNKNOWN";
    }
}

inline DriveMode stringToDriveMode(const String& modeStr) {
    String mode = modeStr;
    mode.toLowerCase();
    
    if (mode == "legacy") return DriveMode::LEGACY;
    if (mode == "regen") return DriveMode::REGEN;
    if (mode == "opd") return DriveMode::OPD;
    
    return DriveMode::REGEN; // Default
}