/**
 * @file configuration.h
 * @brief Runtime Configuration and Persistent Storage Management - Updated with Curtis Power Limiting
 * 
 * Manages user-configurable parameters that persist across reboots:
 * - Driving mode selection
 * - Maximum torque limit
 * - Maximum state of charge (SOC) limit  
 * - Maximum AC charging current
 * - Curtis-style power limiting curves
 * - Neutral braking parameters
 */

#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

/**
 * @brief Configuration management system with Curtis Power Limiting
 * 
 * Handles runtime configurable parameters and persistent storage to ESP32 flash.
 * Provides validation, default values, and serialization for all parameters.
 * 
 * NEW: Curtis-style power limiting with configurable speed zones and power curves
 */
class Configuration {
public:
    bool setDriveModeFromByte(uint8_t modeByte);

    /**
     * @brief Initialize configuration system
     * Loads stored settings from flash if available
     * Initializes default values if no stored settings exist
     */
    void begin();

    /**
     * @brief Save current settings to flash
     * @return true if save was successful
     */
    bool save();

    /**
     * @brief Load settings from flash
     * @return true if load was successful
     */
    bool load();

    /**
     * @brief Reset all settings to default values
     */
    void resetToDefaults();

    // Existing configuration methods
    DriveMode getDriveMode() const { return driveMode; }
    bool setDriveMode(DriveMode mode);
    bool setDriveMode(const String& modeStr);
    int getMaxTorque() const { return maxTorque; }
    bool setMaxTorque(int torque);
    uint8_t getMaxSOC() const { return maxSOC; }
    bool setMaxSOC(uint8_t soc);
    uint8_t getMaxChargingCurrent() const { return maxChargingCurrent; }
    bool setMaxChargingCurrent(uint8_t current);
    String getDriveModeString() const;

    // NEW: Curtis Power Limiting Configuration
    /**
     * @brief Get base speed where power limiting starts
     * @return Base speed in RPM
     */
    float getBaseSpeed() const { return baseSpeed; }

    /**
     * @brief Set base speed for power limiting
     * @param speed Base speed in RPM (500-5000)
     * @return true if valid and set
     */
    bool setBaseSpeed(float speed);

    /**
     * @brief Get delta speed increment between zones
     * @return Delta speed in RPM
     */
    float getDeltaSpeed() const { return deltaSpeed; }

    /**
     * @brief Set delta speed increment
     * @param speed Delta speed in RPM (100-2000)
     * @return true if valid and set
     */
    bool setDeltaSpeed(float speed);

    /**
     * @brief Get nominal power percentage for base zone
     * @return Nominal power percentage (0-100%)
     */
    float getNominalPower() const { return nominalPower; }

    /**
     * @brief Set nominal power percentage
     * @param power Nominal power (50-100%)
     * @return true if valid and set
     */
    bool setNominalPower(float power);

    /**
     * @brief Get drive power limits array
     * @return Pointer to 5-element array of power limits
     */
    const float* getDrivePowerLimits() const { return drivePowerLimits; }

    /**
     * @brief Get regen power limits array
     * @return Pointer to 5-element array of power limits
     */
    const float* getRegenPowerLimits() const { return regenPowerLimits; }

    /**
     * @brief Set drive power limit for specific zone
     * @param zone Zone index (0-4)
     * @param power Power percentage (10-120%)
     * @return true if valid and set
     */
    bool setDrivePowerLimit(int zone, float power);

    /**
     * @brief Set regen power limit for specific zone
     * @param zone Zone index (0-4)
     * @param power Power percentage (10-100%)
     * @return true if valid and set
     */
    bool setRegenPowerLimit(int zone, float power);

    // NEW: Neutral Braking Configuration
    /**
     * @brief Get neutral braking baseline update rate
     * @return Update rate (0.01-0.2)
     */
    float getBaselineUpdateRate() const { return baselineUpdateRate; }

    /**
     * @brief Set baseline update rate (how fast baseline follows torque)
     * @param rate Update rate (0.01-0.2)
     * @return true if valid and set
     */
    bool setBaselineUpdateRate(float rate);

    /**
     * @brief Get neutral braking decay rate
     * @return Decay rate (0.9-0.999)
     */
    float getBaselineDecayRate() const { return baselineDecayRate; }

    /**
     * @brief Set baseline decay rate (how fast baseline decays)
     * @param rate Decay rate (0.9-0.999)
     * @return true if valid and set
     */
    bool setBaselineDecayRate(float rate);

    /**
     * @brief Get regen multiplier
     * @return Regen multiplier (0.5-3.0)
     */
    float getRegenMultiplier() const { return regenMultiplier; }

    /**
     * @brief Set regen strength multiplier
     * @param multiplier Regen multiplier (0.5-3.0)
     * @return true if valid and set
     */
    bool setRegenMultiplier(float multiplier);

    /**
     * @brief Reset Curtis power limits to motor-optimized defaults
     */
    void resetCurtisDefaults();

private:
    Preferences preferences;         ///< ESP32 preferences handle
    static const char* NAMESPACE;    ///< Preferences namespace
    
    // Existing configuration parameters
    DriveMode driveMode;             ///< Current driving mode
    int maxTorque;                   ///< Maximum motor torque (Nm)
    uint8_t maxSOC;                  ///< Maximum state of charge (%)
    uint8_t maxChargingCurrent;      ///< Maximum AC charging current (A)
    
    // NEW: Curtis Power Limiting Parameters
    float baseSpeed;                 ///< Base speed where limiting starts (RPM)
    float deltaSpeed;                ///< Speed increment between zones (RPM)
    float nominalPower;              ///< Power percentage for base zone (%)
    float drivePowerLimits[5];       ///< Power limits for drive zones (%)
    float regenPowerLimits[5];       ///< Power limits for regen zones (%)
    
    // NEW: Neutral Braking Parameters
    float baselineUpdateRate;        ///< How fast baseline follows actual torque
    float baselineDecayRate;         ///< How fast baseline decays when coasting
    float regenMultiplier;           ///< Regen strength vs acceleration
    
    // Storage keys for existing parameters
    static const char* KEY_DRIVE_MODE;
    static const char* KEY_MAX_TORQUE;
    static const char* KEY_MAX_SOC;
    static const char* KEY_MAX_CHARGING_CURRENT;
    
    // NEW: Storage keys for Curtis parameters
    static const char* KEY_BASE_SPEED;
    static const char* KEY_DELTA_SPEED;
    static const char* KEY_NOMINAL_POWER;
    static const char* KEY_DRIVE_LIMITS;
    static const char* KEY_REGEN_LIMITS;
    static const char* KEY_BASELINE_UPDATE_RATE;
    static const char* KEY_BASELINE_DECAY_RATE;
    static const char* KEY_REGEN_MULTIPLIER;
    
    // Validation limits for existing parameters
    static constexpr int MIN_TORQUE_LIMIT = 100;
    static constexpr int MAX_TORQUE_LIMIT = 850;
    static constexpr int MIN_SOC_LIMIT = 50;
    static constexpr int MAX_SOC_LIMIT = 100;
    static constexpr int MIN_CHARGING_CURRENT = 6;
    static constexpr int MAX_CHARGING_CURRENT = 32;
    
    // NEW: Validation limits for Curtis parameters
    static constexpr float MIN_BASE_SPEED = 500.0f;
    static constexpr float MAX_BASE_SPEED = 5000.0f;
    static constexpr float MIN_DELTA_SPEED = 100.0f;
    static constexpr float MAX_DELTA_SPEED = 2000.0f;
    static constexpr float MIN_NOMINAL_POWER = 50.0f;
    static constexpr float MAX_NOMINAL_POWER = 100.0f;
    static constexpr float MIN_POWER_LIMIT = 10.0f;
    static constexpr float MAX_DRIVE_POWER_LIMIT = 120.0f;
    static constexpr float MAX_REGEN_POWER_LIMIT = 100.0f;
    static constexpr float MIN_BASELINE_UPDATE_RATE = 0.01f;
    static constexpr float MAX_BASELINE_UPDATE_RATE = 0.2f;
    static constexpr float MIN_BASELINE_DECAY_RATE = 0.9f;
    static constexpr float MAX_BASELINE_DECAY_RATE = 0.999f;
    static constexpr float MIN_REGEN_MULTIPLIER = 0.5f;
    static constexpr float MAX_REGEN_MULTIPLIER = 3.0f;
};

// Global configuration instance
extern Configuration config;