/**
 * @file configuration.h - UPDATED with Torque Transition Configuration
 * @brief Enhanced Configuration with pedal zones, Curtis power limiting, and torque transitions
 */

#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "config.h"

/**
 * @brief Enhanced Configuration Class - With Torque Transition Support
 */
class Configuration {
public:
    /**
     * @brief Initialize configuration system
     */
    void begin();
    
    /**
     * @brief Save current settings to flash
     */
    bool save();
    
    /**
     * @brief Load settings from flash
     */
    bool load();
    
    /**
     * @brief Reset all settings to default values
     */
    void resetToDefaults();
    
    // === EXISTING INTERFACE (backwards compatible) ===
    DriveMode getDriveMode() const { return driveMode; }
    int getMaxTorque() const { return maxTorque; }
    uint8_t getMaxSOC() const { return maxSOC; }
    uint8_t getMaxChargingCurrent() const { return maxChargingCurrent; }
    String getDriveModeString() const;
    
    bool setDriveMode(DriveMode mode);
    bool setDriveMode(const String& modeStr);
    bool setMaxTorque(int torque);
    bool setMaxSOC(uint8_t soc);
    bool setMaxChargingCurrent(uint8_t current);
    bool setDriveModeFromByte(uint8_t modeByte);
    
    // === CURTIS POWER LIMITING (existing - keep for delta-based power map) ===
    float getBaseSpeed() const { return baseSpeed; }
    float getDeltaSpeed() const { return deltaSpeed; }
    float getNominalPower() const { return nominalPower; }
    const float* getDrivePowerLimits() const { return drivePowerLimits; }
    const float* getRegenPowerLimits() const { return regenPowerLimits; }
    
    bool setBaseSpeed(float speed);
    bool setDeltaSpeed(float speed);
    bool setNominalPower(float power);
    bool setDrivePowerLimit(int zone, float power);
    bool setRegenPowerLimit(int zone, float power);
    void resetCurtisDefaults();
    
    // === SIMPLIFIED PEDAL ZONES ===
    float getRegenZoneEnd() const { return regenZoneEnd; }
    float getCoastZoneEnd() const { return coastZoneEnd; }
    float getRegenProgression() const { return regenProgression; }
    float getAccelProgression() const { return accelProgression; }
    
    bool setRegenZoneEnd(float value);
    bool setCoastZoneEnd(float value);
    bool setRegenProgression(float value);
    bool setAccelProgression(float value);
    void resetPedalDefaults();
    
    // === NEW: TORQUE TRANSITION TIMING ===
    float getRegenEngageTime() const { return regenEngageTime; }
    float getRegenReleaseTime() const { return regenReleaseTime; }
    float getPowerEngageTime() const { return powerEngageTime; }
    float getPowerReleaseTime() const { return powerReleaseTime; }
    float getCrossoverTime() const { return crossoverTime; }
    
    bool setRegenEngageTime(float timeMs);
    bool setRegenReleaseTime(float timeMs);
    bool setPowerEngageTime(float timeMs);
    bool setPowerReleaseTime(float timeMs);
    bool setCrossoverTime(float timeMs);
    void resetTransitionDefaults();
    
    // === JSON INTERFACE ===
    String toJSON();
    bool fromJSON(const String& json);
    String getCategoryJSON(const String& category);
    bool setCategoryJSON(const String& category, const String& json);
    
private:
    Preferences preferences;
    static const char* NAMESPACE;
    
    // === MEMBER VARIABLES ===
    // Basic configuration
    DriveMode driveMode;
    int maxTorque;
    uint8_t maxSOC;
    uint8_t maxChargingCurrent;
    
    // Curtis power limiting (keep for delta-based power map)
    float baseSpeed;
    float deltaSpeed;
    float nominalPower;
    float drivePowerLimits[5];
    float regenPowerLimits[5];
    
    // Simplified pedal zones
    float regenZoneEnd;        ///< End of regen zone (0-50%)
    float coastZoneEnd;        ///< End of coast zone (regenZoneEnd-60%)
    float regenProgression;    ///< Regen curve factor (1.0-3.0)
    float accelProgression;    ///< Accel curve factor (1.0-3.0)
    
    // NEW: Torque transition timing (milliseconds)
    float regenEngageTime;     ///< Regen engagement time (0 -> negative torque)
    float regenReleaseTime;    ///< Regen release time (negative -> 0 torque)
    float powerEngageTime;     ///< Power engagement time (0 -> positive torque)
    float powerReleaseTime;    ///< Power release time (positive -> 0 torque)
    float crossoverTime;       ///< Crossover transition time (regen <-> power)
    
    // Storage keys - Basic
    static const char* KEY_DRIVE_MODE;
    static const char* KEY_MAX_TORQUE;
    static const char* KEY_MAX_SOC;
    static const char* KEY_MAX_CHARGING_CURRENT;
    
    // Storage keys - Curtis
    static const char* KEY_BASE_SPEED;
    static const char* KEY_DELTA_SPEED;
    static const char* KEY_NOMINAL_POWER;
    static const char* KEY_DRIVE_LIMITS;
    static const char* KEY_REGEN_LIMITS;
    
    // Storage keys - Pedal zones
    static const char* KEY_REGEN_ZONE_END;
    static const char* KEY_COAST_ZONE_END;
    static const char* KEY_REGEN_PROGRESSION;
    static const char* KEY_ACCEL_PROGRESSION;
    
    // NEW: Storage keys - Transition timing
    static const char* KEY_REGEN_ENGAGE_TIME;
    static const char* KEY_REGEN_RELEASE_TIME;
    static const char* KEY_POWER_ENGAGE_TIME;
    static const char* KEY_POWER_RELEASE_TIME;
    static const char* KEY_CROSSOVER_TIME;
    
    // Validation limits - Basic
    static constexpr int MIN_TORQUE_LIMIT = 100;
    static constexpr int MAX_TORQUE_LIMIT = 850;
    static constexpr int MIN_SOC_LIMIT = 50;
    static constexpr int MAX_SOC_LIMIT = 100;
    static constexpr int MIN_CHARGING_CURRENT = 6;
    static constexpr int MAX_CHARGING_CURRENT = 32;
    
    // Validation limits - Curtis
    static constexpr float MIN_BASE_SPEED = 500.0f;
    static constexpr float MAX_BASE_SPEED = 5000.0f;
    static constexpr float MIN_DELTA_SPEED = 100.0f;
    static constexpr float MAX_DELTA_SPEED = 2000.0f;
    static constexpr float MIN_NOMINAL_POWER = 50.0f;
    static constexpr float MAX_NOMINAL_POWER = 100.0f;
    static constexpr float MIN_POWER_LIMIT = 10.0f;
    static constexpr float MAX_DRIVE_POWER_LIMIT = 120.0f;
    static constexpr float MAX_REGEN_POWER_LIMIT = 100.0f;
    
    // NEW: Validation limits - Transition timing
    static constexpr float MIN_TRANSITION_TIME = 0.0f;
    static constexpr float MAX_TRANSITION_TIME = 1000.0f;
    
    // Helper methods
    JsonDocument createDrivingJSON();
    JsonDocument createCurtisJSON();
    JsonDocument createPedalJSON();
    JsonDocument createTransitionJSON();  // NEW
    
    bool parseDrivingJSON(const JsonObject& obj);
    bool parseCurtisJSON(const JsonObject& obj);
    bool parsePedalJSON(const JsonObject& obj);
    bool parseTransitionJSON(const JsonObject& obj);  // NEW
};

// Global configuration instance (backwards compatible)
extern Configuration config;