/**
 * @file vehicle_control.h - SIMPLIFIED Control System
 * @brief Vehicle Control System with Simple Pedal Zones and Delta-Based Power Limiting
 * 
 * This class manages the simplified vehicle control logic including:
 * - Three-zone pedal system (regen/coast/accel)
 * - Progressive curves for natural feel
 * - Delta-based power limiting with configurable curves
 * - Advanced gear transition protection
 */

#pragma once
#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include "config.h"
#include "vehicle_parameters.h"
#include "ADS1X15.h"

class CANManager;

class VehicleControl {
public:
    /**
     * @brief Constructs the vehicle control system with simplified pedal zones
     * @param ads Reference to ADS1115 ADC for pedal position reading
     */
    explicit VehicleControl(ADS1115& ads);
    
    /**
     * @brief Calculate motor torque percentage using simplified pedal zones
     * @return Calculated torque percentage (-100% to +100%)
     * 
     * Features:
     * - Three-zone pedal system (regen/coast/accel)
     * - Progressive curves for natural pedal feel
     * - Delta-based power limiting by motor speed
     * - Configurable zone boundaries and progression factors
     */
    float calculateTorquePercentage();

    /**
     * @brief Updates gear state based on switch inputs with anti-jerk protection
     * Handles gear selection with anti-jerk protection
     */
    void updateGearState();

    /**
     * @brief Updates current motor speed
     * @param speed Motor speed in RPM
     */
    void setMotorSpeed(float speed);

    /**
     * @brief Sets current gear state (Drive/Neutral/Reverse)
     * @param gear New gear state
     */
    void setCurrentGear(GearState gear);

    /**
     * @brief Sets driving mode (Legacy/Regen/OPD)
     * @param mode New driving mode
     */
    void setDrivingMode(DriveMode mode);

    /**
     * @brief Checks if DMC (motor controller) should be enabled
     * @return true if DMC should be enabled (torque != 0)
     */
    bool isDMCEnabled() const;
    
    /**
     * @brief Check if currently in a gear transition
     * @return true if gear change is in progress
     */
    bool isGearTransitionInProgress() const { return isInGearTransition; }
    
    /**
     * @brief Force clear all torque and reset control state
     * Useful for emergency stops or when aborting operations
     */
    void clearTorqueState() {
        lastTorquePercent = 0.0f;
        filteredTorquePercent = 0.0f;
        enableDMC = false;
        wasInDeadband = false;
        isInGearTransition = false;
    }
    
    // Configuration methods
    void setCanManager(CANManager* canMgr) { canManager = canMgr; }
    void setGearRatio(GearRatio ratio) { currentGearRatio = ratio; }
    
    static constexpr float MAX_VEHICLE_SPEED = 120.0f;  // kph

private:
    /**
     * @brief Sample pedal position from ADC with averaging
     * @return Raw ADC value averaged over 4 samples
     */
    int32_t samplePedalPosition();

    /**
     * @brief Calculate current vehicle speed based on motor RPM and gear ratios
     * @return Vehicle speed in kph
     */
    float calculateVehicleSpeed();
    
    /**
     * @brief Calculate power limit based on current motor speed using delta curves
     * @param motorSpeed Current motor speed in RPM
     * @param isDriving true for drive power limits, false for regen limits
     * @return Power limit percentage (0-120%)
     */
    float calculatePowerLimit(float motorSpeed, bool isDriving = true);
    
    /**
     * @brief Interpolate power limit between speed zones
     * @param motorSpeed Current motor speed in RPM
     * @param powerLimits Array of power limits for the 5 zones
     * @return Interpolated power limit percentage
     */
    float interpolatePowerLimit(float motorSpeed, const float* powerLimits);
    
    /**
     * @brief NEW: Apply simplified three-zone pedal mapping
     * @param throttlePercent Raw throttle position (0-100%)
     * @return Torque percentage with zone mapping applied
     */
    float applyPedalZones(float throttlePercent);
    
    /**
     * @brief Apply progressive curve to zone value
     * @param zonePosition Position within zone (0-1)
     * @param progression Progression factor (1.0=linear, >1.0=progressive)
     * @return Curved output value (0-1)
     */
    float applyProgressiveCurve(float zonePosition, float progression);

    /**
     * @brief Apply gear transition protection to prevent jerking
     * @param torquePercent Input torque percentage
     * @return Modified torque with gear transition protection
     */
    float applyGearTransitionProtection(float torquePercent);

    /**
     * @brief Apply deadband hysteresis around zero
     * @param torquePercent Input torque percentage
     * @return Processed torque with deadband
     */
    float applyDeadbandHysteresis(float torquePercent);
    
    // Member variables
    ADS1115& ads;                    // Reference to ADC
    DriveMode currentDrivingMode;    // Current driving mode
    GearState currentGear;           // Current gear state
    GearRatio currentGearRatio;      // Current gear ratio
    bool shiftAttempted;             // Track shift attempts at high speed
    
    bool enableDMC;                  // DMC enable flag
    bool wasInDeadband;              // Deadband hysteresis state
    bool wasEnabled;                 // Previous enable state
    
    // Gear transition state management
    bool isInGearTransition;         // Flag indicating gear change in progress
    unsigned long gearTransitionStartTime; // Timestamp for gear transition timing
    GearState previousGear;          // Track previous gear for transition detection
    
    // Basic torque tracking
    float lastTorquePercent;         // Last calculated torque percentage
    float filteredTorquePercent;     // Filtered torque percentage
    float motorSpeed;                // Current motor speed
    CANManager* canManager = nullptr; 
};