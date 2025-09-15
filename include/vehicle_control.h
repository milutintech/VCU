/**
 * @file vehicle_control.h
 * @brief Vehicle Control System with Curtis-Style Neutral Braking and Power Limiting
 * 
 * This class manages the core vehicle control logic including:
 * - Curtis-style neutral braking (immediate throttle response)
 * - Speed-based power limiting with configurable curves
 * - Smooth torque baseline tracking
 * - Advanced gear transition protection
 * - Configurable driving characteristics
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
     * @brief Constructs the vehicle control system with Curtis-style control
     * @param ads Reference to ADS1115 ADC for pedal position reading
     */
    explicit VehicleControl(ADS1115& ads);
    
    /**
     * @brief NEW: Calculate motor torque percentage using Curtis-style neutral braking
     * @return Calculated torque percentage (-100% to +100%)
     * 
     * Features:
     * - Immediate throttle response (no zones)
     * - Speed-based power limiting
     * - Torque baseline tracking for neutral braking
     * - Configurable power curves
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
        torqueBaseline = 0.0f;  // NEW: Reset baseline
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
     * @brief NEW: Calculate power limit based on current motor speed using Curtis curves
     * @param motorSpeed Current motor speed in RPM
     * @param isDriving true for drive power limits, false for regen limits
     * @return Power limit percentage (0-120%)
     */
    float calculatePowerLimit(float motorSpeed, bool isDriving = true);
    
    /**
     * @brief NEW: Interpolate power limit between Curtis speed zones
     * @param motorSpeed Current motor speed in RPM
     * @param powerLimits Array of power limits for the 5 zones
     * @return Interpolated power limit percentage
     */
    float interpolatePowerLimit(float motorSpeed, const float* powerLimits);
    
    /**
     * @brief NEW: Get Curtis speed zone boundaries based on configuration
     * @param zone Zone index (0-4)
     * @return Speed boundary for the zone in RPM
     */
    float getCurtisSpeedBoundary(int zone);
    
    /**
     * @brief NEW: Update torque baseline for neutral braking
     * @param currentTorque Current actual torque demand
     * 
     * Tracks recent torque history to create the "neutral point" that makes
     * any throttle reduction feel like immediate braking.
     */
    void updateTorqueBaseline(float currentTorque);
    
    /**
     * @brief NEW: Apply Curtis-style neutral braking logic
     * @param throttlePercent Raw throttle position (0-100%)
     * @param powerLimit Current power limit based on speed
     * @return Calculated torque percentage with neutral braking applied
     */
    float applyCurtisNeutralBraking(float throttlePercent, float powerLimit);

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
    
    // NEW: Curtis Neutral Braking State Variables
    float torqueBaseline;            // Current torque baseline for neutral braking
    unsigned long lastBaselineUpdate; // Timestamp for baseline updates
    float previousThrottlePercent;   // Previous throttle position for delta calculation
    bool neutralBrakingActive;       // Flag indicating neutral braking is engaged
    
    // NEW: Curtis Power Limiting State
    float currentPowerLimit;         // Current power limit percentage
    float previousPowerLimit;        // Previous power limit for smoothing
    unsigned long lastPowerUpdate;   // Timestamp for power limit updates
};