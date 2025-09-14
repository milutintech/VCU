/**
 * @file vehicle_control.h
 * @brief Vehicle Control System for Electric Vehicle - Updated with Percentage-Based Torque
 * 
 * This class manages the core vehicle control logic including:
 * - Smooth one-foot driving pedal system
 * - Percentage-based torque control (-100% to +100%)
 * - Advanced filtering without lag
 * - Anti-jerk gear transitions
 * - Speed-adaptive pedal response
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
     * @brief Constructs the vehicle control system
     * @param ads Reference to ADS1115 ADC for pedal position reading
     */
    explicit VehicleControl(ADS1115& ads);
    
    /**
     * @brief Calculates motor torque percentage based on smooth one-foot driving
     * @return Calculated torque percentage (-100% to +100%)
     */
    float calculateTorquePercentage();

    /**
     * @brief Updates gear state based on switch inputs and speed
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
     * @brief Samples pedal position from ADC
     * @return Raw ADC value averaged over 4 samples
     */
    int32_t samplePedalPosition();

    /**
     * @brief Calculates current vehicle speed based on motor RPM and gear ratios
     * @return Vehicle speed in kph
     */
    float calculateVehicleSpeed();
    
    /**
     * @brief NEW: Handle smooth one-foot driving with percentage-based torque
     * @param throttlePosition Processed pedal position (0-100%)
     * @param speed Current vehicle speed in kph
     * @return Calculated torque percentage (-100% to +100%)
     */
    float handleSmoothDriving(float throttlePosition, float speed);
    
    /**
     * @brief NEW: Apply advanced filtering to prevent spikes without lag
     * @param requestedTorquePercent Raw calculated torque percentage
     * @return Filtered torque percentage
     */
    float applyAdvancedFiltering(float requestedTorquePercent);

    /**
     * @brief NEW: Apply gear transition protection to prevent jerking
     * @param torquePercent Input torque percentage
     * @return Modified torque with gear transition protection
     */
    float applyGearTransitionProtection(float torquePercent);

    /**
     * @brief NEW: Apply speed-adaptive pedal response
     * @param throttlePosition Raw pedal position (0-100%)
     * @param speed Current vehicle speed (kph)
     * @return Modified pedal position for speed-dependent behavior
     */
    float applySpeedAdaptation(float throttlePosition, float speed);

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
    
    // NEW: Gear transition state management
    bool isInGearTransition;         // Flag indicating gear change in progress
    unsigned long gearTransitionStartTime; // Timestamp for gear transition timing
    GearState previousGear;          // Track previous gear for transition detection
    
    // NEW: Percentage-based torque tracking
    float lastTorquePercent;         // Last calculated torque percentage
    float filteredTorquePercent;     // Filtered torque percentage
    float motorSpeed;                // Current motor speed
    CANManager* canManager = nullptr; 
    
    // NEW: Advanced filtering state variables
    float previousFilteredTorque;    // Previous filtered value for spike detection
    bool spikeDetected;              // Spike detection flag
    unsigned long lastFilterTime;    // For timing calculations
    
    // NEW: Pedal zone tracking for smooth transitions
    int currentPedalZone;            // Track which pedal zone we're in (0=regen, 1=coast, 2=accel)
    int previousPedalZone;           // Previous pedal zone for transition detection
    float zoneTransitionFactor;      // Smoothing factor for zone transitions
};