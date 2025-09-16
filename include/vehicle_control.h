/**
 * @file vehicle_control.h - ENHANCED with Smooth Torque Transitions
 * @brief Vehicle Control System with Configurable Transition Timing and Three-Zone Pedal System
 * 
 * This class manages the enhanced vehicle control logic including:
 * - Three-zone pedal system (regen/coast/accel)
 * - Smooth torque transitions with configurable timing
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
     * @brief Constructs the enhanced vehicle control system
     * @param ads Reference to ADS1115 ADC for pedal position reading
     */
    explicit VehicleControl(ADS1115& ads);
    
    /**
     * @brief Calculate motor torque percentage with smooth transitions
     * @return Smoothly transitioned torque percentage (-100% to +100%)
     * 
     * Features:
     * - Three-zone pedal system (regen/coast/accel)
     * - Configurable smooth transitions for all torque changes
     * - Progressive curves for natural pedal feel
     * - Delta-based power limiting by motor speed
     * - Separate timing for regen/power engage/release
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
    void clearTorqueState();
    
    // Configuration methods
    void setCanManager(CANManager* canMgr) { canManager = canMgr; }
    void setGearRatio(GearRatio ratio) { currentGearRatio = ratio; }
    
    // NEW: Torque transition configuration methods
    /**
     * @brief Set regen engagement transition time
     * @param timeMs Time in milliseconds (0-1000ms)
     */
    void setRegenEngageTime(float timeMs);
    
    /**
     * @brief Set regen release transition time
     * @param timeMs Time in milliseconds (0-1000ms)
     */
    void setRegenReleaseTime(float timeMs);
    
    /**
     * @brief Set power engagement transition time
     * @param timeMs Time in milliseconds (0-1000ms)
     */
    void setPowerEngageTime(float timeMs);
    
    /**
     * @brief Set power release transition time
     * @param timeMs Time in milliseconds (0-1000ms)
     */
    void setPowerReleaseTime(float timeMs);
    
    /**
     * @brief Set crossover transition time (regen <-> power)
     * @param timeMs Time in milliseconds (0-1000ms)
     */
    void setCrossoverTime(float timeMs);
    
    /**
     * @brief Get current torque output (for monitoring/debugging)
     * @return Current smoothed torque output percentage
     */
    float getCurrentTorqueOutput() const { return currentTorqueOutput; }
    
    /**
     * @brief Get target torque from pedal (for monitoring/debugging)
     * @return Target torque percentage from pedal input
     */
    float getTargetTorqueFromPedal() const { return targetTorqueFromPedal; }
    
    /**
     * @brief Check if currently in a torque transition
     * @return true if transition is active
     */
    bool isInTorqueTransition() const { return currentTransition != TransitionType::NONE; }
    
    static constexpr float MAX_VEHICLE_SPEED = 120.0f;  // kph

private:
    /**
     * @brief Transition types for smooth torque control
     */
    enum class TransitionType {
        NONE,
        REGEN_ENGAGE,      ///< 0 -> negative torque (lift-off regen)
        REGEN_RELEASE,     ///< negative -> 0 torque (regen release)
        POWER_ENGAGE,      ///< 0 -> positive torque (acceleration)
        POWER_RELEASE,     ///< positive -> 0 torque (deceleration)
        REGEN_TO_POWER,    ///< negative -> positive (regen to accel)
        POWER_TO_REGEN     ///< positive -> negative (accel to regen)
    };

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
     * @brief Calculate immediate torque target from pedal input
     * @return Target torque percentage without transitions applied
     */
    float calculateImmediateTorqueFromPedal();
    
    /**
     * @brief Apply smooth torque transitions
     * @param targetTorque Target torque from pedal input
     * @return Smoothly transitioned torque
     */
    float applyTorqueTransition(float targetTorque);
    
    /**
     * @brief Detect what type of transition is occurring
     * @param current Current torque output
     * @param target Target torque from pedal
     * @return Detected transition type
     */
    TransitionType detectTransitionType(float current, float target);
    
    /**
     * @brief Get transition time for specific transition type
     * @param type Transition type
     * @return Transition time in milliseconds
     */
    float getTransitionTime(TransitionType type);
    
    /**
     * @brief Apply smooth interpolation curve (ease-in-out)
     * @param progress Transition progress (0.0 to 1.0)
     * @return Curved progress value
     */
    float applySmoothCurve(float progress);
    
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
     * @brief Apply simplified three-zone pedal mapping
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
    
    // Core member variables
    ADS1115& ads;                    ///< Reference to ADC
    DriveMode currentDrivingMode;    ///< Current driving mode
    GearState currentGear;           ///< Current gear state
    GearRatio currentGearRatio;      ///< Current gear ratio
    bool shiftAttempted;             ///< Track shift attempts at high speed
    
    bool enableDMC;                  ///< DMC enable flag
    bool wasInDeadband;              ///< Deadband hysteresis state
    bool wasEnabled;                 ///< Previous enable state
    
    // Gear transition state management
    bool isInGearTransition;         ///< Flag indicating gear change in progress
    unsigned long gearTransitionStartTime; ///< Timestamp for gear transition timing
    GearState previousGear;          ///< Track previous gear for transition detection
    
    // Basic torque tracking
    float lastTorquePercent;         ///< Last calculated torque percentage
    float filteredTorquePercent;     ///< Filtered torque percentage
    float motorSpeed;                ///< Current motor speed
    CANManager* canManager = nullptr; 
    
    // NEW: Smooth torque transition system
    float currentTorqueOutput;       ///< Actual torque being output
    float targetTorqueFromPedal;     ///< Target torque from pedal input
    unsigned long lastTransitionTime; ///< Last transition update time
    
    TransitionType currentTransition; ///< Current transition type
    float transitionStartTorque;     ///< Torque value when transition started
    unsigned long transitionStartTime; ///< Timestamp when transition started
    
    // Configurable transition times (milliseconds)
    float regenEngageTimeMs;         ///< Regen engagement time (0 -> negative torque)
    float regenReleaseTimeMs;        ///< Regen release time (negative -> 0 torque)
    float powerEngageTimeMs;         ///< Power engagement time (0 -> positive torque)
    float powerReleaseTimeMs;        ///< Power release time (positive -> 0 torque)
    float crossoverTimeMs;           ///< Crossover transition time (regen <-> power)
    
    // Validation constants
    static constexpr float MIN_TRANSITION_TIME = 0.0f;   ///< Minimum transition time
    static constexpr float MAX_TRANSITION_TIME = 1000.0f; ///< Maximum transition time
};