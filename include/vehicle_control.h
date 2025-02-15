/**
 * @file vehicle_control.h
 * @brief Vehicle Control System for Electric Vehicle
 * 
 * This class manages the core vehicle control logic including:
 * - Pedal interpretation and torque calculation
 * - Driving modes (Legacy, Regenerative, OPD)
 * - Motor control and gear management
 * - Vehicle speed and acceleration control
 */

#pragma once
#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include "config.h"
#include "vehicle_parameters.h"
#include "ADS1X15.h"

// Simple PID controller for OPD anti-rollback protection.
// (This class can be moved to a separate file if desired.)
class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), prevError_(0.0f), setpoint_(0.0f) {}

    /**
     * @brief Update the PID controller.
     * @param measurement Current measurement (e.g. vehicle speed in kph).
     * @param dt Time step in seconds.
     * @return PID output used as a torque command.
     */
    float update(float measurement, float dt) {
        float error = setpoint_ - measurement;
        integral_ += error * dt;
        float derivative = (error - prevError_) / dt;
        prevError_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }
    
    void setSetpoint(float sp) { setpoint_ = sp; }
    
private:
    float kp_, ki_, kd_;
    float integral_, prevError_;
    float setpoint_;
};

class VehicleControl {
public:
    /**
     * @brief Constructs the vehicle control system
     * @param ads Reference to ADS1115 ADC for pedal position reading
     */
    explicit VehicleControl(ADS1115& ads);
    
    /**
     * @brief Calculates motor torque based on current driving mode and conditions
     * @return Calculated torque demand in Nm (-850 to 850)
     */
    int16_t calculateTorque();

    /**
     * @brief Updates gear state based on switch inputs and speed
     * Handles gear selection with safety checks
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
     * @brief Checks if DMC (motor controller) is enabled
     * @return true if DMC is enabled
     */
    bool isDMCEnabled() const;
    
    // Configuration methods
    void setOPDEnabled(bool enabled) { isOPDEnabled = enabled; }
    void setRegenEnabled(bool enabled) { isRegenEnabled = enabled; }
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
    
    // Driving mode handlers
    int16_t handleLegacyMode(float throttlePosition);
    int16_t handleRegenMode(float throttlePosition, float speed);
    int16_t handleOPDMode(float throttlePosition, float speed);
    
    // (Optional) Additional functions for OPD calculations.
    float calculateCoastUpperBound(float speed);
    float calculateCoastLowerBound(float speed);
    float calculateMaxRegenerativeTorque(float speed);
    float calculateMaxDriveTorque(float speed);
    
    /**
     * @brief Applies rate limiting and maximum torque constraints
     * @param requestedTorque Raw calculated torque
     * @return Limited torque value
     */
    int16_t applyTorqueLimits(int16_t requestedTorque);

    /**
     * @brief Applies deadband hysteresis to prevent torque oscillation
     * @param torque Input torque value
     * @return Processed torque with hysteresis
     */
    int16_t applyDeadbandHysteresis(int16_t torque);
    
    // Member variables
    ADS1115& ads;                    // Reference to ADC
    DriveMode currentDrivingMode;    // Current driving mode
    GearState currentGear;           // Current gear state
    GearRatio currentGearRatio;      // Current gear ratio
    bool shiftAttempted;             // Track shift attempts at high speed
    
    bool isOPDEnabled;               // OPD mode flag
    bool isRegenEnabled;             // Regeneration enabled flag
    bool enableDMC;                  // DMC enable flag
    bool wasInDeadband;              // Deadband hysteresis state
    bool wasEnabled;                 // Previous enable state
    
    float lastTorque;                // Last calculated torque
    float motorSpeed;                // Current motor speed

    // PID controller for OPD anti-rollback protection.
    // This member is used to hold the vehicle when speed is near zero.
    PIDController opdPid;
};