/**
 * @file vehicle_control.cpp
 * @brief Implementation of vehicle driving dynamics and motor control system
 * 
 * Manages vehicle driving characteristics including:
 * - Pedal interpretation and mapping
 * - Torque calculation and limiting
 * - Multiple driving modes (Legacy, Regen, OPD)
 * - Speed monitoring and control
 * - Safety checks and limits
 */

#include "vehicle_control.h"
#include "config.h"
#include "ADS1X15.h"

//-------------------------------
// VehicleControl Class Implementation
//-------------------------------

/**
 * @brief Constructor - initializes vehicle control system.
 * 
 * Initializes control system with safe defaults and creates the PID controller
 * for OPD anti-rollback protection. (Tuning of the PID gains may be required.)
 */
VehicleControl::VehicleControl(ADS1115& ads) 
    : ads(ads)
    , currentDrivingMode(DriveMode::REGEN)
    , currentGear(GearState::NEUTRAL)
    , currentGearRatio(GearRatio::NORMAL)
    , shiftAttempted(false)
    , isOPDEnabled(false)
    , isRegenEnabled(true)
    , enableDMC(false)
    , wasInDeadband(false)
    , wasEnabled(false)
    , lastTorque(0)
    , motorSpeed(0)
    // Initialize the OPD anti-rollback PID controller with example gains
    , opdPid(100.0f, 0.0f, 10.0f)
{
}

/**
 * @brief Calculate motor torque demand based on driving mode and conditions.
 * @return Calculated torque demand in Nm (range: -850 to 850).
 * 
 * Main control flow:
 * 1. Sample pedal position.
 * 2. Map raw pedal value (with gamma correction).
 * 3. Select processing based on the driving mode.
 * 4. Apply safety limits and deadband.
 */
int16_t VehicleControl::calculateTorque() {
    // Sample pedal position.
    int32_t sampledPotiValue = samplePedalPosition();
    
    // Map raw pedal value to 0-100%.
    float rawThrottle = map(sampledPotiValue, ADC::MinValPot, ADC::MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Quick exit for legacy mode with a released pedal.
    if (!isOPDEnabled && !isRegenEnabled && rawThrottle < 1.0f) {
        lastTorque = 0;
        enableDMC = false;
        return 0;
    }
    
    // Calculate throttle position with gamma correction.
    float throttlePosition = pow(rawThrottle / 100.0f, VehicleParams::Control::PEDAL_GAMMA) * 100.0f;
    
    // Update reverse light based on gear state.
    digitalWrite(Pins::BCKLIGHT, currentGear == GearState::REVERSE ? HIGH : LOW);

    // Handle neutral gear.
    if (currentGear == GearState::NEUTRAL) {
        lastTorque = 0;
        digitalWrite(Pins::BCKLIGHT, LOW);  // Ensure reverse light is off in neutral.
        return 0;
    }
    
    // Calculate vehicle speed (in kph).
    float speed = calculateVehicleSpeed();
    
    int16_t calculatedTorque = 0;
    
    switch(currentDrivingMode) {
        case DriveMode::LEGACY:
            calculatedTorque = handleLegacyMode(throttlePosition);
            break;
            
        case DriveMode::REGEN:
            calculatedTorque = handleRegenMode(throttlePosition, speed);
            break;
            
        case DriveMode::OPD:
            calculatedTorque = handleOPDMode(throttlePosition, speed);
            break;
    }
    
    // Apply torque rate limiting.
    calculatedTorque = applyTorqueLimits(calculatedTorque);
    
    // Apply deadband with hysteresis.
    calculatedTorque = applyDeadbandHysteresis(calculatedTorque);

    //Makes sure no reverse driving posible in drive.
    calculatedTorque = applyTorqueCutoff(calculatedTorque);

    
    return calculatedTorque;
}

/**
 * @brief Sample pedal position from ADC.
 * @return Averaged ADC reading for pedal position.
 * 
 * Takes 4 samples and averages them to reduce noise.
 */
int32_t VehicleControl::samplePedalPosition() {
    int32_t total = 0;
    for (int i = 0; i < 4; i++) {
        total += ads.readADC(ADC::GASPEDAL1);
    }
    return total / 4;
}

/**
 * @brief Calculate current vehicle speed from motor speed.
 * @return Vehicle speed in kph.
 * 
 * Considers:
 * - Current gear ratio.
 * - Differential ratio.
 * - Wheel circumference.
 */
float VehicleControl::calculateVehicleSpeed() {
    float ratio = (currentGearRatio == GearRatio::REDUCED) ? 
                  VehicleParams::Transmission::REDUCED_RATIO : 
                  VehicleParams::Transmission::NORMAL_RATIO;
                  
    return motorSpeed * 60.0f / ratio / 
           VehicleParams::Transmission::DIFF_RATIO * 
           VehicleParams::Transmission::WHEEL_CIRC;
}

/**
 * @brief Handle Legacy driving mode.
 * @param throttlePosition Processed pedal position (0-100%).
 * @return Calculated torque for legacy mode.
 * 
 * Uses a simple linear mapping based on pedal position.
 */
int16_t VehicleControl::handleLegacyMode(float throttlePosition) {
    float normalizedThrottle = pow(throttlePosition / 100.0f, VehicleParams::Control::PEDAL_GAMMA);
    return normalizedThrottle * (currentGear == GearState::DRIVE ? 
           -VehicleParams::Motor::MAX_TRQ : VehicleParams::Motor::MAX_REVERSE_TRQ);
}

/**
 * @brief Handle Regenerative driving mode.
 * @param throttlePosition Processed pedal position (0-100%).
 * @param speed Current vehicle speed in kph.
 * @return Calculated torque for regen mode.
 */
int16_t VehicleControl::handleRegenMode(float throttlePosition, float speed) {
    // Define key points in the throttle range
    const float REGEN_END = VehicleParams::Regen::END_POINT;     // Max regen threshold (e.g., 20% throttle)
    const float COAST_END = VehicleParams::Regen::COAST_END;     // Start of acceleration (e.g., 30% throttle)
    const float ZERO_SPEED_DEADBAND = VehicleParams::Regen::ZERO_SPEED;  // Speed deadband (e.g., 1 kph)

    // If in neutral, no torque should be applied
    if (currentGear == GearState::NEUTRAL) {
        return 0;
    }

    // Standstill deadband: No regen or acceleration when in deadband range
    if (fabs(speed) < ZERO_SPEED_DEADBAND) {
        return 0;
    }

    // ** Regenerative Braking Zone (Throttle 0% - REGEN_END%) **
    if (throttlePosition < REGEN_END) {
        float regenFactor = 1.0f - (throttlePosition / REGEN_END);
        return regenFactor * VehicleParams::Motor::MAX_REQ_TRQ;  // Regen torque (positive value)
    }

    // ** Coast Zone (Throttle REGEN_END% - COAST_END%) **
    if (throttlePosition < COAST_END) {
        return 0;  // No torque (coasting)
    }

    // ** Acceleration Zone (Throttle > COAST_END%) **
    float accelFactor = (throttlePosition - COAST_END) / (100.0f - COAST_END);
    return -accelFactor * VehicleParams::Motor::MAX_TRQ;  // Negative torque for forward acceleration
}

/**
 * @brief Handle One Pedal Drive (OPD) mode with PID anti-rollback and torque capping.
 * @param throttlePosition Processed pedal position (0-100%).
 * @param speed Current vehicle speed in kph.
 * @return Calculated torque for OPD mode.
 * 
 * Features:
 * - Speed-dependent pedal mapping.
 * - Built-in regenerative braking.
 * - Anti-rollback protection: when speed is very low, a PID controller holds the vehicle at 0 kph.
 * - Dynamic torque limiting and a regen torque cap for slow speeds so that, in the worst case,
 *   the vehicle only accelerates slowly backward.
 */
int16_t VehicleControl::handleOPDMode(float throttlePosition, float speed) {
    float absSpeed = fabs(speed);
    
    // Near-zero speed: if vehicle is nearly stopped, use PID control for anti-rollback.
    if (absSpeed < VehicleParams::OPD::ZERO_SPEED_THRESHOLD) {
        float pidOutput = opdPid.update(speed, VehicleParams::Control::CONTROL_DT);
        return pidOutput;
    }
    
    // Speed percentage relative to max OPD speed.
    float speedPercent = constrain(absSpeed / VehicleParams::OPD::MAX_SPEED, 0.0f, 1.0f);
    
    // Improved coast zone boundaries.
    float coastUpper = VehicleParams::OPD::PHI * pow(speedPercent, 1.0f / VehicleParams::OPD::SHAPE_FACTOR);
    float coastLower = coastUpper - VehicleParams::OPD::COAST_RANGE * speedPercent;
    
    // Ensure reasonable coast range.
    coastUpper = max(5.0f, coastUpper);
    coastLower = max(2.0f, coastLower);
    
    // Coast logic.
    if (throttlePosition >= coastLower && throttlePosition <= coastUpper) {
        return 0;
    }
    
    // Acceleration logic with proper throttle scaling.
    if (throttlePosition > coastUpper) {
        float normalizedPosition = (throttlePosition - coastUpper) / (100.0f - coastUpper);
        float maxTorque = VehicleParams::Motor::MAX_TRQ;

        // No low-speed torque reduction
        float torque = maxTorque * pow(normalizedPosition, 1.5f);

        return (currentGear == GearState::DRIVE) ? -torque : torque;
    } 
    
    // Regen logic remains unchanged.
    float normalizedPosition = throttlePosition / coastLower;
    float maxRegen = VehicleParams::OPD::MAX_REGEN * (VehicleParams::Motor::MAX_TRQ / 100.0f);
    
    return (currentGear == GearState::DRIVE) ? maxRegen * (1.0f - normalizedPosition) : -maxRegen * (1.0f - normalizedPosition);
}
/* OLD TRY
int16_t VehicleControl::handleOPDMode(float throttlePosition, float speed) {
    float absSpeed = fabs(speed);
    
    // Near-zero speed: if vehicle is nearly stopped, use PID control for anti-rollback.
    if (absSpeed < VehicleParams::OPD::ZERO_SPEED_THRESHOLD) {
        float pidOutput = opdPid.update(speed, VehicleParams::Control::CONTROL_DT);
        return pidOutput;
    }
    
    // Speed percentage relative to max OPD speed.
    float speedPercent = constrain(absSpeed / VehicleParams::OPD::MAX_SPEED, 0.0f, 1.0f);
    
    // Improved coast zone boundaries.
    float coastUpper = VehicleParams::OPD::PHI * pow(speedPercent, 1.0f / VehicleParams::OPD::SHAPE_FACTOR);
    float coastLower = coastUpper - VehicleParams::OPD::COAST_RANGE * speedPercent;
    
    // Ensure reasonable coast range.
    coastUpper = max(5.0f, coastUpper);
    coastLower = max(2.0f, coastLower);
    
    // Coast logic.
    if (throttlePosition >= coastLower && throttlePosition <= coastUpper) {
        return 0;
    }
    
    // Acceleration logic with proper throttle scaling.
    if (throttlePosition > coastUpper) {
        float normalizedPosition = (throttlePosition - coastUpper) / (100.0f - coastUpper);
        float maxTorque = VehicleParams::Motor::MAX_TRQ;

        // No low-speed torque reduction
        float torque = maxTorque * pow(normalizedPosition, 1.5f);

        return (currentGear == GearState::DRIVE) ? -torque : torque;
    } 
    
    // Regen logic remains unchanged.
    float normalizedPosition = throttlePosition / coastLower;
    float maxRegen = VehicleParams::OPD::MAX_REGEN * (VehicleParams::Motor::MAX_TRQ / 100.0f);
    
    return (currentGear == GearState::DRIVE) ? maxRegen * (1.0f - normalizedPosition) : -maxRegen * (1.0f - normalizedPosition);
}
*/

/**
 * @brief Apply rate limiting to torque changes.
 * @param requestedTorque Raw calculated torque.
 * @return Rate-limited torque value.
 * 
 * Implements different limits for acceleration and deceleration.
 */
int16_t VehicleControl::applyTorqueLimits(int16_t requestedTorque) {
    float torqueDiff = requestedTorque - lastTorque;
    
    if (abs(requestedTorque) > abs(lastTorque)) {
        torqueDiff = constrain(torqueDiff, 
                             -VehicleParams::Motor::MAX_DECEL_STEP,
                             VehicleParams::Motor::MAX_ACCEL_STEP);
    } else {
        torqueDiff = constrain(torqueDiff, 
                             -VehicleParams::Motor::MAX_DECEL_STEP,
                             VehicleParams::Motor::MAX_DECEL_STEP);
    }
    
    lastTorque = lastTorque + torqueDiff;
    return lastTorque;
}

/**
 * @brief Apply torque cutoff based on RPM, direction, and gear state.
 * @param requestedTorque The raw calculated torque.
 * @return Modified torque after applying directional restrictions.
 */
int16_t VehicleControl::applyTorqueCutoff(int16_t requestedTorque) {
    if (currentGear == GearState::DRIVE) {
        // Prevent positive torque (forward acceleration) if at 0 RPM or in reverse direction
        if (motorSpeed >= 0 && requestedTorque > 0) {
            return 0;
        }
    } else if (currentGear == GearState::REVERSE) {
        // Prevent negative torque (reverse acceleration) if at 0 RPM or moving forward
        if (motorSpeed <= 0 && requestedTorque < 0) {
            return 0;
        }
    }
    return requestedTorque;
}

/**
 * @brief Apply deadband hysteresis to torque output.
 * @param torque Input torque value.
 * @return Processed torque with deadband hysteresis.
 * 
 * Prevents oscillation around zero torque:
 * - Uses a higher threshold to exit the deadband and a lower threshold to enter.
 */
int16_t VehicleControl::applyDeadbandHysteresis(int16_t torque) {
    if (wasInDeadband) {
        if (abs(torque) > VehicleParams::Motor::TORQUE_DEADBAND_HIGH) {
            wasInDeadband = false;
            enableDMC = true;
        } else {
            torque = 0;
            enableDMC = true;
        }
    } else {
        if (abs(torque) < VehicleParams::Motor::TORQUE_DEADBAND_LOW) {
            wasInDeadband = true;
            torque = 0;
            enableDMC = true;
        } else {
            enableDMC = true;
        }
    }
    
    return torque;
}

/**
 * @brief Update current motor speed.
 * @param speed New motor speed in RPM.
 */
void VehicleControl::setMotorSpeed(float speed) {
    motorSpeed = speed;
}

/**
 * @brief Set current gear state.
 * @param gear New gear state (DRIVE/NEUTRAL/REVERSE).
 */
void VehicleControl::setCurrentGear(GearState gear) {
    currentGear = gear;
}

/**
 * @brief Set current driving mode.
 * @param mode New driving mode (LEGACY/REGEN/OPD).
 */
void VehicleControl::setDrivingMode(DriveMode mode) {
    currentDrivingMode = mode;
}

/**
 * @brief Check if the motor controller is enabled.
 * @return true if DMC is enabled.
 */
bool VehicleControl::isDMCEnabled() const {
    return enableDMC;
}

/**
 * @brief Update the gear state based on switch inputs and speed.
 */
void VehicleControl::updateGearState() {
    int32_t forwardValue = ads.readADC(2);  // Drive switch on A2.
    int32_t reverseValue = ads.readADC(3);  // Reverse switch on A3.
    
    bool isForwardHigh = forwardValue > 200;
    bool isReverseHigh = reverseValue > 200;
    
    // Handle neutral selection – always allowed.
    if (!isForwardHigh && !isReverseHigh) {
        currentGear = GearState::NEUTRAL;
        shiftAttempted = false;
        return;
    }

    // Handle transitions at low speed.
    if (abs(motorSpeed) < VehicleParams::Transmission::RPM_SHIFT_THRESHOLD) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = GearState::DRIVE;
            shiftAttempted = false;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = GearState::REVERSE;
            shiftAttempted = false;
        }
    } else {
        // At high speed, prevent switching between drive and reverse.
        if (isForwardHigh && currentGear == GearState::REVERSE) {
            shiftAttempted = true;
            // Stay in current gear until speed drops.
        } else if (isReverseHigh && currentGear == GearState::DRIVE) {
            shiftAttempted = true;
            // Stay in current gear until speed drops.
        }
    }
    
    // Allow reengaging desired gear when speed drops.
    if (shiftAttempted && abs(motorSpeed) < VehicleParams::Transmission::RPM_SHIFT_THRESHOLD) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = GearState::DRIVE;
            shiftAttempted = false;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = GearState::REVERSE;
            shiftAttempted = false;
        }
    }
}