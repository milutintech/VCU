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

/**
 * @brief Constructor - initializes vehicle control system
 * @param ads Reference to ADC for pedal position reading
 * 
 * Initializes control system with safe defaults:
 * - REGEN drive mode enabled
 * - Zero torque output
 * - Safety systems active
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
{
}

/**
 * @brief Calculate motor torque demand based on driving mode and conditions
 * @return Calculated torque demand in Nm (range: -850 to 850)
 * 
 * Main control flow:
 * 1. Sample pedal position
 * 2. Apply gamma correction
 * 3. Process based on driving mode
 * 4. Apply safety limits
 * 5. Handle deadband
 */
int16_t VehicleControl::calculateTorque() {
    // Sample pedal position
    int32_t sampledPotiValue = samplePedalPosition();
    
    // Map raw pedal value to 0-100%
    float rawThrottle = map(sampledPotiValue, ADC::MinValPot, ADC::MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Quick exit for legacy mode with released pedal
    if (!isOPDEnabled && !isRegenEnabled && rawThrottle < 1.0f) {
        lastTorque = 0;
        enableDMC = false;
        return 0;
    }
    
    // Calculate throttle position with gamma correction
    float throttlePosition = pow(rawThrottle / 100.0f, VehicleParams::Control::PEDAL_GAMMA) * 100.0f;
    
    // Update reverse light based on gear state
    digitalWrite(Pins::BCKLIGHT, currentGear == GearState::REVERSE ? HIGH : LOW);

    // Handle neutral gear
    if (currentGear == GearState::NEUTRAL) {
        lastTorque = 0;
        digitalWrite(Pins::BCKLIGHT, LOW);  // Ensure reverse light is off in neutral
        return 0;
    }
    

    // Calculate vehicle speed
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
    
    // Apply torque rate limiting
    calculatedTorque = applyTorqueLimits(calculatedTorque);
    
    // Apply deadband with hysteresis
    calculatedTorque = applyDeadbandHysteresis(calculatedTorque);
    
    return calculatedTorque;
}

/**
 * @brief Sample pedal position from ADC
 * @return Averaged ADC reading for pedal position
 * 
 * Takes 4 samples and averages them to reduce noise.
 * Uses dedicated ADC channel for pedal position.
 */
int32_t VehicleControl::samplePedalPosition() {
    int32_t total = 0;
    for (int i = 0; i < 4; i++) {
        total += ads.readADC(ADC::GASPEDAL1);
    }
    return total / 4;
}

/**
 * @brief Calculate current vehicle speed from motor speed
 * @return Vehicle speed in kph
 * 
 * Considers:
 * - Current gear ratio
 * - Differential ratio
 * - Wheel circumference
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
 * @brief Handle Legacy driving mode
 * @param throttlePosition Processed pedal position (0-100%)
 * @return Calculated torque for legacy mode
 * 
 * Simple linear torque mapping based on pedal position.
 * Direction depends on selected gear.
 */
int16_t VehicleControl::handleLegacyMode(float throttlePosition) {
    float normalizedThrottle = pow(throttlePosition / 100.0f, VehicleParams::Control::PEDAL_GAMMA);
    return normalizedThrottle * (currentGear == GearState::DRIVE ? 
           -VehicleParams::Motor::MAX_TRQ : VehicleParams::Motor::MAX_REVERSE_TRQ);
}

/**
 * @brief Handle Regenerative driving mode
 * @param throttlePosition Processed pedal position (0-100%)
 * @param speed Current vehicle speed in kph
 * @return Calculated torque for regen mode
 * 
 * Implements three zones:
 * 1. Regenerative braking (low pedal)
 * 2. Coast (mid pedal)
 * 3. Acceleration (high pedal)
 */
int16_t VehicleControl::handleRegenMode(float throttlePosition, float speed) {
    if (currentGear == GearState::DRIVE) {
        if (abs(motorSpeed) < VehicleParams::Regen::ZERO_SPEED) {
            // At standstill
            if (throttlePosition > VehicleParams::Regen::COAST_END) {
                float accelFactor = (throttlePosition - VehicleParams::Regen::COAST_END) / 
                                  (100.0f - VehicleParams::Regen::COAST_END);
                return -accelFactor * VehicleParams::Motor::MAX_TRQ;
            }
            return 0;
        }
        else if (motorSpeed < -VehicleParams::Regen::MIN_SPEED) {
            // Forward motion
            if (throttlePosition < VehicleParams::Regen::END_POINT) {
                // Regen zone
                float regenFactor = 1.0f - (throttlePosition / VehicleParams::Regen::END_POINT);
                return regenFactor * VehicleParams::Motor::MAX_REQ_TRQ;
            }
            else if (throttlePosition < VehicleParams::Regen::COAST_END) {
                // Coast zone
                return 0;
            }
            else {
                // Acceleration zone
                float accelFactor = (throttlePosition - VehicleParams::Regen::COAST_END) / 
                                  (100.0f - VehicleParams::Regen::COAST_END);
                return -accelFactor * VehicleParams::Motor::MAX_TRQ;
            }
        }
    }
    return 0;
}

/**
 * @brief Handle One Pedal Drive mode
 * @param throttlePosition Processed pedal position (0-100%)
 * @param speed Current vehicle speed in kph
 * @return Calculated torque for OPD mode
 * 
 * Features:
 * - Speed-dependent pedal mapping
 * - Built-in regenerative braking
 * - Anti-rollback protection
 * - Dynamic torque limiting
 */
int16_t VehicleControl::handleOPDMode(float throttlePosition, float speed) {
    // Speed limit enforcement
    if (speed >= VehicleParams::OPD::MAX_SPEED) {
        // Only allow regen/negative torque at max speed
        float coastUpper = VehicleParams::OPD::PHI;
        if (throttlePosition <= coastUpper) {
            float normalizedPosition = throttlePosition / coastUpper;
            float regenTorque = VehicleParams::OPD::MAX_REGEN * 
                               (VehicleParams::Motor::MAX_TRQ / 100.0f) * 
                               (1.0f - normalizedPosition);
            digitalWrite(Pins::BCKLIGHT, regenTorque > VehicleParams::OPD::BRAKE_LIGHT_THRESHOLD ? HIGH : LOW);
            return currentGear == GearState::DRIVE ? regenTorque : -regenTorque;
        }
        return 0;
    }

    const float speedPercent = constrain(abs(speed) / VehicleParams::OPD::MAX_SPEED, 0.0f, 1.0f);
    
    // Calculate coast boundaries
    float coastUpper = VehicleParams::OPD::PHI * pow(speedPercent, 1.0f/VehicleParams::OPD::SHAPE_FACTOR);
    float coastLower = coastUpper - VehicleParams::OPD::COAST_RANGE * speedPercent;
    
    // Anti-rollback protection
    if (speed < VehicleParams::OPD::ROLLBACK_SPEED && throttlePosition < coastLower) {
        float rollbackTorque = VehicleParams::OPD::ROLLBACK_TORQUE * 
                              (VehicleParams::Motor::MAX_TRQ / 100.0f);
        return currentGear == GearState::DRIVE ? -rollbackTorque : rollbackTorque;
    }
    
    // Coast zone handling
    if (throttlePosition >= coastLower && throttlePosition <= coastUpper) {
        return 0;
    }
    
    if (throttlePosition > coastUpper) {
        // Acceleration with speed-based limiting
        float normalizedPosition = (throttlePosition - coastUpper) / (100.0f - coastUpper);
        float maxTorque = VehicleParams::Motor::MAX_TRQ;
        
        // Progressively reduce torque as we approach max speed
        float speedFactor = 1.0f - pow(speedPercent, 2.0f);
        
        if (speed < 20.0f) {
            maxTorque = maxTorque * (0.8f + (20.0f - speed) / 100.0f);
        }
        
        float torque = maxTorque * speedFactor * pow(normalizedPosition, 1.5f);
        return currentGear == GearState::DRIVE ? -torque : torque;
    } else {
        // Regenerative braking
        float normalizedPosition = throttlePosition / coastLower;
        float maxRegen = VehicleParams::OPD::MAX_REGEN * 
                        (VehicleParams::Motor::MAX_TRQ / 100.0f);
        
        if (speed > 60.0f) {
            maxRegen *= (1.0f - (speed - 60.0f) * 0.005f);
        }
        
        float regenTorque = maxRegen * (1.0f - normalizedPosition);
        digitalWrite(Pins::BCKLIGHT, regenTorque > VehicleParams::OPD::BRAKE_LIGHT_THRESHOLD ? HIGH : LOW);
        
        return currentGear == GearState::DRIVE ? regenTorque : -regenTorque;
    }
}

/**
 * @brief Apply rate limiting to torque changes
 * @param requestedTorque Raw calculated torque
 * @return Rate-limited torque value
 * 
 * Implements different limits for:
 * - Acceleration (slower)
 * - Deceleration (faster)
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
 * @brief Apply deadband hysteresis to torque output
 * @param torque Input torque value
 * @return Processed torque with deadband hysteresis
 * 
 * Prevents oscillation around zero torque:
 * - Higher threshold to exit deadband
 * - Lower threshold to enter deadband
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
 * @brief Update current motor speed
 * @param speed New motor speed in RPM
 */
void VehicleControl::setMotorSpeed(float speed) {
    motorSpeed = speed;
}

/**
 * @brief Set current gear state
 * @param gear New gear state (DRIVE/NEUTRAL/REVERSE)
 */
void VehicleControl::setCurrentGear(GearState gear) {
    currentGear = gear;
}

/**
 * @brief Set current driving mode
 * @param mode New driving mode (LEGACY/REGEN/OPD)
 */
void VehicleControl::setDrivingMode(DriveMode mode) {
    currentDrivingMode = mode;
}

/**
 * @brief Check if motor controller is enabled
 * @return true if DMC is enabled
 */
bool VehicleControl::isDMCEnabled() const {
    return enableDMC;
}

void VehicleControl::updateGearState() {
    int32_t forwardValue = ads.readADC(2);  // Drive switch on A2
    int32_t reverseValue = ads.readADC(3);  // Reverse switch on A3
    
    bool isForwardHigh = forwardValue > 200;
    bool isReverseHigh = reverseValue > 200;
    
    // Handle neutral selection - always allowed
    if (!isForwardHigh && !isReverseHigh) {
        currentGear = GearState::NEUTRAL;
        shiftAttempted = false;
        return;
    }

    // Handle transitions at low speed
    if (abs(motorSpeed) < VehicleParams::Transmission::RPM_SHIFT_THRESHOLD) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = GearState::DRIVE;
            shiftAttempted = false;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = GearState::REVERSE;
            shiftAttempted = false;
        }
    } else {
        // At high speed, prevent switching between drive and reverse
        if (isForwardHigh && currentGear == GearState::REVERSE) {
            shiftAttempted = true;
            // Stay in current gear until speed drops
        } else if (isReverseHigh && currentGear == GearState::DRIVE) {
            shiftAttempted = true;
            // Stay in current gear until speed drops
        }
    }
    
    // Allow reengaging desired gear when speed drops
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