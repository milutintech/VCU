/**
 * @file vehicle_control.cpp
 * @brief Implementation of Curtis-style neutral braking with configurable power limiting
 * 
 * Features:
 * - Curtis-style neutral braking for immediate throttle response
 * - Speed-based power limiting with configurable curves (1,2,4,8 delta zones)
 * - Smooth torque baseline tracking
 * - Full throttle range utilization (no clamping)
 * - Configurable driving characteristics
 */

#include "vehicle_control.h"
#include "can_manager.h" 
#include "config.h"
#include "ADS1X15.h"
#include "configuration.h"

/**
 * @brief Constructor - initializes vehicle control system with Curtis-style control
 */
VehicleControl::VehicleControl(ADS1115& ads) 
    : ads(ads)
    , currentDrivingMode(DriveMode::REGEN)
    , currentGear(GearState::NEUTRAL)
    , currentGearRatio(GearRatio::NORMAL)
    , shiftAttempted(false)
    , enableDMC(false)
    , wasInDeadband(false)
    , wasEnabled(false)
    , isInGearTransition(false)
    , gearTransitionStartTime(0)
    , previousGear(GearState::NEUTRAL)
    , lastTorquePercent(0.0f)
    , filteredTorquePercent(0.0f)
    , motorSpeed(0.0f)
    , torqueBaseline(0.0f)
    , lastBaselineUpdate(0)
    , previousThrottlePercent(0.0f)
    , neutralBrakingActive(false)
    , currentPowerLimit(100.0f)
    , previousPowerLimit(100.0f)
    , lastPowerUpdate(0)
{
}

/**
 * @brief Calculate motor torque percentage using Curtis-style neutral braking
 * @return Calculated torque percentage (-100% to +100%)
 * 
 * Curtis Logic Flow:
 * 1. Sample and map throttle position 
 * 2. Calculate current power limit based on motor speed
 * 3. Apply Curtis neutral braking logic
 * 4. Update torque baseline for next iteration
 * 5. Apply gear direction and safety limits
 */
float VehicleControl::calculateTorquePercentage() {
    // Sample pedal position
    int32_t sampledPotiValue = samplePedalPosition();
    
    // Map using correct ADC values from config.h (0-100%)
    float rawThrottle = map(sampledPotiValue, ADC::MinValPot, ADC::MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Apply gamma correction for more natural pedal feel
    float throttlePosition = pow(rawThrottle / 100.0f, VehicleParams::Pedal::GAMMA) * 100.0f;
    
    // Debug output
    Serial.printf("ADC: %d -> %.1f%% throttle", sampledPotiValue, throttlePosition);
    
    // Update reverse light based on gear state
    digitalWrite(Pins::BCKLIGHT, currentGear == GearState::REVERSE ? HIGH : LOW);
    digitalWrite(19, currentGear == GearState::REVERSE ? HIGH : LOW);

    // Handle neutral gear - always zero torque
    if (currentGear == GearState::NEUTRAL) {
        torqueBaseline = 0.0f;  // Reset baseline in neutral
        lastTorquePercent = 0.0f;
        filteredTorquePercent = 0.0f;
        enableDMC = false;
        digitalWrite(19, LOW);  
        digitalWrite(Pins::BCKLIGHT, LOW);
        Serial.println(" -> NEUTRAL: 0%");
        return 0.0f;
    }
    
    // Calculate current power limit based on motor speed
    bool isDriving = (currentGear == GearState::DRIVE) ? (throttlePosition > previousThrottlePercent) : 
                     (throttlePosition > previousThrottlePercent);
    float powerLimit = calculatePowerLimit(abs(motorSpeed), isDriving);
    
    // Apply Curtis neutral braking logic
    float calculatedTorquePercent = applyCurtisNeutralBraking(throttlePosition, powerLimit);
    
    // Apply proper direction based on gear state
    if (currentGear == GearState::DRIVE) {
        calculatedTorquePercent = -calculatedTorquePercent;  // Negative = forward in drive
    }
    // In REVERSE, positive torque = reverse motion (no sign change needed)
    
    // Apply gear transition protection
    calculatedTorquePercent = applyGearTransitionProtection(calculatedTorquePercent);
    
    // Apply deadband hysteresis
    calculatedTorquePercent = applyDeadbandHysteresis(calculatedTorquePercent);
    
    // Update DMC enable logic
    enableDMC = (abs(calculatedTorquePercent) > 0.5f);
    
    // Update torque baseline for next iteration
    updateTorqueBaseline(calculatedTorquePercent);
    
    // Store for next iteration
    lastTorquePercent = calculatedTorquePercent;
    filteredTorquePercent = calculatedTorquePercent;
    previousThrottlePercent = throttlePosition;
    
    Serial.printf(" -> Power Limit: %.1f%%, Baseline: %.1f%%, Final: %.1f%% (DMC: %s)\n", 
                  powerLimit, torqueBaseline, calculatedTorquePercent, enableDMC ? "ON" : "OFF");
    
    return calculatedTorquePercent;
}

/**
 * @brief Calculate power limit based on current motor speed using Curtis curves
 * @param motorSpeed Current motor speed in RPM
 * @param isDriving true for drive power limits, false for regen limits
 * @return Power limit percentage (0-120%)
 */
float VehicleControl::calculatePowerLimit(float motorSpeed, bool isDriving) {
    const float* powerLimits = isDriving ? config.getDrivePowerLimits() : config.getRegenPowerLimits();
    return interpolatePowerLimit(motorSpeed, powerLimits);
}

/**
 * @brief Interpolate power limit between Curtis speed zones
 * @param motorSpeed Current motor speed in RPM
 * @param powerLimits Array of power limits for the 5 zones
 * @return Interpolated power limit percentage
 */
float VehicleControl::interpolatePowerLimit(float motorSpeed, const float* powerLimits) {
    float baseSpeed = config.getBaseSpeed();
    float deltaSpeed = config.getDeltaSpeed();
    
    // Curtis speed zones: Base+1×Δ, Base+2×Δ, Base+4×Δ, Base+8×Δ
    float speedZones[5] = {
        baseSpeed + 1 * deltaSpeed,  // Zone 0 boundary
        baseSpeed + 2 * deltaSpeed,  // Zone 1 boundary  
        baseSpeed + 4 * deltaSpeed,  // Zone 2 boundary
        baseSpeed + 8 * deltaSpeed,  // Zone 3 boundary
        999999.0f                    // Zone 4 (no upper limit)
    };
    
    // Find which zone we're in and interpolate
    for (int i = 0; i < 5; i++) {
        if (motorSpeed <= speedZones[i]) {
            if (i == 0) {
                // Below first zone boundary - use nominal power
                return powerLimits[0];
            }
            
            // Linear interpolation between zones
            float prevSpeed = (i == 1) ? 0.0f : speedZones[i-2];
            float zoneProgress = (motorSpeed - prevSpeed) / (speedZones[i] - prevSpeed);
            zoneProgress = constrain(zoneProgress, 0.0f, 1.0f);
            
            return powerLimits[i-1] + (powerLimits[i] - powerLimits[i-1]) * zoneProgress;
        }
    }
    
    // Above all zones - use final zone power
    return powerLimits[4];
}

/**
 * @brief Apply Curtis-style neutral braking logic
 * @param throttlePercent Raw throttle position (0-100%)
 * @param powerLimit Current power limit based on speed
 * @return Calculated torque percentage with neutral braking applied
 */
float VehicleControl::applyCurtisNeutralBraking(float throttlePercent, float powerLimit) {
    // Scale throttle to use full available power range (NO CLAMPING!)
    float scaledThrottlePercent = (throttlePercent / 100.0f) * powerLimit;
    
    // Convert to torque percentage (relative to max possible torque)
    float maxPossibleTorquePercent = (float)config.getMaxTorque() / (float)VehicleParams::Motor::MAX_TRQ * 100.0f;
    float requestedTorquePercent = (scaledThrottlePercent / 100.0f) * maxPossibleTorquePercent;
    
    // Curtis Neutral Braking Logic:
    // Calculate difference from current baseline (this is the magic!)
    float torqueDelta = requestedTorquePercent - torqueBaseline;
    
    // Apply neutral braking characteristics
    if (torqueDelta < 0) {
        // Requesting less torque than baseline = REGEN
        neutralBrakingActive = true;
        
        // Apply regen multiplier for stronger braking feel
        float regenTorque = torqueDelta * config.getRegenMultiplier();
        
        // Limit regen based on current power limits
        float maxRegenPercent = (powerLimit / 100.0f) * maxPossibleTorquePercent;
        regenTorque = constrain(regenTorque, -maxRegenPercent, 0.0f);
        
        return torqueBaseline + regenTorque;
    } else {
        // Requesting more torque than baseline = ACCELERATION
        neutralBrakingActive = false;
        
        // Limit acceleration based on current power limits
        float maxAccelPercent = (powerLimit / 100.0f) * maxPossibleTorquePercent;
        float finalTorque = constrain(requestedTorquePercent, 0.0f, maxAccelPercent);
        
        return finalTorque;
    }
}

/**
 * @brief Update torque baseline for neutral braking
 * @param currentTorque Current actual torque demand
 * 
 * The baseline tracks recent torque history to create the "neutral point"
 * that makes any throttle reduction feel like immediate braking.
 */
void VehicleControl::updateTorqueBaseline(float currentTorque) {
    unsigned long currentTime = millis();
    
    // Skip updates if too frequent (minimum 10ms between updates)
    if (currentTime - lastBaselineUpdate < 10) {
        return;
    }
    
    lastBaselineUpdate = currentTime;
    
    // Curtis-style baseline tracking
    float updateRate = config.getBaselineUpdateRate();
    float decayRate = config.getBaselineDecayRate();
    
    if (abs(currentTorque) > 1.0f) {
        // Active torque demand - baseline follows current torque
        torqueBaseline = (updateRate * currentTorque) + ((1.0f - updateRate) * torqueBaseline);
    } else {
        // Low/zero torque - baseline decays toward zero for natural coasting
        torqueBaseline *= decayRate;
        
        // Prevent baseline from getting stuck at very small values
        if (abs(torqueBaseline) < 0.5f) {
            torqueBaseline = 0.0f;
        }
    }
    
    // Safety limits - prevent baseline from exceeding reasonable bounds
    float maxBaselinePercent = (float)config.getMaxTorque() / (float)VehicleParams::Motor::MAX_TRQ * 100.0f;
    torqueBaseline = constrain(torqueBaseline, -maxBaselinePercent, maxBaselinePercent);
}

/**
 * @brief Apply gear transition protection to prevent jerking
 * @param torquePercent Input torque percentage
 * @return Modified torque with gear transition protection
 */
float VehicleControl::applyGearTransitionProtection(float torquePercent) {
    unsigned long currentTime = millis();
    
    // Detect gear transitions
    if (previousGear != currentGear && previousGear != GearState::NEUTRAL) {
        isInGearTransition = true;
        gearTransitionStartTime = currentTime;
        torqueBaseline = 0.0f;  // Reset baseline during gear changes
        Serial.println("Gear transition detected - applying anti-jerk protection");
    }
    
    // During gear transition, force zero torque for specified time
    if (isInGearTransition) {
        if (currentTime - gearTransitionStartTime < VehicleParams::GearTransition::TRANSITION_TIME_MS) {
            // Force zero torque during transition period
            previousGear = currentGear; // Update after forcing zero
            return 0.0f;
        } else {
            // Transition period complete, allow gradual ramp-up
            unsigned long rampTime = currentTime - gearTransitionStartTime - VehicleParams::GearTransition::TRANSITION_TIME_MS;
            float rampFactor = min(1.0f, rampTime * VehicleParams::GearTransition::RAMPUP_RATE / 100.0f);
            
            // Once fully ramped up, clear transition state
            if (rampFactor >= 1.0f) {
                isInGearTransition = false;
            }
            
            previousGear = currentGear;
            return torquePercent * rampFactor;
        }
    }
    
    previousGear = currentGear;
    return torquePercent;
}

/**
 * @brief Apply deadband hysteresis around zero to prevent oscillation
 * @param torquePercent Input torque percentage
 * @return Processed torque with deadband
 */
float VehicleControl::applyDeadbandHysteresis(float torquePercent) {
    float absValue = abs(torquePercent);
    
    if (wasInDeadband) {
        // Higher threshold to exit deadband (prevents chattering)
        if (absValue > VehicleParams::Motor::DEADZONE_THRESHOLD + VehicleParams::Filtering::DEADZONE_HYSTERESIS) {
            wasInDeadband = false;
            return torquePercent;
        } else {
            return 0.0f; // Stay in deadband
        }
    } else {
        // Lower threshold to enter deadband
        if (absValue < VehicleParams::Motor::DEADZONE_THRESHOLD) {
            wasInDeadband = true;
            return 0.0f;
        } else {
            return torquePercent;
        }
    }
}

/**
 * @brief Sample pedal position from ADC with averaging
 * @return Averaged ADC reading for pedal position
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
 * @brief Update current motor speed
 * @param speed Motor speed in RPM
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
 * @param mode New driving mode
 */
void VehicleControl::setDrivingMode(DriveMode mode) {
    currentDrivingMode = mode;
    // Note: Curtis neutral braking works with all modes
}

/**
 * @brief Check if the motor controller should be enabled
 * @return true if DMC should be enabled (torque != 0)
 */
bool VehicleControl::isDMCEnabled() const {
    return enableDMC;
}

/**
 * @brief Update gear state based on switch inputs with anti-jerk protection
 */
void VehicleControl::updateGearState() {
    static unsigned long errorClearStartTime = 0;
    static bool inErrorClearSequence = false;
    
    int32_t forwardValue = ads.readADC(2);  // Drive switch on A2
    int32_t reverseValue = ads.readADC(3);  // Reverse switch on A3
    
    bool isForwardHigh = forwardValue > 200;
    bool isReverseHigh = reverseValue > 200;
    
    // Handle neutral selection - always allowed
    if (!isForwardHigh && !isReverseHigh) {
        // Only execute error clearing sequence when newly transitioning to neutral
        if (currentGear != GearState::NEUTRAL && !inErrorClearSequence) {
            inErrorClearSequence = true;
            errorClearStartTime = millis();
            enableDMC = false;
            torqueBaseline = 0.0f;  // Reset baseline when going to neutral
            canManager->setNeedsClearError(true);
        } 
        else if (inErrorClearSequence) {
            if (millis() - errorClearStartTime >= 100) {
                canManager->setNeedsClearError(false);
                inErrorClearSequence = false;
                currentGear = GearState::NEUTRAL;
                if (canManager) {
                    canManager->setCurrentGear(currentGear);
                }
                shiftAttempted = false;
            }
        } 
        else {
            currentGear = GearState::NEUTRAL;
            shiftAttempted = false;
            enableDMC = false;        
        }
        return;
    }
    
    // Abort error clear sequence if pedals are pressed
    if (inErrorClearSequence) {
        inErrorClearSequence = false;
        canManager->setNeedsClearError(false);
    }
    
    // Handle transitions at low speed
    if (abs(motorSpeed) < VehicleParams::Transmission::RPM_SHIFT_THRESHOLD) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = GearState::DRIVE;
            shiftAttempted = false;
            enableDMC = true;
            if (canManager) {
                canManager->setCurrentGear(currentGear);
            }
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = GearState::REVERSE;
            shiftAttempted = false;
            enableDMC = true;
            if (canManager) {
                canManager->setCurrentGear(currentGear);
            }
        }
    } else {
        // At high speed, prevent switching between drive and reverse
        if (isForwardHigh && currentGear == GearState::REVERSE) {
            shiftAttempted = true;
        } else if (isReverseHigh && currentGear == GearState::DRIVE) {
            shiftAttempted = true;
        }
    }
    
    // Allow reengaging desired gear when speed drops
    if (shiftAttempted && abs(motorSpeed) < VehicleParams::Transmission::RPM_SHIFT_THRESHOLD) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = GearState::DRIVE;
            shiftAttempted = false;
            if (canManager) {
                canManager->setCurrentGear(currentGear);
            }
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = GearState::REVERSE;
            shiftAttempted = false;
            if (canManager) {
                canManager->setCurrentGear(currentGear);
            }
        }
    }
}