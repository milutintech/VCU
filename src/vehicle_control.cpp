/**
 * @file vehicle_control.cpp - SIMPLIFIED Implementation
 * @brief Implementation of simplified three-zone pedal system with delta-based power limiting
 * 
 * Features:
 * - Simple three-zone pedal system (regen/coast/accel)
 * - Progressive curves for natural pedal feel  
 * - Delta-based power limiting (keeps existing Curtis power maps)
 * - Configurable zone boundaries and progression factors
 */

#include "vehicle_control.h"
#include "can_manager.h" 
#include "config.h"
#include "ADS1X15.h"
#include "configuration.h"

/**
 * @brief Constructor - initializes simplified vehicle control system
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
{
}

/**
 * @brief SIMPLIFIED: Calculate motor torque percentage using three-zone pedal system
 * @return Calculated torque percentage (-100% to +100%)
 */
float VehicleControl::calculateTorquePercentage() {
    // Sample pedal position
    int32_t sampledPotiValue = samplePedalPosition();
    
    // Map using correct ADC values from config.h (0-100%)
    float rawThrottle = map(sampledPotiValue, ADC::MinValPot, ADC::MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Debug output
    //Serial.printf("ADC: %d -> %.1f%% throttle", sampledPotiValue, rawThrottle);
    
    // Update reverse light based on gear state
    digitalWrite(Pins::BCKLIGHT, currentGear == GearState::REVERSE ? HIGH : LOW);
    digitalWrite(19, currentGear == GearState::REVERSE ? HIGH : LOW);

    // Handle neutral gear - always zero torque
    if (currentGear == GearState::NEUTRAL) {
        lastTorquePercent = 0.0f;
        filteredTorquePercent = 0.0f;
        enableDMC = false;
        digitalWrite(19, LOW);  
        digitalWrite(Pins::BCKLIGHT, LOW);
        //Serial.println(" -> NEUTRAL: 0%");
        return 0.0f;
    }
    
    // Apply simplified three-zone pedal mapping
    float baseTorquePercent = applyPedalZones(rawThrottle);
    
    // Apply delta-based power limiting (keep existing Curtis power maps)
    bool isDriving = (baseTorquePercent > 0);  // Positive = acceleration, Negative = regen
    float powerLimit = calculatePowerLimit(abs(motorSpeed), isDriving);
    
    // Apply power limiting
    float maxPossibleTorquePercent = (float)config.getMaxTorque() / (float)VehicleParams::Motor::MAX_TRQ * 100.0f;
    float limitedTorquePercent;
    

    if (isDriving) {
        // Scale acceleration proportionally based on speed zones
        float drivePowerLimit = calculatePowerLimit(abs(motorSpeed), true);  // Use drive zones
        limitedTorquePercent = baseTorquePercent * (drivePowerLimit / 100.0f);
    } else {
        // Scale regen proportionally based on speed zones  
        float regenPowerLimit = calculatePowerLimit(abs(motorSpeed), false); // Use regen zones
        limitedTorquePercent = baseTorquePercent * (regenPowerLimit / 100.0f);
    }
    // Apply gear direction
    float calculatedTorquePercent = limitedTorquePercent;
    if (currentGear == GearState::DRIVE) {
        calculatedTorquePercent = -limitedTorquePercent;  // Negative = forward in drive
    }
    // In REVERSE, positive torque = reverse motion (no sign change needed)
    
    // Apply gear transition protection
    calculatedTorquePercent = applyGearTransitionProtection(calculatedTorquePercent);
    
    // Apply deadband hysteresis
    calculatedTorquePercent = applyDeadbandHysteresis(calculatedTorquePercent);
    
    // Update DMC enable logic
    enableDMC = (abs(calculatedTorquePercent) > 0.5f);
    
    // Store for next iteration
    lastTorquePercent = calculatedTorquePercent;
    filteredTorquePercent = calculatedTorquePercent;
    
    //Serial.printf(" -> Zones: %.1f%%, Power Limit: %.1f%%, Final: %.1f%% (DMC: %s)\n", 
       //           baseTorquePercent, powerLimit, calculatedTorquePercent, enableDMC ? "ON" : "OFF");
    
    return calculatedTorquePercent;
}

/**
 * @brief NEW: Apply simplified three-zone pedal mapping
 * @param throttlePercent Raw throttle position (0-100%)
 * @return Torque percentage with zone mapping applied
 */
float VehicleControl::applyPedalZones(float throttlePercent) {
    // Get configurable zone boundaries
    float regenZoneEnd = config.getRegenZoneEnd();
    float coastZoneEnd = config.getCoastZoneEnd();
    float regenProgression = config.getRegenProgression();
    float accelProgression = config.getAccelProgression();
    
    if (throttlePercent <= regenZoneEnd) {
        // REGEN ZONE: 0% to regenZoneEnd% -> -100% to 0% torque (inverted for lift-off regen)
        float zonePosition = 1.0f - (throttlePercent / regenZoneEnd);  // INVERTED: 1.0 at 0% pedal, 0.0 at regenZoneEnd%
        float curvedPosition = applyProgressiveCurve(zonePosition, regenProgression);
        float torquePercent = -curvedPosition * 100.0f;  // Negative for regen
        
        Serial.printf(" -> REGEN ZONE (%.1f%% in zone)", zonePosition * 100.0f);
        return torquePercent;
    }
    else if (throttlePercent <= coastZoneEnd) {
        // COAST ZONE: regenZoneEnd% to coastZoneEnd% -> 0% torque
        //Serial.printf(" -> COAST ZONE");
        return 0.0f;
    }
    else {
        // ACCEL ZONE: coastZoneEnd% to 100% -> 0% to +100% torque
        float accelZoneSize = 100.0f - coastZoneEnd;
        float zonePosition = (throttlePercent - coastZoneEnd) / accelZoneSize;  // 0-1 within accel zone
        float curvedPosition = applyProgressiveCurve(zonePosition, accelProgression);
        float torquePercent = curvedPosition * 100.0f;  // Positive for accel
        
        //Serial.printf(" -> ACCEL ZONE (%.1f%% in zone)", zonePosition * 100.0f);
        return torquePercent;
    }
}

/**
 * @brief Apply progressive curve to zone value
 * @param zonePosition Position within zone (0-1)
 * @param progression Progression factor (1.0=linear, >1.0=progressive)
 * @return Curved output value (0-1)
 */
float VehicleControl::applyProgressiveCurve(float zonePosition, float progression) {
    // Apply power curve: output = input^progression
    // progression = 1.0 -> linear
    // progression > 1.0 -> progressive (gentle start, aggressive end)
    // progression < 1.0 -> regressive (aggressive start, gentle end)
    
    return pow(zonePosition, progression);
}

/**
 * @brief Calculate power limit based on current motor speed using delta curves
 * @param motorSpeed Current motor speed in RPM
 * @param isDriving true for drive power limits, false for regen limits
 * @return Power limit percentage (0-120%)
 */
float VehicleControl::calculatePowerLimit(float motorSpeed, bool isDriving) {
    const float* powerLimits = isDriving ? config.getDrivePowerLimits() : config.getRegenPowerLimits();
    return interpolatePowerLimit(motorSpeed, powerLimits);
}

/**
 * @brief Interpolate power limit between speed zones (UNCHANGED - keep Curtis delta system)
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
 * @brief Apply gear transition protection to prevent jerking (UNCHANGED)
 * @param torquePercent Input torque percentage
 * @return Modified torque with gear transition protection
 */
float VehicleControl::applyGearTransitionProtection(float torquePercent) {
    unsigned long currentTime = millis();
    
    // Detect gear transitions
    if (previousGear != currentGear && previousGear != GearState::NEUTRAL) {
        isInGearTransition = true;
        gearTransitionStartTime = currentTime;
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
 * @brief Apply deadband hysteresis around zero (UNCHANGED)
 * @param torquePercent Input torque percentage
 * @return Processed torque with deadband
 */
float VehicleControl::applyDeadbandHysteresis(float torquePercent) {
    float absValue = abs(torquePercent);
    
    if (wasInDeadband) {
        // Higher threshold to exit deadband (prevents chattering)
        if (absValue > VehicleParams::Motor::DEADZONE_THRESHOLD + 1.0f) {
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
 * @brief Sample pedal position from ADC with averaging (UNCHANGED)
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
 * @brief Calculate current vehicle speed from motor speed (UNCHANGED)
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
 * @brief Update current motor speed (UNCHANGED)
 * @param speed Motor speed in RPM
 */
void VehicleControl::setMotorSpeed(float speed) {
    motorSpeed = speed;
}

/**
 * @brief Set current gear state (UNCHANGED)
 * @param gear New gear state (DRIVE/NEUTRAL/REVERSE)
 */
void VehicleControl::setCurrentGear(GearState gear) {
    currentGear = gear;
}

/**
 * @brief Set current driving mode (UNCHANGED)
 * @param mode New driving mode
 */
void VehicleControl::setDrivingMode(DriveMode mode) {
    currentDrivingMode = mode;
    // Note: Simplified system works with all modes
}

/**
 * @brief Check if the motor controller should be enabled (UNCHANGED)
 * @return true if DMC should be enabled (torque != 0)
 */
bool VehicleControl::isDMCEnabled() const {
    return enableDMC;
}

/**
 * @brief Update gear state based on switch inputs with anti-jerk protection (UNCHANGED)
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