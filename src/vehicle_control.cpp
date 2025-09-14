/**
 * @file vehicle_control.cpp
 * @brief Implementation of smooth one-foot driving system with percentage-based torque
 * 
 * New Features:
 * - Progressive pedal zones for natural brake-to-accelerate feel
 * - Percentage-based torque system (-100% to +100%)
 * - Advanced filtering to prevent spikes without lag
 * - Anti-jerk gear transition protection  
 * - Speed-adaptive pedal response
 * - Proper DMC enable logic
 */

#include "vehicle_control.h"
#include "can_manager.h" 
#include "config.h"
#include "ADS1X15.h"
#include "configuration.h"  

/**
 * @brief Constructor - initializes vehicle control system with new smooth driving
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
    , previousFilteredTorque(0.0f)
    , spikeDetected(false)
    , lastFilterTime(0)
    , currentPedalZone(-1)
    , previousPedalZone(-1)
    , zoneTransitionFactor(1.0f)
{
}

/**
 * @brief Calculate motor torque percentage based on smooth one-foot driving
 * @return Calculated torque percentage (-100% to +100%)
 * 
 * Main control flow:
 * 1. Sample and map pedal position
 * 2. Apply speed adaptation
 * 3. Calculate torque based on pedal zones
 * 4. Apply advanced filtering
 * 5. Apply gear transition protection
 * 6. Update DMC enable logic
 */
 
// PROGRESSIVE SMOOTH FORMULA: Replace calculateTorquePercentage() in vehicle_control.cpp

float VehicleControl::calculateTorquePercentage() {
    // Sample pedal position
    int32_t sampledPotiValue = samplePedalPosition();
    
    // Map using correct ADC values from config.h
    float rawThrottle = map(sampledPotiValue, ADC::MinValPot, ADC::MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Apply gamma correction for more natural pedal feel
    float throttlePosition = pow(rawThrottle / 100.0f, VehicleParams::Pedal::GAMMA) * 100.0f;
    
    // Debug output
    Serial.printf("ADC: %d -> %.1f%%", sampledPotiValue, throttlePosition);
    
    // Update reverse light based on gear state
    digitalWrite(Pins::BCKLIGHT, currentGear == GearState::REVERSE ? HIGH : LOW);
    digitalWrite(19, currentGear == GearState::REVERSE ? HIGH : LOW);

    // Handle neutral gear
    if (currentGear == GearState::NEUTRAL) {
        lastTorquePercent = 0.0f;
        filteredTorquePercent = 0.0f;
        enableDMC = false;
        digitalWrite(19, LOW);  
        digitalWrite(Pins::BCKLIGHT, LOW);
        Serial.println(" -> NEUTRAL: 0%");
        return 0.0f;
    }
    
    // PROGRESSIVE FORMULA - One continuous smooth curve
    float calculatedTorquePercent = 0.0f;
    
    // Define key points for the curve
    const float REGEN_END = 15.0f;      // End of regen zone
    const float COAST_END = 17.0f;      // End of coast zone (narrow 2% zone)
    const float MAX_REGEN = -35.0f;     // Maximum regen at 0% pedal
    const float MAX_ACCEL = 100.0f;     // Maximum acceleration at 100% pedal
    
    if (throttlePosition <= REGEN_END) {
        // 0-15%: SMOOTH REGEN CURVE
        // Use exponential decay curve: strong regen at 0%, fading to 0 at 15%
        float t = throttlePosition / REGEN_END; // 0.0 to 1.0
        
        // Smooth exponential curve: y = max_regen * (1 - t)^power
        calculatedTorquePercent = MAX_REGEN * pow(1.0f - t, 1.2f);
        
        Serial.printf(" -> REGEN: %.1f%%", calculatedTorquePercent);
    }
    else if (throttlePosition <= COAST_END) {
        // 15-17%: NARROW COAST ZONE with smooth transitions
        float t = (throttlePosition - REGEN_END) / (COAST_END - REGEN_END); // 0.0 to 1.0
        
        // Smooth cosine transition from regen to zero
        calculatedTorquePercent = (MAX_REGEN * pow(0.0f, 1.8f)) * (1.0f - t);
        
        Serial.printf(" -> COAST: %.1f%%", calculatedTorquePercent);
    }
    else {
        // 17-100%: SMOOTH ACCELERATION CURVE
        float t = (throttlePosition - COAST_END) / (100.0f - COAST_END); // 0.0 to 1.0
        
        // Progressive acceleration curve with three segments blended smoothly
        float accelPercent;
        
        if (t < 0.3f) {
            // First 30% of accel zone (17-42% pedal): Gentle start
            float segment_t = t / 0.3f; // 0.0 to 1.0
            accelPercent = 25.0f * pow(segment_t, 1.5f); // 0% to 25%
        }
        else if (t < 0.7f) {
            // Middle 40% of accel zone (42-75% pedal): Linear progression  
            float segment_t = (t - 0.3f) / 0.4f; // 0.0 to 1.0
            accelPercent = 25.0f + (45.0f * segment_t); // 25% to 70%
        }
        else {
            // Last 30% of accel zone (75-100% pedal): Power curve
            float segment_t = (t - 0.7f) / 0.3f; // 0.0 to 1.0
            accelPercent = 70.0f + (30.0f * pow(segment_t, 0.8f)); // 70% to 100%
        }
        
        // Apply user-configurable max torque scaling
        float maxTorquePercent = (float)config.getMaxTorque() / (float)VehicleParams::Motor::MAX_TRQ * 100.0f;
        calculatedTorquePercent = (accelPercent / 100.0f) * maxTorquePercent;
        
        Serial.printf(" -> ACCEL: %.1f%%", calculatedTorquePercent);
    }
    
    // OPTIONAL: Add slight smoothing filter for extra smoothness
    static float smoothedTorque = 0.0f;
    const float SMOOTH_FACTOR = 0.15f; // 15% new, 85% previous
    smoothedTorque = (SMOOTH_FACTOR * calculatedTorquePercent) + ((1.0f - SMOOTH_FACTOR) * smoothedTorque);
    calculatedTorquePercent = smoothedTorque;
    
    // Apply proper direction based on gear state
    if (currentGear == GearState::DRIVE) {
        calculatedTorquePercent = -calculatedTorquePercent;
    }
    
    // Update DMC enable logic
    enableDMC = (abs(calculatedTorquePercent) > 0.3f);
    
    // Store for next iteration
    lastTorquePercent = calculatedTorquePercent;
    filteredTorquePercent = calculatedTorquePercent;
    
    Serial.printf(" -> FINAL: %.1f%% (DMC: %s)\n", 
                  calculatedTorquePercent, enableDMC ? "ON" : "OFF");
    
    return calculatedTorquePercent;
}

/**
 * @brief Handle smooth one-foot driving with progressive pedal zones
 * @param throttlePosition Adapted pedal position (0-100%)
 * @param speed Current vehicle speed in kph
 * @return Calculated torque percentage (-100% to +100%)
 */
float VehicleControl::handleSmoothDriving(float throttlePosition, float speed) {
    // Determine current pedal zone
    int newPedalZone;
    if (throttlePosition <= VehicleParams::Pedal::STRONG_REGEN_END) {
        newPedalZone = 0; // Strong regen zone (0-15%)
    } else if (throttlePosition <= VehicleParams::Pedal::LIGHT_REGEN_END) {
        newPedalZone = 1; // Light regen zone (15-25%)
    } else if (throttlePosition <= VehicleParams::Pedal::COAST_ZONE_END) {
        newPedalZone = 2; // Coast zone (25-30%)
    } else {
        newPedalZone = 3; // Acceleration zone (30-100%)
    }
    
    // Detect zone transitions for smooth blending
    bool zoneTransition = (previousPedalZone != newPedalZone) && (previousPedalZone != -1);
    if (zoneTransition) {
        zoneTransitionFactor = 0.7f; // Start with reduced response during transition
    } else {
        // Gradually return to full response
        zoneTransitionFactor = min(1.0f, zoneTransitionFactor + 0.05f);
    }
    
    float torquePercent = 0.0f;
    
    switch (newPedalZone) {
        case 0: { // Strong regen zone (0-15%) - Replaces brake pedal
            float normalizedPosition = throttlePosition / VehicleParams::Pedal::STRONG_REGEN_END;
            // Inverted mapping: 0% pedal = max regen, 15% pedal = min regen
            float regenIntensity = 1.0f - normalizedPosition;
            
            // Progressive curve for better feel
            regenIntensity = pow(regenIntensity, 0.8f);
            
            // Calculate regen torque percentage
            torquePercent = VehicleParams::Pedal::MAX_STRONG_REGEN + 
                           (VehicleParams::Pedal::MIN_STRONG_REGEN - VehicleParams::Pedal::MAX_STRONG_REGEN) * (1.0f - regenIntensity);
            
            break;
        }
        
        case 1: { // Light regen zone (15-25%) - Gentle slowing
            float normalizedPosition = (throttlePosition - VehicleParams::Pedal::STRONG_REGEN_END) / 
                                     (VehicleParams::Pedal::LIGHT_REGEN_END - VehicleParams::Pedal::STRONG_REGEN_END);
            // Inverted mapping: 15% pedal = max light regen, 25% pedal = min light regen
            float regenIntensity = 1.0f - normalizedPosition;
            
            // Smooth curve for gentle transition
            regenIntensity = pow(regenIntensity, 0.9f);
            
            torquePercent = VehicleParams::Pedal::MAX_LIGHT_REGEN + 
                           (VehicleParams::Pedal::MIN_LIGHT_REGEN - VehicleParams::Pedal::MAX_LIGHT_REGEN) * (1.0f - regenIntensity);
            
            break;
        }
        
        case 2: { // Coast zone (25-30%) - Zero torque
            // Smooth fade from light regen to zero during zone transitions
            if (previousPedalZone == 1) {
                float fadePosition = (throttlePosition - VehicleParams::Pedal::LIGHT_REGEN_END) / 
                                   (VehicleParams::Pedal::COAST_ZONE_END - VehicleParams::Pedal::LIGHT_REGEN_END);
                float fadeRatio = pow(1.0f - fadePosition, 0.5f);
                torquePercent = VehicleParams::Pedal::MIN_LIGHT_REGEN * fadeRatio * 0.3f; // Max 30% carry-over
            } else {
                torquePercent = 0.0f; // Pure coast
            }
            break;
        }
        
        case 3: { // Acceleration zone (30-100%) - Progressive power
            float normalizedPosition = (throttlePosition - VehicleParams::Pedal::COAST_ZONE_END) / 
                                     (100.0f - VehicleParams::Pedal::COAST_ZONE_END);
            
            // Progressive acceleration curve for better control
            float accelIntensity;
            if (normalizedPosition < 0.4f) {
                // Gentle initial acceleration (30-50% pedal)
                accelIntensity = pow(normalizedPosition / 0.4f, 1.4f) * 0.3f;
            } else {
                // More aggressive acceleration (50-100% pedal)
                accelIntensity = 0.3f + pow((normalizedPosition - 0.4f) / 0.6f, 1.1f) * 0.7f;
            }
            
            // Apply user-configurable torque limit
            float maxTorquePercent = (float)config.getMaxTorque() / (float)VehicleParams::Motor::MAX_TRQ * 100.0f;
            torquePercent = accelIntensity * maxTorquePercent;
            
            break;
        }
    }
    
    // Apply zone transition smoothing
    torquePercent *= zoneTransitionFactor;
    
    // Apply proper direction based on gear state
    // DRIVE: Negative torque = forward accel, Positive torque = regen (opposes forward motion)  
    // REVERSE: Positive torque = reverse accel, Negative torque = regen (opposes reverse motion)
    if (currentGear == GearState::REVERSE) {
        torquePercent = -torquePercent; // Flip signs for reverse
    }
    
    // Store zone information for next iteration
    previousPedalZone = currentPedalZone;
    currentPedalZone = newPedalZone;
    
    return torquePercent;
}

/**
 * @brief Apply advanced filtering to prevent torque spikes without lag
 * @param requestedTorquePercent Raw calculated torque percentage
 * @return Filtered torque percentage
 */
float VehicleControl::applyAdvancedFiltering(float requestedTorquePercent) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastFilterTime) / 1000.0f; // Convert to seconds
    lastFilterTime = currentTime;
    
    // Prevent division by zero
    if (deltaTime == 0.0f) deltaTime = 0.001f;
    
    // Calculate rate of change
    float torqueChange = requestedTorquePercent - filteredTorquePercent;
    float changeRate = abs(torqueChange) / deltaTime;
    
    // Spike detection: Check if change rate exceeds threshold
    spikeDetected = (changeRate > VehicleParams::Filtering::SPIKE_THRESHOLD);
    
    // Apply rate limiting based on spike detection
    float maxChangePerCycle = spikeDetected ? 
        VehicleParams::Filtering::SPIKE_RATE_LIMIT : 
        VehicleParams::Filtering::NORMAL_RATE_LIMIT;
    
    // Limit the change rate
    if (abs(torqueChange) > maxChangePerCycle) {
        torqueChange = (torqueChange > 0) ? maxChangePerCycle : -maxChangePerCycle;
    }
    
    // Apply exponential smoothing filter for additional smoothness
    float targetTorque = filteredTorquePercent + torqueChange;
    filteredTorquePercent = (VehicleParams::Filtering::FILTER_ALPHA * targetTorque) + 
                           ((1.0f - VehicleParams::Filtering::FILTER_ALPHA) * filteredTorquePercent);
    
    return filteredTorquePercent;
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
 * @brief Apply speed-adaptive pedal response
 * @param throttlePosition Raw pedal position (0-100%)
 * @param speed Current vehicle speed (kph)
 * @return Modified pedal position for speed-dependent behavior
 */
float VehicleControl::applySpeedAdaptation(float throttlePosition, float speed) {
    float absSpeed = abs(speed);
    
    // At very low speeds, reduce regen sensitivity to prevent jerky stops
    if (absSpeed < VehicleParams::Pedal::LOW_SPEED_LIMIT) {
        // Reduce regen zones and expand coast zone at low speeds
        if (throttlePosition <= VehicleParams::Pedal::STRONG_REGEN_END) {
            // Reduce strong regen intensity by 40% at low speeds
            float regenReduction = 0.6f;
            throttlePosition = throttlePosition / regenReduction;
            throttlePosition = min(throttlePosition, VehicleParams::Pedal::STRONG_REGEN_END);
        }
    }
    // At medium speeds, full regen available
    else if (absSpeed < VehicleParams::Pedal::MEDIUM_SPEED_LIMIT) {
        // Normal operation - no modification needed
    }
    // At high speeds, optimize for efficiency
    else if (absSpeed > VehicleParams::Pedal::HIGH_SPEED_LIMIT) {
        // Slightly increase coast zone for better efficiency at highway speeds
        if (throttlePosition > VehicleParams::Pedal::LIGHT_REGEN_END && 
            throttlePosition < VehicleParams::Pedal::COAST_ZONE_END + 5.0f) {
            // Expand coast zone by 5% at high speeds
            throttlePosition = VehicleParams::Pedal::COAST_ZONE_END;
        }
    }
    
    return throttlePosition;
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
 * @param mode New driving mode (currently only REGEN supported for smooth driving)
 */
void VehicleControl::setDrivingMode(DriveMode mode) {
    currentDrivingMode = mode;
    // Note: Legacy and OPD modes can be implemented later using the same percentage system
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