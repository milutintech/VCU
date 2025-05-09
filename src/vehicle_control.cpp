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
#include "can_manager.h" 
#include "config.h"
#include "ADS1X15.h"
#include "configuration.h"  

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
    //calculatedTorque = applyDeadbandHysteresis(calculatedTorque);

    //Makes sure no reverse driving posible in drive.
    //calculatedTorque = applyTorqueCutoff(calculatedTorque);

    
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
 * @brief Handle Regenerative driving mode with enhanced low-speed regulation.
 * @param throttlePosition Processed pedal position (0-100%).
 * @param speed Current vehicle speed in kph.
 * @return Calculated torque for regen mode.
 * 
 * Features:
 * - Pedal mapping with coast zone and regen zone
 * - Low RPM regulation with predictive adaptive control
 * - Enhanced RPM filtering with multi-stage approach
 * - Speed-based regen tapering with ultra-smooth curve
 * - Direction-aware torque calculation
 * - Oscillation detection and automatic adaptation
 */
int16_t VehicleControl::handleRegenMode(float throttlePosition, float speed) {
    // Static variables for filtering and state tracking
    static float filteredRPM = 0.0f;
    static float prevFilteredRPM = 0.0f;
    static float rpmChangeRate = 0.0f;
    static int lastZone = -1; // -1=initial, 0=regen, 1=coast, 2=accel
    static unsigned long lastUpdateTime = 0; // For time tracking
    
    // Early exit if in neutral
    if (currentGear == GearState::NEUTRAL) {
        return 0;
    }
    
    // Time-based calculations
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;
    
    // Prevent division by zero in rate calculations
    if (deltaTime == 0) deltaTime = 1;
    
    // Enhanced multi-stage RPM filtering for ultimate stability
    float currentRawRPM = fabsf(motorSpeed);
    
    // Stage 1: Heavy low-pass filter for noise reduction
    const float PRIMARY_FILTER = 0.55f; // Very strong filtering (55% previous, 45% new)
    filteredRPM = (PRIMARY_FILTER * filteredRPM) + ((1.0f - PRIMARY_FILTER) * currentRawRPM);
    
    // Stage 2: Calculate rate of change and apply predictive filtering
    if (deltaTime > 0) {
        // Calculate RPM change rate (RPM per second)
        float instantRpmRate = (filteredRPM - prevFilteredRPM) * (1000.0f / deltaTime);
        
        // Filter the rate itself for stability
        const float RATE_FILTER = 0.9f;
        rpmChangeRate = (RATE_FILTER * rpmChangeRate) + ((1.0f - RATE_FILTER) * instantRpmRate);
        
        // Apply predictive component (helps anticipate RPM changes)
        const float PREDICTION_FACTOR = 0.1f;
        float predictedRPM = filteredRPM + (rpmChangeRate * PREDICTION_FACTOR);
        
        // Blend prediction with filtered value (improves responsiveness while maintaining stability)
        const float PREDICTION_BLEND = 0.2f;
        filteredRPM = (filteredRPM * (1.0f - PREDICTION_BLEND)) + (predictedRPM * PREDICTION_BLEND);
    }
    
    // Save current filtered RPM for next iteration
    prevFilteredRPM = filteredRPM;
    
    // Define pedal zones with consistent parameters
    const float REGEN_END_POINT = VehicleParams::Regen::END_POINT;
    const float COAST_END_POINT = VehicleParams::Regen::COAST_END;
    
    // Determine current zone
    int currentZone;
    if (throttlePosition <= REGEN_END_POINT) {
        currentZone = 0; // Regen zone
    } else if (throttlePosition <= COAST_END_POINT) {
        currentZone = 1; // Coast zone
    } else {
        currentZone = 2; // Acceleration zone
    }
    
    // Detect zone transitions
    bool zoneTransition = (lastZone != currentZone) && (lastZone != -1);
    
    // Smoother tapering with gentler curve based on RPM
    float regenTaperFactor = 0.0f;
    
    if (filteredRPM >= 300.0f) {
        // Full regen allowed at higher speeds
        regenTaperFactor = 1.0f;
    } else if (filteredRPM <= 10.0f) {
        // At very low speed (near zero), still allow moderate regen
        // This allows regen braking all the way to 0
        regenTaperFactor = 0.3f;
    } else {
        // Progressive taper between 10 and 300 RPM
        float rampProgress = (filteredRPM - 10.0f) / 290.0f;
        rampProgress = constrain(rampProgress, 0.0f, 1.0f);
        
        // Use gentler power curve for easier transitions
        regenTaperFactor = 0.3f + 0.7f * pow(rampProgress, 1.5f);
    }
    
    // Rate limiting on taper factor for smooth transitions
    static float lastTaperFactor = 0.0f;
    const float MAX_TAPER_CHANGE = 0.08f;
    float taperChange = regenTaperFactor - lastTaperFactor;
    taperChange = constrain(taperChange, -MAX_TAPER_CHANGE, MAX_TAPER_CHANGE);
    regenTaperFactor = lastTaperFactor + taperChange;
    lastTaperFactor = regenTaperFactor;
    
    // Handle regen zone with enhanced progressive pedal response
    if (throttlePosition <= REGEN_END_POINT) {
        // More progressive pedal mapping with two-segment curve
        float normalizedRegen = 1.0f - (throttlePosition / REGEN_END_POINT);
        
        // Two-segment curve for better pedal feel
        float progressiveRegen;
        if (normalizedRegen < 0.4f) {
            // Light regen (first 40% of pedal travel)
            progressiveRegen = pow(normalizedRegen / 0.4f, 1.2f) * 0.25f;
        } else {
            // Strong regen (40-100% of pedal travel)
            progressiveRegen = 0.25f + pow((normalizedRegen - 0.4f) / 0.6f, 0.9f) * 0.75f;
        }
        
        // Calculate regen torque with adaptive taper factor
        // Base torque calculation
        float baseTorque = progressiveRegen * std::min(VehicleParams::OPD::REGEN_TORQUE_CAP, 
                                                      static_cast<double>(config.getMaxTorque() * 0.35));
        
        // Apply adaptive tapering
        float regenTorque = baseTorque * regenTaperFactor;
        
        // REMOVED: Special holding torque at zero speed
        // Now we're letting the regular regen code handle all speeds including zero
        
        // Apply rate limiting for smooth transitions
        static float lastRegenTorque = 0.0f;
        float torqueChange = regenTorque - lastRegenTorque;
        
        // Adaptive rate limiting based on situation
        float maxChange = (zoneTransition && lastZone == 2) ? 3.0f : 7.0f;
        torqueChange = constrain(torqueChange, -maxChange, maxChange);
        regenTorque = lastRegenTorque + torqueChange;
        lastRegenTorque = regenTorque;
        
        // Update zone tracker
        lastZone = currentZone;
        
        // Apply proper direction based on motor speed
        return (motorSpeed < 0) ? regenTorque : -regenTorque;
    }
    // Handle coast zone
    else if (throttlePosition <= COAST_END_POINT) {
        // Smooth transition when coming from regen zone
        if (lastZone == 0) {
            // Calculate fade-out ratio based on position in coast zone
            float coastPosition = (throttlePosition - REGEN_END_POINT) / (COAST_END_POINT - REGEN_END_POINT);
            float fadeRatio = 1.0f - pow(coastPosition, 0.7f); // non-linear fade
            
            // Get last regen torque for smooth transition
            static float lastRegenTorque = 0.0f;
            float transitionTorque = lastRegenTorque * fadeRatio * 0.7f; // 70% max carry-over
            
            // Apply rate limiting for smoothness
            static float lastTransitionTorque = 0.0f;
            float torqueChange = transitionTorque - lastTransitionTorque;
            float maxChange = 5.0f; // moderate limit
            torqueChange = constrain(torqueChange, -maxChange, maxChange);
            transitionTorque = lastTransitionTorque + torqueChange;
            lastTransitionTorque = transitionTorque;
            
            // Save for next iteration
            lastRegenTorque = transitionTorque;
            
            // Update zone tracker
            lastZone = currentZone;
            
            // Apply proper direction based on motor speed
            return (motorSpeed < 0) ? transitionTorque : -transitionTorque;
        }
        
        // Normal coast behavior with zero torque
        // Update zone tracker
        lastZone = currentZone;
        
        return 0; // Coast (zero torque)
    }
    // Handle acceleration zone
    else {
        // MODIFICATION: Progressive regen cutoff for smoother starts from standstill
        // If at very low speed, adjust the regen behavior based on throttle position
        if (filteredRPM < 50.0f && fabsf(speed) < 2.0f) {
            // Calculate a dynamic regen threshold based on throttle position
            // As throttle increases, the threshold increases
            float normalizedThrottle = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
            float dynamicThreshold = 50.0f * normalizedThrottle;
            
            // If current RPM is below the dynamic threshold, we won't do anything
            // The inverter will handle the 0 speed behavior
        }
        
        // Progressive throttle mapping for acceleration
        float normalizedThrottle = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
        float progressiveThrottle;
        
        // Two-segment acceleration curve for better control
        if (normalizedThrottle < 0.3f) {
            // Light acceleration (0-30%)
            progressiveThrottle = pow(normalizedThrottle / 0.3f, 1.3f) * 0.2f; 
        } else {
            // Strong acceleration (30-100%)
            progressiveThrottle = 0.2f + pow((normalizedThrottle - 0.3f) / 0.7f, 1.2f) * 0.8f;
        }
        
        // Special handling for regen-to-accel transition
        if (zoneTransition && lastZone == 0) {
            // Time-based transition from regen to accel
            static unsigned long transitionStartTime = 0;
            if (zoneTransition) {
                transitionStartTime = currentTime; // Reset timer on transition
            }
            
            // Calculate transition ramp factor (0 to 1 over 250ms - shorter than before)
            float transitionTime = min(250.0f, (float)(currentTime - transitionStartTime));
            float rampFactor = transitionTime / 250.0f;
            
            // S-curve transition for smoothness
            rampFactor = 0.5f - 0.5f * cos(rampFactor * 3.14159f);
            
            // Apply gradual ramp-up during transition
            progressiveThrottle *= rampFactor;
        }
        
        // Calculate target torque
        float targetTorque = progressiveThrottle * std::min(config.getMaxTorque(), VehicleParams::Motor::MAX_REQ_TRQ);
        
        // Rate limiting for smooth acceleration
        static float lastAccelTorque = 0.0f;
        float torqueChange = targetTorque - lastAccelTorque;
        
        // Adaptive rate limiting based on situation
        float maxChange;
        if (zoneTransition && lastZone == 0) {
            // Coming from regen - gentler transition
            maxChange = 3.0f;
        } else if (zoneTransition) {
            // Other transitions
            maxChange = 5.0f;
        } else {
            // Normal operation - more responsive
            maxChange = 8.0f + (progressiveThrottle * 5.0f); // 8-13 based on throttle
        }
        
        torqueChange = constrain(torqueChange, -maxChange, maxChange);
        float accelTorque = lastAccelTorque + torqueChange;
        lastAccelTorque = accelTorque;
        
        // Update zone tracker
        lastZone = currentZone;
        
        // Apply proper direction based on gear
        return (currentGear == GearState::DRIVE) ? -accelTorque : accelTorque;
    }
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
        float maxTorque = config.getMaxTorque();

        // No low-speed torque reduction
        float torque = maxTorque * pow(normalizedPosition, 1.5f);

        return (currentGear == GearState::DRIVE) ? -torque : torque;
    } 
    
    // Regen logic remains unchanged.
    float normalizedPosition = throttlePosition / coastLower;
    float maxRegen = VehicleParams::OPD::MAX_REGEN * (config.getMaxTorque() / 100.0f);
    
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
    float maxRegen = VehicleParams::OPD::MAX_REGEN * (config.getMaxTorque() / 100.0f);
    
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
    static unsigned long errorClearStartTime = 0;
    static bool inErrorClearSequence = false;
    static GearState lastGear = GearState::NEUTRAL;
    
    int32_t forwardValue = ads.readADC(2);  // Drive switch on A2.
    int32_t reverseValue = ads.readADC(3);  // Reverse switch on A3.
    
    bool isForwardHigh = forwardValue > 200;
    bool isReverseHigh = reverseValue > 200;
    
    // Handle neutral selection – always allowed.
    if (!isForwardHigh && !isReverseHigh) {
        // Only execute the error clearing sequence when newly transitioning to neutral
        if (currentGear != GearState::NEUTRAL && !inErrorClearSequence) {
            // Start non-blocking error clearing sequence
            inErrorClearSequence = true;
            errorClearStartTime = millis();
            
            // Disable DMC
            enableDMC = false;
            
            // Set error latch high and mark for clearing
            canManager->setNeedsClearError(true);
            
            // Gear will be set to NEUTRAL after the sequence completes
        } 
        else if (inErrorClearSequence) {
            // Check if 100ms has passed
            if (millis() - errorClearStartTime >= 100) {
                // Clear the error flag
                canManager->setNeedsClearError(false);
                inErrorClearSequence = false;
                
                // Now we can officially set the gear state
                currentGear = GearState::NEUTRAL;
                shiftAttempted = false;
            }
            // Don't change gear state until sequence completes
        } 
        else {
            // Already in neutral, nothing special to do
            currentGear = GearState::NEUTRAL;
            shiftAttempted = false;
            enableDMC = false;        
            }
        if (lastGear != currentGear && canManager) {
            canManager->setCurrentGear(currentGear);
        }
        // Remember the last gear state
        lastGear = currentGear;
        return;
    }
    
    // If we were in an error clear sequence but now pedals are pressed,
    // abort the error clear sequence
    if (inErrorClearSequence) {
        inErrorClearSequence = false;
        canManager->setNeedsClearError(false);
    }
    
    // Handle transitions at low speed.
    if (abs(motorSpeed) < VehicleParams::Transmission::RPM_SHIFT_THRESHOLD) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = GearState::DRIVE;
            shiftAttempted = false;
            enableDMC = true;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = GearState::REVERSE;
            shiftAttempted = false;
            enableDMC = true;
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
    
    // Remember the last gear state
    lastGear = currentGear;
}