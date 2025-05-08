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
    // Simple RPM filtering with single-stage approach
    static float filteredRPM = 0.0f;
    
    // Calculate filtered RPM with moderate filtering
    float currentRawRPM = fabsf(motorSpeed);
    const float RPM_FILTER = 0.7f; // 70% previous, 30% new (moderate filtering)
    filteredRPM = (RPM_FILTER * filteredRPM) + ((1.0f - RPM_FILTER) * currentRawRPM);
    
    // Early exit if in neutral
    if (currentGear == GearState::NEUTRAL) {
        return 0;
    }
    
    // Define pedal zones
    const float REGEN_END_POINT = VehicleParams::Regen::END_POINT;
    const float COAST_END_POINT = VehicleParams::Regen::COAST_END;
    
    // Allow regen all the way to zero RPM with a gentle ramp
    // Use a very short ramp at low speeds to ensure smoothness
    float regenFactor = 1.0f;
    
    // If RPM is very low, slightly reduce regen power for smoothness
    if (filteredRPM < 50.0f) {
        // Apply a gentle curve for the last bit of deceleration
        regenFactor = filteredRPM / 50.0f;
        regenFactor = pow(regenFactor, 0.7f); // Make the curve gentler at low speeds
        regenFactor = max(0.2f, regenFactor); // Keep minimum of 20% regen capability
    }
    
    // Handle regen zone (pedal position 0-REGEN_END_POINT)
    if (throttlePosition <= REGEN_END_POINT) {
        // Progressive pedal mapping with smoother curve
        float normalizedRegen = 1.0f - (throttlePosition / REGEN_END_POINT);
        
        // Smoother curve at the transition point
        float progressiveRegen = pow(normalizedRegen, 1.2f); // Less aggressive curve
        
        // Calculate regen torque with smooth ramp
        float maxRegenTorque = std::min(VehicleParams::OPD::REGEN_TORQUE_CAP, 
                                       static_cast<double>(config.getMaxTorque() * 0.4));
        float regenTorque = progressiveRegen * maxRegenTorque * regenFactor;
        
        // Apply rate limiting for extra smoothness
        static float lastRegenTorque = 0.0f;
        float maxTorqueChange = 6.0f; // Allow slightly faster changes
        float torqueChange = regenTorque - lastRegenTorque;
        torqueChange = constrain(torqueChange, -maxTorqueChange, maxTorqueChange);
        regenTorque = lastRegenTorque + torqueChange;
        lastRegenTorque = regenTorque;
        
        // Apply proper direction based on motor speed
        return (motorSpeed < 0) ? regenTorque : -regenTorque;
    }
    // Handle coast zone (natural deceleration)
    else if (throttlePosition <= COAST_END_POINT) {
        return 0; // Coast (zero torque)
    }
    // Handle acceleration zone
    else {
        // Smoother transition from coast to acceleration
        float normalizedThrottle = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
        
        // Gentler curve at beginning of acceleration zone
        float progressiveThrottle = pow(normalizedThrottle, 1.3f); // Less aggressive curve
        
        // Calculate drive torque with basic rate limiting
        float driveTorque = progressiveThrottle * std::min(config.getMaxTorque(), VehicleParams::Motor::MAX_REQ_TRQ);
        
        static float lastDriveTorque = 0.0f;
        float maxDriveChange = 10.0f; // Allow faster changes for acceleration
        float driveChange = driveTorque - lastDriveTorque;
        driveChange = constrain(driveChange, -maxDriveChange, maxDriveChange);
        driveTorque = lastDriveTorque + driveChange;
        lastDriveTorque = driveTorque;
        
        // Apply proper direction based on gear
        return (currentGear == GearState::DRIVE) ? -driveTorque : driveTorque;
    }
}
 /* OLD
int16_t VehicleControl::handleRegenMode(float throttlePosition, float speed) {
    // Static variables for RPM filtering and state tracking
    static float filteredRPM = 0.0f;
    static float prevFilteredRPM = 0.0f;
    static float rpmChangeRate = 0.0f;
    static bool regenActive = false;
    static unsigned long lastOscillationTime = 0;
    static unsigned long lastUpdateTime = 0;
    static float regenTorqueHistory[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static uint8_t historyIndex = 0;
    static uint8_t oscillationCount = 0;
    static const uint8_t MAX_OSCILLATIONS = 5;
    static const unsigned long OSCILLATION_LOCKOUT_MS = 2000; // 2 second lockout after oscillation
    static int lastZone = -1; // -1=initial, 0=regen, 1=coast, 2=accel
    
    // Early exit if in neutral
    if (currentGear == GearState::NEUTRAL) {
        regenActive = false;
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
    const float PRIMARY_FILTER = 0.55f; // Very strong filtering (85% previous, 15% new)
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
    
    // Adaptive RPM thresholds based on oscillation history
    float MIN_REGEN_RPM_ENTER = 180.0f + (oscillationCount * 20.0f); // Increases with oscillations
    float MIN_REGEN_RPM_EXIT = 100.0f + (oscillationCount * 10.0f);  // Increases with oscillations
    const float REGEN_RAMP_RPM = 400.0f;  // Longer ramp for smoother transition
    
    // Oscillation detection - more sophisticated approach
    bool inLockoutPeriod = (currentTime - lastOscillationTime) < OSCILLATION_LOCKOUT_MS;
    
    // Enhance oscillation detection with torque stability analysis
    bool torqueOscillating = false;
    if (regenActive) {
        // Check for alternating torque direction in history (sign changes)
        int signChanges = 0;
        float prevTorque = regenTorqueHistory[0];
        for (int i = 1; i < 5; i++) {
            if ((prevTorque * regenTorqueHistory[i] < 0) && (fabsf(regenTorqueHistory[i]) > 5.0f)) {
                signChanges++;
            }
            prevTorque = regenTorqueHistory[i];
        }
        
        torqueOscillating = (signChanges >= 2); // Multiple sign changes indicate oscillation
        
        // Near threshold oscillation detection
        bool nearThreshold = (filteredRPM < MIN_REGEN_RPM_EXIT + 30.0f) && 
                             (filteredRPM > MIN_REGEN_RPM_EXIT - 30.0f);
        
        if ((torqueOscillating && nearThreshold) || 
            (nearThreshold && fabsf(rpmChangeRate) > 50.0f)) { // Rapid RPM changes near threshold
            // Oscillation detected - increase thresholds and disable regen temporarily
            regenActive = false;
            lastOscillationTime = currentTime;
            
            // Increment oscillation counter for adaptive behavior (max 5)
            if (oscillationCount < MAX_OSCILLATIONS) {
                oscillationCount++;
            }
        }
    }
    
    // Apply hysteresis to RPM threshold with intelligent lockout
    bool canRegen = false;
    if (!inLockoutPeriod) {
        if (regenActive) {
            // Once regen is active, keep it active until RPM drops below exit threshold
            // Add momentum factor - if RPM is falling rapidly, deactivate early to prevent overshoot
            float momentumFactor = constrain(rpmChangeRate * -0.2f, 0.0f, 50.0f);
            canRegen = (filteredRPM >= (MIN_REGEN_RPM_EXIT + momentumFactor));
            if (!canRegen) {
                regenActive = false;
            }
        } else {
            // To activate, need stable RPM above threshold
            canRegen = (filteredRPM >= MIN_REGEN_RPM_ENTER) && (fabsf(rpmChangeRate) < 100.0f);
            if (canRegen) {
                regenActive = true;
            }
        }
    }
    
    // Ultra-smooth tapering with adaptive curve
    float regenTaperFactor = 0.0f;
    if (canRegen && filteredRPM > MIN_REGEN_RPM_EXIT) {
        if (filteredRPM >= REGEN_RAMP_RPM) {
            regenTaperFactor = 1.0f; // Full regen available
        } else {
            // Adaptive curve based on oscillation history
            float rampProgress = (filteredRPM - MIN_REGEN_RPM_EXIT) / (REGEN_RAMP_RPM - MIN_REGEN_RPM_EXIT);
            rampProgress = constrain(rampProgress, 0.0f, 1.0f);
            
            // Use higher power curve when oscillations detected for even gentler onset
            float curvePower = 3.0f + (oscillationCount * 0.5f); // 3.0 to 5.5 based on oscillations
            regenTaperFactor = pow(rampProgress, curvePower);
            
            // Additional rate limiting on taper factor itself
            static float lastTaperFactor = 0.0f;
            const float MAX_TAPER_CHANGE = 0.05f; // Maximum change per update
            float taperChange = regenTaperFactor - lastTaperFactor;
            taperChange = constrain(taperChange, -MAX_TAPER_CHANGE, MAX_TAPER_CHANGE);
            regenTaperFactor = lastTaperFactor + taperChange;
            lastTaperFactor = regenTaperFactor;
        }
    }
    
    // Handle regen zone (pedal position 0-REGEN_END_POINT)
    if (throttlePosition <= REGEN_END_POINT) {
        // Progressive pedal mapping
        float normalizedRegen = 1.0f - (throttlePosition / REGEN_END_POINT);
        float progressiveRegen = pow(normalizedRegen, 1.5f);
        
        // Calculate regen torque with adaptive taper factor
        float regenTorque = 0.0f;
        
        if (canRegen) {
            // Base torque calculation
            float baseTorque = progressiveRegen * std::min(VehicleParams::OPD::REGEN_TORQUE_CAP, static_cast<double>(config.getMaxTorque() * 0.35));
            
            // Apply adaptive tapering
            regenTorque = baseTorque * regenTaperFactor;
            
            // Apply rate limiting for ultimate smoothness
            static float lastRegenTorque = 0.0f;
            float torqueChange = regenTorque - lastRegenTorque;
            
            // Apply stronger rate limiting after zone transitions from accel to regen
            float maxChange = (zoneTransition && lastZone == 2) ? 3.0f : 5.0f + (filteredRPM * 0.05f);
            torqueChange = constrain(torqueChange, -maxChange, maxChange);
            regenTorque = lastRegenTorque + torqueChange;
            lastRegenTorque = regenTorque;
        }
        
        // Store torque in history buffer for oscillation detection
        regenTorqueHistory[historyIndex] = regenTorque * ((motorSpeed < 0) ? 1.0f : -1.0f); // Store with direction
        historyIndex = (historyIndex + 1) % 5;
        
        // Update zone tracker
        lastZone = currentZone;
        
        // Apply proper direction based on motor speed
        return (motorSpeed < 0) ? regenTorque : -regenTorque;
    }
    // Handle coast zone
    else if (throttlePosition <= COAST_END_POINT) {
        // Reset torque history in coast mode
        for (int i = 0; i < 5; i++) {
            regenTorqueHistory[i] = 0.0f;
        }
        
        // Update zone tracker
        lastZone = currentZone;
        
        return 0; // Coast (zero torque)
    }
    // Handle acceleration zone
    else {
        // Reset torque history in acceleration mode
        for (int i = 0; i < 5; i++) {
            regenTorqueHistory[i] = 0.0f;
        }
        
        // Progressive throttle mapping
        float normalizedThrottle = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
        float progressiveThrottle = pow(normalizedThrottle, 1.7f);
        
        // Adaptive torque calculation with rate limiting
        static float lastAccelTorque = 0.0f;
        float targetTorque = progressiveThrottle * std::min(config.getMaxTorque(), VehicleParams::Motor::MAX_REQ_TRQ);
        
        // Apply rate limiting with stricter limits during zone transitions
        float torqueChange = targetTorque - lastAccelTorque;
        
        // Use much stricter limit after zone transition from regen to accel
        float maxChange = (zoneTransition && lastZone == 0) ? 2.0f : 10.0f;
        torqueChange = constrain(torqueChange, -maxChange, maxChange);
        float accelTorque = lastAccelTorque + torqueChange;
        lastAccelTorque = accelTorque;
        
        // Update zone tracker
        lastZone = currentZone;
        
        // Apply proper direction based on gear
        return (currentGear == GearState::DRIVE) ? -accelTorque : accelTorque;
    }
}

/*
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
            canManager->setEnableDMC(false);
            
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
            canManager->setEnableDMC(false);
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
    
    // Remember the last gear state
    lastGear = currentGear;
}