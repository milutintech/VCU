/**
 * @file state_manager.cpp
 * @brief Implementation of the vehicle state management system
 * 
 * Manages the vehicle's operational states and transitions:
 * - STANDBY: Low power state with minimal systems active
 * - RUN: Normal driving operation with motor systems enabled
 * - CHARGING: Battery charging state with charger enabled
 * 
 * Handles critical safety systems including:
 * - Battery precharge sequence
 * - Contactor control
 * - Cooling system management
 * - System wake-up and sleep control
 */

#include "state_manager.h"
#include "can_manager.h"  // Add this include
#include "config.h"
#include <esp_sleep.h>
#include "configuration.h"  // Add this include at the top


/**
 * @brief Constructor - initializes state manager with safe defaults
 * 
 * Sets initial system state to STANDBY and ensures all
 * subsystems start in a safe, disabled state.
 */
StateManager::StateManager(CANManager& canMgr)
    : canManager(canMgr)
    , currentState(VehicleState::STANDBY)
    , batteryArmed(false)
    , hasPreCharged(false)
    , nlgCharged(false)
    , connectorLocked(false)
    , unlockPersist(false)
    , conUlockInterrupt(false)
    , lastModeChangeTime(0)
    , lastPrechargeAttempt(0)
    , enableBSC(false)
    , modeBSC(false)
    , errorLatch(false)
    , chargerState(ChargerStates::NLG_ACT_SLEEP)
    , chargerStateDemand(ChargerStates::NLG_DEM_STANDBY)
    , chargeLedDemand(0)
    , batteryVoltage(0)
    , hvVoltageActual(0)
    , hvVoltage(0)
    , inverterTemp(0)
    , motorTemp(0)
    , coolingRequest(0)
    , enableDMC(false)
    , lastNlgWakeupCheck(0)
    , wasNlgWakeupHigh(false)
{
    // Initialize GPIO outputs to safe states
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    digitalWrite(Pins::CONTACTOR, LOW);
    digitalWrite(Pins::PUMP, LOW);
}

/**
 * @brief Process wake-up event and determine initial state
 * 
 * Checks wake-up source and transitions to appropriate state:
 * - CHARGING if woken by charger
 * - RUN if woken by ignition
 * - STANDBY if no wake signals present
 */
void StateManager::handleWakeup() {
    Serial.println("Handling wakeup...");
    uint8_t reason = getWakeupReason();
    Serial.print("Wakeup reason: ");
    Serial.println(reason);
    
    if (digitalRead(Pins::NLG_HW_Wakeup)) {
        Serial.println("NLG_HW_Wakeup pin is HIGH");
        if(!unlockPersist){
            // When waking up due to charger, remove NLG_HW_Wakeup from wake sources
            removePinFromWakeSources(Pins::NLG_HW_Wakeup);
            transitionToCharging();
        } else {
            transitionToStandby();
        }
    } else if (digitalRead(Pins::IGNITION)) {
        Serial.println("IGNITION pin is HIGH");
        unlockPersist = false;
        // When waking up due to ignition, add NLG_HW_Wakeup back to wake sources
        addPinToWakeSources(Pins::NLG_HW_Wakeup);
        transitionToRun();
    } else {
        Serial.println("No wake signals, going to standby");
        transitionToStandby();
    }
}

/**
 * @brief Main state machine update function
 * 
 * Updates system based on current state:
 * - STANDBY: Monitor wake sources, manage sleep
 * - RUN: Handle driving operations
 * - CHARGING: Manage charging process
 */
void StateManager::update() {
    switch(currentState) {
        case VehicleState::STANDBY:
            handleStandbyState();
            break;
            
        case VehicleState::RUN:
            handleRunState();
            break;
            
        case VehicleState::CHARGING:
            handleChargingState();
            break;
    }
}

/**
 * @brief Handle system behavior in STANDBY state
 * 
 * Manages low-power standby state:
 * - Disables all major systems
 * - Monitors wake-up sources
 * - Enters deep sleep if no active inputs
 */
void StateManager::handleStandbyState() {
    // Disable all systems in standby
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    
    // Reset all flags
    batteryArmed = false;
    hasPreCharged = false;
    enableBSC = false;
    enableDMC = false;
    
    nlgCharged = false;
    
    armBattery(false);
    armCoolingSys(false);
    
    // Check for state transitions
    if(digitalRead(Pins::NLG_HW_Wakeup) && !unlockPersist) {
        wasNlgWakeupHigh = true;
        removePinFromWakeSources(Pins::NLG_HW_Wakeup);
        transitionToCharging();
        return;
    }
    
    if(digitalRead(Pins::IGNITION)) {
        addPinToWakeSources(Pins::NLG_HW_Wakeup);
        transitionToRun();
        return;
    }
    
    unlockConnectorRequest = false;
    
    // Make sure connector is unlocked before sleeping
    if (canManager.getNLGData().connectorLocked) {
        canManager.setNLGUnlockRequest(true);
        delay(100); // Give a small delay for processing
        return; // Don't sleep yet, wait until connector is unlocked
    }
    
    // Once we're sure connector is unlocked if needed, we can sleep
    Serial.println("No wake signals, entering deep sleep");
    delay(100); // Small delay to allow serial to finish
    esp_deep_sleep_start();
}

void StateManager::updateDriveMode(DriveMode mode) {
    if (vehicleControl) {
        vehicleControl->setDrivingMode(mode);
    }
}

/**
 * @brief Handle system behavior in RUN state
 * 
 * Manages normal driving operation:
 * - Maintains system enables
 * - Controls cooling system
 * - Manages battery power
 * - Updates status indicators
 * 
 * @note Transitions to STANDBY if ignition is turned off
 */
void StateManager::handleRunState() {
    if (!digitalRead(Pins::IGNITION)) {
        transitionToStandby();
        return;
    }
    
    armCoolingSys(true);
    armBattery(true);
    
    // Only enable DMC if HV system is ready
    if (batteryArmed && hasPreCharged && 
        hvVoltageActual >= VehicleParams::Battery::MIN_VOLTAGE) {
        digitalWrite(Pins::DMCKL15, HIGH);
    } else {
        digitalWrite(Pins::DMCKL15, LOW);  // Keep DMC off until HV is ready
    }
    
    digitalWrite(Pins::DMCKL15, HIGH);
    digitalWrite(Pins::BSCKL15, HIGH);
}

/**
 * @brief Handle system behavior in CHARGING state
 * 
 * Manages charging operation:
 * - Controls charger power
 * - Manages battery state
 * - Handles connector locking
 * - Controls cooling system
 */
void StateManager::handleChargingState() {
    digitalWrite(Pins::NLGKL15, HIGH);
    digitalWrite(Pins::BSCKL15, HIGH);
    
    if(conUlockInterrupt) {
        Serial.println("Connector unlock interrupt detected");
        handleConnectorUnlock();
        //conUlockInterrupt = false; // Clear the interrupt flag after handling
    }
    
    armCoolingSys(true);
    chargeManage();
    canManager.setNLGStateDemand(chargerStateDemand);
    canManager.setNLGLedDemand(chargeLedDemand);
    canManager.setNLGUnlockRequest(unlockConnectorRequest);
}


/**
 * @brief Control battery system state and precharge
 * @param arm true to enable battery system, false to disable
 * 
 * Controls battery power system including:
 * - Precharge sequence management
 * - Contactor control
 * - Mode switching
 * - Error handling
 */
void StateManager::armBattery(bool arm) {
    if (arm) {
        if (!hasPreCharged) {
            // Start precharge process
            if (modeBSC != BSCModes::BSC6_BOOST) {
                Serial.println("Starting precharge - Setting BOOST mode");
                modeBSC = BSCModes::BSC6_BOOST;
                lastModeChangeTime = millis();
                enableBSC = false;
                canManager.setModeBSC(modeBSC);      // Update when mode changes
                canManager.setEnableBSC(enableBSC);  // Update when enable changes
            }

            if (millis() - lastModeChangeTime >= Constants::MODE_CHANGE_DELAY_MS) {
                Serial.println("Precharge Status:");
                Serial.println("Battery Voltage: " + String(batteryVoltage) + "V");
                Serial.println("HV Actual: " + String(hvVoltageActual) + "V");
                Serial.println("BSC Enable State: " + String(enableBSC));
                Serial.println("BSC Mode: " + String(modeBSC));
                hvVoltage = batteryVoltage;  // Set target voltage
                canManager.setHVVoltage(hvVoltage);
                enableBSC = true;
                canManager.setEnableBSC(enableBSC);  // Update when enable changes
                
                // Check if precharge is complete with detailed logging
                if ((hvVoltageActual >= (batteryVoltage - 20)) && 
                    (hvVoltageActual <= (batteryVoltage + 20)) && 
                    (hvVoltageActual > 50)) {
                    Serial.println("Precharge complete - Closing contactor");
                    hasPreCharged = true;
                    digitalWrite(Pins::CONTACTOR, HIGH);
                    digitalWrite(Pins::LWP5, HIGH);
                    enableBSC = false;
                    canManager.setEnableBSC(enableBSC);  // Update when enable changes
                }
                else if (millis() - lastModeChangeTime > Constants::PRECHARGE_TIMEOUT_MS) {
                    Serial.println("Precharge timeout - Voltage not reached");
                    enableBSC = false;
                    canManager.setEnableBSC(enableBSC);  // Update when enable changes
                    lastModeChangeTime = millis(); // Reset for next attempt
                }
            }
        }
        else {
            batteryArmed = true;
            errorLatch = true;
            delay(100);
            errorLatch = false;

            if (modeBSC != BSCModes::BSC6_BUCK) {
                Serial.println("Switching to BUCK mode for normal operation");
                modeBSC = BSCModes::BSC6_BUCK;
                lastModeChangeTime = millis();
                enableBSC = false;
                canManager.setModeBSC(modeBSC);      // Update when mode changes
                canManager.setEnableBSC(enableBSC);  // Update when enable changes
            }

            if (millis() - lastModeChangeTime >= Constants::MODE_CHANGE_DELAY_MS) {
                enableBSC = true;
                canManager.setEnableBSC(enableBSC);  // Update when enable changes
            }
        }
    } else {
        Serial.println("Disabling battery system");
        batteryArmed = false;
        enableBSC = false;
        hasPreCharged = false;
        canManager.setEnableBSC(enableBSC);  // Update when disabling
        digitalWrite(Pins::CONTACTOR, LOW);
        digitalWrite(Pins::LWP5, LOW);
    }
}
/**
 * @brief Control cooling system operation
 * @param arm true to enable cooling, false to disable
 * 
 * Manages cooling system based on:
 * - Temperature thresholds
 * - Cooling requests
 * - Battery system state
 */
void StateManager::armCoolingSys(bool arm) {
    if (batteryArmed) {
        if (arm) {
            // Separate cooling logic based on vehicle state
            if (currentState == VehicleState::CHARGING) {
                // When charging, only consider cooling request
                if (coolingRequest > 50) {
                    digitalWrite(Pins::LWP6, HIGH);
                    digitalWrite(Pins::LWP7, HIGH);
                } else if(coolingRequest < 2){
                    digitalWrite(Pins::LWP6, LOW);
                    digitalWrite(Pins::LWP7, LOW);
                }
            } else if (currentState == VehicleState::RUN) {
                // When driving, only consider motor and inverter temps
                if ((inverterTemp > VehicleParams::Temperature::INV_HIGH) || 
                    (motorTemp > VehicleParams::Temperature::MOT_HIGH)) {
                    digitalWrite(Pins::LWP6, HIGH);
                    digitalWrite(Pins::LWP7, HIGH);
                } else if ((inverterTemp < VehicleParams::Temperature::INV_LOW) && 
                          (motorTemp < VehicleParams::Temperature::MOT_LOW)) {
                    digitalWrite(Pins::LWP6, LOW);
                    digitalWrite(Pins::LWP7, LOW);
                }
            } else {
                // In standby, cooling should be off
                digitalWrite(Pins::LWP6, LOW);
                digitalWrite(Pins::LWP7, LOW);
            }
        } else {
            // If not armed, turn cooling off
            digitalWrite(Pins::LWP6, LOW);
            digitalWrite(Pins::LWP7, LOW);
        }
    } else {
        // If battery not armed, turn cooling off
        digitalWrite(Pins::LWP6, LOW);
        digitalWrite(Pins::LWP7, LOW);
    } 
}

/**
 * @brief Manage charging process state machine
 * 
 * Controls charging sequence including:
 * - State transitions
 * - LED indicators
 * - Connector locking
 * - Charge completion
 */
void StateManager::chargeManage() {
    static unsigned long unlockTimeout = 0;
    static unsigned long connectorUnlockStartTime = 0;
    static bool waitingForUnlockComplete = false;
    static unsigned long connectorLockDelay = 0;
    static bool delayingConnectorLock = true;
    unsigned long currentTime = millis();

    // Regular charger state detection check
    if (currentTime - lastNlgWakeupCheck >= NLG_WAKEUP_CHECK_INTERVAL) {
        bool currentNlgWakeup = digitalRead(Pins::NLG_HW_Wakeup);
        
        // If NLG_HW_Wakeup transitions from HIGH to LOW, charger was unplugged
        if (wasNlgWakeupHigh && !currentNlgWakeup) {
            Serial.println("Charger unplugged detected");
            unlockConnectorRequest = true;
            // We won't immediately sleep - make sure connector is unlocked first
            waitingForUnlockComplete = true;
            connectorUnlockStartTime = currentTime;
        }
        
        wasNlgWakeupHigh = currentNlgWakeup;
        lastNlgWakeupCheck = currentTime;
    }

    // Handle waiting period after connector unlocked before going to sleep
    if (waitingForUnlockComplete) {
        if (!canManager.getNLGData().connectorLocked) {
            // Connector is successfully unlocked, wait for timeout before sleeping
            if (currentTime - connectorUnlockStartTime >= 10000) { // 10 second wait
                waitingForUnlockComplete = false;
                transitionToStandby();
                return;
            }
        } else if (currentTime - connectorUnlockStartTime >= 15000) {
            // Safety timeout if unlock fails after 15 seconds
            waitingForUnlockComplete = false;
            transitionToStandby();
            return;
        }
    }

    if (currentState == VehicleState::CHARGING) {
        // Implement delay before locking connector when first entering charging state
        if (delayingConnectorLock) {
            if (connectorLockDelay == 0) {
                connectorLockDelay = currentTime;
                // Don't lock the connector yet
                unlockConnectorRequest = true;
            } else if (currentTime - connectorLockDelay >= 5000) { // 5 second delay
                // 5 seconds passed, now allow connector to lock
                delayingConnectorLock = false;
                unlockConnectorRequest = false;
            }
        }

        chargerState = canManager.getNLGData().stateAct;
        
        // Check SOC limit - now with special case for 100% SOC
        if (config.getMaxSOC() < 100 && canManager.getBMSData().soc >= config.getMaxSOC()) {
            // For less than 100% SOC, use the standard behavior
            Serial.println("SOC limit reached! Stopping charge.");
            Serial.print("Max SOC limit: ");
            Serial.print(config.getMaxSOC());
            Serial.println("%");
            chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
            unlockConnectorRequest = true;
            unlockPersist = true;
            unlockTimeout = currentTime;
        } else if (config.getMaxSOC() == 100) {
            // For 100% SOC, we check current instead
            if (canManager.getBMSData().soc >= 100 && 
                canManager.getNLGData().dcHvCurrentAct < 30) { // 3 Amps (assuming scale factor of 10)
                Serial.println("Charge current below 3A with 100% SOC - BMS drift reset complete");
                chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
                unlockConnectorRequest = true;
                unlockPersist = true;
                unlockTimeout = currentTime;
            }
        }

        switch (chargerState) {
            case ChargerStates::NLG_ACT_SLEEP:
                chargeLedDemand = 0;
                if (!digitalRead(Pins::NLG_HW_Wakeup)) {
                    // Charger is unplugged, wait for connector to unlock before going to standby
                    unlockConnectorRequest = true;
                    waitingForUnlockComplete = true;
                    connectorUnlockStartTime = currentTime;
                } else {
                    chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                }
                break;

            case ChargerStates::NLG_ACT_STANDBY:
                chargeLedDemand = 1;
                if (nlgCharged) {
                    chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
                    unlockConnectorRequest = true;
                    unlockPersist = true;
                    unlockTimeout = currentTime;
                    // Don't transition to standby yet - wait for connector to unlock
                    waitingForUnlockComplete = true;
                    connectorUnlockStartTime = currentTime;
                }
                break;

            case ChargerStates::NLG_ACT_READY2CHARGE:
                chargeLedDemand = 3;
                if (unlockPersist || unlockConnectorRequest) {
                    chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                } else if (hasPreCharged) {
                    chargerStateDemand = ChargerStates::NLG_DEM_CHARGE;
                }
                armBattery(true);
                break;

            case ChargerStates::NLG_ACT_CHARGE:
                chargeLedDemand = 4;
                armBattery(true);
                break;

            default:
                armBattery(false);
                break;
        }

        if (unlockPersist) {
            handlePersistentUnlock(unlockTimeout);
        }
    } else {
        armBattery(false);
        digitalWrite(Pins::NLGKL15, LOW);
        
        // Reset delay variables when not in charging state
        delayingConnectorLock = true;
        connectorLockDelay = 0;
    }
}

/**
 * @brief Handle persistent connector unlock request
 * @param unlockTimeout Reference to unlock timeout counter
 * 
 * Manages connector unlocking sequence including:
 * - Timeout handling
 * - State transitions
 * - Charging system shutdown
 */
void StateManager::handlePersistentUnlock(unsigned long& unlockTimeout) {
    unlockConnectorRequest = true;
    if (!connectorLocked) {
        chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
        transitionToStandby();
        chargeLedDemand = 0;
    } else if (millis() - unlockTimeout > Constants::UNLOCK_TIMEOUT_MS) {
        unlockTimeout = millis();
    }
}

/**
 * @brief Handle connector unlock interrupt
 * 
 * Processes connector unlock requests:
 * - Validates connector state
 * - Updates charge completion status
 * - Manages state transitions
 */
void StateManager::handleConnectorUnlock() {
    Serial.println("Processing connector unlock request");
    
    // Get current connector status from NLG
    connectorLocked = canManager.getNLGData().connectorLocked;
    
    if (connectorLocked) {
        Serial.println("Connector locked - requesting unlock");
        unlockConnectorRequest = true;
        nlgCharged = true;
        // Force charger to standby state
        chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
    } else {
        Serial.println("Connector already unlocked - transitioning to standby");
        transitionToStandby();
    }
    // Clear the interrupt flag after handling
    conUlockInterrupt = false;
}


/**
 * @brief Get system wake-up source
 * @return Wake-up pin identifier
 * 
 * Determines which GPIO triggered the wake-up event
 */
uint8_t StateManager::getWakeupReason() {
    uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    return (log(GPIO_reason)) / log(2);
}

/**
 * @brief Transition to STANDBY state
 * 
 * Performs safe shutdown sequence:
 * - Disables all systems
 * - Resets state flags
 * - Prepares for sleep
 */
void StateManager::transitionToStandby() {
    Serial.println("Transitioning to STANDBY state");
    currentState = VehicleState::STANDBY;
    nlgCharged = false;
    enableBSC = false;
    enableDMC = false;
    armBattery(false);
    armCoolingSys(false);
    
    // Make sure the connector is unlocked before going to sleep
    if (digitalRead(Pins::NLG_HW_Wakeup) && canManager.getNLGData().connectorLocked) {
        Serial.println("Ensuring connector is unlocked before sleeping");
        canManager.setNLGUnlockRequest(true);
        // Note: In a complete implementation, we'd add a wait loop here with timeout
        // to ensure the connector is actually unlocked before sleeping
    }
    
    Serial.println("Now in STANDBY state");
}

/**
 * @brief Transition to RUN state
 * 
 * Initializes driving operation:
 * - Sets operational state
 * - Resets charging flags
 */
void StateManager::transitionToRun() {
    Serial.println("Transitioning to RUN state");
    currentState = VehicleState::RUN;
    nlgCharged = false;
    Serial.println("Now in RUN state");
}

/**
 * @brief Transition to CHARGING state
 * 
 * Initializes charging operation:
 * - Sets charging state
 * - Prepares charging system
 */
void StateManager::transitionToCharging() {
    Serial.println("Transitioning to CHARGING state");
    currentState = VehicleState::CHARGING;
    Serial.println("Now in CHARGING state");
}

void StateManager::removePinFromWakeSources(uint8_t pin) {
    // Remove pin from EXT1 wake sources
    uint64_t mask = esp_sleep_get_ext1_wakeup_status();
    mask &= ~(1ULL << pin);
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.print("Removed pin ");
    Serial.print(pin);
    Serial.println(" from wake sources");
}

void StateManager::addPinToWakeSources(uint8_t pin) {
    // Add pin to EXT1 wake sources
    uint64_t mask = esp_sleep_get_ext1_wakeup_status();
    mask |= (1ULL << pin);
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.print("Added pin ");
    Serial.print(pin);
    Serial.println(" to wake sources");
}