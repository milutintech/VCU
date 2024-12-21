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
        transitionToCharging();
    } else if (digitalRead(Pins::IGNITION)) {
        Serial.println("IGNITION pin is HIGH");
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
    
    batteryArmed = false;
    hasPreCharged = false;
    enableBSC = false;
    enableDMC = false;
    
    armBattery(false);
    armCoolingSys(false);
    
    // Check for state transitions
    if(digitalRead(Pins::NLG_HW_Wakeup)) {
        transitionToCharging();
    }
    if(digitalRead(Pins::IGNITION)) {
        transitionToRun();
    }
    
    // Enter deep sleep if no active inputs
    if((!digitalRead(Pins::IGNITION)) && (!digitalRead(Pins::NLG_HW_Wakeup))) {
        esp_deep_sleep_start();
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
    
    digitalWrite(Pins::DMCKL15, HIGH);
    digitalWrite(Pins::BSCKL15, HIGH);
    
    // Update reverse light based on gear state
    digitalWrite(Pins::BCKLIGHT, currentGear == GearState::REVERSE ? HIGH : LOW);
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
        handleConnectorUnlock();
    }
    
    armCoolingSys(true);
    chargeManage();
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
                modeBSC = BSCModes::BSC6_BOOST;
                lastModeChangeTime = millis();
                enableBSC = false;
            }

            if (millis() - lastModeChangeTime >= Constants::MODE_CHANGE_DELAY_MS) {
                Serial.println("Battery Voltage: " + String(batteryVoltage));
                hvVoltage = batteryVoltage;  // Set the target voltage
                canManager.setHVVoltage(hvVoltage);  // NEW LINE: Update CAN manager
                enableBSC = true;
                
                // Check if precharge is complete
                if ((hvVoltageActual >= (batteryVoltage - 20)) && 
                    (hvVoltageActual <= (batteryVoltage + 20)) && 
                    (hvVoltageActual > 50)) {
                    hasPreCharged = true;
                    digitalWrite(Pins::CONTACTOR, HIGH);
                    digitalWrite(Pins::LWP5, HIGH);
                    enableBSC = false;
                }
            }
        }
        else {
            batteryArmed = true;
            errorLatch = true;
            delay(100);
            errorLatch = false;

            if (modeBSC != BSCModes::BSC6_BUCK) {
                modeBSC = BSCModes::BSC6_BUCK;
                lastModeChangeTime = millis();
                enableBSC = false;
            }

            if (millis() - lastModeChangeTime >= Constants::MODE_CHANGE_DELAY_MS) {
                enableBSC = true;
            }
        }
    } else {
        batteryArmed = false;
        enableBSC = false;
        hasPreCharged = false;
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
        if (arm && ((inverterTemp > VehicleParams::Temperature::INV_HIGH) || 
                   (motorTemp > VehicleParams::Temperature::MOT_HIGH) || 
                   (coolingRequest > 50))) {
            digitalWrite(Pins::LWP6, HIGH);
            digitalWrite(Pins::LWP7, HIGH);
        } else if ((inverterTemp < VehicleParams::Temperature::INV_LOW) && 
                  (motorTemp < VehicleParams::Temperature::MOT_LOW) && 
                  (coolingRequest < 0)) {
            digitalWrite(Pins::LWP6, LOW);
            digitalWrite(Pins::LWP7, LOW); 
        }
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

    if (currentState == VehicleState::CHARGING) {
        switch (chargerState) {
            case ChargerStates::NLG_ACT_SLEEP:
                chargeLedDemand = 0;
                chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                break;

            case ChargerStates::NLG_ACT_STANDBY:
                chargeLedDemand = 1;
                if (nlgCharged) {
                    chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
                    transitionToStandby();
                    unlockConnectorRequest = true;
                    unlockPersist = true;
                    unlockTimeout = millis();
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
    if (connectorLocked) {
        unlockConnectorRequest = true;
        nlgCharged = true;
    } else {
        conUlockInterrupt = false;
        chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
        transitionToStandby();
    }
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