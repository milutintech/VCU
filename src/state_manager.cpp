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
#include "fault_handler.h"  // FaultLimits::NLG_TIMEOUT_MS, shared with FaultHandler's own NLG_TIMEOUT fault
#include <esp_sleep.h>
#include "configuration.h"  // Add this include at the top
#include "vehicle_control.h"
#include "pi_power.h"
#include "ethernet_link.h"
#include "error_monitor.h"


/**
 * @brief Constructor - initializes state manager with safe defaults
 * 
 * Sets initial system state to STANDBY and ensures all
 * subsystems start in a safe, disabled state.
 */
StateManager::StateManager(CANManager& canMgr, VehicleControl* vc)
    : canManager(canMgr)
    , vehicleControl(vc) 
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
{
    // Initialize GPIO outputs to safe states
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    digitalWrite(Pins::CONTACTOR, LOW);
    digitalWrite(Pins::FAN, LOW);
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
    
    // Get the actual wakeup reason from ESP32
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    uint8_t gpio_reason = getWakeupReason();
    
    Serial.print("ESP32 wakeup cause: ");
    Serial.println(wakeup_reason);
    Serial.print("GPIO wakeup reason: ");
    Serial.println(gpio_reason);
    
    // First check for specific wake sources based on ESP32's wake cause
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        // EXT0 is configured for IGNITION pin
        Serial.println("Woken up by IGNITION (EXT0)");
        unlockPersist = false;
        transitionToRun();
        return;
    } 
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        // For EXT1, we need to check which pin caused the wakeup
        Serial.println("Woken up by EXT1 source");
        
        // Check if NLG_HW_Wakeup pin is high (charging wake)
        if (digitalRead(Pins::NLG_HW_Wakeup) == HIGH) {
            Serial.println("NLG_HW_Wakeup pin is HIGH");
            if (!unlockPersist) {
                transitionToCharging();
                return;
            }
        }
        
        // Check if IGNITION pin is high
        if (digitalRead(Pins::IGNITION) == HIGH) {
            Serial.println("IGNITION pin is HIGH");
            unlockPersist = false;
            transitionToRun();
            return;
        }
        
        // Check if UNLCKCON pin is high
        if (digitalRead(Pins::UNLCKCON) == HIGH) {
            Serial.println("UNLCKCON pin is HIGH");
            // Handle unlock button specific wakeup if needed
        }
    }
    
    // If we get here, no specific wake source was identified from ESP32's mechanism
    // Fall back to direct pin reading as a secondary method
    
    if (digitalRead(Pins::NLG_HW_Wakeup) == HIGH) {
        Serial.println("NLG_HW_Wakeup pin is HIGH (direct check)");
        if (!unlockPersist) {
            transitionToCharging();
            return;
        }
    } 
    else if (digitalRead(Pins::IGNITION) == HIGH) {
        Serial.println("IGNITION pin is HIGH (direct check)");
        unlockPersist = false;
        transitionToRun();
        return;
    }
    
    // If no wake signals detected through any method, go to standby
    Serial.println("No wake signals detected, going to standby");
    transitionToStandby(EventLog::StopReason::NO_WAKE_SIGNAL);
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
    updatePlugState();
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

    if (vehicleControl) vehicleControl->setReverseSignals(false);   // also held through deep sleep
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
    
    // A session ended on purpose (unlock button, SOC limit) stays ended while the plug is in.
    // Unplugging clears that, so plugging in again starts a new session without an ignition cycle.
    if (plugRemoved && unlockPersist) {
        Serial.println("Plug removed: next plug-in starts a new charging session");
        unlockPersist = false;
    }

    // Check for state transitions
    const bool plugHigh = digitalRead(Pins::NLG_HW_Wakeup);
    if (plugHigh && !unlockPersist) {
        transitionToCharging();
        return;
    }
    
    if(digitalRead(Pins::IGNITION)) {
        transitionToRun();
        return;
    }
    
    unlockConnectorRequest = false;

    // Never deep sleep with the plug in: NLG_HW_Wakeup is a wake source (ANY_HIGH) and would wake
    // the VCU at once. Stay awake until it is unplugged.
    if (plugHigh) {
        if (!waitUnplugLogged) {
            waitUnplugLogged = true;
            Serial.println("Charging finished, plug still in: waiting for unplug before deep sleep");
            logEvent(EventLog::Ev::WAIT_UNPLUG);
        }
        return;
    }
    
    // Make sure connector is unlocked before sleeping
    /*if (canManager.getNLGData().connectorLocked) {
        canManager.setNLGUnlockRequest(true);
        delay(100); // Give a small delay for processing
        return; // Don't sleep yet, wait until connector is unlocked
    }*/
    
    // The Pi gets its shutdown first: no deep sleep while it has power or is shutting down.
    // The transitions above stay live, so the car can still start in the meantime.
    if (piPower && !piPower->mayDeepSleep()) {
        return;
    }

    // Once we're sure connector is unlocked if needed, we can sleep
    Serial.println("No wake signals, entering deep sleep");
    configureWakeSources();
    logEvent(EventLog::Ev::DEEP_SLEEP,
             (digitalRead(Pins::IGNITION) ? 1 : 0) | (digitalRead(Pins::NLG_HW_Wakeup) ? 2 : 0) |
             (digitalRead(Pins::UNLCKCON) ? 4 : 0));
    if (piPower) piPower->holdForSleep();
    EthernetLink::holdForSleep();
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
    // Check if ignition is off
    if (!digitalRead(Pins::IGNITION)) {
        transitionToStandby(EventLog::StopReason::IGNITION_OFF);
        return;
    }

    // Check if charger is connected - transition to CHARGING
    // NLG_HW_Wakeup pin HIGH indicates charger is connected
    if (digitalRead(Pins::NLG_HW_Wakeup) == HIGH) {
        Serial.println("[STATE] Charger detected during RUN state, transitioning to CHARGING");
        transitionToCharging();
        return;
    }

    armCoolingSys(true);
    armBattery(true);

    // DMC KL15 (IO13) also powers the Orion BMS, so it is on for all of RUN: precharge needs the
    // BMS pack voltage. It is only the DMC's logic supply; the DMC is enabled to drive (0x210)
    // only once precharged (canTask, isPreCharged()).
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
    if (vehicleControl) vehicleControl->setReverseSignals(false);   // plugged in while in R
    digitalWrite(Pins::NLGKL15, HIGH);
    digitalWrite(Pins::BSCKL15, HIGH);

    // CRITICAL: CHARGING state should NEVER transition to RUN, even if ignition is on
    // User must unplug charger first, which will transition to SLEEP/STANDBY
    // Then turning on ignition will go SLEEP -> RUN
    if (digitalRead(Pins::IGNITION) == HIGH) {
        Serial.println("[STATE] WARNING: Ignition turned on during CHARGING - staying in CHARGING state");
        Serial.println("[STATE] Unplug charger first, then turn on ignition to drive");
        // Do NOT transition to RUN - this is intentional safety behavior
    }

    if(conUlockInterrupt) {
        Serial.println("Connector unlock interrupt detected");
        handleConnectorUnlock();
        //conUlockInterrupt = false; // Clear the interrupt flag after handling
    }

    armCoolingSys(true);
    chargeManage();
    static uint8_t lastDemand = 0xFF;
    if (chargerStateDemand != lastDemand) {
        logEvent(EventLog::Ev::CHARGER_DEMAND, lastDemand, chargerStateDemand);
        lastDemand = chargerStateDemand;
    }
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
    static unsigned long errorClearStartTime = 0;
    static bool inErrorClearSequence = false;
    
    if (arm) {
        if (!hasPreCharged) {
            if (prechargeFailed) {
                return;   // gave up, BSC already off; STANDBY resets it
            }
            const bool voltageOk = packVoltageValid();
            if (!voltageOk || !hvDrawAllowed()) {
                // No attempt without a plausible pack voltage, or in RUN while the BMS forbids
                // discharging: BSC off, and the attempt (including the mode change delay)
                // starts over once both are fine. Waiting does not count as an attempt.
                static unsigned long lastWaitPrint = 0;
                if (millis() - lastWaitPrint >= 5000) {
                    lastWaitPrint = millis();
                    if (!voltageOk) {
                        Serial.println("Precharge waiting: pack voltage " + String(batteryVoltage) + "V not in " +
                                       String(VehicleParams::Battery::MIN_VOLTAGE) + "-" +
                                       String(VehicleParams::Battery::MAX_VOLTAGE) + "V or no BMS data");
                    } else {
                        Serial.println("Precharge waiting: BMS does not allow discharging");
                    }
                }
                if (enableBSC) {
                    enableBSC = false;
                    canManager.setEnableBSC(enableBSC);
                }
                inErrorClearSequence = false;
                lastModeChangeTime = millis();
                return;
            }

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
                // Status once per second, this runs every control cycle
                static unsigned long lastStatusPrint = 0;
                if (millis() - lastStatusPrint >= 1000) {
                    lastStatusPrint = millis();
                    Serial.println("Precharge Status:");
                    Serial.println("Battery Voltage: " + String(batteryVoltage) + "V");
                    Serial.println("HV Actual: " + String(hvVoltageActual) + "V");
                    Serial.println("BSC Enable State: " + String(enableBSC));
                    Serial.println("BSC Mode: " + String(modeBSC));
                }
                hvVoltage = batteryVoltage;  // Set target voltage
                canManager.setHVVoltage(hvVoltage);
                enableBSC = true;
                canManager.setEnableBSC(enableBSC);  // Update when enable changes
                
                // Check if precharge is complete with detailed logging
                if ((hvVoltageActual >= (batteryVoltage - 20)) && 
                    (hvVoltageActual <= (batteryVoltage + 20)) && 
                    (hvVoltageActual > 50)) {
                    
                    // Start non-blocking error clearing sequence if not already in progress
                    if (!inErrorClearSequence) {
                        Serial.println("Precharge complete - Beginning error clearing sequence");
                        // First disable BSC before error latch sequence
                        enableBSC = false;
                        canManager.setEnableBSC(enableBSC);
                        
                        // Start error clearing sequence
                        errorLatch = true;
                        canManager.setNeedsClearError(true);
                        errorClearStartTime = millis();
                        inErrorClearSequence = true;
                    }
                    // Check if error clearing sequence has completed its 100ms timing
                    else if (millis() - errorClearStartTime >= 100) {
                        Serial.println("Error clearing sequence completed - Closing contactor");
                        // Clear error latch
                        errorLatch = false;
                        canManager.setNeedsClearError(false);
                        inErrorClearSequence = false;
                        
                        // Now complete the precharge sequence
                        hasPreCharged = true;
                        logEvent(EventLog::Ev::PRECHARGE_DONE, 0, static_cast<uint16_t>(hvVoltageActual));
                        digitalWrite(Pins::CONTACTOR, HIGH);
                        digitalWrite(Pins::LWP5, HIGH);
                    }
                    // While in the 100ms wait period, continue normal operation
                }
                else if (millis() - lastModeChangeTime > Constants::PRECHARGE_TIMEOUT_MS) {
                    enableBSC = false;
                    canManager.setEnableBSC(enableBSC);  // Update when enable changes
                    lastModeChangeTime = millis(); // Reset for next attempt
                    prechargeAttempts++;
                    Serial.println("Precharge timeout - Voltage not reached (attempt " +
                                   String(prechargeAttempts) + "/" + String(MAX_PRECHARGE_ATTEMPTS) + ")");
                    logEvent(EventLog::Ev::PRECHARGE_TIMEOUT, prechargeAttempts,
                             static_cast<uint16_t>(hvVoltageActual));
                    if (prechargeAttempts >= MAX_PRECHARGE_ATTEMPTS) {
                        prechargeFailed = true;
                        Serial.println("Precharge failed, giving up until ignition off");
                        logEvent(EventLog::Ev::PRECHARGE_FAILED, prechargeAttempts);
                        if (errorMonitor) {
                            errorMonitor->raiseFault(ErrorCode::PRECHARGE_FAILED, ErrorSeverity::ERROR,
                                                     "Precharge failed after " + String(prechargeAttempts) +
                                                     " attempts", hvVoltageActual, "BSC");
                        }
                    }
                }
            }
        }
        else {
            batteryArmed = true;

            if (modeBSC != BSCModes::BSC6_BUCK) {
                Serial.println("Switching to BUCK mode for normal operation");
                modeBSC = BSCModes::BSC6_BUCK;
                lastModeChangeTime = millis();
                enableBSC = false;
                canManager.setModeBSC(modeBSC);      // Update when mode changes
                canManager.setEnableBSC(enableBSC);  // Update when enable changes
            }

            if (!hvDrawAllowed()) {
                // Buck draws from the pack. The contactor stays closed (DMC is already
                // disabled by the fault); the BSC comes back 2 s after discharge is allowed.
                if (enableBSC) {
                    Serial.println("BSC off: BMS does not allow discharging");
                    enableBSC = false;
                    canManager.setEnableBSC(enableBSC);
                }
                lastModeChangeTime = millis();
            } else if (millis() - lastModeChangeTime >= Constants::MODE_CHANGE_DELAY_MS) {
                enableBSC = true;
                canManager.setEnableBSC(enableBSC);  // Update when enable changes
            }
        }
    } else {
        // Called every cycle while disarmed: only report when something was actually on.
        if (batteryArmed || hasPreCharged || enableBSC || inErrorClearSequence) {
            Serial.println("Disabling battery system");
            logEvent(EventLog::Ev::BATTERY_OFF, static_cast<uint8_t>(currentState));
        }
        batteryArmed = false;
        enableBSC = false;
        hasPreCharged = false;
        inErrorClearSequence = false;  // Reset error clearing sequence state
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
    unsigned long currentTime = millis();
    
    if (batteryArmed) {
        if (arm) {
            // Separate cooling logic based on vehicle state
            if (currentState == VehicleState::CHARGING) {
                // Pump control when charging
                if (coolingRequest > 50) {
                    digitalWrite(Pins::PUMP, HIGH); // Turn on pump
                    if (!pumpRunningTimer) {
                        pumpStartTime = currentTime;
                        pumpRunningTimer = true;
                    }
                } else if(coolingRequest < 2) {
                    // Only turn off if minimum run time has elapsed
                    if (!pumpRunningTimer || (currentTime - pumpStartTime >= MIN_RUN_TIME)) {
                        digitalWrite(Pins::PUMP, LOW); // Turn off pump
                        pumpRunningTimer = false;
                    }
                }
                
                // Fan control when charging
                if (coolingRequest >= 53) {
                    digitalWrite(Pins::FAN, HIGH); // Turn on fan
                    if (!fanRunningTimer) {
                        fanStartTime = currentTime;
                        fanRunningTimer = true;
                    }
                } else if(coolingRequest == 0) {
                    // Only turn off if minimum run time has elapsed
                    if (!fanRunningTimer || (currentTime - fanStartTime >= MIN_RUN_TIME)) {
                        digitalWrite(Pins::FAN, LOW); // Turn off fan
                        fanRunningTimer = false;
                    }
                }
            } else if (currentState == VehicleState::RUN) {
                // Reset timers when not charging
                pumpRunningTimer = false;
                fanRunningTimer = false;
                
                // Pump control when driving
                if ((inverterTemp > VehicleParams::Temperature::INV_HIGH) || 
                    (motorTemp > VehicleParams::Temperature::MOT_HIGH)) {
                    digitalWrite(Pins::PUMP, HIGH); // Turn on pump
                } else if ((inverterTemp < VehicleParams::Temperature::INV_LOW) &&
                          (motorTemp < VehicleParams::Temperature::MOT_LOW)) {
                    digitalWrite(Pins::PUMP, LOW); // Turn off pump
                }
                
                // Fan control when driving
                if ((inverterTemp > VehicleParams::Temperature::FAN_ON) || 
                    (motorTemp > VehicleParams::Temperature::FAN_ON)) {
                    digitalWrite(Pins::FAN, HIGH); // Turn on fan
                } else if ((inverterTemp < VehicleParams::Temperature::INV_LOW) &&
                          (motorTemp < VehicleParams::Temperature::MOT_LOW)) {
                    digitalWrite(Pins::FAN, LOW); // Turn off fan
                }
            } else {
                // In standby, cooling should be off
                digitalWrite(Pins::FAN, LOW); // Fan off
                digitalWrite(Pins::PUMP, LOW); // Pump off
                // Reset timers
                pumpRunningTimer = false;
                fanRunningTimer = false;
            }
        } else {
            // If not armed, turn cooling off
            digitalWrite(Pins::FAN, LOW); // Fan off
            digitalWrite(Pins::PUMP, LOW); // Pump off
            // Reset timers
            pumpRunningTimer = false;
            fanRunningTimer = false;
        }
    } else {
        // If battery not armed, turn cooling off
        digitalWrite(Pins::FAN, LOW); // Fan off
        digitalWrite(Pins::PUMP, LOW); // Pump off
        // Reset timers
        pumpRunningTimer = false;
        fanRunningTimer = false;
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
    unsigned long currentTime = millis();

    // Unplugged: NLG_HW_Wakeup LOW for NLG_UNPLUG_DEBOUNCE_MS (updatePlugState). Unplugging also
    // ends a session that was finished on purpose, so the next plug-in charges again.
    if (plugRemoved) {
        Serial.println("Charger unplugged detected");
        unlockConnectorRequest = true;
        unlockPersist = false;
        transitionToStandby(EventLog::StopReason::UNPLUGGED);
        return;
    }

    // Handle waiting period after connector unlocked before going to sleep
    if (waitingForUnlockComplete) {
        if (!canManager.getNLGData().connectorLocked) {
            // Connector is successfully unlocked, wait for timeout before sleeping
            if (currentTime - connectorUnlockStartTime >= 4000) { // 4 second wait
                waitingForUnlockComplete = false;
                transitionToStandby(EventLog::StopReason::UNLOCK_DONE);
                return;
            }
        } else if (currentTime - connectorUnlockStartTime >= 15000) {
            // Safety timeout if unlock fails after 15 seconds
            waitingForUnlockComplete = false;
            transitionToStandby(EventLog::StopReason::UNLOCK_TIMEOUT);
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

        // NLG CAN link lost: chargerState above is stale, so don't drive the switch below
        // off it (it could keep demanding CHARGE on a charger we can no longer hear from).
        // Fail safe by holding at STANDBY until a fresh NLG_ACT frame arrives.
        const unsigned long lastNlg = canManager.getLastNLGMs();
        const unsigned long nlgAge = msSince(lastNlg, currentTime);
        const bool nlgTimedOut = lastNlg != 0 && nlgAge > FaultLimits::NLG_TIMEOUT_MS;
        static bool wasNlgTimedOut = false;
        if (nlgTimedOut != wasNlgTimedOut) {
            wasNlgTimedOut = nlgTimedOut;
            logEvent(EventLog::Ev::NLG_TIMEOUT, nlgTimedOut ? 1 : 0, static_cast<uint16_t>(min(nlgAge, 65535UL)));
        }
        if (nlgTimedOut) {
            chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
        } else {
            // SOC limit. At 100 % the SOC is ignored: charging continues until unplugged and
            // is governed only by the BMS (charge limit CCL and charge enable flag).
            if (config.getMaxSOC() < 100 && canManager.getBMSData().soc >= config.getMaxSOC()) {
                // For less than 100% SOC, use the standard behavior
                if (!unlockPersist) logEvent(EventLog::Ev::SOC_LIMIT, config.getMaxSOC());
                Serial.println("SOC limit reached! Stopping charge.");
                Serial.print("Max SOC limit: ");
                Serial.print(config.getMaxSOC());
                Serial.println("%");
                chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
                unlockConnectorRequest = true;
                unlockPersist = true;
                unlockTimeout = currentTime;
            }

            switch (chargerState) {
                case ChargerStates::NLG_ACT_SLEEP:
                    // Plug still in (an unplug is handled at the top): wake the charger up.
                    // This used to start the unlock wait on one LOW reading of the plug line.
                    chargeLedDemand = 0;
                    chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                    break;

                case ChargerStates::NLG_ACT_STANDBY:
                    chargeLedDemand = 1;
                    if (nlgCharged) {
                        chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
                        unlockConnectorRequest = true;
                        unlockPersist = true;
                        unlockTimeout = currentTime;
                        // Don't transition to standby yet - wait for connector to unlock.
                        // Start the timeout once: this branch runs on every pass, restarting it
                        // here kept the 4 s / 15 s exits below from ever expiring.
                        if (!waitingForUnlockComplete) {
                            waitingForUnlockComplete = true;
                            connectorUnlockStartTime = currentTime;
                        }
                    } else if (!unlockPersist && !unlockConnectorRequest) {
                        // The charger ended a charge by itself (CHARGE -> SHUTDOWN -> STANDBY) and
                        // waits here. It only goes to READY2CHARGE while the demand is STANDBY (as
                        // at the start of the session); a CHARGE demand left over kept it in
                        // STANDBY for good. READY2CHARGE then asks for CHARGE again. The battery
                        // stays armed (no new precharge). Owner, 2026-11-20.
                        chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                    }
                    break;

                case ChargerStates::NLG_ACT_READY2CHARGE:
                    chargeLedDemand = 3;
                    if (unlockPersist || unlockConnectorRequest) {
                        chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                    } else if (hasPreCharged) {
                        chargerStateDemand = ChargerStates::NLG_DEM_CHARGE;
                    }
                    break;

                case ChargerStates::NLG_ACT_CHARGE:
                    chargeLedDemand = 4;
                    // Restore the CHARGE demand: one that dropped to STANDBY for a cycle (NLG
                    // timeout) used to stay there until the charger left CHARGE. A stop on purpose
                    // (unlock button, SOC limit, unlock request) keeps its demand.
                    if (!unlockPersist && !unlockConnectorRequest && hasPreCharged) {
                        chargerStateDemand = ChargerStates::NLG_DEM_CHARGE;
                    }
                    break;

                default:
                    break;
            }
        }

        // The battery stays armed for the whole charging session, whatever the charger reports:
        // it used to be disarmed on any other charger state (WAKEUP, SHUTDOWN, a bad frame), which
        // opened the contactor and re-ran precharge many times per session and left the 12 V side
        // without the DC-DC. It is disarmed when the session ends (transitionToStandby).
        armBattery(true);

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
    // Follow the charger: the flag was only refreshed by the unlock interrupt, so it stayed
    // "locked" and this function never reached standby.
    connectorLocked = canManager.getNLGData().connectorLocked;
    if (!connectorLocked) {
        chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
        transitionToStandby(EventLog::StopReason::PERSIST_UNLOCK);
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
    logEvent(EventLog::Ev::UNLOCK_BUTTON, connectorLocked ? 1 : 0);
    // The button ends the session: no new one until the plug has been removed (STANDBY waits).
    unlockPersist = true;
    
    if (connectorLocked) {
        Serial.println("Connector locked - requesting unlock");
        unlockConnectorRequest = true;
        nlgCharged = true;
        // Force charger to standby state
        chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
        
        // Add these lines to enable proper transition to standby
        waitingForUnlockComplete = true;
        connectorUnlockStartTime = millis();
    } else {
        Serial.println("Connector already unlocked - transitioning to standby");
        transitionToStandby(EventLog::StopReason::BUTTON_UNLOCKED);
    }
    // Clear the interrupt flag after handling
    conUlockInterrupt = false;
}

/**
 * @brief Get system wake-up source GPIO number
 * @return Wake-up pin identifier or 0 if no GPIO wakeup
 * 
 * Determines which GPIO triggered the wake-up event from EXT1 source
 */
uint8_t StateManager::getWakeupReason() {
    uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    
    // If no GPIO was triggered, return 0
    if (GPIO_reason == 0) {
        return 0;
    }
    
    // Use logarithm method to find the highest bit set
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
void StateManager::transitionToStandby(EventLog::StopReason reason) {
    Serial.println("Transitioning to STANDBY state");
    logEvent(EventLog::Ev::STATE, static_cast<uint8_t>(VehicleState::STANDBY), static_cast<uint16_t>(reason));
    resetPrecharge();
    currentState = VehicleState::STANDBY;
    nlgCharged = false;
    waitingForUnlockComplete = false;
    waitUnplugLogged = false;
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
    logEvent(EventLog::Ev::STATE, static_cast<uint8_t>(VehicleState::RUN));
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
    logEvent(EventLog::Ev::STATE, static_cast<uint8_t>(VehicleState::CHARGING));
    currentState = VehicleState::CHARGING;
    // New session: nothing from the previous one carries over (unlock wait, connector lock delay).
    nlgCharged = false;
    waitingForUnlockComplete = false;
    delayingConnectorLock = true;
    connectorLockDelay = 0;
    Serial.println("Now in CHARGING state");
}

/** Fresh BMS frame and a pack voltage within the battery limits. */
bool StateManager::packVoltageValid() const {
    const unsigned long lastBms = canManager.getLastBMSMs();
    return lastBms != 0 &&
           msSince(lastBms, millis()) <= VehicleParams::Timing::BMS_TIMEOUT_MS &&
           batteryVoltage >= VehicleParams::Battery::MIN_VOLTAGE &&
           batteryVoltage <= VehicleParams::Battery::MAX_VOLTAGE;
}

/**
 * May the BSC draw from the pack? In RUN only while the BMS allows discharging. In CHARGING
 * always: the Orion clears discharge-allowed while plugged in (interlock), and the DC-DC has
 * to keep the 12 V side up during charging.
 */
bool StateManager::hvDrawAllowed() const {
    return currentState != VehicleState::RUN || canManager.dischargeAllowed();
}

/** New ignition cycle (or charge session): precharge may try again. */
void StateManager::resetPrecharge() {
    prechargeAttempts = 0;
    if (prechargeFailed) {
        prechargeFailed = false;
        // Cleared here, not in FaultHandler: only this sequence knows when it may retry.
        if (errorMonitor) errorMonitor->clearFault(ErrorCode::PRECHARGE_FAILED);
    }
}

/**
 * Wake sources for deep sleep, set right before sleeping. The old add/remove helpers built the
 * mask from esp_sleep_get_ext1_wakeup_status(), which is the pin that caused the last wake-up,
 * not the configured mask: after a charging session the plug line was often missing, so the VCU
 * did not wake up when the charger came back. Deep sleep only happens with the plug line LOW
 * (handleStandbyState), so it can always be a wake source.
 */
void StateManager::configureWakeSources() {
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(Pins::IGNITION), 1);
    const uint64_t mask = (1ULL << Pins::NLG_HW_Wakeup) | (1ULL << Pins::IGNITION) | (1ULL << Pins::UNLCKCON);
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);
}

/** Debounces NLG_HW_Wakeup: plugRemoved once it stayed LOW for NLG_UNPLUG_DEBOUNCE_MS. */
void StateManager::updatePlugState() {
    const unsigned long now = millis();
    if (digitalRead(Pins::NLG_HW_Wakeup)) {
        if (nlgWakeLowSince != 0) {
            const unsigned long lowMs = now - nlgWakeLowSince;
            if (!plugRemoved) {
                // Came back before the debounce: logged so short drops show up in the event log.
                Serial.println("NLG wake line LOW for " + String(lowMs) + " ms, ignored");
                logEvent(EventLog::Ev::PLUG_GLITCH, 0, static_cast<uint16_t>(min(lowMs, 65535UL)));
            }
            nlgWakeLowSince = 0;
        }
        plugRemoved = false;
    } else {
        if (nlgWakeLowSince == 0) nlgWakeLowSince = now ? now : 1;
        plugRemoved = now - nlgWakeLowSince >= NLG_UNPLUG_DEBOUNCE_MS;
    }
}

void StateManager::logEvent(EventLog::Ev ev, uint8_t a, uint16_t b) {
    EventLog::add(ev, a, b, batteryVoltage, canManager.getBMSData().soc);
}
