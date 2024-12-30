/**
 * @file state_manager.h
 * @brief Vehicle State Management System
 * 
 * Manages the vehicle's operational states (Standby, Run, Charging) and handles:
 * - State transitions and safety checks
 * - Battery system control and precharging
 * - Cooling system management
 * - Charging system coordination
 * - Wake-up and sleep management
 */

#pragma once
#include <Arduino.h>
#include "config.h"
#include "vehicle_parameters.h"

class CANManager; // Forward declaration
class StateManager {
public:
    explicit StateManager(CANManager& canMgr);    
    /**
     * @brief Main state machine update function
     * Called regularly to manage state transitions and system controls
     */
    void update();

    /**
     * @brief Handles system wake-up from sleep
     * Determines wake-up source and transitions to appropriate state
     */
    void handleWakeup();

    /**
     * @brief Gets current vehicle state
     * @return Current VehicleState (STANDBY/RUN/CHARGING)
     */
    VehicleState getCurrentState() const { return currentState; }
    
    // State transitions
    /**
     * @brief Transitions to STANDBY state
     * Disables all systems and prepares for sleep
     */
    void transitionToStandby();

    /**
     * @brief Transitions to RUN state
     * Enables drive systems and motor controller
     */
    void transitionToRun();

    /**
     * @brief Transitions to CHARGING state
     * Enables charging systems and prepares battery
     */
    void transitionToCharging();
    
    // System control
    /**
     * @brief Controls battery system state
     * @param arm true to enable battery system, false to disable
     * Manages precharge sequence and contactors
     */
    void armBattery(bool arm);

    /**
     * @brief Controls cooling system
     * @param arm true to enable cooling, false to disable
     * Manages pump and fan control based on temperature
     */
    void armCoolingSys(bool arm);

    /**
     * @brief Manages charging process
     * Handles charging states, connector locking, and charge completion
     */
    void chargeManage();
    
    // Status methods
    /**
     * @brief Checks if battery system is armed
     * @return true if battery system is active and ready
     */
    bool isBatteryArmed() const { return batteryArmed; }

    /**
     * @brief Checks if precharge is complete
     * @return true if precharge sequence finished successfully
     */
    bool isPreCharged() const { return hasPreCharged; }

    /**
     * @brief Checks if vehicle is actively charging
     * @return true if in CHARGING state
     */
    bool isCharging() const { return currentState == VehicleState::CHARGING; }

    /**
     * @brief Checks if charge connector is locked
     * @return true if connector is locked
     */
    bool isConnectorLocked() const { return connectorLocked; }

    /**
     * @brief Gets the wake-up source
     * @return Wake-up pin or source identifier
     */
    uint8_t getWakeupReason();
    
    // Configuration methods
    void setBatteryVoltage(uint16_t voltage) { batteryVoltage = voltage; }
    void setInverterTemp(float temp) { inverterTemp = temp; }
    void setMotorTemp(float temp) { motorTemp = temp; }
    void setCoolingRequest(uint8_t request) { coolingRequest = request; }
    void setHVVoltageActual(float voltage) { hvVoltageActual = voltage; }
    void setHVVoltage(uint16_t voltage) { hvVoltage = voltage; }
    uint16_t getHVVoltage() const { return hvVoltage; }
    float getHVVoltageActual() const { return hvVoltageActual; }
private:
    // Private member functions
    void handleStandbyState();
    void handleRunState();
    void handleChargingState();
    void handleConnectorUnlock();
    void handlePersistentUnlock(unsigned long& unlockTimeout);

    // System references
    CANManager& canManager;

    // Internal state variables
    VehicleState currentState;     // Current vehicle operational state
    bool batteryArmed;            // Battery system active flag
    bool hasPreCharged;           // Precharge complete flag
    bool nlgCharged;              // Charge complete flag
    bool connectorLocked;         // Charge connector state
    bool unlockPersist;           // Persistent unlock request flag
    bool conUlockInterrupt;       // Connector unlock interrupt flag
    bool enableBSC;               // BSC enable state
    bool modeBSC;                 // BSC mode (buck/boost)
    bool errorLatch;              // Error state latch
    bool enableDMC;               // DMC enable state
    
    GearState currentGear = GearState::NEUTRAL;  
    bool unlockConnectorRequest = false;         

    // System parameters
    uint16_t batteryVoltage;      // Current battery voltage
    float inverterTemp;           // Inverter temperature
    float motorTemp;              // Motor temperature
    uint8_t coolingRequest;       // Cooling system demand
    uint8_t chargerState;         // Current charger state
    uint8_t chargerStateDemand;   // Requested charger state
    uint8_t chargeLedDemand;      // Charge LED state request
    float hvVoltageActual;        // Actual HV bus voltage
    uint16_t hvVoltage;           // Target HV bus voltage
    
    // Timing management
    unsigned long lastModeChangeTime;      // Last state change timestamp
    unsigned long lastPrechargeAttempt;    // Last precharge attempt timestamp
};