/**
 * @file SerialConsole.h
 * @brief Debug Console Interface for Vehicle Control System
 * 
 * Provides comprehensive debug interface for:
 * - Real-time system monitoring
 * - Parameter adjustment
 * - Diagnostics and testing
 * - State control
 * 
 * Command Structure:
 * - GET commands: get:<system>:<parameter>
 * - SET commands: set:<system>:<parameter>:<value>
 * 
 * Supported Systems:
 * - NLG (Charger): State, voltage, current, temperature
 * - BMS: SOC, voltage, current limits
 * - DMC (Motor): Temperature, status, torque
 * - BSC (DC-DC): Voltage, current, mode
 * - VCU: State, drive mode, system control
 */

#pragma once
#include <Arduino.h>
#include "can_manager.h"
#include "state_manager.h"
#include "vehicle_control.h"
#include "config.h"

class SerialConsole {
public:
    /**
     * @brief Constructs debug console interface
     * @param canManager Reference to CAN communication system
     * @param stateManager Reference to vehicle state control
     * @param vehicleControl Reference to motor control system
     * 
     * Initializes monitoring interface and command parser
     */
    SerialConsole(CANManager& canManager, StateManager& stateManager, VehicleControl& vehicleControl);

    /**
     * @brief Process serial input and handle commands
     * Should be called in main control loop
     * 
     * Handles:
     * - Command buffering
     * - Command parsing
     * - Response formatting
     * - Error handling
     */
    void update();
    
private:
    /**
     * @brief Process received command string
     * @param command Complete command string to process
     * 
     * Supported Commands:
     * 1. NLG (Charger) Commands:
     *    - get:nlg:state - Current charger state
     *    - get:nlg:voltage - Charging voltage
     *    - get:nlg:current - Charging current
     *    - get:nlg:temp - Charger temperature
     *    - get:nlg:all - All charger parameters
     * 
     * 2. BMS Commands:
     *    - get:bms:maxcurrent - Maximum allowed current
     *    - get:bms:soc - State of charge
     *    - get:bms:voltage - Battery voltage
     *    - get:bms:current - Battery current
     *    - get:bms:all - All battery parameters
     * 
     * 3. DMC (Motor) Commands:
     *    - get:dmc:motortemp - Motor temperature
     *    - get:dmc:invertertemp - Inverter temperature
     *    - get:dmc:status - Operation status
     *    - get:dmc:all - All motor parameters
     * 
     * 4. BSC (DC-DC) Commands:
     *    - get:bsc:hvvoltage - High voltage
     *    - get:bsc:lvcurrent - Low voltage current
     *    - get:bsc:mode - Operating mode
     *    - get:bsc:all - All converter parameters
     * 
     * 5. VCU Control Commands:
     *    - get:vcu:state - System state
     *    - get:vcu:all - All system parameters
     *    - set:vcu:drivemode:[legacy|regen|opd] - Set drive mode
     *    - set:vcu:bsckl15:[0|1] - Control BSC power
     *    - set:vcu:dmckl15:[0|1] - Control DMC power
     *    - set:vcu:nlgkl15:[0|1] - Control NLG power
     *    - set:vcu:pump:[0|1] - Control cooling pump
     */
    void handleCommand(String command);

    /**
     * @brief Handle get commands for system monitoring
     * @param target System to query (nlg/bms/dmc/bsc/vcu)
     * @param parameter Specific parameter to read
     * 
     * Response Format:
     * "<parameter_name>: <value> <unit>"
     * 
     * Error Handling:
     * - Invalid target: "Unknown system: <target>"
     * - Invalid parameter: "Unknown parameter: <parameter>"
     * - Read error: "Error reading <parameter>"
     */
    void handleGet(String target, String parameter);

    /**
     * @brief Handle set commands for system control
     * @param target System to control
     * @param parameter Parameter to modify
     * @param value New value to set
     * 
     * Safety Checks:
     * - Value range validation
     * - System state validation
     * - Operation permissions
     * 
     * Error Handling:
     * - Invalid value: "Invalid value for <parameter>"
     * - Operation not allowed: "Operation not allowed in current state"
     */
    void handleSet(String target, String parameter, String value);

    /**
     * @brief Display help information and command list
     * Shows all available commands with descriptions
     * Groups commands by subsystem for clarity
     */
    void printHelp();
    
    /**
     * @brief Print integer value with optional unit
     * @param name Parameter name
     * @param value Integer value
     * @param unit Optional unit string (V, A, °C, etc)
     */
    void printValue(const String& name, int value, const String& unit = "");

    /**
     * @brief Print float value with optional unit
     * @param name Parameter name
     * @param value Float value
     * @param unit Optional unit string
     */
    void printValue(const String& name, float value, const String& unit = "");

    /**
     * @brief Print boolean value
     * @param name Parameter name
     * @param value Boolean state
     * 
     * Outputs "True" or "False" for better readability
     */
    void printValue(const String& name, bool value);
    
    CANManager& canManager;        ///< Reference to CAN system
    StateManager& stateManager;    ///< Reference to state system
    VehicleControl& vehicleControl;///< Reference to vehicle control
    String inputBuffer;            ///< Command input buffer
};