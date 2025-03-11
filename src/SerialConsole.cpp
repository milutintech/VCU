/**
 * @file SerialConsole.cpp
 * @brief Implementation of debug console interface for vehicle control system
 * 
 * Provides a command-line interface for:
 * - Real-time system monitoring
 * - Parameter adjustment
 * - System control and configuration
 * - Diagnostic output
 * 
 * Command format:
 * - GET: get:<system>:<parameter>
 * - SET: set:<system>:<parameter>:<value>
 */

#include "SerialConsole.h"

/**
 * @brief Construct Serial Console interface
 * @param canManager Reference to CAN communication system
 * @param stateManager Reference to vehicle state manager
 * @param vehicleControl Reference to vehicle control system
 */
SerialConsole::SerialConsole(CANManager& canManager, StateManager& stateManager, VehicleControl& vehicleControl)
    : canManager(canManager)
    , stateManager(stateManager)
    , vehicleControl(vehicleControl)
    , inputBuffer("")
{
}

/**
 * @brief Process serial input and handle commands
 * 
 * Reads available serial data and builds commands character by character.
 * Commands are processed when a newline is received.
 */
void SerialConsole::update() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                handleCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}

/**
 * @brief Process received command string
 * @param command Complete command to process
 * 
 * Parses command string in format:
 * action:target:parameter[:value]
 * 
 * Actions:
 * - get: Read system parameters
 * - set: Modify system parameters
 * - help: Show command list
 */
void SerialConsole::handleCommand(String command) {
    command.trim();
    command.toLowerCase();
    
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);
    
    if (firstColon == -1) {
        if (command == "help") {
            printHelp();
            return;
        }
        Serial.println("Invalid command format");
        return;
    }
    
    String action = command.substring(0, firstColon);
    
    if (action == "get") {
        if (secondColon == -1) {
            Serial.println("Invalid get command format");
            return;
        }
        String target = command.substring(firstColon + 1, secondColon);
        String parameter = command.substring(secondColon + 1);
        handleGet(target, parameter);
    }
    else if (action == "set") {
        int valueStart = command.indexOf(':', secondColon + 1);
        if (secondColon == -1 || valueStart == -1) {
            Serial.println("Invalid set command format");
            return;
        }
        String target = command.substring(firstColon + 1, secondColon);
        String parameter = command.substring(secondColon + 1, valueStart);
        String value = command.substring(valueStart + 1);
        handleSet(target, parameter, value);
    }
    else {
        Serial.println("Unknown command: " + action);
    }
}

/**
 * @brief Handle get commands for system monitoring
 * @param target System to query (nlg/bms/dmc/bsc/vcu)
 * @param parameter Parameter to read
 * 
 * Retrieves and displays current system parameters.
 * Supports individual parameter queries and 'all' for complete status.
 */
void SerialConsole::handleGet(String target, String parameter) {
    if (target == "nlg") {
        // Existing NLG code
    }
    else if (target == "bms") {
        // Existing BMS code
    }
    else if (target == "dmc") {
        // Existing DMC code
    }
    else if (target == "bsc") {
        // Existing BSC code
    }
    else if (target == "vcu") {
        // Existing VCU code
    }
    else if (target == "config") {
        if (parameter == "drivemode") {
            Serial.println("Drive Mode: " + config.getDriveModeString());
        }
        else if (parameter == "maxtorque") {
            printValue("Max Torque", config.getMaxTorque(), "Nm");
        }
        else if (parameter == "maxsoc") {
            printValue("Max SOC", config.getMaxSOC(), "%");
        }
        else if (parameter == "maxcurrent") {
            printValue("Max Charging Current", config.getMaxChargingCurrent(), "A");
        }
        else if (parameter == "all") {
            Serial.println("Configuration Settings:");
            Serial.println("Drive Mode: " + config.getDriveModeString());
            printValue("Max Torque", config.getMaxTorque(), "Nm");
            printValue("Max SOC", config.getMaxSOC(), "%");
            printValue("Max Charging Current", config.getMaxChargingCurrent(), "A");
        }
        else {
            Serial.println("Unknown config parameter: " + parameter);
        }
    }
    else {
        Serial.println("Unknown target: " + target);
    }
}


/**
 * @brief Handle set commands for system control
 * @param target System to control
 * @param parameter Parameter to modify
 * @param value New value to set
 * 
 * Processes system control commands and applies requested changes.
 */
void SerialConsole::handleSet(String target, String parameter, String value) {
    if (target == "vcu") {
        // Existing VCU code
    }
    else if (target == "config") {
        if (parameter == "drivemode") {
            if (config.setDriveMode(value)) {
                vehicleControl.setDrivingMode(config.getDriveMode());
                Serial.println("Drive Mode set to: " + config.getDriveModeString());
            } else {
                Serial.println("Invalid drive mode. Use: legacy, regen, or opd");
            }
        }
        else if (parameter == "maxtorque") {
            int torque = value.toInt();
            if (config.setMaxTorque(torque)) {
                Serial.print("Max Torque set to: ");
                Serial.print(torque);
                Serial.println(" Nm");
            } else {
                Serial.println("Invalid torque value. Range: 100-850 Nm");
            }
        }
        else if (parameter == "maxsoc") {
            int soc = value.toInt();
            if (config.setMaxSOC(soc)) {
                Serial.print("Max SOC set to: ");
                Serial.print(soc);
                Serial.println("%");
            } else {
                Serial.println("Invalid SOC value. Range: 50-100%");
            }
        }
        else if (parameter == "maxcurrent") {
            int current = value.toInt();
            if (config.setMaxChargingCurrent(current)) {
                Serial.print("Max Charging Current set to: ");
                Serial.print(current);
                Serial.println(" A");
            } else {
                Serial.println("Invalid current value. Range: 6-32 A");
            }
        }
        else if (parameter == "save") {
            if (config.save()) {
                Serial.println("Configuration saved to flash");
            } else {
                Serial.println("Error saving configuration");
            }
        }
        else if (parameter == "reset") {
            config.resetToDefaults();
            vehicleControl.setDrivingMode(config.getDriveMode());
            Serial.println("Configuration reset to defaults");
        }
        else {
            Serial.println("Unknown config parameter: " + parameter);
        }
    }
    else {
        Serial.println("Unknown target: " + target);
    }
}

/**
 * @brief Display available commands and usage information
 * 
 * Shows comprehensive list of supported commands grouped by subsystem.
 */
void SerialConsole::printHelp() {
    Serial.println("Available commands:");
    Serial.println("\nGet commands:");
    Serial.println("  get:nlg:[state|voltage|current|temp|all]");
    Serial.println("  get:bms:[soc|voltage|current|maxcurrent|all]");
    Serial.println("  get:dmc:[motortemp|invertertemp|status|all]");
    Serial.println("  get:bsc:[hvvoltage|lvvoltage|hvcurrent|lvcurrent|mode|all]");
    Serial.println("  get:vcu:[state|all]");
    Serial.println("\nSet commands:");
    Serial.println("  set:vcu:drivemode:[legacy|regen|opd]");
    Serial.println("  set:vcu:bsckl15:[0|1]");
    Serial.println("  set:vcu:dmckl15:[0|1]");
    Serial.println("  set:vcu:nlgkl15:[0|1]");
    Serial.println("  set:vcu:pump:[0|1]");
    Serial.println("\nOther commands:");
    Serial.println("  help - Show this help message");
    Serial.println("\nConfiguration commands:");
    Serial.println("  get:config:[drivemode|maxtorque|maxsoc|maxcurrent|all]");
    Serial.println("  set:config:drivemode:[legacy|regen|opd]");
    Serial.println("  set:config:maxtorque:[100-850]");
    Serial.println("  set:config:maxsoc:[50-100]");
    Serial.println("  set:config:maxcurrent:[6-32]");
    Serial.println("  set:config:save - Save configuration to flash");
    Serial.println("  set:config:reset - Reset to default configuration");
}

/**
 * @brief Print integer value with optional unit
 * @param name Parameter name
 * @param value Integer value
 * @param unit Optional unit string (V, A, °C, etc)
 */
void SerialConsole::printValue(const String& name, int value, const String& unit) {
    Serial.print(name + ": ");
    Serial.print(value);
    if (unit.length() > 0) {
        Serial.print(unit);
    }
    Serial.println();
}

/**
 * @brief Print float value with optional unit
 * @param name Parameter name
 * @param value Float value
 * @param unit Optional unit string
 */
void SerialConsole::printValue(const String& name, float value, const String& unit) {
    Serial.print(name + ": ");
    Serial.print(value, 1);
    if (unit.length() > 0) {
        Serial.print(unit);
    }
    Serial.println();
}

/**
 * @brief Print boolean value
 * @param name Parameter name
 * @param value Boolean state
 */
void SerialConsole::printValue(const String& name, bool value) {
    Serial.println(name + ": " + (value ? "True" : "False"));
}
