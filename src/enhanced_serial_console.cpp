/**
 * @file enhanced_serial_console.cpp
 * @brief Implementation of enhanced console interface for vehicle control system
 * 
 * Provides both legacy and JSON command interfaces for:
 * - Real-time system monitoring
 * - Parameter adjustment
 * - System control and configuration
 * - Diagnostic output
 */

#include "enhanced_serial_console.h"

/**
 * @brief Construct Enhanced Serial Console interface
 * @param canManager Reference to CAN communication system
 * @param stateManager Reference to vehicle state manager
 * @param vehicleControl Reference to vehicle control system
 * @param errorMonitor Reference to error monitoring system
 */
EnhancedSerialConsole::EnhancedSerialConsole(CANManager& canManager, StateManager& stateManager, 
                                           VehicleControl& vehicleControl, ErrorMonitor& errorMonitor)
    : canManager(canManager)
    , stateManager(stateManager)
    , vehicleControl(vehicleControl)
    , errorMonitor(errorMonitor)
    , inputBuffer("")
    , currentStreamingMode(StreamingMode::NONE)
    , streamingInterval(100)
    , lastStreamTime(0)
{
}

/**
 * @brief Process serial input and handle commands
 * 
 * Reads available serial data and builds commands character by character.
 * Commands are processed when a newline is received.
 */
void EnhancedSerialConsole::update() {
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
    
    // Process streaming if active
    processStreaming();
}

/**
 * @brief Process received command string
 * @param command Complete command to process
 */
void EnhancedSerialConsole::handleCommand(const String& command) {
    String cmd = command;
    cmd.trim();
    
    // Check if it's a JSON command
    if (cmd.startsWith("{") && cmd.endsWith("}")) {
        handleJSONCommand(cmd);
        return;
    }
    
    // Handle legacy commands
    handleLegacyCommand(cmd);
}

/**
 * @brief Handle legacy command format
 * @param command Legacy format command
 */
void EnhancedSerialConsole::handleLegacyCommand(const String& command) {
    String cmd = command;
    cmd.toLowerCase();
    
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    
    if (firstColon == -1) {
        if (cmd == "help") {
            printHelp();
            return;
        } else if (cmd == "json_help") {
            printJSONHelp();
            return;
        }
        Serial.println("Invalid command format");
        return;
    }
    
    String action = cmd.substring(0, firstColon);
    
    if (action == "get") {
        if (secondColon == -1) {
            Serial.println("Invalid get command format");
            return;
        }
        String target = cmd.substring(firstColon + 1, secondColon);
        String parameter = cmd.substring(secondColon + 1);
        handleGet(target, parameter);
    }
    else if (action == "set") {
        int valueStart = cmd.indexOf(':', secondColon + 1);
        if (secondColon == -1 || valueStart == -1) {
            Serial.println("Invalid set command format");
            return;
        }
        String target = cmd.substring(firstColon + 1, secondColon);
        String parameter = cmd.substring(secondColon + 1, valueStart);
        String value = cmd.substring(valueStart + 1);
        handleSet(target, parameter, value);
    }
    else {
        Serial.println("Unknown command: " + action);
    }
}

/**
 * @brief Handle JSON commands
 * @param command JSON format command
 */
void EnhancedSerialConsole::handleJSONCommand(const String& command) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, command);
    
    if (error) {
        sendJSONError("Invalid JSON format");
        return;
    }
    
    if (!doc["cmd"].is<const char*>()) {
        sendJSONError("Missing 'cmd' field");
        return;
    }
    
    String cmd = doc["cmd"].as<String>();
    
    if (cmd == "config") {
        handleConfigCommand(doc);
    } else if (cmd == "monitor") {
        handleMonitorCommand(doc);
    } else if (cmd == "help") {
        sendJSONResponse("success", "Available commands: config, monitor");
    } else {
        sendJSONError("Unknown command: " + cmd);
    }
}

/**
 * @brief Handle configuration commands - FIXED for ArduinoJson v7
 * @param doc JSON document with config command
 */
void EnhancedSerialConsole::handleConfigCommand(const JsonDocument& doc) {
    String action = doc["action"].as<String>();
    
    if (action == "get") {
        String category = doc["category"].as<String>();
        if (category.isEmpty()) {
            // Return all config
            sendJSONResponse("success", config.toJSON());
        } else {
            // Return specific category
            sendJSONResponse("success", config.getCategoryJSON(category));
        }
    } else if (action == "set") {
        String category = doc["category"].as<String>();
        
        // FIXED: Proper JsonObject handling for ArduinoJson v7
        if (!doc["data"].is<JsonObjectConst>()) {
            sendJSONError("Missing or invalid data field");
            return;
        }
        
        JsonObjectConst data = doc["data"].as<JsonObjectConst>();
        
        if (category.isEmpty() || data.isNull()) {
            sendJSONError("Missing category or data");
            return;
        }
        
        String dataStr;
        serializeJson(data, dataStr);
        
        if (config.setCategoryJSON(category, dataStr)) {
            sendJSONResponse("success", "Configuration updated");
            
            // Apply changes immediately
            if (category == "driving") {
                vehicleControl.setDrivingMode(config.getDriveMode());
            }
        } else {
            sendJSONError("Invalid configuration data");
        }
    } else if (action == "save") {
        if (config.save()) {
            sendJSONResponse("success", "Configuration saved");
        } else {
            sendJSONError("Failed to save configuration");
        }
    } else if (action == "reset") {
        config.resetToDefaults();
        vehicleControl.setDrivingMode(config.getDriveMode());
        sendJSONResponse("success", "Configuration reset to defaults");
    } else {
        sendJSONError("Unknown config action: " + action);
    }
}
/**
 * @brief Handle monitoring commands
 * @param doc JSON document with monitor command
 */
void EnhancedSerialConsole::handleMonitorCommand(const JsonDocument& doc) {
    String action = doc["action"].as<String>();
    
    if (action == "get") {
        // Create monitoring data JSON
        JsonDocument monDoc;
        
        // Vehicle status
        monDoc["timestamp"] = millis();
        monDoc["vehicleState"] = (int)stateManager.getCurrentState();
        monDoc["batteryArmed"] = stateManager.isBatteryArmed();
        monDoc["prechargeComplete"] = stateManager.isPreCharged();
        monDoc["charging"] = stateManager.isCharging();
        
        // BMS data
        const BMSData& bms = canManager.getBMSData();
        JsonObject bmsObj = monDoc["bms"].to<JsonObject>();
        bmsObj["soc"] = bms.soc;
        bmsObj["voltage"] = bms.voltage;
        bmsObj["current"] = bms.current;
        bmsObj["maxDischarge"] = bms.maxDischarge;
        bmsObj["maxCharge"] = bms.maxCharge;
        
        // DMC data
        const DMCData& dmc = canManager.getDMCData();
        JsonObject dmcObj = monDoc["dmc"].to<JsonObject>();
        dmcObj["ready"] = dmc.ready;
        dmcObj["running"] = dmc.running;
        dmcObj["torqueActual"] = dmc.torqueActual;
        dmcObj["speedActual"] = dmc.speedActual;
        dmcObj["tempInverter"] = dmc.tempInverter;
        dmcObj["tempMotor"] = dmc.tempMotor;
        
        // BSC data
        const BSCData& bsc = canManager.getBSCData();
        JsonObject bscObj = monDoc["bsc"].to<JsonObject>();
        bscObj["hvVoltageAct"] = bsc.hvVoltageAct;
        bscObj["lvVoltageAct"] = bsc.lvVoltageAct;
        bscObj["hvCurrentAct"] = bsc.hvCurrentAct;
        bscObj["lvCurrentAct"] = bsc.lvCurrentAct;
        bscObj["mode"] = bsc.mode;
        
        // NLG data
        const NLGData& nlg = canManager.getNLGData();
        JsonObject nlgObj = monDoc["nlg"].to<JsonObject>();
        nlgObj["stateAct"] = nlg.stateAct;
        nlgObj["dcHvVoltageAct"] = nlg.dcHvVoltageAct;
        nlgObj["dcHvCurrentAct"] = nlg.dcHvCurrentAct;
        nlgObj["connectorLocked"] = nlg.connectorLocked;
        nlgObj["tempCoolPlate"] = nlg.tempCoolPlate;
        
        String result;
        serializeJson(monDoc, result);
        sendJSONResponse("success", result);
        
    } else {
        sendJSONError("Unknown monitor action: " + action);
    }
}

/**
 * @brief Send JSON response
 * @param status Response status
 * @param data Optional data payload
 * @param error Optional error message
 */
void EnhancedSerialConsole::sendJSONResponse(const String& status, const String& data, const String& error) {
    JsonDocument doc;
    doc["status"] = status;
    if (!data.isEmpty()) {
        if (data.startsWith("{") || data.startsWith("[")) {
            JsonDocument dataDoc;
            deserializeJson(dataDoc, data);
            doc["data"] = dataDoc;
        } else {
            doc["message"] = data;
        }
    }
    if (!error.isEmpty()) {
        doc["error"] = error;
    }
    
    String response;
    serializeJson(doc, response);
    Serial.println(response);
}

/**
 * @brief Send JSON error response
 * @param message Error message
 */
void EnhancedSerialConsole::sendJSONError(const String& message) {
    JsonDocument doc;
    doc["status"] = "error";
    doc["message"] = message;
    
    String response;
    serializeJson(doc, response);
    Serial.println(response);
}

/**
 * @brief Handle get commands for system monitoring
 * @param target System to query
 * @param parameter Parameter to read
 */
void EnhancedSerialConsole::handleGet(const String& target, const String& parameter) {
    if (target == "config") {
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
 */
void EnhancedSerialConsole::handleSet(const String& target, const String& parameter, const String& value) {
    if (target == "config") {
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
 */
void EnhancedSerialConsole::printHelp() {
    Serial.println("Available commands:");
    Serial.println("\nGet commands:");
    Serial.println("  get:config:[drivemode|maxtorque|maxsoc|maxcurrent|all]");
    Serial.println("\nSet commands:");
    Serial.println("  set:config:drivemode:[legacy|regen|opd]");
    Serial.println("  set:config:maxtorque:[100-850]");
    Serial.println("  set:config:maxsoc:[50-100]");
    Serial.println("  set:config:maxcurrent:[6-32]");
    Serial.println("  set:config:save - Save configuration to flash");
    Serial.println("  set:config:reset - Reset to default configuration");
    Serial.println("\nOther commands:");
    Serial.println("  help - Show this help message");
    Serial.println("  json_help - Show JSON API commands");
}

/**
 * @brief Display JSON API help
 */
void EnhancedSerialConsole::printJSONHelp() {
    Serial.println("JSON API Commands:");
    Serial.println("Configuration:");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"get\"} - Get all config");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"get\",\"category\":\"driving\"} - Get driving config");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"get\",\"category\":\"curtis\"} - Get Curtis power limits");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"get\",\"category\":\"pedal\"} - Get pedal zones");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"get\",\"category\":\"transitions\"} - Get transition timing");  // NEW
    Serial.println("  {\"cmd\":\"config\",\"action\":\"set\",\"category\":\"transitions\",\"data\":{...}} - Set transitions");  // NEW
    Serial.println("  {\"cmd\":\"config\",\"action\":\"save\"} - Save to flash");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"reset\"} - Reset to defaults");
    Serial.println("");
    Serial.println("Monitoring:");
    Serial.println("  {\"cmd\":\"monitor\",\"action\":\"get\"} - Get system status");
    Serial.println("");
    Serial.println("Available categories: driving, curtis, pedal, transitions");  // Updated
}


/**
 * @brief Print integer value with optional unit
 * @param name Parameter name
 * @param value Integer value
 * @param unit Optional unit string
 */
void EnhancedSerialConsole::printValue(const String& name, int value, const String& unit) {
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
void EnhancedSerialConsole::printValue(const String& name, float value, const String& unit) {
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
void EnhancedSerialConsole::printValue(const String& name, bool value) {
    Serial.println(name + ": " + (value ? "True" : "False"));
}

/**
 * @brief Process streaming output
 */
void EnhancedSerialConsole::processStreaming() {
    if (currentStreamingMode == StreamingMode::NONE) {
        return;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastStreamTime < streamingInterval) {
        return;
    }
    
    lastStreamTime = currentTime;
    
    // Implement streaming based on mode
    switch (currentStreamingMode) {
        case StreamingMode::MONITORING:
            // Stream monitoring data
            break;
        case StreamingMode::CAN_MESSAGES:
            // Stream CAN messages
            break;
        default:
            break;
    }
}

/**
 * @brief Set streaming mode
 * @param mode Streaming mode to activate
 * @param intervalMs Update interval in milliseconds
 */
void EnhancedSerialConsole::setStreamingMode(StreamingMode mode, unsigned long intervalMs) {
    currentStreamingMode = mode;
    streamingInterval = intervalMs;
    lastStreamTime = 0;
}

/**
 * @brief Stop streaming
 */
void EnhancedSerialConsole::stopStreaming() {
    currentStreamingMode = StreamingMode::NONE;
}

// Stub implementations for additional methods (to be implemented as needed)
void EnhancedSerialConsole::handleConfigGet(const String& category) {}
void EnhancedSerialConsole::handleConfigSet(const String& category, const String& jsonData) {}
void EnhancedSerialConsole::handleConfigUpload(const String& jsonData) {}
void EnhancedSerialConsole::handleConfigDownload() {}
void EnhancedSerialConsole::handleConfigBackup() {}
void EnhancedSerialConsole::handleConfigRestore(const String& backupData) {}
void EnhancedSerialConsole::handleConfigReset(const String& category) {}
void EnhancedSerialConsole::handleMonitoringGet() {}
void EnhancedSerialConsole::handlePerformanceGet() {}
void EnhancedSerialConsole::handleErrorLogGet(int maxEntries) {}
void EnhancedSerialConsole::handleCANLogGet(int maxEntries) {}
void EnhancedSerialConsole::handleSystemHealthGet() {}
void EnhancedSerialConsole::handleStreamStart(const String& type, int intervalMs) {}
void EnhancedSerialConsole::handleStreamStop() {}
void EnhancedSerialConsole::handleSystemTest(const String& component) {}
void EnhancedSerialConsole::handleCalibrationStart(const String& type) {}
void EnhancedSerialConsole::handleCalibrationSet(const String& param, float value) {}
void EnhancedSerialConsole::handleCalibrationSave() {}
void EnhancedSerialConsole::handleCANMonitorToggle(bool enable) {}
void EnhancedSerialConsole::handleErrorClear() {}
void EnhancedSerialConsole::handlePerformanceReset() {}
void EnhancedSerialConsole::handleDeviceControl(const String& device, const String& action, const String& value) {}
void EnhancedSerialConsole::handleEmergencyStop() {}
void EnhancedSerialConsole::handleSafeMode(bool enable) {}
void EnhancedSerialConsole::handleCANSend(uint32_t id, const String& hexData) {}
void EnhancedSerialConsole::handleCANFilter(const String& filterConfig) {}
void EnhancedSerialConsole::handleLegacyGet(const String& target, const String& parameter) {}
void EnhancedSerialConsole::handleLegacySet(const String& target, const String& parameter, const String& value) {}
void EnhancedSerialConsole::sendError(const String& message) {}
void EnhancedSerialConsole::sendSuccess(const String& message) {}
void EnhancedSerialConsole::sendData(const String& data) {}
bool EnhancedSerialConsole::isJSONCommand(const String& command) { return false; }
JsonDocument EnhancedSerialConsole::parseJSON(const String& json) { return JsonDocument(); }
String EnhancedSerialConsole::createResponse(const String& status, const JsonObject& data) { return ""; }
bool EnhancedSerialConsole::validateJSONStructure(const JsonObject& obj, const String& expectedType) { return false; }
bool EnhancedSerialConsole::validateParameterRange(const String& param, float value, float min, float max) { return false; }