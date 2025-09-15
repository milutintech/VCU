/**
 * Add these methods to your existing SerialConsole.cpp 
 * Insert after your existing handleSet method
 */

void SerialConsole::handleCommand(String command) {
    command.trim();
    
    // Check if it's a JSON command
    if (command.startsWith("{") && command.endsWith("}")) {
        handleJSONCommand(command);
        return;
    }
    
    // Handle legacy commands (your existing code)
    command.toLowerCase();
    
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);
    
    if (firstColon == -1) {
        if (command == "help") {
            printHelp();
            return;
        } else if (command == "json_help") {
            printJSONHelp();
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

void SerialConsole::handleJSONCommand(const String& command) {
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

void SerialConsole::handleConfigCommand(const JsonDocument& doc) {
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
        JsonObject data = doc["data"];
        
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

void SerialConsole::handleMonitorCommand(const JsonDocument& doc) {
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

void SerialConsole::sendJSONResponse(const String& status, const String& data) {
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
    
    String response;
    serializeJson(doc, response);
    Serial.println(response);
}

void SerialConsole::sendJSONError(const String& message) {
    JsonDocument doc;
    doc["status"] = "error";
    doc["message"] = message;
    
    String response;
    serializeJson(doc, response);
    Serial.println(response);
}

void SerialConsole::printJSONHelp() {
    Serial.println("JSON API Commands:");
    Serial.println("Configuration:");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"get\"} - Get all config");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"get\",\"category\":\"driving\"} - Get driving config");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"set\",\"category\":\"driving\",\"data\":{\"maxTorque\":500}} - Set config");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"save\"} - Save to flash");
    Serial.println("  {\"cmd\":\"config\",\"action\":\"reset\"} - Reset to defaults");
    Serial.println("");
    Serial.println("Monitoring:");
    Serial.println("  {\"cmd\":\"monitor\",\"action\":\"get\"} - Get system status");
    Serial.println("");
    Serial.println("Available categories: driving, curtis");
}