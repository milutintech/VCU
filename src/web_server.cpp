#include "web_server.h"

// Helper functions for state/gear conversion
static String stateToString(VehicleState state) {
    switch(state) {
        case VehicleState::STANDBY: return "STANDBY";
        case VehicleState::RUN: return "RUN";
        case VehicleState::CHARGING: return "CHARGING";
        default: return "UNKNOWN";
    }
}

static String gearToString(GearState gear) {
    switch(gear) {
        case GearState::NEUTRAL: return "N";
        case GearState::DRIVE: return "D";
        case GearState::REVERSE: return "R";
        default: return "N";
    }
}

VCUWebServer::VCUWebServer(Configuration* cfg, StateManager* sm, VehicleControl* vc,
                           CANManager* can, ErrorMonitor* err)
    : server(80)
    , ws("/ws")
    , config(cfg)
    , stateManager(sm)
    , vehicleControl(vc)
    , canManager(can)
    , errorMonitor(err)
    , debugMode(false)
    , lastWSBroadcast(0)
{
}

void VCUWebServer::begin() {
    Serial.println("\n========================================");
    Serial.println("    Initializing Web Server...");
    Serial.println("========================================");

    // Initialize LittleFS
    Serial.print("[WebServer] Mounting LittleFS... ");
    if (!LittleFS.begin(true)) {
        Serial.println("FAILED!");
        Serial.println("[WebServer] ERROR: LittleFS Mount Failed");
        Serial.println("[WebServer] Web interface files may not be available");
        Serial.println("[WebServer] Run: pio run --target uploadfs");
        // Continue anyway - API will still work
    } else {
        Serial.println("OK");
        Serial.println("[WebServer] LittleFS Mounted Successfully");

        // List files in LittleFS for debugging
        Serial.println("[WebServer] Files in LittleFS:");
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        int fileCount = 0;
        while (file) {
            Serial.print("  - ");
            Serial.print(file.name());
            Serial.print(" (");
            Serial.print(file.size());
            Serial.println(" bytes)");
            file = root.openNextFile();
            fileCount++;
        }
        if (fileCount == 0) {
            Serial.println("  (No files found - run: pio run --target uploadfs)");
        }
    }

    // Setup routes and handlers
    Serial.print("[WebServer] Setting up routes... ");
    setupRoutes();
    Serial.println("OK");

    Serial.print("[WebServer] Setting up WebSocket handlers... ");
    setupWebSocketHandlers();
    Serial.println("OK");

    // Start server
    Serial.print("[WebServer] Starting HTTP server on port 80... ");
    server.begin();
    Serial.println("OK");

    Serial.println("[WebServer] HTTP server started successfully");
    Serial.println("========================================");
    Serial.println("Web Server Ready!");
    Serial.println("========================================\n");
}

void VCUWebServer::update() {
    unsigned long now = millis();

    // Broadcast live data via WebSocket
    if (now - lastWSBroadcast >= WS_BROADCAST_INTERVAL) {
        lastWSBroadcast = now;
        broadcastLiveData();
    }

    // Cleanup WebSocket clients
    ws.cleanupClients();
}

void VCUWebServer::setDebugMode(bool enabled) {
    debugMode = enabled;
    Serial.print("[WebServer] Debug mode: ");
    Serial.println(enabled ? "ENABLED" : "DISABLED");
}

void VCUWebServer::setupRoutes() {
    // Register API routes first (they take priority over static files)
    // This ensures /api/* requests go to handlers, not filesystem

    // Add fallback root handler in case LittleFS is empty
    server.on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // Try to serve from LittleFS first
        if (LittleFS.exists("/index.html")) {
            request->send(LittleFS, "/index.html", "text/html");
            return;
        }

        // If no index.html, serve a simple fallback page
        String html = "<!DOCTYPE html><html><head><title>VCU Web Server</title></head><body>";
        html += "<h1>VCU Web Server Running!</h1>";
        html += "<p>The web server is operational, but web interface files are not uploaded.</p>";
        html += "<h2>System Status</h2>";
        html += "<p>State: " + stateToString(stateManager->getCurrentState()) + "</p>";
        html += "<p>Battery SOC: " + String(canManager->getBMSData().soc) + "%</p>";
        html += "<p>Battery Voltage: " + String(canManager->getBMSData().voltage / 10.0f) + "V</p>";
        html += "<h2>How to upload web interface:</h2>";
        html += "<pre>pio run --target uploadfs</pre>";
        html += "<h2>Available API Endpoints:</h2>";
        html += "<ul>";
        html += "<li><a href='/api/status'>/api/status</a> - Full system status</li>";
        html += "<li><a href='/api/status/live'>/api/status/live</a> - Live data</li>";
        html += "<li><a href='/api/config'>/api/config</a> - Configuration</li>";
        html += "</ul>";
        html += "</body></html>";
        request->send(200, "text/html", html);
    });

    // Status endpoints
    server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetStatus(request);
    });

    server.on("/api/status/live", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetLiveData(request);
    });

    server.on("/api/status/errors", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetErrors(request);
    });

    // Configuration endpoints - GET
    // IMPORTANT: Register ALL specific routes BEFORE the generic /api/config route to avoid prefix matching
    server.on("/api/config/driving", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetDrivingConfig(request);
    });

    server.on("/api/config/charging", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetChargingConfig(request);
    });

    server.on("/api/config/limits", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetLimitsConfig(request);
    });

    server.on("/api/config/pedal", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetPedalConfig(request);
    });

    server.on("/api/config/transition", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetTransitionConfig(request);
    });

    server.on("/api/config/throttle", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetThrottleConfig(request);
    });

    server.on("/api/throttle/raw", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetRawThrottle(request);
    });

    // Generic config endpoint - MUST be last to avoid catching specific routes
    server.on("/api/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetConfig(request);
    });

    // Configuration endpoints - POST (with JSON body)
    AsyncCallbackWebHandler* drivingHandler = new AsyncCallbackWebHandler();
    drivingHandler->setUri("/api/config/driving");
    drivingHandler->setMethod(HTTP_POST);
    drivingHandler->onRequest([this](AsyncWebServerRequest* request) {
        // Body handled in onBody callback
    });
    drivingHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleSetDrivingConfig(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(drivingHandler);

    AsyncCallbackWebHandler* chargingHandler = new AsyncCallbackWebHandler();
    chargingHandler->setUri("/api/config/charging");
    chargingHandler->setMethod(HTTP_POST);
    chargingHandler->onRequest([](AsyncWebServerRequest* request) {});
    chargingHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleSetChargingConfig(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(chargingHandler);

    AsyncCallbackWebHandler* limitsHandler = new AsyncCallbackWebHandler();
    limitsHandler->setUri("/api/config/limits");
    limitsHandler->setMethod(HTTP_POST);
    limitsHandler->onRequest([](AsyncWebServerRequest* request) {});
    limitsHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleSetLimitsConfig(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(limitsHandler);

    AsyncCallbackWebHandler* pedalHandler = new AsyncCallbackWebHandler();
    pedalHandler->setUri("/api/config/pedal");
    pedalHandler->setMethod(HTTP_POST);
    pedalHandler->onRequest([](AsyncWebServerRequest* request) {});
    pedalHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleSetPedalConfig(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(pedalHandler);

    AsyncCallbackWebHandler* transitionHandler = new AsyncCallbackWebHandler();
    transitionHandler->setUri("/api/config/transition");
    transitionHandler->setMethod(HTTP_POST);
    transitionHandler->onRequest([](AsyncWebServerRequest* request) {});
    transitionHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleSetTransitionConfig(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(transitionHandler);

    AsyncCallbackWebHandler* throttleHandler = new AsyncCallbackWebHandler();
    throttleHandler->setUri("/api/config/throttle");
    throttleHandler->setMethod(HTTP_POST);
    throttleHandler->onRequest([](AsyncWebServerRequest* request) {});
    throttleHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleSetThrottleConfig(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(throttleHandler);

    // System endpoints
    server.on("/api/system/save", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSaveConfig(request);
    });

    server.on("/api/system/reset", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleResetConfig(request);
    });

    server.on("/api/system/export", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleExportConfig(request);
    });

    AsyncCallbackWebHandler* importHandler = new AsyncCallbackWebHandler();
    importHandler->setUri("/api/system/import");
    importHandler->setMethod(HTTP_POST);
    importHandler->onRequest([](AsyncWebServerRequest* request) {});
    importHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleImportConfig(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(importHandler);

    server.on("/api/system/info", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleSystemInfo(request);
    });

    AsyncCallbackWebHandler* debugHandler = new AsyncCallbackWebHandler();
    debugHandler->setUri("/api/system/debug");
    debugHandler->setMethod(HTTP_POST);
    debugHandler->onRequest([](AsyncWebServerRequest* request) {});
    debugHandler->onBody([this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            JsonVariant json = doc.as<JsonVariant>();
            handleSetDebugMode(request, json);
        } else {
            request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        }
    });
    server.addHandler(debugHandler);

    // CAN Monitor endpoint
    server.on("/api/can/messages", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetCANMessages(request);
    });

    // Serve static files from LittleFS (AFTER all API routes)
    // API routes registered above take priority, so this won't interfere
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // 404 handler
    server.onNotFound([](AsyncWebServerRequest* request) {
        request->send(404, "text/plain", "Not Found");
    });
}

void VCUWebServer::setupWebSocketHandlers() {
    ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                     AwsEventType type, void* arg, uint8_t* data, size_t len) {
        handleWebSocketEvent(server, client, type, arg, data, len);
    });

    server.addHandler(&ws);
}

// ===== STATUS ENDPOINTS =====

void VCUWebServer::handleGetStatus(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["state"] = stateToString(stateManager->getCurrentState());
    doc["gear"] = gearToString(vehicleControl ? vehicleControl->getCurrentGear() : GearState::NEUTRAL);
    doc["uptime"] = millis() / 1000;
    doc["debugMode"] = debugMode;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void VCUWebServer::handleGetLiveData(AsyncWebServerRequest* request) {
    String json = createLiveDataJSON();
    request->send(200, "application/json", json);
}

void VCUWebServer::handleGetErrors(AsyncWebServerRequest* request) {
    // Get recent errors from error monitor
    JsonDocument doc;
    JsonArray errors = doc["errors"].to<JsonArray>();

    // Add error log entries (this would need to be implemented in ErrorMonitor)
    // For now, return empty array
    errors.add("Error log not yet implemented");

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// ===== CONFIGURATION ENDPOINTS - GET =====

void VCUWebServer::handleGetConfig(AsyncWebServerRequest* request) {
    String json = config->toJSON();
    request->send(200, "application/json", json);
}

void VCUWebServer::handleGetDrivingConfig(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["driveMode"] = config->getDriveModeString();
    doc["maxTorque"] = config->getMaxTorque();
    doc["maxSOC"] = config->getMaxSOC();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void VCUWebServer::handleGetChargingConfig(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["maxChargingCurrent"] = config->getMaxChargingCurrent();
    doc["maxSOC"] = config->getMaxSOC();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void VCUWebServer::handleGetLimitsConfig(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["maxTorque"] = config->getMaxTorque();
    doc["baseSpeed"] = config->getBaseSpeed();
    doc["deltaSpeed"] = config->getDeltaSpeed();
    doc["nominalPower"] = config->getNominalPower();

    JsonArray driveArray = doc["drivePowerLimits"].to<JsonArray>();
    const float* driveLimits = config->getDrivePowerLimits();
    for (int i = 0; i < 5; i++) {
        driveArray.add(driveLimits[i]);
    }

    JsonArray regenArray = doc["regenPowerLimits"].to<JsonArray>();
    const float* regenLimits = config->getRegenPowerLimits();
    for (int i = 0; i < 5; i++) {
        regenArray.add(regenLimits[i]);
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void VCUWebServer::handleGetPedalConfig(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["regenZoneEnd"] = config->getRegenZoneEnd();
    doc["coastZoneEnd"] = config->getCoastZoneEnd();
    doc["regenProgression"] = config->getRegenProgression();
    doc["accelProgression"] = config->getAccelProgression();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void VCUWebServer::handleGetTransitionConfig(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["regenEngageTime"] = config->getRegenEngageTime();
    doc["regenReleaseTime"] = config->getRegenReleaseTime();
    doc["powerEngageTime"] = config->getPowerEngageTime();
    doc["powerReleaseTime"] = config->getPowerReleaseTime();
    doc["crossoverTime"] = config->getCrossoverTime();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void VCUWebServer::handleGetThrottleConfig(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["throttleMinADC"] = config->getThrottleMinADC();
    doc["throttleMaxADC"] = config->getThrottleMaxADC();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void VCUWebServer::handleGetRawThrottle(AsyncWebServerRequest* request) {
    JsonDocument doc;

    doc["rawADC"] = vehicleControl->getRawThrottleADC();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// ===== CONFIGURATION ENDPOINTS - POST =====

void VCUWebServer::handleSetDrivingConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    JsonObject obj = json.as<JsonObject>();
    bool success = true;

    if (!obj["driveMode"].isNull()) {
        success &= config->setDriveMode(obj["driveMode"].as<String>());
        vehicleControl->setDrivingMode(config->getDriveMode());
    }

    if (!obj["maxTorque"].isNull()) {
        success &= config->setMaxTorque(obj["maxTorque"].as<int>());
    }

    if (!obj["maxSOC"].isNull()) {
        success &= config->setMaxSOC(obj["maxSOC"].as<uint8_t>());
    }

    if (success) {
        config->save();
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid parameters\"}");
    }
}

void VCUWebServer::handleSetChargingConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    JsonObject obj = json.as<JsonObject>();
    bool success = true;

    Serial.println("[WebServer] Setting charging config...");

    if (!obj["maxChargingCurrent"].isNull()) {
        uint8_t current = obj["maxChargingCurrent"].as<uint8_t>();
        Serial.printf("[WebServer] Setting max charging current: %d A\n", current);
        success &= config->setMaxChargingCurrent(current);
    }

    if (!obj["maxSOC"].isNull()) {
        uint8_t soc = obj["maxSOC"].as<uint8_t>();
        Serial.printf("[WebServer] Setting max SOC: %d%%\n", soc);
        success &= config->setMaxSOC(soc);
    }

    if (success) {
        Serial.println("[WebServer] Saving config to flash...");
        bool saved = config->save();
        Serial.printf("[WebServer] Config save result: %s\n", saved ? "SUCCESS" : "FAILED");

        // Verify what was saved
        Serial.printf("[WebServer] Verifying saved values - maxSOC: %d%%, maxChargingCurrent: %d A\n",
                     config->getMaxSOC(), config->getMaxChargingCurrent());

        request->send(200, "application/json", "{\"success\":true}");
    } else {
        Serial.println("[WebServer] Invalid parameters");
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid parameters\"}");
    }
}

void VCUWebServer::handleSetLimitsConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    JsonObject obj = json.as<JsonObject>();
    bool success = true;

    Serial.println("[WebServer] Setting limits config...");

    if (!obj["maxTorque"].isNull()) {
        int torque = obj["maxTorque"].as<int>();
        Serial.printf("[WebServer] Setting max torque: %d Nm\n", torque);
        success &= config->setMaxTorque(torque);
    }

    if (!obj["baseSpeed"].isNull()) {
        success &= config->setBaseSpeed(obj["baseSpeed"].as<float>());
    }

    if (!obj["deltaSpeed"].isNull()) {
        success &= config->setDeltaSpeed(obj["deltaSpeed"].as<float>());
    }

    if (!obj["nominalPower"].isNull()) {
        success &= config->setNominalPower(obj["nominalPower"].as<float>());
    }

    if (!obj["drivePowerLimits"].isNull()) {
        JsonArray arr = obj["drivePowerLimits"].as<JsonArray>();
        for (size_t i = 0; i < arr.size() && i < 5; i++) {
            success &= config->setDrivePowerLimit(i, arr[i].as<float>());
        }
    }

    if (!obj["regenPowerLimits"].isNull()) {
        JsonArray arr = obj["regenPowerLimits"].as<JsonArray>();
        for (size_t i = 0; i < arr.size() && i < 5; i++) {
            success &= config->setRegenPowerLimit(i, arr[i].as<float>());
        }
    }

    if (success) {
        Serial.println("[WebServer] Saving config to flash...");
        bool saved = config->save();
        Serial.printf("[WebServer] Config save result: %s\n", saved ? "SUCCESS" : "FAILED");

        Serial.printf("[WebServer] Verifying saved values - maxTorque: %d Nm\n", config->getMaxTorque());

        request->send(200, "application/json", "{\"success\":true}");
    } else {
        Serial.println("[WebServer] Invalid parameters");
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid parameters\"}");
    }
}

void VCUWebServer::handleSetPedalConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    JsonObject obj = json.as<JsonObject>();
    bool success = true;

    if (!obj["regenZoneEnd"].isNull()) {
        float value = obj["regenZoneEnd"].as<float>();
        success &= config->setRegenZoneEnd(value);
        // vehicleControl->setRegenZoneEnd(value); // TODO: Add this method
    }

    if (!obj["coastZoneEnd"].isNull()) {
        float value = obj["coastZoneEnd"].as<float>();
        success &= config->setCoastZoneEnd(value);
        // vehicleControl->setCoastZoneEnd(value); // TODO: Add this method
    }

    if (!obj["regenProgression"].isNull()) {
        float value = obj["regenProgression"].as<float>();
        success &= config->setRegenProgression(value);
        // vehicleControl->setRegenProgression(value); // TODO: Add this method
    }

    if (!obj["accelProgression"].isNull()) {
        float value = obj["accelProgression"].as<float>();
        success &= config->setAccelProgression(value);
        // vehicleControl->setAccelProgression(value); // TODO: Add this method
    }

    if (success) {
        config->save();
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid parameters\"}");
    }
}

void VCUWebServer::handleSetTransitionConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    JsonObject obj = json.as<JsonObject>();
    bool success = true;

    if (!obj["regenEngageTime"].isNull()) {
        float value = obj["regenEngageTime"].as<float>();
        success &= config->setRegenEngageTime(value);
        vehicleControl->setRegenEngageTime(value);
    }

    if (!obj["regenReleaseTime"].isNull()) {
        float value = obj["regenReleaseTime"].as<float>();
        success &= config->setRegenReleaseTime(value);
        vehicleControl->setRegenReleaseTime(value);
    }

    if (!obj["powerEngageTime"].isNull()) {
        float value = obj["powerEngageTime"].as<float>();
        success &= config->setPowerEngageTime(value);
        vehicleControl->setPowerEngageTime(value);
    }

    if (!obj["powerReleaseTime"].isNull()) {
        float value = obj["powerReleaseTime"].as<float>();
        success &= config->setPowerReleaseTime(value);
        vehicleControl->setPowerReleaseTime(value);
    }

    if (!obj["crossoverTime"].isNull()) {
        float value = obj["crossoverTime"].as<float>();
        success &= config->setCrossoverTime(value);
        vehicleControl->setCrossoverTime(value);
    }

    if (success) {
        config->save();
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid parameters\"}");
    }
}

void VCUWebServer::handleSetThrottleConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    JsonObject obj = json.as<JsonObject>();
    bool success = true;

    if (!obj["throttleMinADC"].isNull()) {
        int value = obj["throttleMinADC"].as<int>();
        success &= config->setThrottleMinADC(value);
    }

    if (!obj["throttleMaxADC"].isNull()) {
        int value = obj["throttleMaxADC"].as<int>();
        success &= config->setThrottleMaxADC(value);
    }

    if (success) {
        config->save();
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid parameters\"}");
    }
}

// ===== SYSTEM ENDPOINTS =====

void VCUWebServer::handleSaveConfig(AsyncWebServerRequest* request) {
    bool success = config->save();

    if (success) {
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(500, "application/json", "{\"success\":false,\"error\":\"Save failed\"}");
    }
}

void VCUWebServer::handleResetConfig(AsyncWebServerRequest* request) {
    config->resetToDefaults();
    config->save();

    request->send(200, "application/json", "{\"success\":true}");
}

void VCUWebServer::handleExportConfig(AsyncWebServerRequest* request) {
    String json = config->toJSON();
    request->send(200, "application/json", json);
}

void VCUWebServer::handleImportConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    String jsonStr;
    serializeJson(json, jsonStr);

    bool success = config->fromJSON(jsonStr);

    if (success) {
        config->save();
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid configuration\"}");
    }
}

void VCUWebServer::handleSystemInfo(AsyncWebServerRequest* request) {
    String json = createSystemInfoJSON();
    request->send(200, "application/json", json);
}

void VCUWebServer::handleSetDebugMode(AsyncWebServerRequest* request, JsonVariant& json) {
    JsonObject obj = json.as<JsonObject>();

    if (!obj["enabled"].isNull()) {
        setDebugMode(obj["enabled"].as<bool>());
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(400, "application/json", "{\"success\":false,\"error\":\"Missing 'enabled' field\"}");
    }
}

// ===== CAN MONITOR ENDPOINTS =====

void VCUWebServer::handleGetCANMessages(AsyncWebServerRequest* request) {
    String json = createCANMessagesJSON();
    request->send(200, "application/json", json);
}

// ===== WEBSOCKET HANDLERS =====

void VCUWebServer::handleWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("[WebSocket] Client #%u connected from %s\n",
                         client->id(), client->remoteIP().toString().c_str());
            break;

        case WS_EVT_DISCONNECT:
            Serial.printf("[WebSocket] Client #%u disconnected\n", client->id());
            break;

        case WS_EVT_DATA:
            // Handle incoming WebSocket data if needed
            break;

        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void VCUWebServer::broadcastLiveData() {
    if (ws.count() > 0) {
        String json = createLiveDataJSON();
        ws.textAll(json);
    }
}

// ===== HELPER FUNCTIONS =====

String VCUWebServer::createLiveDataJSON() {
    JsonDocument doc;

    // Vehicle status
    doc["state"] = stateToString(stateManager->getCurrentState());
    doc["gear"] = gearToString(vehicleControl ? vehicleControl->getCurrentGear() : GearState::NEUTRAL);

    // BMS data
    const BMSData& bms = canManager->getBMSData();
    JsonObject battery = doc["battery"].to<JsonObject>();
    battery["soc"] = bms.soc;
    battery["voltage"] = bms.voltage;
    battery["current"] = bms.current / 10.0f;  // Scale: divide by 10
    battery["minCell"] = 0.0f;

    // DMC data
    const DMCData& dmc = canManager->getDMCData();
    JsonObject motor = doc["motor"].to<JsonObject>();
    motor["speed"] = dmc.speedActual;
    motor["torque"] = dmc.torqueActual;
    motor["tempMotor"] = dmc.tempMotor;
    motor["tempInverter"] = dmc.tempInverter;
    motor["power"] = dmc.mechPower;

    // Vehicle control - calculate actual vehicle speed from motor RPM
    float vehicleSpeedKph = 0.0f;
    if (vehicleControl) {
        // Calculate vehicle speed from motor RPM using wheel diameter and gear ratio
        // wheel_circumference = PI * WHEEL_DIAMETER (0.53m) = 1.665m
        // vehicle_speed (m/s) = (motor_rpm * wheel_circumference) / (60 * GEAR_RATIO)
        // vehicle_speed (kph) = vehicle_speed (m/s) * 3.6
        const float WHEEL_CIRCUMFERENCE = 3.14159f * 0.53f;  // meters
        const float GEAR_RATIO = 10.0f;  // Typical EV gear ratio
        vehicleSpeedKph = (abs(dmc.speedActual) * WHEEL_CIRCUMFERENCE * 3.6f) / (60.0f * GEAR_RATIO);
    }

    JsonObject vehicle = doc["vehicle"].to<JsonObject>();
    vehicle["speed"] = vehicleSpeedKph;
    vehicle["power"] = dmc.mechPower / 1000.0f;  // Convert W to kW
    vehicle["torqueDemand"] = dmc.torqueActual;  // Actual torque in Nm

    // Inputs
    JsonObject inputs = doc["inputs"].to<JsonObject>();
    // Get throttle percentage from vehicle control
    float throttlePercent = vehicleControl ? vehicleControl->getThrottlePercentage() : 0.0f;
    float torquePercent = vehicleControl ? vehicleControl->getTorquePercentage() : 0.0f;

    inputs["throttle"] = throttlePercent;
    inputs["torque"] = torquePercent;
    inputs["rawThrottleADC"] = vehicleControl ? vehicleControl->getRawThrottleADC() : 0;
    inputs["connectorLock"] = stateManager->isConnectorLocked();
    inputs["ignition"] = digitalRead(Pins::IGNITION) == HIGH;

    // Safety status
    JsonObject safety = doc["safety"].to<JsonObject>();
    safety["throttleBlockingShift"] = vehicleControl ? vehicleControl->isThrottleBlockingShift() : false;

    // Temperatures
    JsonObject temps = doc["temperatures"].to<JsonObject>();
    temps["motor"] = dmc.tempMotor;
    temps["inverter"] = dmc.tempInverter;
    temps["battery"] = 0;

    // Charging status
    const NLGData& nlg = canManager->getNLGData();
    JsonObject charging = doc["charging"].to<JsonObject>();

    // Map NLG state to readable string
    String chargerState = "UNKNOWN";
    switch(nlg.stateAct) {
        case 0: chargerState = "SLEEP"; break;
        case 1: chargerState = "WAKEUP"; break;
        case 2: chargerState = "STANDBY"; break;
        case 3: chargerState = "READY"; break;
        case 4: chargerState = "CHARGING"; break;
        case 5: chargerState = "SHUTDOWN"; break;
        default: chargerState = "UNKNOWN"; break;
    }

    charging["state"] = chargerState;
    charging["current"] = nlg.dcHvCurrentAct / 100.0f;  // Scale: divide by 100
    charging["voltage"] = nlg.dcHvVoltageAct;
    charging["connectorLocked"] = nlg.connectorLocked;

    String response;
    serializeJson(doc, response);
    return response;
}

String VCUWebServer::createSystemInfoJSON() {
    JsonDocument doc;

    doc["chipModel"] = ESP.getChipModel();
    doc["chipRevision"] = ESP.getChipRevision();
    doc["chipCores"] = ESP.getChipCores();
    doc["cpuFreqMHz"] = ESP.getCpuFreqMHz();
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["heapSize"] = ESP.getHeapSize();
    doc["freePSRAM"] = ESP.getFreePsram();
    doc["psramSize"] = ESP.getPsramSize();
    doc["flashSize"] = ESP.getFlashChipSize();
    doc["sketchSize"] = ESP.getSketchSize();
    doc["freeSketchSpace"] = ESP.getFreeSketchSpace();
    doc["uptime"] = millis() / 1000;
    doc["debugMode"] = debugMode;

    String response;
    serializeJson(doc, response);
    return response;
}

String VCUWebServer::createCANMessagesJSON() {
    JsonDocument doc;
    JsonArray messages = doc["messages"].to<JsonArray>();

    // Get CAN message data from CAN manager
    const BMSData& bms = canManager->getBMSData();
    const DMCData& dmc = canManager->getDMCData();
    const BSCData& bsc = canManager->getBSCData();
    const NLGData& nlg = canManager->getNLGData();

    // BMS message
    JsonObject bmsMsg = messages.add<JsonObject>();
    bmsMsg["id"] = "0x0F1";
    bmsMsg["name"] = "BMS";
    bmsMsg["soc"] = bms.soc;
    bmsMsg["voltage"] = bms.voltage;
    bmsMsg["current"] = bms.current / 10.0f;  // Scale: divide by 10

    // DMC message
    JsonObject dmcMsg = messages.add<JsonObject>();
    dmcMsg["id"] = "0x280";
    dmcMsg["name"] = "DMC";
    dmcMsg["speed"] = dmc.speedActual;
    dmcMsg["torque"] = dmc.torqueActual;

    // BSC message
    JsonObject bscMsg = messages.add<JsonObject>();
    bscMsg["id"] = "0x26A";
    bscMsg["name"] = "BSC";
    bscMsg["hvVoltage"] = bsc.hvVoltageAct;
    bscMsg["lvVoltage"] = bsc.lvVoltageAct;

    // NLG message
    JsonObject nlgMsg = messages.add<JsonObject>();
    nlgMsg["id"] = "0x728";
    nlgMsg["name"] = "NLG";
    nlgMsg["state"] = 0;
    nlgMsg["hvCurrent"] = nlg.dcHvCurrentAct;

    String response;
    serializeJson(doc, response);
    return response;
}
