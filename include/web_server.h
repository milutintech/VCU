#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "config.h"
#include "configuration.h"
#include "state_manager.h"
#include "vehicle_control.h"
#include "can_manager.h"
#include "error_monitor.h"

class VCUWebServer {
public:
    VCUWebServer(Configuration* cfg, StateManager* sm, VehicleControl* vc, CANManager* can, ErrorMonitor* err);

    // Initialize web server and routes
    void begin();

    // Update (for WebSocket broadcasts)
    void update();

    // Enable/disable debug mode
    void setDebugMode(bool enabled);
    bool getDebugMode() const { return debugMode; }

private:
    AsyncWebServer server;
    AsyncWebSocket ws;

    // References to system components
    Configuration* config;
    StateManager* stateManager;
    VehicleControl* vehicleControl;
    CANManager* canManager;
    ErrorMonitor* errorMonitor;

    // System flags
    bool debugMode;

    // Last WebSocket broadcast time
    unsigned long lastWSBroadcast;
    static constexpr unsigned long WS_BROADCAST_INTERVAL = 100;  // 100ms = 10Hz

    // Setup routes
    void setupRoutes();
    void setupWebSocketHandlers();

    // API Endpoints - Status
    void handleGetStatus(AsyncWebServerRequest* request);
    void handleGetLiveData(AsyncWebServerRequest* request);
    void handleGetErrors(AsyncWebServerRequest* request);

    // API Endpoints - Configuration
    void handleGetConfig(AsyncWebServerRequest* request);
    void handleGetDrivingConfig(AsyncWebServerRequest* request);
    void handleSetDrivingConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetChargingConfig(AsyncWebServerRequest* request);
    void handleSetChargingConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetLimitsConfig(AsyncWebServerRequest* request);
    void handleSetLimitsConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetPedalConfig(AsyncWebServerRequest* request);
    void handleSetPedalConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetTransitionConfig(AsyncWebServerRequest* request);
    void handleSetTransitionConfig(AsyncWebServerRequest* request, JsonVariant& json);

    // API Endpoints - System
    void handleSaveConfig(AsyncWebServerRequest* request);
    void handleResetConfig(AsyncWebServerRequest* request);
    void handleExportConfig(AsyncWebServerRequest* request);
    void handleImportConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleSystemInfo(AsyncWebServerRequest* request);
    void handleSetDebugMode(AsyncWebServerRequest* request, JsonVariant& json);

    // API Endpoints - CAN Monitor
    void handleGetCANMessages(AsyncWebServerRequest* request);

    // WebSocket handlers
    void handleWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                             AwsEventType type, void* arg, uint8_t* data, size_t len);
    void broadcastLiveData();

    // Helper functions
    String createLiveDataJSON();
    String createSystemInfoJSON();
    String createCANMessagesJSON();
};

#endif // WEB_SERVER_H
