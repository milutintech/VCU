#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "configuration.h"
#include "state_manager.h"
#include "vehicle_control.h"
#include "can_manager.h"
#include "error_monitor.h"
#include "wifi_manager.h"
#include "pi_power.h"
#include "ethernet_link.h"

class VCUWebServer {
public:
    VCUWebServer(Configuration* cfg, StateManager* sm, VehicleControl* vc, CANManager* can, ErrorMonitor* err);

    // Initialize web server and routes
    void begin();

    // Update (for WebSocket broadcasts)
    void update();

    void setWifiManager(WiFiManager* wm) { wifiManager = wm; }
    void setPiPower(PiPower* p) { piPower = p; }
    void setEthernet(EthernetLink* e) { ethernet = e; }

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
    WiFiManager* wifiManager = nullptr;
    PiPower* piPower = nullptr;
    EthernetLink* ethernet = nullptr;

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
    void handleGetFaults(AsyncWebServerRequest* request);
    void handleGetBmsCells(AsyncWebServerRequest* request);
    void handleGetBmsConfig(AsyncWebServerRequest* request);
    void handleSetBmsConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetWifiConfig(AsyncWebServerRequest* request);
    void handleGetPi(AsyncWebServerRequest* request);
    void handlePiAck(AsyncWebServerRequest* request);
    void handleSetWifiConfig(AsyncWebServerRequest* request, JsonVariant& json);

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
    void handleGetThrottleConfig(AsyncWebServerRequest* request);
    void handleSetThrottleConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetRawThrottle(AsyncWebServerRequest* request);

    // API Endpoints - System
    void handleSaveConfig(AsyncWebServerRequest* request);
    void handleResetConfig(AsyncWebServerRequest* request);
    void handleExportConfig(AsyncWebServerRequest* request);
    void handleImportConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleSystemInfo(AsyncWebServerRequest* request);
    void handleGetLimits(AsyncWebServerRequest* request);
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
