/**
 * @file enhanced_serial_console.h
 * @brief Enhanced Serial Console with JSON Support and Streaming
 */

#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "can_manager.h"
#include "state_manager.h"
#include "vehicle_control.h"
#include "error_monitor.h"
#include "configuration.h"

/**
 * @brief Streaming Mode Configuration
 */
enum class StreamingMode {
    NONE,
    MONITORING,
    CAN_MESSAGES,
    ERRORS,
    PERFORMANCE
};

/**
 * @brief Enhanced Serial Console Class
 */
class EnhancedSerialConsole {
public:
    EnhancedSerialConsole(CANManager& canManager, StateManager& stateManager, 
                         VehicleControl& vehicleControl, ErrorMonitor& errorMonitor);
    
    void update();
    void setStreamingMode(StreamingMode mode, unsigned long intervalMs = 100);
    void stopStreaming();

private:
    // Core system references
    CANManager& canManager;
    StateManager& stateManager;
    VehicleControl& vehicleControl;
    ErrorMonitor& errorMonitor;
    
    // Input handling
    String inputBuffer;
    const size_t MAX_BUFFER_SIZE = 2048;
    
    // Streaming
    StreamingMode currentStreamingMode;
    unsigned long streamingInterval;
    unsigned long lastStreamTime;
    
    // Command processing
    void handleCommand(const String& command);
    void handleLegacyCommand(const String& command);
    void handleJSONCommand(const String& command);
    void handleConfigCommand(const JsonDocument& doc);
    void handleMonitorCommand(const JsonDocument& doc);
    
    // Configuration commands
    void handleConfigGet(const String& category);
    void handleConfigSet(const String& category, const String& jsonData);
    void handleConfigUpload(const String& jsonData);
    void handleConfigDownload();
    void handleConfigBackup();
    void handleConfigRestore(const String& backupData);
    void handleConfigReset(const String& category);
    
    // Monitoring commands
    void handleMonitoringGet();
    void handlePerformanceGet();
    void handleErrorLogGet(int maxEntries = 100);
    void handleCANLogGet(int maxEntries = 100);
    void handleSystemHealthGet();
    
    // Streaming commands
    void handleStreamStart(const String& type, int intervalMs = 100);
    void handleStreamStop();
    void processStreaming();
    
    // Diagnostic commands
    void handleSystemTest(const String& component);
    void handleCalibrationStart(const String& type);
    void handleCalibrationSet(const String& param, float value);
    void handleCalibrationSave();
    void handleCANMonitorToggle(bool enable);
    void handleErrorClear();
    void handlePerformanceReset();
    
    // Device control commands
    void handleDeviceControl(const String& device, const String& action, const String& value);
    void handleEmergencyStop();
    void handleSafeMode(bool enable);
    
    // Raw CAN commands
    void handleCANSend(uint32_t id, const String& hexData);
    void handleCANFilter(const String& filterConfig);
    
    // Legacy command handlers (existing functionality)
    void handleLegacyGet(const String& target, const String& parameter);
    void handleLegacySet(const String& target, const String& parameter, const String& value);
    
    // Response helpers
    void sendJSONResponse(const String& status, const String& data = "", const String& error = "");
    void sendJSONError(const String& message);
    void sendError(const String& message);
    void sendSuccess(const String& message = "");
    void sendData(const String& data);
    
    // Legacy methods for backwards compatibility
    void handleGet(const String& target, const String& parameter);
    void handleSet(const String& target, const String& parameter, const String& value);
    void printHelp();
    void printValue(const String& name, int value, const String& unit = "");
    void printValue(const String& name, float value, const String& unit = "");
    void printValue(const String& name, bool value);
    void printJSONHelp();  // Removed duplicate declaration
    
    // Utility methods
    bool isJSONCommand(const String& command);
    JsonDocument parseJSON(const String& json);
    String createResponse(const String& status, const JsonObject& data = JsonObject());
    
    // Validation
    bool validateJSONStructure(const JsonObject& obj, const String& expectedType);
    bool validateParameterRange(const String& param, float value, float min, float max);
};