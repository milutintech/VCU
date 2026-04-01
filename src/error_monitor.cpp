/**
 * @file error_monitor.cpp
 * @brief Implementation of error monitoring and system logging
 */

#include "error_monitor.h"
#include <esp_system.h>

// Global instance
ErrorMonitor systemMonitor(1000, 500);

/**
 * @brief Constructor
 */
ErrorMonitor::ErrorMonitor(size_t maxLogEntries, size_t maxCANMessages)
    : maxLogEntries(maxLogEntries)
    , maxCANMessages(maxCANMessages)
    , canLoggingEnabled(false)
    , lastPerformanceUpdate(0)
    , lastHealthCheck(0)
{
    // Log system startup
    logInfo("Error monitoring system initialized", "SYSTEM");
}

/**
 * @brief Log error with severity and code
 */
void ErrorMonitor::logError(ErrorSeverity severity, ErrorCode code, const String& description, 
                          float value, const String& component) {
    LogEntry entry(severity, code, description, value, component);
    
    // Add to log
    errorLog.push_back(entry);
    
    // Maintain size limit
    if (errorLog.size() > maxLogEntries) {
        errorLog.pop_front();
    }
    
    // Print to serial for debugging
    String severityStr = severityToString(severity);
    String codeStr = errorCodeToString(code);
    
    Serial.printf("[%s] %s (%s): %s", 
                  severityStr.c_str(), 
                  codeStr.c_str(),
                  component.c_str(),
                  description.c_str());
    
    if (value != 0.0f) {
        Serial.printf(" (Value: %.2f)", value);
    }
    Serial.println();
}

/**
 * @brief Log info message
 */
void ErrorMonitor::logInfo(const String& message, const String& component) {
    logError(ErrorSeverity::INFO, ErrorCode::SYSTEM_BOOT, message, 0.0f, component);
}

/**
 * @brief Log warning message
 */
void ErrorMonitor::logWarning(const String& message, const String& component) {
    logError(ErrorSeverity::WARNING, ErrorCode::SYSTEM_BOOT, message, 0.0f, component);
}

/**
 * @brief Log error message
 */
void ErrorMonitor::logError(const String& message, const String& component) {
    logError(ErrorSeverity::ERROR, ErrorCode::SYSTEM_BOOT, message, 0.0f, component);
}

/**
 * @brief Log critical message
 */
void ErrorMonitor::logCritical(const String& message, const String& component) {
    logError(ErrorSeverity::CRITICAL, ErrorCode::SYSTEM_BOOT, message, 0.0f, component);
}

/**
 * @brief Clear error log
 */
void ErrorMonitor::clearErrorLog() {
    errorLog.clear();
    logInfo("Error log cleared", "MONITOR");
}

/**
 * @brief Get error count by severity
 */
uint16_t ErrorMonitor::getErrorCount(ErrorSeverity minSeverity) {
    uint16_t count = 0;
    for (const auto& entry : errorLog) {
        if (entry.severity >= minSeverity) {
            count++;
        }
    }
    return count;
}

/**
 * @brief Get error log as JSON
 */
String ErrorMonitor::getErrorLogJSON(uint16_t maxEntries) {
    JsonDocument doc;
    JsonArray errors = doc["errors"].to<JsonArray>();
    
    uint16_t count = 0;
    for (auto it = errorLog.rbegin(); it != errorLog.rend() && count < maxEntries; ++it, ++count) {
        JsonObject error = errors.add<JsonObject>();
        error["timestamp"] = it->timestamp;
        error["severity"] = severityToString(it->severity);
        error["code"] = errorCodeToString(it->errorCode);
        error["description"] = it->description;
        error["component"] = it->component;
        if (it->associatedValue != 0.0f) {
            error["value"] = it->associatedValue;
        }
    }
    
    doc["totalEntries"] = errorLog.size();
    doc["returned"] = count;
    
    String result;
    serializeJson(doc, result);
    return result;
}

/**
 * @brief Update monitoring data
 */
void ErrorMonitor::updateMonitoringData(const CANManager& canManager, const StateManager& stateManager) {
    currentData.timestamp = millis();
    
    // System status
    currentData.vehicleState = stateManager.getCurrentState();
    currentData.batteryArmed = stateManager.isBatteryArmed();
    currentData.prechargeComplete = stateManager.isPreCharged();
    
    // BMS data
    const BMSData& bms = canManager.getBMSData();
    currentData.soc = bms.soc;
    currentData.packVoltage = bms.voltage;
    currentData.packCurrent = bms.current;
    currentData.maxDischarge = bms.maxDischarge;
    currentData.maxCharge = bms.maxCharge;
    
    // DMC data
    const DMCData& dmc = canManager.getDMCData();
    currentData.motorTemp = dmc.tempMotor;
    currentData.inverterTemp = dmc.tempInverter;
    currentData.motorSpeedRPM = dmc.speedActual;
    
    // BSC data
    const BSCData& bsc = canManager.getBSCData();
    currentData.bscHVVoltage = bsc.hvVoltageAct;
    currentData.bscLVVoltage = bsc.lvVoltageAct;
    currentData.bscHVCurrent = bsc.hvCurrentAct;
    currentData.bscLVCurrent = bsc.lvCurrentAct;
    currentData.bscMode = bsc.mode;
    
    // NLG data
    const NLGData& nlg = canManager.getNLGData();
    currentData.chargerState = nlg.stateAct;
    currentData.chargingVoltage = nlg.dcHvVoltageAct;
    currentData.chargingCurrent = nlg.dcHvCurrentAct;
    currentData.connectorLocked = nlg.connectorLocked;
    currentData.chargerTemp = nlg.tempCoolPlate;
    currentData.coolingRequest = nlg.coolingRequest;
    
    // System health
    currentData.uptime = millis();
    currentData.freeMemory = ESP.getFreeHeap();
    currentData.activeErrors = getErrorCount(ErrorSeverity::ERROR);
}

/**
 * @brief Get monitoring data as JSON
 */
String ErrorMonitor::getMonitoringJSON() {
    JsonDocument doc;
    
    // System info
    doc["timestamp"] = currentData.timestamp;
    doc["uptime"] = currentData.uptime;
    doc["vehicleState"] = vehicleStateToString(currentData.vehicleState);
    doc["batteryArmed"] = currentData.batteryArmed;
    doc["prechargeComplete"] = currentData.prechargeComplete;
    
    // Power system
    JsonObject power = doc["power"].to<JsonObject>();
    power["soc"] = currentData.soc;
    power["packVoltage"] = currentData.packVoltage;
    power["packCurrent"] = currentData.packCurrent;
    power["maxDischarge"] = currentData.maxDischarge;
    power["maxCharge"] = currentData.maxCharge;
    power["bscHVVoltage"] = currentData.bscHVVoltage;
    power["bscLVVoltage"] = currentData.bscLVVoltage;
    
    // Thermal
    JsonObject thermal = doc["thermal"].to<JsonObject>();
    thermal["motorTemp"] = currentData.motorTemp;
    thermal["inverterTemp"] = currentData.inverterTemp;
    thermal["chargerTemp"] = currentData.chargerTemp;
    thermal["coolingRequest"] = currentData.coolingRequest;
    
    // Motion
    JsonObject motion = doc["motion"].to<JsonObject>();
    motion["motorSpeedRPM"] = currentData.motorSpeedRPM;
    motion["vehicleSpeedKPH"] = currentData.vehicleSpeedKPH;
    motion["torquePercent"] = currentData.torquePercent;
    
    // Charging
    JsonObject charging = doc["charging"].to<JsonObject>();
    charging["state"] = currentData.chargerState;
    charging["voltage"] = currentData.chargingVoltage;
    charging["current"] = currentData.chargingCurrent;
    charging["connectorLocked"] = currentData.connectorLocked;
    
    // System health
    JsonObject health = doc["health"].to<JsonObject>();
    health["freeMemory"] = currentData.freeMemory;
    health["activeErrors"] = currentData.activeErrors;
    
    String result;
    serializeJson(doc, result);
    return result;
}

/**
 * @brief Log CAN message
 */
void ErrorMonitor::logCANMessage(uint32_t id, const uint8_t* data, uint8_t length, 
                                bool transmitted, const String& description) {
    if (!canLoggingEnabled) {
        return;
    }
    
    CANMessageInfo info;
    info.timestamp = millis();
    info.id = id;
    info.length = length;
    info.transmitted = transmitted;
    info.description = description;
    
    if (data && length <= 8) {
        memcpy(info.data, data, length);
    }
    
    canLog.push_back(info);
    
    // Maintain size limit
    if (canLog.size() > maxCANMessages) {
        canLog.pop_front();
    }
}

/**
 * @brief Clear CAN log
 */
void ErrorMonitor::clearCANLog() {
    canLog.clear();
}

/**
 * @brief Get CAN log as JSON - FIXED toUpperCase() issue
 */
String ErrorMonitor::getCANLogJSON(uint16_t maxEntries) {
    JsonDocument doc;
    JsonArray messages = doc["messages"].to<JsonArray>();
    
    uint16_t count = 0;
    for (auto it = canLog.rbegin(); it != canLog.rend() && count < maxEntries; ++it, ++count) {
        JsonObject msg = messages.add<JsonObject>();
        msg["timestamp"] = it->timestamp;
        msg["id"] = "0x" + String(it->id, HEX);
        msg["direction"] = it->transmitted ? "TX" : "RX";
        msg["length"] = it->length;
        
        String hexData;
        for (uint8_t i = 0; i < it->length; i++) {
            if (i > 0) hexData += " ";
            if (it->data[i] < 0x10) hexData += "0";
            hexData += String(it->data[i], HEX);
        }
        // FIXED: toUpperCase() returns void, so call it first then assign
        hexData.toUpperCase();
        msg["data"] = hexData;
        
        if (!it->description.isEmpty()) {
            msg["description"] = it->description;
        }
    }
    
    doc["totalEntries"] = canLog.size();
    doc["returned"] = count;
    
    String result;
    serializeJson(doc, result);
    return result;
}

/**
 * @brief Update performance metrics
 */
void ErrorMonitor::updatePerformanceMetrics(const MonitoringData& data) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastPerformanceUpdate < 1000) {
        return; // Update only once per second
    }
    
    lastPerformanceUpdate = currentTime;
    
    // Update basic metrics
    performance.totalOperatingTime = currentTime;
    
    // Calculate efficiency if we have current data
    if (data.packCurrent > 0) {
        performance.instantEfficiency = (data.packVoltage * data.packCurrent) / 1000.0f; // kW
    }
    
    // Track maximum values
    if (data.motorSpeedRPM > performance.maxSpeedReached) {
        performance.maxSpeedReached = data.motorSpeedRPM;
    }
    
    performance.totalCANMessages++;
}

/**
 * @brief Get performance metrics as JSON
 */
String ErrorMonitor::getPerformanceJSON() {
    JsonDocument doc;
    
    doc["totalOperatingTime"] = performance.totalOperatingTime;
    doc["instantEfficiency"] = performance.instantEfficiency;
    doc["avgEfficiency"] = performance.avgEfficiency;
    doc["totalDistance"] = performance.totalDistance;
    doc["totalEnergy"] = performance.totalEnergy;
    doc["maxSpeedReached"] = performance.maxSpeedReached;
    doc["totalCANMessages"] = performance.totalCANMessages;
    
    String result;
    serializeJson(doc, result);
    return result;
}

/**
 * @brief Reset performance metrics
 */
void ErrorMonitor::resetPerformanceMetrics() {
    memset(&performance, 0, sizeof(performance));
    logInfo("Performance metrics reset", "MONITOR");
}

/**
 * @brief Check system health
 */
bool ErrorMonitor::isSystemHealthy() {
    // Check for critical errors
    if (getErrorCount(ErrorSeverity::CRITICAL) > 0) {
        return false;
    }
    
    // Check memory
    if (ESP.getFreeHeap() < 10000) { // Less than 10KB free
        return false;
    }
    
    // Check data freshness (basic check)
    if (millis() - currentData.timestamp > 5000) { // Data older than 5 seconds
        return false;
    }
    
    return true;
}

/**
 * @brief Get system health as JSON
 */
String ErrorMonitor::getSystemHealthJSON() {
    JsonDocument doc;
    
    doc["healthy"] = isSystemHealthy();
    doc["criticalErrors"] = getErrorCount(ErrorSeverity::CRITICAL);
    doc["errors"] = getErrorCount(ErrorSeverity::ERROR);
    doc["warnings"] = getErrorCount(ErrorSeverity::WARNING);
    doc["freeMemory"] = ESP.getFreeHeap();
    doc["dataAge"] = millis() - currentData.timestamp;
    doc["uptime"] = millis();
    
    String result;
    serializeJson(doc, result);
    return result;
}

// Helper methods implementation

String ErrorMonitor::severityToString(ErrorSeverity severity) {
    switch (severity) {
        case ErrorSeverity::INFO: return "INFO";
        case ErrorSeverity::WARNING: return "WARNING";
        case ErrorSeverity::ERROR: return "ERROR";
        case ErrorSeverity::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

String ErrorMonitor::errorCodeToString(ErrorCode code) {
    switch (code) {
        case ErrorCode::SYSTEM_BOOT: return "SYSTEM_BOOT";
        case ErrorCode::CAN_INIT_FAILED: return "CAN_INIT_FAILED";
        case ErrorCode::BATTERY_UNDERVOLTAGE: return "BATTERY_UNDERVOLTAGE";
        case ErrorCode::BATTERY_OVERVOLTAGE: return "BATTERY_OVERVOLTAGE";
        case ErrorCode::MOTOR_OVERTEMP: return "MOTOR_OVERTEMP";
        case ErrorCode::INVERTER_OVERTEMP: return "INVERTER_OVERTEMP";
        // Add more codes as needed
        default: return "UNKNOWN_ERROR";
    }
}

String ErrorMonitor::vehicleStateToString(VehicleState state) {
    switch (state) {
        case VehicleState::STANDBY: return "STANDBY";
        case VehicleState::RUN: return "RUN";
        case VehicleState::CHARGING: return "CHARGING";
        default: return "UNKNOWN";
    }
}