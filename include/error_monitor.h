/**
 * @file error_monitor.h
 * @brief Error Logging and System Monitoring
 */

#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <deque>
#include "config.h"
#include "can_manager.h"
#include "state_manager.h"

/**
 * @brief Error Severity Levels
 */
enum class ErrorSeverity : uint8_t {
    INFO = 0,      // Informational
    WARNING = 1,   // Warning condition
    ERROR = 2,     // Error condition
    CRITICAL = 3   // Critical error
};

/**
 * @brief Error Codes
 */
enum class ErrorCode : uint16_t {
    // System Errors (100-199)
    SYSTEM_BOOT = 100,
    SYSTEM_SHUTDOWN = 101,
    WATCHDOG_RESET = 102,
    MEMORY_ERROR = 103,
    
    // Communication Errors (200-299)
    CAN_INIT_FAILED = 200,
    CAN_BUS_OFF = 201,
    BMS_TIMEOUT = 202,
    DMC_TIMEOUT = 203,
    BSC_TIMEOUT = 204,
    NLG_TIMEOUT = 205,
    
    // Battery Errors (300-399)
    BATTERY_UNDERVOLTAGE = 300,
    BATTERY_OVERVOLTAGE = 301,
    BATTERY_OVERCURRENT = 302,
    PRECHARGE_FAILED = 303,
    PRECHARGE_TIMEOUT = 304,
    SOC_OUT_OF_RANGE = 305,
    
    // Motor Errors (400-499)
    MOTOR_OVERTEMP = 400,
    INVERTER_OVERTEMP = 401,
    DMC_ERROR = 402,
    TORQUE_LIMIT_EXCEEDED = 403,
    SPEED_LIMIT_EXCEEDED = 404,
    
    // Charging Errors (500-599)
    CHARGER_ERROR = 500,
    CHARGER_OVERTEMP = 501,
    CONNECTOR_ERROR = 502,
    CHARGE_CURRENT_ERROR = 503,
    
    // Configuration Errors (600-699)
    CONFIG_INVALID = 600,
    CONFIG_SAVE_FAILED = 601,
    CONFIG_LOAD_FAILED = 602,
    CALIBRATION_ERROR = 603,
    
    // User Errors (700-799)
    GEAR_SHIFT_ERROR = 700,
    PEDAL_ERROR = 701,
    USER_ABORT = 702
};

/**
 * @brief Log Entry Structure
 */
struct LogEntry {
    unsigned long timestamp;
    ErrorSeverity severity;
    ErrorCode errorCode;
    String description;
    float associatedValue;
    String component;
    
    LogEntry() : timestamp(0), severity(ErrorSeverity::INFO), 
                 errorCode(ErrorCode::SYSTEM_BOOT), associatedValue(0.0f) {}
    
    LogEntry(ErrorSeverity sev, ErrorCode code, const String& desc, 
             float value = 0.0f, const String& comp = "") 
        : timestamp(millis()), severity(sev), errorCode(code), 
          description(desc), associatedValue(value), component(comp) {}
};

/**
 * @brief Real-time Monitoring Data Structure
 */
struct MonitoringData {
    // System Status
    unsigned long timestamp;
    VehicleState vehicleState;
    GearState gearState;
    DriveMode driveMode;
    bool batteryArmed;
    bool prechargeComplete;
    bool dmcEnabled;
    
    // Power & Motion
    float torquePercent;
    float torqueNm;
    float motorSpeedRPM;
    float vehicleSpeedKPH;
    float powerLimitPercent;
    
    // Battery Data
    uint8_t soc;
    uint16_t packVoltage;
    int16_t packCurrent;
    uint16_t maxDischarge;
    uint16_t maxCharge;
    
    // Temperatures
    float motorTemp;
    float inverterTemp;
    float chargerTemp;
    float ambientTemp;
    
    // Charging Data
    uint8_t chargerState;
    uint16_t chargingVoltage;
    uint16_t chargingCurrent;
    bool connectorLocked;
    uint8_t coolingRequest;
    
    // DC-DC Converter
    float bscHVVoltage;
    float bscLVVoltage;
    float bscHVCurrent;
    float bscLVCurrent;
    uint8_t bscMode;
    
    // System Health
    unsigned long uptime;
    float cpuUsage;
    uint16_t freeMemory;
    uint8_t canBusLoad;
    uint16_t activeErrors;
    
    // Performance
    float energyConsumption;
    float regenEfficiency;
    unsigned long totalDistance;
    
    MonitoringData() : timestamp(millis()) {}
};

/**
 * @brief CAN Message Monitor Structure
 */
struct CANMessageInfo {
    unsigned long timestamp;
    uint32_t id;
    uint8_t length;
    uint8_t data[8];
    bool transmitted;  // true = TX, false = RX
    String description;
    
    CANMessageInfo() : timestamp(millis()), id(0), length(0), transmitted(false) {
        memset(data, 0, 8);
    }
};

/**
 * @brief Performance Metrics
 */
struct PerformanceMetrics {
    // Efficiency
    float instantEfficiency;      // Current Wh/km
    float avgEfficiency;          // Average Wh/km
    float regenRecovered;         // Energy recovered via regen (Wh)
    
    // Usage Statistics
    unsigned long totalDistance;  // Total distance (m)
    unsigned long totalEnergy;    // Total energy consumed (Wh)
    unsigned long totalChargeCycles;
    unsigned long totalOperatingTime;
    
    // System Performance
    float avgResponseTime;        // Average response time (ms)
    uint32_t totalCANMessages;
    uint16_t maxTorqueUsed;
    float maxSpeedReached;
    
    // Component Health
    unsigned long pumpRunTime;
    unsigned long fanRunTime;
    uint32_t contactorCycles;
    uint32_t gearShifts;
    
    PerformanceMetrics() {
        memset(this, 0, sizeof(PerformanceMetrics));
    }
};

/**
 * @brief Error Monitor and System Logger Class
 */
class ErrorMonitor {
public:
    ErrorMonitor(size_t maxLogEntries = 1000, size_t maxCANMessages = 500);
    
    // Error Logging
    void logError(ErrorSeverity severity, ErrorCode code, const String& description, 
                  float value = 0.0f, const String& component = "");
    void logInfo(const String& message, const String& component = "");
    void logWarning(const String& message, const String& component = "");
    void logError(const String& message, const String& component = "");
    void logCritical(const String& message, const String& component = "");
    
    // Log Management
    std::deque<LogEntry>& getErrorLog() { return errorLog; }
    void clearErrorLog();
    uint16_t getErrorCount(ErrorSeverity minSeverity = ErrorSeverity::WARNING);
    String getErrorLogJSON(uint16_t maxEntries = 100);
    
    // Monitoring Data
    void updateMonitoringData(const CANManager& canManager, const StateManager& stateManager);
    const MonitoringData& getMonitoringData() const { return currentData; }
    String getMonitoringJSON();
    
    // CAN Message Monitoring
    void logCANMessage(uint32_t id, const uint8_t* data, uint8_t length, bool transmitted, const String& description = "");
    std::deque<CANMessageInfo>& getCANLog() { return canLog; }
    void clearCANLog();
    String getCANLogJSON(uint16_t maxEntries = 100);
    void setCANLogging(bool enabled) { canLoggingEnabled = enabled; }
    
    // Performance Tracking
    void updatePerformanceMetrics(const MonitoringData& data);
    const PerformanceMetrics& getPerformanceMetrics() const { return performance; }
    String getPerformanceJSON();
    void resetPerformanceMetrics();
    
    // System Health
    bool isSystemHealthy();
    String getSystemHealthJSON();
    
private:
    std::deque<LogEntry> errorLog;
    std::deque<CANMessageInfo> canLog;
    MonitoringData currentData;
    PerformanceMetrics performance;
    
    size_t maxLogEntries;
    size_t maxCANMessages;
    bool canLoggingEnabled;
    
    unsigned long lastPerformanceUpdate;
    unsigned long lastHealthCheck;
    
    // Helper methods
    String severityToString(ErrorSeverity severity);
    String errorCodeToString(ErrorCode code);
    String vehicleStateToString(VehicleState state);
    String gearStateToString(GearState state);
    String driveModeToString(DriveMode mode);
    
    // Performance calculation helpers
    void updateEfficiencyMetrics();
    void updateUsageStatistics();
    void updateComponentHealth();
};

// Global instance
extern ErrorMonitor systemMonitor;