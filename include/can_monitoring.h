/**
 * @file can_monitoring.h - FIXED VERSION
 * @brief CAN Message Monitoring and Logging Extensions
 */

#pragma once
#include <Arduino.h>
#include <map>
#include <set>  // Added missing include
#include "error_monitor.h"

/**
 * @brief CAN Message Definition Structure - FIXED with default constructor
 */
struct CANMessageDef {
    uint32_t id;
    String name;
    String description;
    uint8_t expectedLength;
    String signalNames[8];  // Names for each byte
    String units[8];        // Units for each byte
    float scales[8];        // Scale factors for each byte
    float offsets[8];       // Offset values for each byte
    
    // FIXED: Add default constructor for std::map compatibility
    CANMessageDef() : id(0), name(""), description(""), expectedLength(0) {
        for (int i = 0; i < 8; i++) {
            scales[i] = 1.0f;
            offsets[i] = 0.0f;
        }
    }
    
    CANMessageDef(uint32_t msgId, const String& msgName, const String& desc, uint8_t len) 
        : id(msgId), name(msgName), description(desc), expectedLength(len) {
        // Initialize arrays
        for (int i = 0; i < 8; i++) {
            scales[i] = 1.0f;
            offsets[i] = 0.0f;
        }
    }
};

/**
 * @brief CAN Monitor Class - Extension to CANManager
 */
class CANMonitor {
public:
    CANMonitor();
    
    // Message definition management
    void initializeMessageDefinitions();
    void addMessageDefinition(const CANMessageDef& def);
    String getMessageDescription(uint32_t id);
    String decodeMessage(uint32_t id, const uint8_t* data, uint8_t length);
    
    // Message logging
    void logMessage(uint32_t id, const uint8_t* data, uint8_t length, bool transmitted);
    void setLoggingEnabled(bool enabled) { loggingEnabled = enabled; }
    bool isLoggingEnabled() const { return loggingEnabled; }
    
    // Statistics
    void updateStatistics(uint32_t id, bool transmitted);
    String getStatisticsJSON();
    void resetStatistics();
    
    // Filtering
    void addFilter(uint32_t id) { filters.insert(id); }
    void removeFilter(uint32_t id) { filters.erase(id); }
    void clearFilters() { filters.clear(); }
    bool isFiltered(uint32_t id) { return filters.empty() || filters.count(id) > 0; }
    
private:
    std::map<uint32_t, CANMessageDef> messageDefs;
    std::map<uint32_t, uint32_t> rxCounts;
    std::map<uint32_t, uint32_t> txCounts;
    std::map<uint32_t, unsigned long> lastSeen;
    std::set<uint32_t> filters;
    
    bool loggingEnabled;
    unsigned long totalRxMessages;
    unsigned long totalTxMessages;
    unsigned long startTime;
    
    // Helper methods
    String formatHexData(const uint8_t* data, uint8_t length);
    String decodeSignal(const uint8_t* data, int byteIndex, float scale, float offset, const String& unit);
};