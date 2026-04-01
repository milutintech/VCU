/**
 * @file can_monitoring.cpp - FIXED VERSION
 * @brief Implementation of CAN message monitoring and logging
 */

#include "can_monitoring.h"
#include "config.h"

/**
 * @brief Constructor
 */
CANMonitor::CANMonitor() 
    : loggingEnabled(false)
    , totalRxMessages(0)
    , totalTxMessages(0)
    , startTime(millis())
{
}

/**
 * @brief Initialize message definitions
 */
void CANMonitor::initializeMessageDefinitions() {
    // BMS Messages
    addMessageDefinition(CANMessageDef(0x010, "BMS_Status", "Battery Management System Status", 8));
    
    // BSC Messages
    addMessageDefinition(CANMessageDef(CANIds::BSC_COMM, "BSC_Control", "BSC Control Message", 3));
    addMessageDefinition(CANMessageDef(CANIds::BSC_LIM, "BSC_Limits", "BSC Limits Message", 6));
    addMessageDefinition(CANMessageDef(CANIds::BSC_VAL, "BSC_Status", "BSC Status Values", 8));
    
    // DMC Messages
    addMessageDefinition(CANMessageDef(CANIds::DMCCTRL, "DMC_Control", "DMC Control Message", 8));
    addMessageDefinition(CANMessageDef(CANIds::DMCLIM, "DMC_Limits", "DMC Limits Message", 8));
    addMessageDefinition(CANMessageDef(0x258, "DMC_Status", "DMC Status Message", 8));
    addMessageDefinition(CANMessageDef(0x259, "DMC_Power", "DMC Power Message", 8));
    addMessageDefinition(CANMessageDef(0x458, "DMC_Temperature", "DMC Temperature Message", 8));
    
    // NLG Messages
    addMessageDefinition(CANMessageDef(CANIds::NLG_DEM_LIM, "NLG_Control", "NLG Control Message", 8));
    addMessageDefinition(CANMessageDef(CANIds::NLG_ACT_LIM, "NLG_Status", "NLG Status Message", 8));
    addMessageDefinition(CANMessageDef(CANIds::NLG_ACT_PLUG, "NLG_Connector", "NLG Connector Status", 8));
    
    // Configuration Messages
    addMessageDefinition(CANMessageDef(CANIds::CONFIG_MESSAGE, "Config_Update", "Configuration Update", 8));
    
    Serial.println("CAN message definitions initialized");
}

/**
 * @brief Add a message definition
 */
void CANMonitor::addMessageDefinition(const CANMessageDef& def) {
    messageDefs[def.id] = def;
}

/**
 * @brief Get message description by ID
 */
String CANMonitor::getMessageDescription(uint32_t id) {
    auto it = messageDefs.find(id);
    if (it != messageDefs.end()) {
        return it->second.name + ": " + it->second.description;
    }
    return "Unknown_Message_" + String(id, HEX);
}

/**
 * @brief Decode message data
 */
String CANMonitor::decodeMessage(uint32_t id, const uint8_t* data, uint8_t length) {
    String result = "ID:0x" + String(id, HEX) + " [" + String(length) + "] ";
    result += formatHexData(data, length);
    
    auto it = messageDefs.find(id);
    if (it != messageDefs.end()) {
        result += " (" + it->second.name + ")";
    }
    
    return result;
}

/**
 * @brief Log CAN message
 */
void CANMonitor::logMessage(uint32_t id, const uint8_t* data, uint8_t length, bool transmitted) {
    if (!loggingEnabled || !isFiltered(id)) {
        return;
    }
    
    updateStatistics(id, transmitted);
    
    // Create log entry
    String direction = transmitted ? "TX" : "RX";
    String message = "[" + direction + "] " + decodeMessage(id, data, length);
    
    Serial.println(message);
}

/**
 * @brief Update statistics
 */
void CANMonitor::updateStatistics(uint32_t id, bool transmitted) {
    if (transmitted) {
        totalTxMessages++;
        txCounts[id]++;
    } else {
        totalRxMessages++;
        rxCounts[id]++;
    }
    
    lastSeen[id] = millis();
}

/**
 * @brief Get statistics as JSON
 */
String CANMonitor::getStatisticsJSON() {
    JsonDocument doc;
    
    doc["uptime"] = millis() - startTime;
    doc["totalRx"] = totalRxMessages;
    doc["totalTx"] = totalTxMessages;
    doc["uniqueMessages"] = messageDefs.size();
    
    JsonObject messages = doc["messages"].to<JsonObject>();
    for (const auto& pair : rxCounts) {
        JsonObject msgObj = messages[String(pair.first, HEX)].to<JsonObject>();
        msgObj["rx"] = pair.second;
        msgObj["tx"] = (txCounts.count(pair.first) > 0) ? txCounts.at(pair.first) : 0;
        msgObj["lastSeen"] = lastSeen[pair.first];
        
        auto it = messageDefs.find(pair.first);
        if (it != messageDefs.end()) {
            msgObj["name"] = it->second.name;
        }
    }
    
    String result;
    serializeJson(doc, result);
    return result;
}

/**
 * @brief Reset statistics
 */
void CANMonitor::resetStatistics() {
    totalRxMessages = 0;
    totalTxMessages = 0;
    startTime = millis();
    rxCounts.clear();
    txCounts.clear();
    lastSeen.clear();
}

/**
 * @brief Format hex data - FIXED toUpperCase() issue
 */
String CANMonitor::formatHexData(const uint8_t* data, uint8_t length) {
    String result;
    for (uint8_t i = 0; i < length; i++) {
        if (i > 0) result += " ";
        if (data[i] < 0x10) result += "0";
        result += String(data[i], HEX);
    }
    // FIXED: toUpperCase() returns void, so call it then return
    result.toUpperCase();
    return result;
}

/**
 * @brief Decode signal helper
 */
String CANMonitor::decodeSignal(const uint8_t* data, int byteIndex, float scale, float offset, const String& unit) {
    if (byteIndex >= 0 && byteIndex < 8) {
        float value = data[byteIndex] * scale + offset;
        return String(value, 2) + unit;
    }
    return "N/A";
}