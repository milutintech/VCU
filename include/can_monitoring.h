/**
 * @file can_monitoring.h
 * @brief CAN Message Monitoring and Logging Extensions
 */

#pragma once
#include <Arduino.h>
#include <map>
#include "error_monitor.h"

/**
 * @brief CAN Message Definition Structure
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

// Add these methods to your existing CANManager class:
/*
class CANManager {
    // ... existing code ...
    
private:
    CANMonitor canMonitor;  // Add this member
    
public:
    // Add these methods to existing CANManager:
    void enableCANLogging(bool enable) { canMonitor.setLoggingEnabled(enable); }
    String getCANStatistics() { return canMonitor.getStatisticsJSON(); }
    void resetCANStatistics() { canMonitor.resetStatistics(); }
    
    // Modify your existing checkAndProcessMessages() method to add logging:
    void checkAndProcessMessages() {
        uint8_t len;
        uint8_t buf[8];
        while (CAN_MSGAVAIL == CAN.checkReceive()) {
            if (CAN.readMsgBuf(&len, buf) == CAN_OK) {
                uint32_t id = CAN.getCanId();
                
                // ADD THIS LINE to log received messages:
                if (canMonitor.isLoggingEnabled()) {
                    canMonitor.logMessage(id, buf, len, false);  // false = received
                    systemMonitor.logCANMessage(id, buf, len, false, canMonitor.getMessageDescription(id));
                }
                
                // ... existing switch statement for message processing ...
            }
        }
    }
    
    // Modify your send methods to add logging:
    void sendBSC() {
        // ... existing code to prepare messages ...
        
        CAN.sendMsgBuf(CANIds::BSC_COMM, 0, 3, controlBufferBSC);
        if (canMonitor.isLoggingEnabled()) {
            canMonitor.logMessage(CANIds::BSC_COMM, controlBufferBSC, 3, true);  // true = transmitted
            systemMonitor.logCANMessage(CANIds::BSC_COMM, controlBufferBSC, 3, true, "BSC Control");
        }
        
        CAN.sendMsgBuf(CANIds::BSC_LIM, 0, 6, limitBufferBSC);
        if (canMonitor.isLoggingEnabled()) {
            canMonitor.logMessage(CANIds::BSC_LIM, limitBufferBSC, 6, true);
            systemMonitor.logCANMessage(CANIds::BSC_LIM, limitBufferBSC, 6, true, "BSC Limits");
        }
    }
    
    // Similar modifications for sendDMC() and sendNLG()
};
*/

/**
 * @brief Predefined CAN Message Definitions
 */
const CANMessageDef MESSAGE_DEFINITIONS[] = {
    // BMS Messages
    {0x010, "BMS_Status", "Battery Management System Status", 8},
    
    // DMC Messages  
    {0x258, "DMC_Status", "Drive Motor Controller Status", 8},
    {0x259, "DMC_Power", "Drive Motor Controller Power Data", 8},
    {0x458, "DMC_Temperature", "Drive Motor Controller Temperature", 8},
    
    // BSC Messages
    {0x260, "BSC_Control", "Battery Switch Controller Control", 3},
    {0x261, "BSC_Limits", "Battery Switch Controller Limits", 6},
    {0x26A, "BSC_Status", "Battery Switch Controller Status", 8},
    
    // NLG Messages
    {0x711, "NLG_Control", "Network Load Gateway Control", 8},
    {0x728, "NLG_Status_Limits", "Network Load Gateway Status & Limits", 8},
    {0x739, "NLG_Status_Plug", "Network Load Gateway Plug Status", 8},
    {0x799, "NLG_Errors", "Network Load Gateway Errors", 8},
    
    // Configuration Messages
    {0x011, "VCU_Config", "VCU Configuration Message", 8}
};

/**
 * @brief Initialize CAN message definitions with detailed signal information
 */
void initializeCANMessageDefinitions(CANMonitor& monitor) {
    // BMS Status (0x010)
    CANMessageDef bms(0x010, "BMS_Status", "Battery pack status and measurements", 8);
    bms.signalNames[0] = "SOC";
    bms.signalNames[1] = "Voltage_High";
    bms.signalNames[2] = "Voltage_Low";  
    bms.signalNames[3] = "Current_High";
    bms.signalNames[4] = "Current_Low";
    bms.signalNames[5] = "MaxDischarge_High";
    bms.signalNames[6] = "MaxDischarge_Low";
    bms.signalNames[7] = "MaxCharge";
    bms.scales[0] = 0.5f;  // SOC in 0.5% steps
    bms.scales[1] = 0.1f;  // Voltage in 0.1V steps
    bms.scales[2] = 0.1f;
    bms.scales[7] = 2.0f;  // Current in 2A steps
    bms.units[0] = "%";
    bms.units[1] = "V";
    bms.units[2] = "V";
    bms.units[3] = "A";
    bms.units[4] = "A";
    bms.units[5] = "A";
    bms.units[6] = "A";
    bms.units[7] = "A";
    monitor.addMessageDefinition(bms);
    
    // DMC Status (0x258)
    CANMessageDef dmcStatus(0x258, "DMC_Status", "Motor controller operational status", 8);
    dmcStatus.signalNames[0] = "Status_Flags";
    dmcStatus.signalNames[1] = "Reserved";
    dmcStatus.signalNames[2] = "Torque_Available_H";
    dmcStatus.signalNames[3] = "Torque_Available_L";
    dmcStatus.signalNames[4] = "Torque_Actual_H";
    dmcStatus.signalNames[5] = "Torque_Actual_L";
    dmcStatus.signalNames[6] = "Speed_H";
    dmcStatus.signalNames[7] = "Speed_L";
    dmcStatus.scales[2] = 0.01f;  // Torque in 0.01Nm
    dmcStatus.scales[3] = 0.01f;
    dmcStatus.scales[4] = 0.01f;
    dmcStatus.scales[5] = 0.01f;
    dmcStatus.units[2] = "Nm";
    dmcStatus.units[3] = "Nm";
    dmcStatus.units[4] = "Nm";
    dmcStatus.units[5] = "Nm";
    dmcStatus.units[6] = "RPM";
    dmcStatus.units[7] = "RPM";
    monitor.addMessageDefinition(dmcStatus);
    
    // Add more message definitions...
    // BSC, NLG, etc. following the same pattern
}

/**
 * @brief Enhanced CAN message decoder with human-readable output
 */
String decodeCANMessage(uint32_t id, const uint8_t* data, uint8_t length) {
    String result = "";
    
    switch (id) {
        case 0x010: // BMS
            {
                uint8_t soc = data[0] / 2;
                uint16_t voltage = (data[2] | (data[1] << 8)) / 10;
                int16_t current = (data[4] | (data[3] << 8));
                uint16_t maxDischarge = (data[6] | (data[5] << 8));
                uint8_t maxCharge = data[7] * 2;
                
                result = "SOC:" + String(soc) + "% V:" + String(voltage) + "V I:" + String(current) + 
                        "A MaxDis:" + String(maxDischarge) + "A MaxCh:" + String(maxCharge) + "A";
            }
            break;
            
        case 0x258: // DMC Status
            {
                bool ready = data[0] & 0x80;
                bool running = data[0] & 0x40;
                float torqueAvail = ((data[2] << 8) | data[3]) * 0.01f;
                float torqueActual = ((data[4] << 8) | data[5]) * 0.01f;
                int16_t speed = (data[6] << 8) | data[7];
                
                result = "Ready:" + String(ready ? "Y" : "N") + " Run:" + String(running ? "Y" : "N") +
                        " TrqAvail:" + String(torqueAvail) + "Nm TrqAct:" + String(torqueActual) + 
                        "Nm Speed:" + String(speed) + "RPM";
            }
            break;
            
        case 0x26A: // BSC Status
            {
                float hvVolt = ((data[0] << 8) | data[1]) * 0.1f;
                float lvVolt = data[2] * 0.1f;
                float hvCurr = (((data[3] << 8) | data[4]) * 0.1f) - 25.0f;
                float lvCurr = ((data[5] << 8) | data[6]) - 280.0f;
                uint8_t mode = data[7] >> 4;
                
                result = "HV:" + String(hvVolt) + "V LV:" + String(lvVolt) + "V HI:" + 
                        String(hvCurr) + "A LI:" + String(lvCurr) + "A Mode:" + String(mode);
            }
            break;
            
        default:
            // Generic hex dump for unknown messages
            result = "Data: ";
            for (int i = 0; i < length; i++) {
                result += String(data[i], HEX);
                if (i < length - 1) result += " ";
            }
            break;
    }
    
    return result;
}