#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include "config.h"
#include "vehicle_parameters.h"
#include "state_manager.h"
#include <WiFi.h>
#include <esp_now.h>

// Forward declarations
class StateManager;
class CANMonitor;
class ErrorMonitor;

/**
 * @brief Battery Management System data structure
 * Contains battery state information from BMS
 */
struct BMSData {
    uint8_t soc;              ///< State of charge (%)
    uint16_t voltage;         ///< Battery voltage (V)
    int16_t current;         ///< Battery current (A)
    uint16_t maxDischarge;    ///< Maximum discharge current (A)
    uint16_t maxCharge;       ///< Maximum charge current (A)
};

/**
 * @brief Drive Motor Controller data structure
 * Contains motor and inverter operational data
 */
struct DMCData {
    bool ready;              ///< DMC ready state
    bool running;            ///< DMC operational state
    float torqueAvailable;   ///< Available torque (Nm)
    float torqueActual;      ///< Current torque output (Nm)
    float speedActual;       ///< Current motor speed (RPM)
    float dcVoltageAct;      ///< DC bus voltage (V)
    float dcCurrentAct;      ///< DC bus current (A)
    float acCurrentAct;      ///< AC phase current (A)
    int32_t mechPower;       ///< Mechanical power output (W)
    float tempInverter;      ///< Inverter temperature (°C)
    float tempMotor;         ///< Motor temperature (°C)
    int8_t tempSystem;       ///< System temperature (°C)
};

/**
 * @brief Battery Switch Controller data structure
 * Contains DC-DC converter operational data
 */
struct BSCData {
    float hvVoltageAct;      ///< High voltage side voltage (V)
    float lvVoltageAct;      ///< Low voltage side voltage (V)
    float hvCurrentAct;      ///< High voltage side current (A)
    float lvCurrentAct;      ///< Low voltage side current (A)
    uint8_t mode;            ///< Operating mode (Buck/Boost)
};

/**
 * @brief Network Load Gateway data structure
 * Contains charging system data
 */
struct NLGData {
    uint8_t stateCtrlPilot;   ///< Control pilot state
    uint16_t dcHvVoltageAct;  ///< Charging voltage (V)
    uint8_t stateAct;         ///< Charger state
    uint16_t dcHvCurrentAct;  ///< Charging current (A)
    bool connectorLocked;     ///< Charge connector status
    uint8_t coolingRequest;   ///< Cooling demand
    float tempCoolPlate;      ///< Cooling plate temperature (°C)
    bool unlockRequest;       ///< Connector unlock request
    uint8_t stateDemand;      ///< Requested charger state
    uint8_t ledDemand;        ///< Charge LED state request
};

// Define ESP-NOW message types
#define MSG_TYPE_BMS 0x01
#define MSG_TYPE_DMC_TEMP 0x02

/**
 * @brief CAN Communication Manager Class
 * 
 * Handles all CAN bus communication for the vehicle control system.
 * Manages message timing, data parsing, and system coordination.
 */
class CANManager {
public:
    /**
     * @brief Construct a new CAN Manager
     * @param cs_pin SPI chip select pin for CAN controller
     */
    explicit CANManager(uint8_t cs_pin);
    ~CANManager();
    
    /**
     * @brief Set torque demand as percentage (-100% to +100%)
     * @param torquePercent Torque percentage where negative = forward/regen, positive = reverse/regen
     * Automatically converts percentage to Nm based on current gear and configuration
     */
    void setTorquePercentage(float torquePercent);

    /**
     * @brief Set the State Manager after initialization
     * @param sm Pointer to the state manager instance
     */
    void setStateManager(StateManager* sm) { stateManager = sm; }

    /**
     * @brief Initialize CAN hardware and communication
     * Sets up MCP2515 controller and message filters
     */
    void begin();

    /**
     * @brief Update CAN communication
     * Handles message reception and transmission timing
     * Should be called in fast control loop
     */
    void update();
    
    /**
     * @brief Send BSC control and limit messages
     * Transmits DC-DC converter control parameters
     */
    void sendBSC();

    /**
     * @brief Send DMC control and limit messages
     * Transmits motor control parameters
     */
    void sendDMC();

    /**
     * @brief Send NLG control messages
     * Transmits charging control parameters
     */
    void sendNLG();
    
    /**
     * @brief Initialize ESP-NOW communication
     * Sets up ESP-NOW for peer-to-peer data transmission
     * @param macAddress Target receiver MAC address
     */
    void beginESPNOW(uint8_t* macAddress);
    
    /**
     * @brief Send BMS data via ESP-NOW
     * Transmits battery management system data to receiver
     */
    void sendBMSDataESPNOW();

    /**
     * @brief Send DMC temperature data via ESP-NOW
     * Transmits motor temperature data to receiver
     */
    void sendDMCTempESPNOW();
    
    // Data access methods
    const BMSData& getBMSData() const { return bmsData; }
    const DMCData& getDMCData() const { return dmcData; }
    const BSCData& getBSCData() const { return bscData; }
    const NLGData& getNLGData() const { return nlgData; }
    
    // Control methods
    void setCurrentGear(GearState gear) { currentGear = gear; }
    void setTorqueDemand(float torque) { torqueDemand = torque; }
    void setSpeedDemand(int16_t speed) { speedDemand = speed; }
    void setEnableDMC(bool enable) { enableDMC = enable; }
    void setEnableBSC(bool enable) { enableBSC = enable; }
    void setModeBSC(bool mode) { modeBSC = mode; }
    void setHVVoltage(uint16_t voltage) { hvVoltage = voltage; }
    void setNLGStateDemand(uint8_t state) { nlgData.stateDemand = state; }
    void setNLGLedDemand(uint8_t led) { nlgData.ledDemand = led; }
    void setNLGUnlockRequest(bool unlock) { nlgData.unlockRequest = unlock; }
    void setNeedsClearError(bool needs) { 
        needsClearError = needs; 
        if (needs && !inErrorClearSequence) {
            inErrorClearSequence = true;
            errorClearStartTime = millis();
            if (stateManager) {
                stateManager->setErrorLatch(true);  // Set error latch high immediately
            }
        } else if (!needs) {
            inErrorClearSequence = false;
            if (stateManager) {
                stateManager->setErrorLatch(false);  // Set error latch low immediately
            }
        }
    }

    // CAN Monitoring methods - FIXED
    void setCANMonitor(CANMonitor* monitor) { canMonitor = monitor; }
    void setSystemMonitor(ErrorMonitor* monitor) { systemMonitor = monitor; }
    void enableCANLogging(bool enable);
    String getCANStatistics();
    void resetCANStatistics();
        
private:

    /**
    * @brief Process external configuration message
    * @param buf Message data buffer
    * 
    * Processes configuration settings received via CAN
    */
    void processConfigMessage(uint8_t* buf);

    /**
     * @brief Process BMS status message
     * @param buf Message data buffer
     */
    void processBMSMessage(uint8_t* buf);

    /**
     * @brief Process BSC status message
     * @param buf Message data buffer
     */
    void processBSCMessage(uint8_t* buf);

    /**
     * @brief Process DMC status messages
     * @param id Message identifier
     * @param buf Message data buffer
     */
    void processDMCMessage(uint32_t id, uint8_t* buf);

    /**
     * @brief Process NLG status messages
     * @param id Message identifier
     * @param buf Message data buffer
     */
    void processNLGMessage(uint32_t id, uint8_t* buf);
    
    /**
     * @brief Reset all message buffers to default state
     */
    void resetMessageBuffers();

    /**
     * @brief Check for and process incoming messages
     */
    void checkAndProcessMessages();
    
    /**
     * @brief Callback function for ESP-NOW send status
     * @param mac_addr MAC address of the receiver
     * @param status Send status (success or failure)
     */
    static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    
    bool needsClearError = false;  // Flag to indicate error clearing needed
    unsigned long errorClearStartTime = 0;  // Timestamp for error clearing
    bool inErrorClearSequence = false;  // Track if we're in the middle of error clearing
    int calculateTaperedDMCCurrent();

    // Hardware interfaces
    mcp2515_can CAN;         ///< CAN controller instance
    SPIClass* customSPI;     ///< Custom SPI interface
    
    // Message buffers for different subsystems
    uint8_t controlBufferDMC[8];  ///< DMC control message buffer
    uint8_t controlBufferBSC[8];  ///< BSC control message buffer
    uint8_t limitBufferBSC[8];    ///< BSC limits message buffer
    uint8_t limitBufferDMC[8];    ///< DMC limits message buffer
    uint8_t controlBufferNLG[8];  ///< NLG control message buffer
    
    // System data storage
    BMSData bmsData;         ///< Battery system data
    DMCData dmcData;         ///< Motor controller data
    BSCData bscData;         ///< DC-DC converter data
    NLGData nlgData;         ///< Charging system data
    
    StateManager* stateManager;  ///< Pointer to state manager
    GearState currentGear = GearState::NEUTRAL;

    // Control parameters
    float torqueDemand;      ///< Requested motor torque
    float torquePercentage;  ///< Current torque demand as percentage (-100 to +100)
    int16_t speedDemand;     ///< Requested motor speed
    bool enableDMC;          ///< Motor controller enable flag
    bool enableBSC;          ///< DC-DC converter enable flag
    bool modeBSC;            ///< DC-DC converter mode
    uint16_t lvVoltage;      ///< Low voltage system voltage
    uint16_t hvVoltage;      ///< High voltage system voltage
    
    // ESP-NOW variables
    uint8_t receiverMacAddress[6]; ///< MAC address of the ESP-NOW receiver
    bool espNowInitialized = false; ///< Flag to indicate if ESP-NOW is initialized
    static uint32_t messagesSent;   ///< Counter for successful messages
    static uint32_t messagesFailed; ///< Counter for failed messages
    
    // Timing management
    unsigned long lastFastCycle;   ///< Last fast update cycle timestamp
    unsigned long lastSlowCycle;   ///< Last slow update cycle timestamp
    unsigned long lastBMSSendTime; ///< Last BMS data send timestamp
    unsigned long lastDMCSendTime; ///< Last DMC data send timestamp

    // CAN Monitoring - FIXED: Use pointers instead of direct objects
    CANMonitor* canMonitor = nullptr;      ///< CAN bus monitor instance
    ErrorMonitor* systemMonitor = nullptr; ///< System error monitor instance
};