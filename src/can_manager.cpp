/**
 * @file can_manager.cpp
 * @brief Implementation of CAN communication management system
 * 
 * Handles all CAN bus communication between vehicle subsystems including:
 * - Message transmission and reception
 * - Data parsing and scaling
 * - Communication timing
 * - System state coordination
 * 
 * Uses MCP2515 CAN controller via SPI interface.
 */

#include "can_manager.h"
#include "state_manager.h"
#include <esp_task_wdt.h>
#include "config.h"
#include <algorithm>
#include "configuration.h"
/**
 * @brief Constructs the CAN manager
 * @param cs_pin SPI chip select pin for CAN controller
 * 
 * Initializes communication buffers and system state.
 */
CANManager::CANManager(uint8_t cs_pin)
    : CAN(cs_pin)
    , stateManager(nullptr)
    , lastFastCycle(0)
    , lastSlowCycle(0)
    , enableDMC(false)
    , enableBSC(false)
    , modeBSC(false)
    , torqueDemand(0)
    , speedDemand(0)
    , lvVoltage(0)
    , hvVoltage(0)
{
    customSPI = new SPIClass(HSPI);
    resetMessageBuffers();
}

/**
 * @brief Destructor - cleans up SPI interface
 */
CANManager::~CANManager() {
    if (customSPI) {
        delete customSPI;
        customSPI = nullptr;
    }
}



/**
 * @brief Initialize CAN communication hardware
 * 
 * Configures SPI interface and initializes MCP2515 CAN controller
 * with 500kbps baudrate. Retries initialization until successful.
 */
void CANManager::begin() {
    if (!customSPI) {
        Serial.println("SPI not initialized");
        return;
    }
    
    customSPI->begin(Pins::SCK, Pins::MISO, Pins::MOSI, Pins::SPI_CS_PIN);
    CAN.setSPI(customSPI);
    
    uint8_t retries = 0;
    const uint8_t MAX_RETRIES = 5;
    
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN BUS Shield init fail");
        delay(100);
        if (++retries >= MAX_RETRIES) {
            Serial.println("CAN init failed after max retries");
            return;
        }
    }
    
    Serial.println("CAN init OK!");
}

/**
 * @brief Main update function for CAN communication
 * 
 * Processes incoming messages and handles periodic transmission:
 * - Fast cycle (10ms): DMC messages in RUN state
 * - Slow cycle (50ms): BSC and NLG messages
 * Also handles ESP-NOW messaging with specified intervals
 */
void CANManager::update() {
    if (!stateManager) {
        return; // Exit if StateManager not set
    }
    
    // Handle error clearing sequence timing
    if (inErrorClearSequence) {
        needsClearError = true;
        // If 100ms has passed and we're still in sequence, auto-clear
        if (millis() - errorClearStartTime >= 100) {
            Serial.println("cleard");
            inErrorClearSequence = false;
            needsClearError = false;
        }
    }
    // Process all incoming CAN messages
    checkAndProcessMessages();

    // Update state manager with latest data
    if (stateManager) {
        stateManager->setBatteryVoltage(bmsData.voltage);
        stateManager->setInverterTemp(dmcData.tempInverter);
        stateManager->setMotorTemp(dmcData.tempMotor);
        stateManager->setCoolingRequest(nlgData.coolingRequest);
        stateManager->setHVVoltageActual(bscData.hvVoltageAct);
    }

    // Handle periodic message transmission
    unsigned long currentTime = millis();
    
    // Fast cycle for motor control
    if ((currentTime - lastFastCycle >= Constants::FAST_CYCLE_MS) && 
        (stateManager->getCurrentState() == VehicleState::RUN)) {
        lastFastCycle = currentTime;
        sendDMC();
    }
    
    // Slow cycle for charging and DC-DC control
    if (currentTime - lastSlowCycle >= Constants::SLOW_CYCLE_MS) {
        lastSlowCycle = currentTime;
        if (stateManager->getCurrentState() == VehicleState::CHARGING) {
            sendNLG();
        } 
        sendBSC();
    }
    
    // ESP-NOW transmission for BMS data
    if (espNowInitialized && currentTime - lastBMSSendTime >= ESPNOW::BMS_SEND_INTERVAL) {
        lastBMSSendTime = currentTime;
        sendBMSDataESPNOW();
    }
    
    // ESP-NOW transmission for DMC temperature data
    if (espNowInitialized && currentTime - lastDMCSendTime >= ESPNOW::DMC_SEND_INTERVAL) {
        lastDMCSendTime = currentTime;
        sendDMCTempESPNOW();
    }
    
    // Print ESP-NOW stats periodically

}

/**
 * @brief Check for and process incoming CAN messages
 */
void CANManager::checkAndProcessMessages() {
    uint8_t len;
    uint8_t buf[8];
    while (CAN_MSGAVAIL == CAN.checkReceive()) {
        if (CAN.readMsgBuf(&len, buf) == CAN_OK) {
            uint32_t id = CAN.getCanId();
            switch(id) {
                case 0x010:  // BMS message
                    processBMSMessage(buf);
                    break;
                case CANIds::CONFIG_MESSAGE:
                    processConfigMessage(buf);
                    break;
                case CANIds::BSC_VAL:
                    processBSCMessage(buf);
                    break;
                    
                case 0x258:  // DMC Status
                case 0x259:  // DMC Power
                case 0x458:  // DMC Temperature
                    processDMCMessage(id, buf);
                    break;
                    
                case CANIds::NLG_ACT_LIM:
                case CANIds::NLG_ACT_PLUG:
                    processNLGMessage(id, buf);
                    break;
            }
        }
    }
}

/**
 * @brief Process BMS status message
 */

unsigned long lastBMSUpdate = 0;  // Variable to track last update timestamp

void CANManager::processBMSMessage(uint8_t* buf) {
    if (!buf) return;
    
    bmsData.soc = buf[0] / 2;
    bmsData.voltage = (buf[2] | (buf[1] << 8)) / 10;
    bmsData.current = (buf[4] | (buf[3] << 8));
    bmsData.maxDischarge = (buf[6] | (buf[5] << 8));
    bmsData.maxCharge = buf[7] * 2;

    lastBMSUpdate = millis(); // Update timestamp on valid message
}

/**
 * @brief Process BSC status message
 */
void CANManager::processBSCMessage(uint8_t* buf) {
    if (!buf || !stateManager) return;
    
    bscData.hvVoltageAct = ((buf[0] << 8) | buf[1]) * 0.1;
    bscData.lvVoltageAct = buf[2] * 0.1;
    bscData.hvCurrentAct = (((buf[3] << 8) | buf[4]) * 0.1) - 25;
    bscData.lvCurrentAct = ((buf[5] << 8) | buf[6]) - 280;
    bscData.mode = buf[7] >> 4;
    
    stateManager->setHVVoltageActual(bscData.hvVoltageAct);
}

/**
 * @brief Process DMC status messages
 */
void CANManager::processDMCMessage(uint32_t id, uint8_t* buf) {
    if (!buf) return;

    switch(id) {
        case 0x258:  // Status message
            dmcData.ready = buf[0] & 0x80;
            dmcData.running = buf[0] & 0x40;
            dmcData.torqueAvailable = ((buf[2] << 8) | buf[3]) * 0.01;
            dmcData.torqueActual = ((buf[4] << 8) | buf[5]) * 0.01;
            dmcData.speedActual = static_cast<float>(static_cast<int16_t>((buf[6] << 8) | buf[7]));
            //Serial.println(dmcData.speedActual);
            break;
            
        case 0x259:  // Power message
            dmcData.dcVoltageAct = ((buf[0] << 8) | buf[1]) * 0.1;
            dmcData.dcCurrentAct = ((buf[2] << 8) | buf[3]) * 0.1;
            dmcData.acCurrentAct = ((buf[4] << 8) | buf[5]) * 0.25;
            dmcData.mechPower = ((buf[6] << 8) | buf[7]) * 16;
            break;
            
        case 0x458:  // Temperature message
            dmcData.tempInverter = ((buf[0] << 8) | buf[1]) * 0.5;
            dmcData.tempMotor = ((buf[2] << 8) | buf[3]) * 0.5;
            dmcData.tempSystem = buf[4] - 50;
            
            
            if (stateManager) {
                stateManager->setInverterTemp(dmcData.tempInverter);
                stateManager->setMotorTemp(dmcData.tempMotor);
            }
            break;
    }
}

/**
 * @brief Process NLG status messages
 */
void CANManager::processNLGMessage(uint32_t id, uint8_t* buf) {
    if (!buf) return;

    switch(id) {
        case CANIds::NLG_ACT_LIM:
            nlgData.stateCtrlPilot = buf[0] >> 5;
            nlgData.dcHvVoltageAct = ((buf[0] & 0x1F) << 8) | buf[1];
            nlgData.stateAct = buf[2] >> 5;
            nlgData.dcHvCurrentAct = ((buf[2] & 0x07) << 8) | buf[3];
            nlgData.connectorLocked = (buf[7] >> 5) & 0x01;
            break;
            
        case CANIds::NLG_ACT_PLUG:
            nlgData.coolingRequest = buf[4];
            nlgData.tempCoolPlate = ((buf[6] << 8) | buf[7]) * 0.1;
            
            if (stateManager) {
                stateManager->setCoolingRequest(nlgData.coolingRequest);
            }
            break;
    }
}

/**
 * @brief Send BSC control messages
 */
void CANManager::sendBSC() {
    uint8_t lvVoltageScale = static_cast<uint8_t>(VehicleParams::Battery::MIN_LVVOLTAGE * 10);
    uint8_t hvVoltageScale = static_cast<uint8_t>(hvVoltage - 220);  // Changed to match DBC
    
    // BSC control message (0x260)
    controlBufferBSC[0] = (enableBSC << 0) | (modeBSC << 1) | 0x80;  // Set BSC6_RUNCOMM
    controlBufferBSC[1] = lvVoltageScale;
    controlBufferBSC[2] = hvVoltageScale;
   
    // BSC limits message (0x261)
    limitBufferBSC[0] = static_cast<uint8_t>(VehicleParams::Battery::MIN_VOLTAGE - 220);
    limitBufferBSC[1] = VehicleParams::Power::BSC_LV_BUCK;
    limitBufferBSC[2] = static_cast<uint8_t>(VehicleParams::Battery::PRECHARGE_CURRENT * 10);
    limitBufferBSC[3] = static_cast<uint8_t>(9 * 10);
    limitBufferBSC[4] = VehicleParams::Power::BSC_LV_BOOST;
    limitBufferBSC[5] = static_cast<uint8_t>(VehicleParams::Battery::PRECHARGE_CURRENT * 10);
    
    CAN.sendMsgBuf(CANIds::BSC_COMM, 0, 3, controlBufferBSC);
    CAN.sendMsgBuf(CANIds::BSC_LIM, 0, 6, limitBufferBSC);
}

/**
 * @brief Send DMC control messages
 */
void CANManager::sendDMC() {
    int16_t scaledTorque = static_cast<int16_t>(torqueDemand * 10);  // 0.01Nm/bit according to DBC
    
    // Set direction control bits based on current gear
    bool enablePosSpeed = true;  // Default: enable both directions
    bool enableNegSpeed = true;
    
    // Set direction bits based on stored gear
    switch(currentGear) {
        case GearState::DRIVE:
            // In DRIVE, only enable negative speed (motor runs backwards)
            enablePosSpeed = false;
            enableNegSpeed = true;
            break;
            
        case GearState::REVERSE:
            // In REVERSE, only enable positive speed (motor runs forwards)
            enablePosSpeed = true; 
            enableNegSpeed = false;
            break;
            
        case GearState::NEUTRAL:
        default:
            // In NEUTRAL, disable both directions for safety
            enablePosSpeed = false;
            enableNegSpeed = false;
            break;
    }

    if (!needsClearError) {
        // Normal operation - Enable bit set, Error clear bit not set
        // Bits:
        // 0: DMC_EnableRq - Enable power stage
        // 1: DMC_ModeRq - 0=torque mode, 1=speed mode
        // 2: DMC_OscLimEnableRq - Enable OscLim
        // 4: DMC_ClrError - Clear error latch (0->1, Enable must be 0)
        // 6: DMC_NegTrqSpd - Enable negative speed/torque
        // 7: DMC_PosTrqSpd - Enable positive speed/torque
        
        // Default configuration for normal operation
        controlBufferDMC[0] = (enableDMC << 0) |      // DMC_EnableRq at bit 0
                             (false << 1) |           // DMC_ModeRq at bit 1 (0 = torque mode)
                             (1 << 2) |               // DMC_OscLimEnableRq at bit 2
                             (0 << 4) |               // DMC_ClrError at bit 4 (not clearing)
                             (enableNegSpeed << 6) |  // DMC_NegTrqSpd at bit 6
                             (enablePosSpeed << 7);   // DMC_PosTrqSpd at bit 7
    } else {
        // Error clearing operation - Enable bit cleared, Error clear bit set
        controlBufferDMC[0] = (0 << 0) |              // DMC_EnableRq at bit 0 (must be 0 to clear error)
                             (0 << 1) |               // DMC_ModeRq at bit 1
                             (0 << 2) |               // DMC_OscLimEnableRq at bit 2
                             (0 << 3) |  
                             (1 << 4) |               // DMC_ClrError at bit 4 (clearing)
                             (0 << 5) |  
                             (0 << 6) |               // DMC_NegTrqSpd - Disabled during error clear
                             (0 << 7);                // DMC_PosTrqSpd - Disabled during error clear
    }
    
    // Speed limit (16-bit signed value in RPM)
    int16_t speedLimit = VehicleParams::Motor::MAX_RPM;
    controlBufferDMC[2] = speedLimit >> 8;
    controlBufferDMC[3] = speedLimit & 0xFF;
    
    // Torque request (16-bit signed value in 0.01Nm)
    controlBufferDMC[4] = scaledTorque >> 8;
    controlBufferDMC[5] = scaledTorque & 0xFF;
    
    // DMC limits message (0x211)
    int dcVoltLimMotor = VehicleParams::Battery::MIN_VOLTAGE * 10;
    int dcVoltLimGen = (VehicleParams::Battery::MAX_VOLTAGE + 4) * 10;
    int dcCurrLimMotor = VehicleParams::Battery::MAX_DMC_CURRENT * 10;
    int dcCurrLimGen = VehicleParams::Power::DMC_DC_GEN * 10;
    
    limitBufferDMC[0] = dcVoltLimMotor >> 8;
    limitBufferDMC[1] = dcVoltLimMotor & 0xFF;
    limitBufferDMC[2] = dcVoltLimGen >> 8;
    limitBufferDMC[3] = dcVoltLimGen & 0xFF;
    limitBufferDMC[4] = dcCurrLimMotor >> 8;
    limitBufferDMC[5] = dcCurrLimMotor & 0xFF;
    limitBufferDMC[6] = dcCurrLimGen >> 8;
    limitBufferDMC[7] = dcCurrLimGen & 0xFF;
    
    CAN.sendMsgBuf(CANIds::DMCCTRL, 0, 8, controlBufferDMC);
    CAN.sendMsgBuf(CANIds::DMCLIM, 0, 8, limitBufferDMC);
}
/**
 * @brief Send NLG control messages
 */
void CANManager::sendNLG() {
    int nlgVoltageScale = static_cast<int>(VehicleParams::Battery::MAX_VOLTAGE * 10);
    // Check if BMS timeout has occurred
    if (millis() - lastBMSUpdate > VehicleParams::Timing::BMS_TIMEOUT_MS) {
        bmsData.maxCharge = 0;  // Set max charge current to 0 if timeout occurs
    }

    // Use the configuration value for maximum charging current
    uint8_t maxNlgCurrentAC = config.getMaxChargingCurrent();
    
    // Limit charging current by smaller of max charger current and BMS max charge
    float limitedCurrentDC = std::min(static_cast<int>(VehicleParams::Battery::MAX_NLG_CURRENT), static_cast<int>(bmsData.maxCharge));
    int nlgCurrentScaleDC = static_cast<int>((limitedCurrentDC + 102.4) * 10);
    int nlgCurrentScaleAC = static_cast<int>((maxNlgCurrentAC  + 102.4) * 10);
    
    controlBufferNLG[0] = (false << 7) | (nlgData.unlockRequest << 6) | (false << 5) | ((nlgVoltageScale >> 8) & 0x1F);
    controlBufferNLG[1] = nlgVoltageScale & 0xFF;
    controlBufferNLG[2] = (nlgData.stateDemand << 5) | ((nlgCurrentScaleDC >> 8) & 0x07);
    controlBufferNLG[3] = nlgCurrentScaleDC & 0xFF;
    controlBufferNLG[4] = (nlgData.ledDemand << 4) | ((nlgCurrentScaleAC >> 8) & 0x07);
    controlBufferNLG[5] = nlgCurrentScaleAC & 0xFF;
    
    CAN.sendMsgBuf(CANIds::NLG_DEM_LIM, 0, 8, controlBufferNLG);
}
/**
 * @brief Process external configuration message
 * @param buf Message data buffer
 * 
 * Receives and applies configuration parameters from other CAN devices.
 * Only valid parameters within allowed ranges are applied.
 */
void CANManager::processConfigMessage(uint8_t* buf) {
    if (!buf) return;
    
    bool configChanged = false;
    
    // Process drive mode
    uint8_t driveModeByte = buf[0];
    if (config.setDriveModeFromByte(driveModeByte)) {
        configChanged = true;
        Serial.print("CAN: Drive mode updated to: ");
        Serial.println(config.getDriveModeString());
    }
    
    // Process max torque
    uint16_t maxTorque = (buf[1] << 8) | buf[2];
    if (config.setMaxTorque(maxTorque)) {
        configChanged = true;
        Serial.print("CAN: Max torque set to: ");
        Serial.print(maxTorque);
        Serial.println(" Nm");
    }
    
    // Process max SOC
    uint8_t maxSOC = buf[3];
    if (config.setMaxSOC(maxSOC)) {
        configChanged = true;
        Serial.print("CAN: Max SOC set to: ");
        Serial.print(maxSOC);
        Serial.println("%");
    }
    
    // Process max charging current
    uint8_t maxChargingCurrent = buf[4];
    if (config.setMaxChargingCurrent(maxChargingCurrent)) {
        configChanged = true;
        Serial.print("CAN: Max charging current set to: ");
        Serial.print(maxChargingCurrent);
        Serial.println(" A");
    }
    
    // If anything changed, save to flash
    if (configChanged) {
        config.save();
        Serial.println("Configuration updated from CAN and saved to flash");
        
        // Apply the changes immediately
        if (stateManager) {
            stateManager->updateDriveMode(config.getDriveMode());
        }
    }
}

/**
 * @brief Reset all CAN message buffers to default state
 * 
 * Initializes/clears all message buffers used for CAN communication:
 * - DMC (Drive Motor Controller) control and limit buffers
 * - BSC (Battery Switch Controller) control and limit buffers 
 * - NLG (Network Load Gateway) control buffer
 * 
 * This prevents sending stale or invalid data after initialization
 * or after error recovery. All buffers are zeroed using memset.
 * 
 * The buffers cleared are:
 * - controlBufferDMC: Motor control messages
 * - controlBufferBSC: DC-DC converter control
 * - limitBufferBSC: DC-DC converter limits
 * - limitBufferDMC: Motor controller limits
 * - controlBufferNLG: Charger control
 */
void CANManager::resetMessageBuffers() {
    memset(controlBufferDMC, 0, 8);
    memset(controlBufferBSC, 0, 8);
    memset(limitBufferBSC, 0, 8);
    memset(limitBufferDMC, 0, 8);
    memset(controlBufferNLG, 0, 8);
}
// Define static member variables first
uint32_t CANManager::messagesSent = 0;
uint32_t CANManager::messagesFailed = 0;

/**
 * @brief Initialize ESP-NOW communication
 * Sets up ESP-NOW for peer-to-peer data transmission
 * @param macAddress Target receiver MAC address
 */
void CANManager::beginESPNOW(uint8_t* macAddress) {
    if (espNowInitialized) {
        return; // Already initialized
    }
    
    // Store the MAC address
    memcpy(receiverMacAddress, macAddress, 6);
    
    // Initialize WiFi in STA mode
    WiFi.mode(WIFI_STA);
    
    // Print MAC Address for debugging
    Serial.print("Sender MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Register callback
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    
    // Print target device
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             receiverMacAddress[0], receiverMacAddress[1], receiverMacAddress[2], 
             receiverMacAddress[3], receiverMacAddress[4], receiverMacAddress[5]);
    Serial.printf("ESP-NOW Target receiver: %s\n", macStr);
    
    espNowInitialized = true;
    lastBMSSendTime = 0;
    lastDMCSendTime = 0;
    
    Serial.println("ESP-NOW initialized successfully");
}

/**
 * @brief Callback function for ESP-NOW send status
 * Static function to handle ESP-NOW send status
 */
void CANManager::OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    
    if (status == ESP_NOW_SEND_SUCCESS) {
        messagesSent++;
    } else {
        messagesFailed++;
    }
    
    Serial.printf("ESP-NOW sent to: %s, Status: %s, Success/Fail: %u/%u\n", 
                  macStr, 
                  status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed",
                  messagesSent,
                  messagesFailed);
}

/**
 * @brief Send BMS data via ESP-NOW
 * Transmits battery management system data to receiver
 */
void CANManager::sendBMSDataESPNOW() {
    if (!espNowInitialized) {
        return; // Not initialized
    }
    
    // Create a message buffer for BMS data
    uint8_t buffer[6]; // Buffer size: 1 (msgType) + 2 (voltage) + 2 (current) + 1 (SOC)
    
    // Prepare the message buffer
    buffer[0] = MSG_TYPE_BMS;  // Message type
    
    // Convert voltage to integer (multiply by 10 to preserve 1 decimal place)
    uint16_t voltageInt = (uint16_t)(bmsData.voltage * 10.0);
    buffer[1] = (voltageInt >> 8) & 0xFF;  // high byte
    buffer[2] = voltageInt & 0xFF;         // low byte
    
    // Current as int16_t 
    buffer[3] = (bmsData.current >> 8) & 0xFF;  // high byte
    buffer[4] = bmsData.current & 0xFF;         // low byte
    
    // SOC as direct byte
    buffer[5] = bmsData.soc;
    
    // Debug print
    Serial.printf("Sending BMS data: SOC=%d%%, Voltage=%.1fV, Current=%dA\n", 
                  bmsData.soc, (float)bmsData.voltage, bmsData.current);
    
    // Debug raw data
    Serial.print("Raw data: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    Serial.println();
    
    // Send the message
    esp_err_t result = esp_now_send(receiverMacAddress, buffer, 6);
    
    if (result != ESP_OK) {
        Serial.println("Error sending BMS message via ESP-NOW");
    }
}

/**
 * @brief Send DMC temperature data via ESP-NOW
 * Transmits motor temperature data to receiver
 */
void CANManager::sendDMCTempESPNOW() {
    if (!espNowInitialized) {
        return; // Not initialized
    }
    
    // Create a message buffer for DMC temperature data
    uint8_t buffer[5]; // Buffer size: 1 (msgType) + 2 (invTemp) + 2 (motorTemp)
    
    // Prepare the message buffer
    buffer[0] = MSG_TYPE_DMC_TEMP;  // Message type
    
    // Convert temperatures to integers (multiply by 10 to preserve 1 decimal place)
    uint16_t invTempInt = (uint16_t)(dmcData.tempInverter * 10.0);
    buffer[1] = (invTempInt >> 8) & 0xFF;  // high byte
    buffer[2] = invTempInt & 0xFF;         // low byte
    
    uint16_t motorTempInt = (uint16_t)(dmcData.tempMotor * 10.0);
    buffer[3] = (motorTempInt >> 8) & 0xFF;  // high byte
    buffer[4] = motorTempInt & 0xFF;         // low byte
    /*
    // Debug print
    Serial.printf("Sending DMC temp data: Inverter=%.1f°C, Motor=%.1f°C\n", 
                  dmcData.tempInverter, dmcData.tempMotor);
    
    // Debug raw data
    Serial.print("Raw data: ");
    for (int i = 0; i < 5; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    Serial.println();
    */
    // Send the message
    esp_err_t result = esp_now_send(receiverMacAddress, buffer, 5);
    
    if (result != ESP_OK) {
        Serial.println("Error sending DMC temp message via ESP-NOW");
    }
}
