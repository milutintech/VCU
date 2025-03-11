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
 */
void CANManager::update() {
    if (!stateManager) {
        return; // Exit if StateManager not set
    }
    
    // Handle error clearing sequence timing
    if (inErrorClearSequence) {
        // If 100ms has passed and we're still in sequence, auto-clear
        if (millis() - errorClearStartTime >= 100) {
            if (stateManager) {
                stateManager->setErrorLatch(false);  // Set error latch low
            }
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
    
    // Fast cycle for motor control (10ms)
    if ((currentTime - lastFastCycle >= Constants::FAST_CYCLE_MS) && 
        (stateManager->getCurrentState() == VehicleState::RUN)) {
        lastFastCycle = currentTime;
        sendDMC();
    }
    
    // Slow cycle for charging and DC-DC control (100ms)
    if (currentTime - lastSlowCycle >= Constants::SLOW_CYCLE_MS) {
        lastSlowCycle = currentTime;
        if (stateManager->getCurrentState() == VehicleState::CHARGING) {
            sendNLG();
        } 
        sendBSC();
    }
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
    int16_t scaledTorque = static_cast<int16_t>(torqueDemand * 100);  // 0.01Nm/bit according to DBC
    
    // DMC control message (0x210)
    // Bits:
    // 0: DMC_EnableRq - Enable power stage
    // 1: DMC_ModeRq - 0=torque mode, 1=speed mode
    // 2: DMC_OscLimEnableRq - Enable OscLim
    // 4: DMC_ClrError - Clear error latch (0->1, Enable must be 0)
    // 6: DMC_NegTrqSpd - Enable negative speed/torque
    // 7: DMC_PosTrqSpd - Enable positive speed/torque
    
    // Default configuration for normal operation
    if (!needsClearError) {
        // Normal operation - Enable bit set, Error clear bit not set
        controlBufferDMC[0] = (enableDMC << 0) |         // DMC_EnableRq at bit 0
                              (1 << 1) |                 // DMC_ModeRq at bit 1 (1 = speed mode)
                              (1 << 2) |                 // DMC_OscLimEnableRq at bit 2
                              (0 << 4) |                 // DMC_ClrError at bit 4 (not clearing)
                              (1 << 6) |                 // DMC_NegTrqSpd at bit 6
                              (1 << 7);                  // DMC_PosTrqSpd at bit 7
    } else {
        // Error clearing operation - Enable bit cleared, Error clear bit set
        controlBufferDMC[0] = (0 << 0) |                 // DMC_EnableRq at bit 0 (must be 0 to clear error)
                              (1 << 1) |                 // DMC_ModeRq at bit 1 (1 = speed mode)
                              (1 << 2) |                 // DMC_OscLimEnableRq at bit 2
                              (1 << 4) |                 // DMC_ClrError at bit 4 (clearing)
                              (1 << 6) |                 // DMC_NegTrqSpd at bit 6
                              (1 << 7);                  // DMC_PosTrqSpd at bit 7
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
    uint8_t maxNlgCurrent = config.getMaxChargingCurrent();
    
    // Limit charging current by smaller of max charger current and BMS max charge
    float limitedCurrent = std::min(static_cast<int>(maxNlgCurrent), static_cast<int>(bmsData.maxCharge));
    int nlgCurrentScale = static_cast<int>((limitedCurrent + 102.4) * 10);
    
    controlBufferNLG[0] = (false << 7) | (nlgData.unlockRequest << 6) | (false << 5) | ((nlgVoltageScale >> 8) & 0x1F);
    controlBufferNLG[1] = nlgVoltageScale & 0xFF;
    controlBufferNLG[2] = (nlgData.stateDemand << 5) | ((nlgCurrentScale >> 8) & 0x07);
    controlBufferNLG[3] = nlgCurrentScale & 0xFF;
    controlBufferNLG[4] = (nlgData.ledDemand << 4) | ((nlgCurrentScale >> 8) & 0x07);
    controlBufferNLG[5] = nlgCurrentScale & 0xFF;
    
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
