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
                    
                case 0x26A:  // BSC message
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
#define BMS_TIMEOUT_MS 1000  // Timeout duration in milliseconds
unsigned long lastBMSUpdate = 0;  // Variable to track last update timestamp

void CANManager::processBMSMessage(uint8_t* buf) {
    if (!buf) return;
    
    bmsData.soc = buf[0] / 2;
    bmsData.voltage = (buf[2] | (buf[1] << 8)) / 10;
    bmsData.current = 400; //(buf[3] | (buf[4] << 8)) / 100;
    bmsData.maxDischarge = (buf[5] | (buf[6] << 8)) / 100;
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
            dmcData.speedActual = static_cast<float>((buf[6] << 8) | buf[7]);
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
    int16_t scaledTorque = static_cast<int16_t>(torqueDemand * 10);
    
    // DMC control message (0x210)
    controlBufferDMC[0] = (enableDMC << 7) | (false << 6) | (1 << 5) | (1 << 1) | 1;
    controlBufferDMC[2] = VehicleParams::Motor::MAX_RPM >> 8;
    controlBufferDMC[3] = VehicleParams::Motor::MAX_RPM & 0xFF;
    controlBufferDMC[4] = scaledTorque >> 8;
    controlBufferDMC[5] = scaledTorque & 0xFF;
    
    // DMC limits message (0x211)
    int dcVoltLimMotor = VehicleParams::Battery::MIN_VOLTAGE * 10;
    int dcVoltLimGen = VehicleParams::Battery::MAX_VOLTAGE * 10;
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
    if (millis() - lastBMSUpdate > BMS_TIMEOUT_MS) {
        bmsData.maxCharge = 0;  // Set max charge current to 0 if timeout occurs
    }

    float limitedCurrent = std::min(VehicleParams::Battery::MAX_NLG_CURRENT, static_cast<int>(bmsData.maxCharge));
    int nlgCurrentScale = static_cast<int>((limitedCurrent + 102.4) * 10);

    controlBufferNLG[0] = (false << 7) | (nlgData.unlockRequest << 6) | 
                         (false << 5) | ((nlgVoltageScale >> 8) & 0x1F);
    controlBufferNLG[1] = nlgVoltageScale & 0xFF;
    controlBufferNLG[2] = (nlgData.stateDemand << 5) | ((nlgCurrentScale >> 8) & 0x07);
    controlBufferNLG[3] = nlgCurrentScale & 0xFF;
    controlBufferNLG[4] = (nlgData.ledDemand << 4) | ((nlgCurrentScale >> 8) & 0x07);
    controlBufferNLG[5] = nlgCurrentScale & 0xFF;
    
    CAN.sendMsgBuf(CANIds::NLG_DEM_LIM, 0, 8, controlBufferNLG);
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
