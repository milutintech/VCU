#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "config.h"
#include "vehicle_parameters.h"
#include "state_manager.h"
#include "bms_cells.h"
#include "dcl_pulse_logic.h"
#include <WiFi.h>

/**
 * @brief mcp2515_can whose frame read holds the SPI bus lock.
 *
 * The library reads received frames (mcp2515_read_canMsg) without beginTransaction(), unlike
 * every other access. With the W5500 on the same bus that read could run in the middle of an
 * Ethernet transfer from the other core (both chip selects low) or at the W5500's 20 MHz
 * (MCP2515 max 10 MHz): garbage or no frames.
 */
class LockedMcp2515 : public mcp2515_can {
public:
    using mcp2515_can::mcp2515_can;

    /**
     * SPI clock for our own accesses (readFrame, queueFrame, readClearOverflow). MCP2515 max is
     * 10 MHz; the library's remaining calls (init, mode) stay at its hard-coded 4 MHz. If frames
     * come in garbled after this change, try 8 MHz.
     */
    static constexpr uint32_t SPI_HZ = 10000000;

    /** One received frame (RX0 first); status and frame are read under one bus lock. */
    bool readFrame(uint32_t& id, uint8_t& len, uint8_t* buf);

    /** Reads EFLG and clears the RX overflow bits, one bus lock. Returns EFLG. */
    uint8_t readClearOverflow();

    // Timing diagnostics, maxima since CANManager::takeTiming(). maxSpiWaitUs is written by the
    // RX task, the others by the CAN task; the reset in takeTiming() may race (diagnostics only).
    uint32_t maxSpiWaitUs = 0;   ///< readFrame: waiting for the shared SPI bus (W5500 holding it)
    uint32_t maxTxUs = 0;        ///< queueFrame: whole call (waits while all 3 TX buffers are busy)
    uint32_t txFails = 0;        ///< queueFrame: no free TX buffer within the library timeout

    /**
     * Puts a standard (11 bit) frame into a free TX buffer and returns; does not wait for the
     * wire. Own implementation: the library's sendMsgBuf() used ~10 separate 4 MHz bus
     * transactions per frame (each one possibly waiting behind the W5500). Here: READ STATUS,
     * LOAD TX BUFFER, RTS in one 10 MHz transaction. Polls (bus released in between) only while
     * all three TX buffers are busy, like the library (50 x 10 us).
     */
    byte queueFrame(uint32_t id, uint8_t len, const uint8_t* buf);
};

// Forward declarations
class StateManager;
class CANMonitor;
class ErrorMonitor;

/**
 * @brief Battery Management System data structure
 * Contains battery state information from BMS
 */
struct BMSData {
    // From BMS_PACK (0x010)
    uint8_t soc = 0;                ///< State of charge (%)
    uint16_t voltage = 0;           ///< Battery voltage (V)
    int16_t current = 0;            ///< Battery current (0.1 A, negative while charging)
    uint16_t maxDischarge = 0;      ///< Maximum discharge current (A)
    uint16_t maxCharge = 0;         ///< Maximum charge current (A)
    // From BMS_MINMAX (0x001)
    int8_t tempHigh = 0;            ///< Highest pack temperature (°C)
    int8_t tempLow = 0;             ///< Lowest pack temperature (°C)
    uint16_t cellLowDeciMv = 0;     ///< Lowest cell voltage (0.1 mV)
    uint16_t cellHighDeciMv = 0;    ///< Highest cell voltage (0.1 mV)
    bool chargeRelay = false;       ///< BMS allows charging (charge enable)
    bool dischargeRelay = false;    ///< BMS allows discharging (discharge enable)
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
    uint8_t stateAct;         ///< Charger state, debounced (NLG_STATE_DEBOUNCE_FRAMES)
    uint8_t stateActRaw;      ///< Charger state from the latest 0x728 frame
    uint16_t dcHvCurrentAct;  ///< Charging current (A)
    bool connectorLocked;     ///< Charge connector status
    uint8_t coolingRequest;   ///< Cooling demand
    float tempCoolPlate;      ///< Cooling plate temperature (°C)
    bool unlockRequest;       ///< Connector unlock request
    uint8_t stateDemand;      ///< Requested charger state
    uint8_t ledDemand;        ///< Charge LED state request
};

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
     * @brief The SPI bus, started on first use. The W5500 shares it: both drivers use
     * SPIClass transactions, whose lock keeps them from talking over each other.
     */
    SPIClass& spiBus();

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
    
    /** Process every frame the RX task has queued (also done at the start of update()). */
    void receive() { checkAndProcessMessages(); }

    /**
     * Body of the CAN RX task (never returns). Woken by the MCP2515 INT (ISR notifies the task
     * that runs this), polls every tick as a fallback. Only reads frames into the queue and
     * checks the overflow flags, so a busy CAN task loop no longer delays reading: the
     * MCP2515 holds only 2 frames (~500 us at 500 kbit/s in a burst). Notifies `consumer`
     * (the CAN task) after queuing frames. Start it after begin().
     */
    void rxTaskLoop(TaskHandle_t consumer);
    static constexpr UBaseType_t RX_QUEUE_LEN = 64;   ///< ~160 ms of frames at the current bus load

    /** Frames counted per CAN ID since boot (diagnostics, to choose MCP2515 filters). */
    struct IdStat {
        uint32_t id;
        uint32_t count;
        bool used;      ///< processed by the VCU (known ID or Orion cell frame)
        uint8_t len;            ///< length of the last frame
        uint8_t last[8];        ///< data of the last frame (UDS raw DIDs 0x4000 + ID)
        uint32_t lastMs;        ///< millis() of the last frame
    };
    static constexpr size_t MAX_ID_STATS = 48;
    /** Entries [0, n) are valid. Written by the CAN task, read by the web task (counts may lag). */
    const IdStat* idStats(size_t& n) const { n = idStatCount; return idStatTable; }
    /** Last frame of `id` (CAN task only, no copy race). false = never seen. */
    bool rawFrame(uint32_t id, uint8_t* out, uint8_t& len, uint32_t& ageMs) const;
    /** Queue one standard frame for sending. CAN task only (the TX buffers are not shared). */
    bool sendFrame(uint32_t id, const uint8_t* buf, uint8_t len) {
        return CAN.queueFrame(id, len, buf) == CAN_OK;
    }
    /** Frames to DIAG_REQ / DIAG_FUNC go here instead of the normal decoding (called in the CAN task). */
    using DiagRxFn = void (*)(void* ctx, uint32_t id, const uint8_t* buf, uint8_t len);
    void setDiagRx(DiagRxFn fn, void* ctx) { diagRxCtx = ctx; diagRx = fn; }
    uint32_t getOtherIdFrames() const { return otherIdFrames; }   ///< frames of IDs beyond the table
    uint32_t getRxQueueDrops() const { return rxQueueDrops; }     ///< frames lost because the queue was full

    /// Receive diagnostics (for testing the INT pin and checking for lost frames)
    static volatile uint32_t intCount;   ///< MCP2515 INT falling edges, counted in the ISR
    /// esp_timer µs (low 32 bit) of the first INT not yet served, 0 = none. Set in the ISR.
    static volatile uint32_t intPendingUs;

    /**
     * Where the receive latency comes from. An RX overflow means a frame arrived while both
     * MCP2515 receive buffers were full, i.e. the frames were not read fast enough.
     */
    struct CanTiming {
        uint32_t intLatencyUs = 0;   ///< INT edge -> start of reading in the RX task
        uint32_t spiWaitUs = 0;      ///< waiting for the SPI bus before a read (W5500 holding it)
        uint32_t txUs = 0;           ///< longest queueFrame() call (TX buffers full)
        uint32_t loopUs = 0;         ///< longest CAN task loop body (frames wait in the queue meanwhile)
        uint32_t txFails = 0;        ///< frames not sent (no free TX buffer)
    };
    /** Maxima since the last call, then reset. CAN task only. */
    CanTiming takeTiming();
    /** Last window from takeTiming(), for the web API. */
    const CanTiming& lastTiming() const { return lastTimingWindow; }
    void noteLoopUs(uint32_t us) { if (us > maxLoopUs) maxLoopUs = us; }
    uint32_t getRxCount() const { return rxCount; }
    /// RX task passes in which the MCP2515 reported a receive overflow (frames lost)
    uint32_t getRxOverflowCount() const { return rxOverflowCount; }

    // Data access methods
    const BMSData& getBMSData() const { return bmsData; }
    const DMCData& getDMCData() const { return dmcData; }
    const BSCData& getBSCData() const { return bscData; }
    const NLGData& getNLGData() const { return nlgData; }
    BmsCells& getBmsCells() { return bmsCells; }

    /// millis() of the last frame received from each node; 0 = nothing received yet.
    unsigned long getLastBMSMs() const { return lastBMSMs; }
    unsigned long getLastDMCMs() const { return lastDMCMs; }
    unsigned long getLastBSCMs() const { return lastBSCMs; }
    bool isBSCEnabled() const { return enableBSC; }
    bool isBSCBoost() const { return modeBSC; }
    uint16_t getHVVoltageDemand() const { return hvVoltage; }
    unsigned long getLastNLGMs() const { return lastNLGMs; }
    unsigned long getLastBMSFlagsMs() const { return lastBMSFlagsMs; }

    /// BMS allow flags (0x001) received within the BMS timeout.
    bool bmsFlagsFresh() const;
    /// Discharge limit last sent to the DMC (A), and acceleration pulse time left (ms).
    int getDmcDriveLimitA() const { return dmcDriveLimitA; }
    uint32_t getPulseRemainingMs() const { return dclPulse.remainingMs(); }
    /** 0x010 (pack voltage, current limits) received within BMS_TIMEOUT_MS. */
    bool bmsPackFresh() const;
    /// Charging (charger and regen) allowed: flag set and fresh, or flags ignored.
    bool chargeAllowed() const;
    /// Discharging (drive current) allowed: flag set and fresh, or flags ignored.
    bool dischargeAllowed() const;
    /// Ignore the BMS allow flags (breakdown, trailer loading, debugging). Not saved: every
    /// boot starts with the flags enforced. Current limits (DCL/CCL) still apply.
    void setIgnoreAllowFlags(bool ignore) { ignoreAllowFlags = ignore; }
    bool getIgnoreAllowFlags() const { return ignoreAllowFlags; }
    
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
     * @brief Process BMS min/max message (0x001): cells, temps, allow flags
     */
    void processBMSMinMaxMessage(uint8_t* buf, uint8_t len);

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
     * @brief Process the frames queued by the RX task
     */
    void checkAndProcessMessages();
    void countId(uint32_t id, bool used, const uint8_t* buf, uint8_t len);
    DiagRxFn diagRx = nullptr;
    void* diagRxCtx = nullptr;

    struct RxFrame {
        uint32_t id;
        uint8_t len;
        uint8_t buf[8];
    };
    QueueHandle_t rxQueue = nullptr;          ///< RX task -> CAN task
    volatile uint32_t rxQueueDrops = 0;
    IdStat idStatTable[MAX_ID_STATS] = {};
    volatile size_t idStatCount = 0;
    volatile uint32_t otherIdFrames = 0;
    
    bool needsClearError = false;  // Flag to indicate error clearing needed
    unsigned long errorClearStartTime = 0;  // Timestamp for error clearing
    bool inErrorClearSequence = false;  // Track if we're in the middle of error clearing
    int calculateTaperedDMCCurrent();
    DclPulseLogic dclPulse;              ///< acceleration pulses above the BMS DCL
    volatile int dmcDriveLimitA = 0;

    // Hardware interfaces
    LockedMcp2515 CAN;       ///< CAN controller instance
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
    NLGData nlgData{};       ///< Charging system data (zeroed: stateAct 0 = SLEEP until debounced)
    BmsCells bmsCells;       ///< Orion BMS per-cell data, decoded from the CAN broadcast
    
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
    
    // Timing management
    unsigned long lastFastCycle;   ///< Last fast update cycle timestamp
    unsigned long lastSlowCycle;   ///< Last slow update cycle timestamp
    volatile unsigned long lastBMSMs = 0; ///< Last BMS frame (millis), written in CAN task
    volatile unsigned long lastDMCMs = 0; ///< Last DMC frame (any of 0x258/0x259/0x458)
    volatile unsigned long lastBSCMs = 0; ///< Last BSC frame
    volatile unsigned long lastNLGMs = 0; ///< Last NLG frame
    uint8_t nlgStateCandidate = 0xFF;     ///< 0x728 state being debounced (CAN task only)
    uint8_t nlgStateCount = 0;            ///< consecutive frames with nlgStateCandidate
    volatile uint32_t rxCount = 0;           ///< frames read from the MCP2515
    volatile uint32_t rxOverflowCount = 0;   ///< see getRxOverflowCount()
    volatile uint32_t maxIntLatencyUs = 0;   ///< written by the RX task
    uint32_t maxLoopUs = 0;
    CanTiming lastTimingWindow;
    volatile unsigned long lastBMSFlagsMs = 0; ///< Last BMS_MINMAX frame (allow flags)
    volatile bool ignoreAllowFlags = false;    ///< set from the web task, read in the CAN task

    // Charger current ramp (CAN task only), see sendNLG()
    float nlgCurrentCmdA = 0.0f;       ///< DC current limit last sent to the NLG (A)
    unsigned long lastNlgRampMs = 0;   ///< millis() of the last sendNLG()

    // CAN Monitoring - FIXED: Use pointers instead of direct objects
    CANMonitor* canMonitor = nullptr;      ///< CAN bus monitor instance
    ErrorMonitor* systemMonitor = nullptr; ///< System error monitor instance
};