/**
 * @file main.cpp - FIXED VERSION
 * @brief Enhanced VCU with comprehensive monitoring and JSON configuration
 */

#include <Arduino.h>
#include <ArduinoJson.h>  // Add this for JSON support
#include <esp_task_wdt.h>
#include <Wire.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include <esp_adc_cal.h>
#include <esp32-hal-adc.h>
#include "ADS1X15.h"

// Enhanced includes
#include "state_manager.h"
#include "can_manager.h"
#include "vehicle_control.h"
#include "setup.h"
#include "config.h"
#include "enhanced_serial_console.h"  // Fixed enhanced console
#include "error_monitor.h"            // New error monitoring
#include "fault_handler.h"            // Fault detection and reactions
#include "configuration.h"            // Extended configuration
#include "can_monitoring.h"           // CAN monitoring extensions
#include "wifi_manager.h"             // WiFi management
#include "pi_power.h"                // Pi power and shutdown sequence
#include "web_server.h"               // Web server

// Global objects - Enhanced
ADS1115 ads(0x48);
CANManager* canManager = nullptr;
StateManager* stateManager = nullptr;
VehicleControl* vehicleControl = nullptr;
EnhancedSerialConsole* serialConsole = nullptr;  // Fixed class name
ErrorMonitor* errorMonitor = nullptr;             // New error monitor
FaultHandler* faultHandler = nullptr;             // Fault detection and reactions
CANMonitor* canMonitor = nullptr;                 // New CAN monitor
WiFiManager* wifiManager = nullptr;               // WiFi manager
PiPower* piPower = nullptr;                       // Power and shutdown of the infotainment Pi
VCUWebServer* webServer = nullptr;                // Web server

// Task handles
TaskHandle_t canTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t adcTaskHandle = nullptr;
TaskHandle_t webTaskHandle = nullptr;

// Interrupt handling (unchanged)
volatile bool unlockInterruptTriggered = false;
unsigned long lastInterruptTime = 0;
const unsigned long debounceTime = 500;

void IRAM_ATTR connectorUnlockISR() {
    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime > debounceTime) {
        unlockInterruptTriggered = true;
        lastInterruptTime = currentTime;
    }
}

void checkAndHandleInterrupt() {
    if (unlockInterruptTriggered) {
        if (stateManager) {
            errorMonitor->logInfo("Connector unlock interrupt triggered", "USER");
            stateManager->handleConnectorUnlockInterrupt();
        }
        unlockInterruptTriggered = false;
    }
}

/**
 * @brief MCP2515 INT (active low, IO37): wake the CAN task to read the frame at once.
 * The chip holds only two frames and the bus has bursts of 3 to 5 within 1 ms.
 */
static void IRAM_ATTR canIntISR() {
    BaseType_t woken = pdFALSE;
    CANManager::intCount = CANManager::intCount + 1;
    if (canTaskHandle) vTaskNotifyGiveFromISR(canTaskHandle, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * @brief Task watchdog: 5 s timeout, panic on expiry (ESP-IDF 5 API)
 *
 * The Arduino core already starts the watchdog at boot, so it is reconfigured;
 * init is only the fallback when it is not running yet. The CAN and control
 * tasks are not subscribed (no esp_task_wdt_add), so they do not call
 * esp_task_wdt_reset(): on IDF 5 that logs "task not found" on every call.
 */
static void configureTaskWdt() {
    esp_task_wdt_config_t cfg = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
    cfg.idle_core_mask |= 1 << 0;
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
    cfg.idle_core_mask |= 1 << 1;
#endif
    if (esp_task_wdt_reconfigure(&cfg) == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_init(&cfg);
    }
}

/**
 * @brief Enhanced CAN Task with comprehensive monitoring
 */
void canTask(void* parameter) {
    Serial.print("Enhanced CAN Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    // Initialize communication hardware
    SPI.begin(Pins::SCK, Pins::MISO, Pins::MOSI, Pins::SPI_CS_PIN);
    
    // Initialize CAN with enhanced monitoring
    canManager->begin();
    // Note: CAN logging methods need to be implemented in CANManager
    // canManager->enableCANLogging(true);  // Enable CAN message logging

    // Attached here so the ISR runs on this task's core
    pinMode(Pins::CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Pins::CAN_INT_PIN), canIntISR, FALLING);
    
    configureTaskWdt();
    
    errorMonitor->logInfo("CAN task started successfully", "SYSTEM");
    const unsigned long taskStartMs = millis();
    
    unsigned long lastMonitorUpdate = 0;
    const unsigned long MONITOR_UPDATE_INTERVAL = 100; // Update monitoring every 100ms
    
    for(;;) {
        // Update CAN communication
        canManager->update();
        
        // Evaluate faults and compute the reaction (torque derate / DMC disable)
        faultHandler->update(*canManager, *stateManager);
        const FaultReaction& reaction = faultHandler->reaction();

        // Update motor speed from DMC data
        const DMCData& dmcData = canManager->getDMCData();
        vehicleControl->setMotorSpeed(dmcData.speedActual);
        
        // Calculate and apply torque demand
        if (stateManager->getCurrentState() == VehicleState::RUN) {
            vehicleControl->updateGearState();
            float torquePercentage = vehicleControl->calculateTorquePercentage();
            canManager->setTorquePercentage(torquePercentage * reaction.torqueScale);
            canManager->setEnableDMC(stateManager->isPreCharged() && vehicleControl->isDMCEnabled() &&
                                     !reaction.disableDMC);
            
            // Log significant torque changes
            static float lastLoggedTorque = 0.0f;
            if (abs(torquePercentage - lastLoggedTorque) > 10.0f) {
                errorMonitor->logInfo("Torque demand: " + String(torquePercentage) + "%", "CONTROL");
                lastLoggedTorque = torquePercentage;
            }
        } else {
            canManager->setTorquePercentage(0.0f);
            canManager->setEnableDMC(false);
        }
        
        // Update monitoring data periodically
        if (millis() - lastMonitorUpdate >= MONITOR_UPDATE_INTERVAL) {
            errorMonitor->updateMonitoringData(*canManager, *stateManager);
            lastMonitorUpdate = millis();
        }
        
        // Receive diagnostics: INT self-test 3 s after start, then lost frames every 10 s
        static unsigned long lastRxReport = 0;
        static bool intChecked = false;
        static uint32_t lastOverflows = 0;
        if (!intChecked && millis() - taskStartMs >= 3000) {
            intChecked = true;
            const uint32_t ints = CANManager::intCount, frames = canManager->getRxCount();
            if (frames == 0) {
                errorMonitor->logWarning("CAN INT (IO37) not tested: no frames received yet", "CAN");
            } else if (ints == 0) {
                errorMonitor->logWarning("CAN INT (IO37) never fired, " + String(frames) +
                                         " frames read by polling only: check the INT line", "CAN");
            } else {
                errorMonitor->logInfo("CAN INT (IO37) ok: " + String(ints) + " interrupts, " +
                                      String(frames) + " frames", "CAN");
            }
        }
        if (millis() - lastRxReport >= 10000) {
            lastRxReport = millis();
            const uint32_t overflows = canManager->getRxOverflowCount();
            if (overflows != lastOverflows) {
                errorMonitor->logWarning("CAN RX overflow (frames lost) in " + String(overflows - lastOverflows) +
                                         " cycles in the last 10 s", "CAN");
                lastOverflows = overflows;
            }
        }

        // Wait one tick as before, but read frames as soon as the MCP2515 signals one.
        const TickType_t waitStart = xTaskGetTickCount();
        while (xTaskGetTickCount() - waitStart < 1) {
            if (ulTaskNotifyTake(pdTRUE, 1) == 0) break;   // tick over, no frame
            canManager->receive();
        }
    }
}

/**
 * @brief ADC task: owns the I2C bus and the ADS1115 (pedal, gear switches).
 *
 * The reads used to run inside the CAN task at 128 SPS (~8 ms per conversion, 6 per
 * loop), which blocked it ~47 ms per cycle in RUN: MCP2515 overflows, late 0x210.
 * Now: 860 SPS, sleeping during conversions, one full set every ~7 ms. The CAN task
 * only uses the cached values (stale after VehicleControl::INPUT_STALE_MS).
 */
void adcTask(void* parameter) {
    Wire.begin(Pins::SDA, Pins::SCL);
    Wire.setClock(400000);
    ads.begin();
    ads.setGain(2);
    ads.setDataRate(7);   // 860 SPS
    errorMonitor->logInfo("ADC task started", "SYSTEM");

    for (;;) {
        vehicleControl->sampleInputs();
        vTaskDelay(1);
    }
}

/**
 * @brief Enhanced Control Task with error monitoring
 */
void controlTask(void* parameter) {
    Serial.print("Enhanced Control Task running on core: ");
    Serial.println(xPortGetCoreID());

    configureTaskWdt();
    errorMonitor->logInfo("Control task started successfully", "SYSTEM");

    stateManager->handleWakeup();

    unsigned long lastPerformanceUpdate = 0;
    const unsigned long PERFORMANCE_UPDATE_INTERVAL = 1000; // Update performance every second

    for(;;) {
        // Check for pending unlock interrupt
        checkAndHandleInterrupt();

        // Update system state
        stateManager->update();

        // Update performance metrics periodically
        if (millis() - lastPerformanceUpdate >= PERFORMANCE_UPDATE_INTERVAL) {
            const MonitoringData& monData = errorMonitor->getMonitoringData();
            errorMonitor->updatePerformanceMetrics(monData);
            lastPerformanceUpdate = millis();
        }

        // Check system health (logged on change only, the faults themselves are logged once)
        static bool wasHealthy = true;
        const bool healthy = errorMonitor->isSystemHealthy();
        if (healthy != wasHealthy) {
            if (healthy) errorMonitor->logInfo("System health check passed", "MONITOR");
            else errorMonitor->logWarning("System health check failed", "MONITOR");
            wasHealthy = healthy;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms cycle time
    }
}

/**
 * @brief Web Server Task - Handles WiFi and HTTP/WebSocket
 */
void webServerTask(void* parameter) {
    Serial.print("Web Server Task running on core: ");
    Serial.println(xPortGetCoreID());

    errorMonitor->logInfo("Web server task started successfully", "SYSTEM");

    for(;;) {
        // Update WiFi connection status
        if (wifiManager) {
            wifiManager->update();
        }

        // The Pi's power sequence runs here, away from the control task
        if (piPower) {
            piPower->update();
        }

        // Update web server (WebSocket broadcasts)
        if (webServer) {
            webServer->update();
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms cycle time
    }
}

/**
 * @brief Enhanced System Setup - ADD this code after creating the canManager
 */
void setup() {
    // Outputs first: every object below may drive them (StateManager sets the contactor and
    // KL15 lines LOW in its constructor), and a write to an unconfigured pin is ignored.
    SystemSetup::initializeGPIO();

    // A TX buffer makes Serial.print return at once instead of waiting for the UART
    // (115200 baud: ~10 ms per log line), so logging does not stall the CAN task.
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n\n==========================================");
    Serial.println("  Enhanced Vehicle Control Unit Startup  ");
    Serial.println("==========================================");
    Serial.println("ESP32 VCU with JSON Config & Monitoring");
    
    // Initialize error monitor first
    errorMonitor = new ErrorMonitor(1000, 500);
    if (!errorMonitor) {
        Serial.println("CRITICAL: Failed to create ErrorMonitor");
        while(1);
    }
    faultHandler = new FaultHandler(*errorMonitor);
    errorMonitor->logInfo("System startup initiated", "SYSTEM");
    
    // Initialize CAN monitor
    canMonitor = new CANMonitor();
    if (!canMonitor) {
        errorMonitor->logCritical("Failed to create CANMonitor", "SYSTEM");
        while(1);
    }
    canMonitor->initializeMessageDefinitions();
    
    // Initialize configuration
    config.begin();
    errorMonitor->logInfo("Configuration system initialized", "CONFIG");
    
    // Create core system components
    canManager = new CANManager(Pins::SPI_CS_PIN);
    if (!canManager) {
        errorMonitor->logCritical("Failed to create CANManager", "SYSTEM");
        while(1);
    }

    // FIXED: Connect monitors to CAN manager after creation
    canManager->setCANMonitor(canMonitor);
    canManager->setSystemMonitor(errorMonitor);

    stateManager = new StateManager(*canManager, nullptr);
    if (!stateManager) {
        errorMonitor->logCritical("Failed to create StateManager", "SYSTEM");
        while(1);
    }
    canManager->setStateManager(stateManager);
    
    vehicleControl = new VehicleControl(ads);
    if (!vehicleControl) {
        errorMonitor->logCritical("Failed to create VehicleControl", "SYSTEM");
        while(1);
    }
    
    if (stateManager && vehicleControl) {
        stateManager->setVehicleControl(vehicleControl);
    }

    // Pi power: owns IO21, leaves the Pi off until the ignition has been on for a moment
    piPower = new PiPower();
    piPower->begin();
    stateManager->setPiPower(piPower);
    vehicleControl->setCanManager(canManager);
    vehicleControl->setErrorMonitor(errorMonitor);
    
    // Initialize WiFi Manager
    wifiManager = new WiFiManager();
    if (!wifiManager) {
        errorMonitor->logCritical("Failed to create WiFiManager", "SYSTEM");
        while(1);
    }

    // ====== WIFI CONFIGURATION OPTIONS ======
    // Uncomment ONE of these options as needed:

    // Option 1: Clear all WiFi settings (force AP mode)
    //wifiManager->clearStationCredentials();

    // Option 2: Set WiFi credentials to connect to your network
    //wifiManager->setStationCredentials("YOUR_SSID", "YOUR_PASSWORD");
    //wifiManager->saveConfig();

    // Option 3: Do nothing - use saved settings or default to AP mode
    // =========================================

    wifiManager->begin();
    errorMonitor->logInfo("WiFi Manager initialized", "NETWORK");

    // Initialize Web Server
    webServer = new VCUWebServer(&config, stateManager, vehicleControl, canManager, errorMonitor);
    if (!webServer) {
        errorMonitor->logCritical("Failed to create VCUWebServer", "SYSTEM");
        while(1);
    }
    webServer->setWifiManager(wifiManager);
    webServer->setPiPower(piPower);
    webServer->begin();
    errorMonitor->logInfo("Web Server initialized", "NETWORK");

    Serial.print("Web Interface: http://");
    Serial.println(wifiManager->getIPAddress());

    // Apply current configuration to vehicle control
    vehicleControl->setDrivingMode(config.getDriveMode());    
    // NEW: Apply transition configuration to vehicle control
    vehicleControl->setRegenEngageTime(config.getRegenEngageTime());
    vehicleControl->setRegenReleaseTime(config.getRegenReleaseTime());
    vehicleControl->setPowerEngageTime(config.getPowerEngageTime());
    vehicleControl->setPowerReleaseTime(config.getPowerReleaseTime());
    vehicleControl->setCrossoverTime(config.getCrossoverTime());
    
    errorMonitor->logInfo("Applied configuration to vehicle control", "CONFIG");
    errorMonitor->logInfo("Transition timing configured", "CONFIG"); 
    errorMonitor->logInfo("Applied configuration to vehicle control", "CONFIG");
    
    // Setup GPIO and interrupts
    pinMode(Pins::UNLCKCON, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(Pins::UNLCKCON), connectorUnlockISR, RISING);
    
    SystemSetup::initializeSleep();
    
    // Print system information (don't force WiFi mode - let WiFiManager handle it)
    Serial.print("Device MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    errorMonitor->logInfo("Hardware initialization complete", "SYSTEM");
    
    // Create enhanced tasks
    BaseType_t canTaskCreated = xTaskCreatePinnedToCore(
        // Core 1, above the control/web/loop tasks (1): core 0 runs WiFi, lwIP and AsyncTCP,
        // which starved this task and overflowed the MCP2515 (lost BMS frames, DMC
        // CanCtrlMsgLost). The loop still waits 1 tick per cycle, so lower tasks keep running.
        canTask, "Enhanced_CAN_Task", 12000, NULL, 5, &canTaskHandle, 1
    );
    
    if (canTaskCreated != pdPASS) {
        errorMonitor->logCritical("Failed to create CAN task", "SYSTEM");
        while(1);
    }
    
    // Below the CAN task (5), above control/web (1); it sleeps during every conversion.
    BaseType_t adcTaskCreated = xTaskCreatePinnedToCore(
        adcTask, "ADC_Task", 4096, NULL, 2, &adcTaskHandle, 1
    );

    if (adcTaskCreated != pdPASS) {
        errorMonitor->logCritical("Failed to create ADC task", "SYSTEM");
        while(1);
    }

    BaseType_t controlTaskCreated = xTaskCreatePinnedToCore(
        controlTask, "Enhanced_Control_Task", 24000, NULL, 1, &controlTaskHandle, 1
    );

    if (controlTaskCreated != pdPASS) {
        errorMonitor->logCritical("Failed to create Control task", "SYSTEM");
        while(1);
    }

    BaseType_t webTaskCreated = xTaskCreatePinnedToCore(
        webServerTask, "Web_Server_Task", 16000, NULL, 1, &webTaskHandle, 1
    );

    if (webTaskCreated != pdPASS) {
        errorMonitor->logCritical("Failed to create Web Server task", "SYSTEM");
        while(1);
    }

    errorMonitor->logInfo("All tasks created successfully", "SYSTEM");
    Serial.println("Enhanced VCU initialization complete!");
    Serial.println("==========================================");
    Serial.print("Web Interface: http://");
    Serial.println(wifiManager->getIPAddress());
    Serial.print("WiFi SSID: ");
    Serial.println(wifiManager->getSSID());
    Serial.println("==========================================");
}


void loop() {
    vTaskDelete(NULL);
}