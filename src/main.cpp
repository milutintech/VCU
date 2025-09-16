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
#include "configuration.h"            // Extended configuration
#include "can_monitoring.h"           // CAN monitoring extensions

// Global objects - Enhanced
ADS1115 ads(0x48);
CANManager* canManager = nullptr;
StateManager* stateManager = nullptr;
VehicleControl* vehicleControl = nullptr;
EnhancedSerialConsole* serialConsole = nullptr;  // Fixed class name
ErrorMonitor* errorMonitor = nullptr;             // New error monitor
CANMonitor* canMonitor = nullptr;                 // New CAN monitor

// Task handles
TaskHandle_t canTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;

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
 * @brief Enhanced CAN Task with comprehensive monitoring
 */
void canTask(void* parameter) {
    Serial.print("Enhanced CAN Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    // Initialize communication hardware
    SPI.begin(Pins::SCK, Pins::MISO, Pins::MOSI, Pins::SPI_CS_PIN);
    Wire.begin(Pins::SDA, Pins::SCL);
    Wire.setClock(400000);
    ads.begin();
    ads.setGain(2);
    
    // Initialize CAN with enhanced monitoring
    canManager->begin();
    // Note: CAN logging methods need to be implemented in CANManager
    // canManager->enableCANLogging(true);  // Enable CAN message logging
    
    esp_task_wdt_init(5, true);
    
    errorMonitor->logInfo("CAN task started successfully", "SYSTEM");
    
    unsigned long lastMonitorUpdate = 0;
    const unsigned long MONITOR_UPDATE_INTERVAL = 100; // Update monitoring every 100ms
    
    for(;;) {
        esp_task_wdt_reset();
        
        // Update CAN communication
        canManager->update();
        
        // Update motor speed from DMC data
        const DMCData& dmcData = canManager->getDMCData();
        vehicleControl->setMotorSpeed(dmcData.speedActual);
        
        // Calculate and apply torque demand
        if (stateManager->getCurrentState() == VehicleState::RUN) {
            vehicleControl->updateGearState();
            float torquePercentage = vehicleControl->calculateTorquePercentage();
            canManager->setTorquePercentage(torquePercentage);
            canManager->setEnableDMC(vehicleControl->isDMCEnabled());
            
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
        
        // Check for system errors using the global config instance
        const BMSData& bmsData = canManager->getBMSData();
        if (bmsData.voltage < VehicleParams::Battery::MIN_VOLTAGE) { // Use constants instead of config
            errorMonitor->logError(ErrorSeverity::WARNING, ErrorCode::BATTERY_UNDERVOLTAGE, 
                                 "Battery voltage low", bmsData.voltage, "BMS");
        }
        
        if (dmcData.tempInverter > VehicleParams::Temperature::INV_HIGH) { // Use constants instead of config
            errorMonitor->logError(ErrorSeverity::ERROR, ErrorCode::INVERTER_OVERTEMP,
                                 "Inverter overtemperature", dmcData.tempInverter, "DMC");
        }
        
        vTaskDelay(1);
    }
}

/**
 * @brief Enhanced Control Task with error monitoring
 */
void controlTask(void* parameter) {
    Serial.print("Enhanced Control Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    esp_task_wdt_init(5, true);
    errorMonitor->logInfo("Control task started successfully", "SYSTEM");
    
    stateManager->handleWakeup();
    
    unsigned long lastPerformanceUpdate = 0;
    const unsigned long PERFORMANCE_UPDATE_INTERVAL = 1000; // Update performance every second
    
    for(;;) {
        esp_task_wdt_reset();
        
        // Check for pending unlock interrupt
        checkAndHandleInterrupt();
        
        // Update system state
        stateManager->update();
        
        // Process serial console commands
        serialConsole->update();
        
        // Update performance metrics periodically
        if (millis() - lastPerformanceUpdate >= PERFORMANCE_UPDATE_INTERVAL) {
            const MonitoringData& monData = errorMonitor->getMonitoringData();
            errorMonitor->updatePerformanceMetrics(monData);
            lastPerformanceUpdate = millis();
        }
        
        // Check system health
        if (!errorMonitor->isSystemHealthy()) {
            errorMonitor->logWarning("System health check failed", "MONITOR");
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms cycle time
    }
}

/**
 * @brief Enhanced System Setup - ADD this code after creating the canManager
 */
void setup() {
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
    vehicleControl->setCanManager(canManager);
    
    // Create enhanced serial console
    serialConsole = new EnhancedSerialConsole(*canManager, *stateManager, *vehicleControl, *errorMonitor);
    if (!serialConsole) {
        errorMonitor->logCritical("Failed to create EnhancedSerialConsole", "SYSTEM");
        while(1);
    }

    
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
    
    SystemSetup::initializeGPIO();
    SystemSetup::initializeSleep();
    
    // Print system information
    WiFi.mode(WIFI_STA);
    Serial.print("Device MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    errorMonitor->logInfo("Hardware initialization complete", "SYSTEM");
    
    // Create enhanced tasks
    BaseType_t canTaskCreated = xTaskCreatePinnedToCore(
        canTask, "Enhanced_CAN_Task", 12000, NULL, 1, &canTaskHandle, 0
    );
    
    if (canTaskCreated != pdPASS) {
        errorMonitor->logCritical("Failed to create CAN task", "SYSTEM");
        while(1);
    }
    
    BaseType_t controlTaskCreated = xTaskCreatePinnedToCore(
        controlTask, "Enhanced_Control_Task", 24000, NULL, 1, &controlTaskHandle, 1
    );
    
    if (controlTaskCreated != pdPASS) {
        errorMonitor->logCritical("Failed to create Control task", "SYSTEM");
        while(1);
    }
    
    
    errorMonitor->logInfo("All tasks created successfully", "SYSTEM");
    Serial.println("Enhanced VCU initialization complete!");
    Serial.println("==========================================");
    Serial.println("Available commands:");
    Serial.println("  help - Show legacy commands");
    Serial.println("  json_help - Show JSON API commands");
    Serial.println("  {\"cmd\":\"help\"} - JSON command help");
    Serial.println("==========================================");
}


void loop() {
    vTaskDelete(NULL);
}