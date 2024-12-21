/**
 * @file main.cpp
 * @brief Main Vehicle Control Unit (VCU) Program
 * 
 * Core system orchestration:
 * - Task management for CAN and control systems
 * - Hardware initialization
 * - Real-time scheduling
 * - System monitoring
 * 
 * Uses dual-core ESP32:
 *  IS Stupid
 * Core 0: CAN communication and fast control loops
 * Core 1: State management and system control
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include <esp_adc_cal.h>
#include <esp32-hal-adc.h>
#include "ADS1X15.h"
#include "AD5593R.h"

#include "state_manager.h"
#include "can_manager.h"
#include "vehicle_control.h"
#include "setup.h"
#include "config.h"
#include "SerialConsole.h"

// Global system instances
ADS1115 ads(0x48);                                    ///< ADC for pedal position
StateManager stateManager;                            ///< Vehicle state management
CANManager canManager(Pins::SPI_CS_PIN, stateManager);///< CAN communication
VehicleControl vehicleControl(ads);                   ///< Vehicle control logic
SerialConsole serialConsole(canManager, stateManager, vehicleControl);  ///< Debug interface

// Task handles for ESP32 dual-core operation
TaskHandle_t canTaskHandle = nullptr;     ///< CAN task handle (Core 0)
TaskHandle_t controlTaskHandle = nullptr; ///< Control task handle (Core 1)

/**
 * @brief CAN and Fast Control Task (Core 0)
 * 
 * Handles:
 * - CAN message processing
 * - Motor control updates
 * - Fast sensor readings
 * - Real-time control loops
 * 
 * @param parameter Task parameters (unused)
 */
void canTask(void* parameter) {
    Serial.print("CAN Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    // Initialize communication hardware
    SPI.begin(Pins::SCK, Pins::MISO, Pins::MOSI, Pins::SPI_CS_PIN);
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    ads.begin();
    ads.setGain(2);         // Set ADC gain for pedal reading
    canManager.begin();
    
    esp_task_wdt_init(5, true);  // 5 second watchdog timeout
    
    for(;;) {
        esp_task_wdt_init(5, true);
        canManager.update();
        
        // Update motor speed from DMC data
        const DMCData& dmcData = canManager.getDMCData();
        vehicleControl.setMotorSpeed(dmcData.speedActual);
        
        // Calculate and apply torque demand in RUN state
        if (stateManager.getCurrentState() == VehicleState::RUN) {
            int16_t torque = vehicleControl.calculateTorque();
            canManager.setTorqueDemand(torque);
            canManager.setEnableDMC(vehicleControl.isDMCEnabled());
        }
        
       // vTaskDelay(pdMS_TO_TICKS(Constants::FAST_CYCLE_MS)); // 10ms cycle
    }
}

/**
 * @brief State Management and System Control Task (Core 1)
 * 
 * Handles:
 * - Vehicle state management
 * - Temperature monitoring
 * - Charging control
 * - User interface
 * - System diagnostics
 * 
 * @param parameter Task parameters (unused)
 */
void controlTask(void* parameter) {
    Serial.print("Control Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    esp_task_wdt_init(5, true);  // 5 second watchdog timeout
    
    stateManager.handleWakeup();  // Initial state determination
    
    for(;;) {
        esp_task_wdt_init(5, true);
        
        // Update system state and interface
        stateManager.update();
        serialConsole.update();
        
        // Update system parameters from CAN data
        const BMSData& bmsData = canManager.getBMSData();
        const DMCData& dmcData = canManager.getDMCData();
        const NLGData& nlgData = canManager.getNLGData();
        
        stateManager.setBatteryVoltage(bmsData.voltage);
        stateManager.setInverterTemp(dmcData.tempInverter);
        stateManager.setMotorTemp(dmcData.tempMotor);
        stateManager.setCoolingRequest(nlgData.coolingRequest);
        
        // Update BSC state based on battery status
        canManager.setEnableBSC(stateManager.isBatteryArmed());
        
        vTaskDelay(pdMS_TO_TICKS(Constants::SLOW_CYCLE_MS)); // 50ms cycle
    }
}

/**
 * @brief System initialization
 * 
 * Performs initial setup:
 * - Serial communication
 * - GPIO configuration
 * - Sleep mode setup
 * - Task creation and scheduling
 */
void setup() {
    Serial.begin(115200);
    SystemSetup::initializeGPIO();
    SystemSetup::initializeSleep();
    
    // Create tasks on specific cores
    xTaskCreatePinnedToCore(
        canTask,         // Task function
        "CAN_Task",      // Name
        10000,           // Stack size (bytes)
        NULL,            // Parameters
        1,              // Priority
        &canTaskHandle,  // Task handle
        0               // Core 0
    );
    
    xTaskCreatePinnedToCore(
        controlTask,         // Task function
        "Control_Task",      // Name
        20000,              // Stack size (bytes)
        NULL,               // Parameters
        1,                  // Priority
        &controlTaskHandle, // Task handle
        1                  // Core 1
    );
}

/**
 * @brief Main program loop
 * 
 * Not used as functionality is handled by tasks.
 * Deletes the setup task and enters idle state.
 */
void loop() {
    vTaskDelete(NULL);  // Delete setup task
}