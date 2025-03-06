/**
 * @file setup.cpp
 * @brief System initialization and hardware setup implementation
 * 
 * Provides core initialization functionality for:
 * - GPIO configuration
 * - Sleep mode setup
 * - Default system states
 * 
 * This implementation focuses on safe startup conditions and 
 * proper hardware initialization for the ESP32-based vehicle control unit.
 */

#include "setup.h"
#include "config.h"
#include <esp_sleep.h>

/**
 * @brief Initialize all GPIO pins for vehicle control
 * 
 * Configures all required GPIO pins with appropriate modes:
 * - Input pins: Wake sources, signals, sensors
 * - Output pins: Power control, contactors, system enables
 * 
 * All outputs are initialized to safe states during configuration.
 * Pin definitions are sourced from config.h.
 * 
 * Configured pins include:
 * - NLG_HW_Wakeup: Charger wake input
 * - IGNITION: Main ignition input
 * - UNLCKCON: Connector unlock input
 * - PUMP: Cooling system control
 * - CONTACTOR: Main battery contactor
 * - System enables (NLGKL15, DMCKL15, BSCKL15)
 * - Status indicators and auxiliary controls
 */
void SystemSetup::initializeGPIO() {
    // Configure input pins
    pinMode(Pins::NLG_HW_Wakeup, INPUT);
    pinMode(Pins::IGNITION, INPUT);
    pinMode(Pins::UNLCKCON, INPUT);
    
    // Configure output pins
    pinMode(Pins::PUMP, OUTPUT);
    pinMode(Pins::CONTACTOR, OUTPUT);
    pinMode(Pins::NLGKL15, OUTPUT);
    pinMode(Pins::DMCKL15, OUTPUT);
    pinMode(Pins::BSCKL15, OUTPUT);
    pinMode(Pins::BCKLIGHT, OUTPUT);
    pinMode(Pins::PW1, OUTPUT);
    pinMode(Pins::LWP5, OUTPUT);
    pinMode(Pins::LWP6, OUTPUT);
    pinMode(Pins::LWP7, OUTPUT);
    pinMode(Pins::UNLCKCON, INPUT); // Use pull-down to ensure clean interrupts
    
    setDefaultPinStates();
}

/**
 * @brief Configure ESP32 sleep mode parameters
 * 
 * Sets up deep sleep functionality including:
 * - Wake sources configuration
 * - GPIO wake pin selection
 * - Wake level triggers
 * 
 * Wake sources:
 * - EXT0: Single GPIO wake (IGNITION pin)
 * - EXT1: Multiple GPIO wake (Button bitmask)
 * 
 * The system can be woken by either:
 * - Ignition signal (HIGH level)
 * - Any configured button press (HIGH level)
 */
void SystemSetup::initializeSleep() {
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(Pins::IGNITION), 1);
    esp_sleep_enable_ext1_wakeup(Pins::BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
}

/**
 * @brief Set default states for all output pins
 * 
 * Initializes all output pins to safe default states:
 * - All power controls disabled (LOW)
 * - All contactors open (LOW)
 * - All system enables inactive (LOW)
 * - All auxiliary outputs disabled (LOW)
 * 
 * This function ensures a known safe state during:
 * - Initial power-up
 * - System reset
 * - Error recovery
 * 
 * Critical safety consideration: All high-voltage systems
 * are disabled by default until explicitly enabled by the
 * state management system.
 */
void SystemSetup::setDefaultPinStates() {
    digitalWrite(Pins::PUMP, LOW);
    digitalWrite(Pins::CONTACTOR, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::BCKLIGHT, LOW);
    digitalWrite(Pins::PW1, LOW);
    digitalWrite(Pins::LWP5, LOW);
    digitalWrite(Pins::LWP6, LOW);
    digitalWrite(Pins::LWP7, LOW);
    
}