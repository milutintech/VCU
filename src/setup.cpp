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
    pinMode(Pins::PUMP, OUTPUT);         // Now Fan (PW0)
    pinMode(Pins::CONTACTOR, OUTPUT);
    pinMode(Pins::NLGKL15, OUTPUT);
    pinMode(Pins::DMCKL15, OUTPUT);
    pinMode(Pins::BSCKL15, OUTPUT);
    pinMode(Pins::BCKLIGHT, OUTPUT);
    pinMode(Pins::PW1, OUTPUT);          // Now Contactor
    pinMode(Pins::LWP5, OUTPUT);
    pinMode(Pins::LWP6, OUTPUT);
    pinMode(Pins::LWP7, OUTPUT);         // Pump
    pinMode(Pins::UNLCKCON, INPUT);
    pinMode(19, OUTPUT);
    
    setDefaultPinStates();
}

/**
 * @brief Configure ESP32 sleep mode parameters
 * 
 * Sets up deep sleep functionality including:
 * - Wake sources configuration
 * - GPIO wake pin selection
 * - Wake level triggers
 */
void SystemSetup::initializeSleep() {
    // Define the wake pins
    gpio_num_t ignitionPin = static_cast<gpio_num_t>(Pins::IGNITION);
    
    // Configure the ignition pin as a wake source (EXT0)
    esp_sleep_enable_ext0_wakeup(ignitionPin, 1);  // Wake on HIGH level
    
    // Define the bitmask for multiple wake sources (EXT1)
    uint64_t wakeBitmask = (1ULL << Pins::NLG_HW_Wakeup) | 
                          (1ULL << Pins::IGNITION) | 
                          (1ULL << Pins::UNLCKCON);
    
    // Enable EXT1 wake sources with the defined bitmask
    esp_sleep_enable_ext1_wakeup(wakeBitmask, ESP_EXT1_WAKEUP_ANY_HIGH);
    
    // Configure GPIO pulldowns using standard Arduino functions
    pinMode(Pins::IGNITION, INPUT_PULLDOWN);
    pinMode(Pins::NLG_HW_Wakeup, INPUT_PULLDOWN);
    pinMode(Pins::UNLCKCON, INPUT_PULLDOWN);
    
    Serial.println("Sleep configuration initialized with wake sources:");
    Serial.print("- IGNITION (EXT0): Pin ");
    Serial.println(Pins::IGNITION);
    Serial.print("- Wake bitmask (EXT1): 0x");
    Serial.println(wakeBitmask, HEX);
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
    digitalWrite(Pins::PUMP, LOW);       // Fan off (will be controlled by PWM)
    digitalWrite(Pins::CONTACTOR, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::BCKLIGHT, LOW);
    digitalWrite(Pins::PW1, LOW);        // Contactor open
    digitalWrite(Pins::LWP5, LOW);
    digitalWrite(Pins::LWP6, LOW);
    digitalWrite(Pins::LWP7, LOW);       // Pump off
    digitalWrite(19, LOW);
}