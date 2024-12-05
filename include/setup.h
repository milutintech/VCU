/**
 * @file setup.h
 * @brief System Initialization and Safety Configuration
 * 
 * This class handles the complete initialization sequence for all hardware and safety systems:
 * - GPIO configuration (inputs, outputs, analog)
 * - Sleep mode and power management
 * - Default safety states for all systems
 * - Pin state verification
 * 
 * Critical functions:
 * - Safe state initialization before any systems are enabled
 * - Proper sequencing of power-up states
 * - Configuration of wakeup sources
 * - Watchdog initialization
 */

#pragma once
#include <Arduino.h>
#include "state_manager.h"
#include "can_manager.h"
#include "vehicle_control.h"
#include "ADS1X15.h"

class SystemSetup {
public:
    /**
     * @brief Initialize all GPIO pins and verify their states
     * 
     * Configures:
     * - Input pins:
     *   - NLG_HW_Wakeup: Charging system wake detection
     *   - IGNITION: Key position detection
     *   - UNLCKCON: Charge connector unlock button
     * 
     * - Output pins:
     *   - PUMP: Cooling system control
     *   - CONTACTOR: Main battery contactor control
     *   - NLGKL15: Charger control power
     *   - DMCKL15: Motor controller power
     *   - BSCKL15: DC-DC converter power
     *   - BCKLIGHT: Reverse light control
     * 
     * - Communication pins:
     *   - SPI for CAN interface (SCK, MOSI, MISO, CS)
     *   - I2C for ADC interface
     * 
     * @note All outputs are initialized to safe (inactive) states
     * @note Input pin states are verified after configuration
     */
    static void initializeGPIO();

    /**
     * @brief Configure ESP32 sleep mode and wake sources
     * 
     * Setup includes:
     * - Deep sleep mode configuration
     * - Wake sources:
     *   - IGNITION pin (ext0 wake source)
     *   - NLG_HW_Wakeup (ext1 wake source)
     *   - Timer-based wakeup for system checks
     * 
     * Wake pin configurations:
     * - Pull-down resistors enabled
     * - Debounce time configured
     * - Wake level triggers defined
     * 
     * @note Multiple wake sources can be active simultaneously
     * @note System state is preserved across sleep cycles
     */
    static void initializeSleep();

private:
    /**
     * @brief Set default safe states for all output pins
     * 
     * Safety sequence:
     * 1. All contactors opened
     *    - Main battery contactor open
     *    - Precharge contactor open
     * 
     * 2. All power outputs disabled
     *    - Motor controller power off
     *    - Charger power off
     *    - DC-DC converter power off
     * 
     * 3. Auxiliary systems disabled
     *    - Cooling pump off
     *    - Indicator lights off
     * 
     * @note Order of operations is critical for safety
     * @note Pin states are verified after setting
     */
    static void setDefaultPinStates();
    
    /** 
     * @brief Prevent instantiation of this utility class
     * All methods are static and class should never be instantiated
     */
    SystemSetup() = delete;
};