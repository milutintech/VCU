/**
 * @file vehicle_parameters.h
 * @brief Vehicle Configuration Parameters and Operating Limits - SIMPLIFIED PEDAL SYSTEM
 * 
 * Simplified to focus on essential parameters with configurable pedal zones:
 * - Regen zone (progressive): 0% to regenZoneEnd%
 * - Coast zone (dead): regenZoneEnd% to coastZoneEnd% 
 * - Accel zone (progressive): coastZoneEnd% to 100%
 */

#pragma once

namespace VehicleParams {
    /**
     * @brief Battery System Parameters
     */
    struct Battery {
        static constexpr int MIN_VOLTAGE = 333;     ///< Minimum pack voltage (3.2V * 104S)
        static constexpr int MAX_VOLTAGE = 437;     ///< Maximum pack voltage (4.2V * 104S)
        static constexpr int MAX_DMC_CURRENT = 450; ///< Maximum motor controller current (A)
        static constexpr int MAX_NLG_CURRENT = 80;  ///< Maximum charging current (A)
        static constexpr int PRECHARGE_CURRENT = 20;///< Precharge current limit (A)
        static constexpr float MIN_LVVOLTAGE = 14.4f; ///< Charge Voltage for the LV Bat (V)
    };

    /**
     * @brief Temperature Management Parameters
     */
    struct Temperature {
        static constexpr float INV_HIGH = 45.0f;   ///< Coolant pump on above this inverter temp (°C)
        static constexpr float MOT_HIGH = 45.0f;   ///< Coolant pump on above this motor temp (°C)
        static constexpr float INV_LOW = 30.0f;    ///< Pump and fan off below this inverter temp (°C)
        static constexpr float MOT_LOW = 30.0f;    ///< Pump and fan off below this motor temp (°C)
        static constexpr float FAN_ON = 65.0f;     ///< Fan on above this inverter or motor temp (°C)
        static constexpr float NLG_MAX = 95.0f;    ///< Maximum charger temperature (°C)
    };

    /**
     * @brief Motor Control Parameters
     */
    struct Motor {
        static constexpr int MAX_TRQ = 850;         ///< Maximum motor torque (Nm)
        static constexpr int MAX_REVERSE_TRQ = 450; ///< Maximum reverse torque (Nm)
        static constexpr int MAX_RPM = 6000;        ///< Maximum motor RPM
        static constexpr float DEADZONE_THRESHOLD = 0.5f; ///< Dead zone around zero (%)
    };

    /**
     * @brief Power Management Parameters
     */
    struct Power {
        static constexpr int DMC_DC_GEN = 420;     ///< Generator power limit (A)
        static constexpr int BSC_LV_BUCK = 100;    ///< DC-DC buck mode limit (A)
        static constexpr int BSC_LV_BOOST = 100;   ///< DC-DC boost mode limit (A)
        static constexpr int NLG_MAX_AC = 32;      ///< Maximum AC charging power (A)
    };

    /**
     * @brief Transmission Parameters
     */
    struct Transmission {
        static constexpr float NORMAL_RATIO = 1.2f;     ///< Normal gear ratio
        static constexpr float REDUCED_RATIO = 2.1f;    ///< Reduced gear ratio
        static constexpr float DIFF_RATIO = 3.9f;       ///< Differential ratio
        static constexpr float WHEEL_CIRC = 2.08f;      ///< Wheel circumference (m)
        static constexpr float RPM_SHIFT_THRESHOLD = 150.0f; ///< Shift RPM threshold
    };

    /**
     * @brief SIMPLIFIED: Three-Zone Pedal System with Progressive Curves
     * 
     * Zone Layout:
     * - 0% to regenZoneEnd%: Progressive regen (0% to -100% torque)
     * - regenZoneEnd% to coastZoneEnd%: Coast zone (0% torque)
     * - coastZoneEnd% to 100%: Progressive accel (0% to +100% torque)
     */
    struct Pedal {
        // Update default values to match
        static constexpr float DEFAULT_REGEN_ZONE_END = 16.0f;   // Change from 25.0f
        static constexpr float DEFAULT_COAST_ZONE_END = 17.0f;   // Change from 30.0f
        static constexpr float DEFAULT_REGEN_PROGRESSION = 1.5f; // Change from 1.1f
        static constexpr float DEFAULT_ACCEL_PROGRESSION = 1.7f; // Change from 1.5f
        
        // Validation limits remain the same
        static constexpr float MIN_REGEN_ZONE_END = 10.0f;
        static constexpr float MAX_REGEN_ZONE_END = 50.0f;
        static constexpr float MIN_COAST_ZONE_END = 15.0f;
        static constexpr float MAX_COAST_ZONE_END = 60.0f;
        static constexpr float MIN_PROGRESSION = 1.0f;
        static constexpr float MAX_PROGRESSION = 3.0f;
    };
    /**
     * @brief Gear Transition Control
     */
    struct GearTransition {
        static constexpr unsigned long TRANSITION_TIME_MS = 200; ///< Zero torque hold time during shifts (ms)
        static constexpr float RAMPUP_RATE = 2.0f;              ///< Torque ramp-up rate after shift (%/cycle)
    };

    /**
     * @brief System Timing Parameters
     */
    struct Timing {
        static constexpr unsigned long FAST_CYCLE_MS = 50;    ///< Fast loop interval (ms)
        static constexpr unsigned long SLOW_CYCLE_MS = 100;   ///< Slow loop interval (ms)
        static constexpr unsigned long BMS_TIMEOUT_MS = 1600;     ///< BMS timeout (ms)
    };

};