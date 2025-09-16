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
        static constexpr int MIN_VOLTAGE = 320;     ///< Minimum pack voltage (3.2V * 100S)
        static constexpr int NOM_VOLTAGE = 367;     ///< Nominal pack voltage (3.67V * 100S)
        static constexpr int MAX_VOLTAGE = 420;     ///< Maximum pack voltage (4.2V * 100S)
        static constexpr int MAX_DMC_CURRENT = 450; ///< Maximum motor controller current (A)
        static constexpr int MAX_NLG_CURRENT = 80;  ///< Maximum charging current (A)
        static constexpr int PRECHARGE_CURRENT = 20;///< Precharge current limit (A)
        static constexpr int MIN_LVVOLTAGE = 14.4;  ///< Charge Voltage for the LV Bat (V)
        static constexpr int MAX_SOC = 100;         ///< Maximum Battery Pack SOC
    };

    /**
     * @brief Temperature Management Parameters
     */
    struct Temperature {
        static constexpr float INV_HIGH = 65.0f;   ///< Inverter high temp limit (°C)
        static constexpr float MOT_HIGH = 65.0f;   ///< Motor high temp limit (°C)
        static constexpr float INV_LOW = 40.0f;    ///< Inverter low temp threshold (°C)
        static constexpr float MOT_LOW = 50.0f;    ///< Motor low temp threshold (°C)
        static constexpr float NLG_MAX = 80.0f;    ///< Maximum charger temperature (°C)
    };

    /**
     * @brief Motor Control Parameters
     */
    struct Motor {
        static constexpr int MAX_TRQ = 850;         ///< Maximum motor torque (Nm)
        static constexpr int MAX_REQ_TRQ = 850;     ///< Maximum request torque (Nm)
        static constexpr int MAX_REVERSE_TRQ = 200; ///< Maximum reverse torque (Nm)
        static constexpr int MAX_RPM = 6000;        ///< Maximum motor RPM
        static constexpr float DEADZONE_THRESHOLD = 0.5f; ///< Dead zone around zero (%)
    };

    /**
     * @brief Power Management Parameters
     */
    struct Power {
        static constexpr int DMC_DC_MOT = 450;     ///< Motor power limit (A)
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
        static constexpr unsigned long NLG_UNLOCK_TIMEOUT = 3000; ///< Charger unlock timeout (ms)
        static constexpr unsigned long PRECHARGE_TIMEOUT = 5000;  ///< Precharge timeout (ms)
        static constexpr unsigned long BMS_TIMEOUT_MS = 1500;     ///< BMS timeout (ms)
    };

    /**
     * @brief Vehicle Speed and Performance Limits
     */
    struct Limits {
        static constexpr float MAX_SPEED = 120.0f;        ///< Maximum speed (kph)
        static constexpr float MAX_REVERSE_SPEED = 20.0f; ///< Maximum reverse speed (kph)
        static constexpr int MAX_MOTOR_RPM = 8000;        ///< Maximum motor speed (RPM)
        static constexpr float MAX_ACCELERATION = 3.0f;   ///< Maximum acceleration (m/s²)
    };
};