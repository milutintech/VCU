/**
 * @file vehicle_parameters.h
 * @brief Vehicle Configuration Parameters and Operating Limits
 * 
 * Defines all critical vehicle parameters including:
 * - Battery system limits and thresholds
 * - Motor and inverter operational parameters
 * - Temperature management thresholds
 * - Power system limits
 * - Vehicle dynamics parameters
 * - New percentage-based pedal control system
 */

#pragma once

namespace VehicleParams {
    /**
     * @brief Battery System Parameters
     * Defines voltage limits and current thresholds for the high-voltage battery
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
     * Defines temperature limits for various systems
     */
    struct Temperature {
        static constexpr float INV_HIGH = 65.0f;   ///< Inverter high temp limit (°C)
        static constexpr float MOT_HIGH = 65.0f;   ///< Motor high temp limit (°C)
        static constexpr float INV_LOW = 40.0f;    ///< Inverter low temp threshold (°C)
        static constexpr float MOT_LOW = 50.0f;    ///< Motor low temp threshold (°C)
        static constexpr float NLG_MAX = 80.0f;    ///< Maximum charger temperature (°C)
    };

    /**
     * @brief Motor Control Parameters - Updated for Percentage System
     * Defines torque limits and control characteristics
     */
    struct Motor {
        static constexpr int MAX_TRQ = 850;         ///< Maximum motor torque (Nm) - for conversion only
        static constexpr int MAX_REQ_TRQ = 850;     ///< Maximum request torque (Nm) - for conversion only
        static constexpr int MAX_REVERSE_TRQ = 200; ///< Maximum reverse torque (Nm) - for conversion only
        static constexpr int MAX_RPM = 6000;        ///< Maximum motor RPM 0-6000
        static constexpr float MAX_ACCEL_STEP = 8.0f;   ///< Percentage ramp-up limit (%/cycle)
        static constexpr float MAX_DECEL_STEP = 15.0f;  ///< Percentage ramp-down limit (%/cycle)
        static constexpr float DEADZONE_THRESHOLD = 0.5f; ///< Dead zone around zero (%)
    };

    /**
     * @brief Power Management Parameters
     * Defines power limits for various systems
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
     * Defines gear ratios and mechanical parameters
     */
    struct Transmission {
        static constexpr float NORMAL_RATIO = 1.2f;     ///< Normal gear ratio
        static constexpr float REDUCED_RATIO = 2.1f;    ///< Reduced gear ratio
        static constexpr float DIFF_RATIO = 3.9f;       ///< Differential ratio
        static constexpr float WHEEL_CIRC = 2.08f;      ///< Wheel circumference (m)
        static constexpr float RPM_SHIFT_THRESHOLD = 150.0f; ///< Shift RPM threshold
    };

    /**
     * @brief NEW: Smooth One-Foot Driving Pedal System
     * Progressive zones for natural brake-to-accelerate feel
     */
    struct Pedal {
        static constexpr float GAMMA = 1.5f;            ///< Pedal response curve exponent
        
        // Pedal Zone Boundaries (% of pedal travel)
        static constexpr float STRONG_REGEN_END = 15.0f;   ///< End of strong regen zone (replaces brake)
        static constexpr float LIGHT_REGEN_END = 25.0f;    ///< End of light regen zone  
        static constexpr float COAST_ZONE_END = 30.0f;     ///< End of coast zone (dead zone)
        // 30-100% = Acceleration zone
        
        // Torque Percentages for each zone
        static constexpr float MAX_STRONG_REGEN = -60.0f;  ///< Maximum strong regen (-60%)
        static constexpr float MIN_STRONG_REGEN = -20.0f;  ///< Minimum strong regen (-20%)
        static constexpr float MAX_LIGHT_REGEN = -20.0f;   ///< Maximum light regen (-20%)
        static constexpr float MIN_LIGHT_REGEN = -5.0f;    ///< Minimum light regen (-5%)
        static constexpr float MAX_ACCELERATION = 100.0f;   ///< Maximum acceleration (100%)
        
        // Speed-dependent behavior thresholds (kph)
        static constexpr float LOW_SPEED_LIMIT = 5.0f;     ///< Below this speed, reduce regen
        static constexpr float MEDIUM_SPEED_LIMIT = 10.0f; ///< Speed for full regen availability
        static constexpr float HIGH_SPEED_LIMIT = 50.0f;   ///< Speed for efficiency optimization
    };

    /**
     * @brief Advanced Filtering Parameters
     * Prevents torque spikes while maintaining responsiveness
     */
    struct Filtering {
        static constexpr float SPIKE_THRESHOLD = 15.0f;    ///< Spike detection threshold (%/cycle)
        static constexpr float NORMAL_RATE_LIMIT = 8.0f;   ///< Normal max change rate (%/cycle)  
        static constexpr float SPIKE_RATE_LIMIT = 4.0f;    ///< Spike-detected max change rate (%/cycle)
        static constexpr float FILTER_ALPHA = 0.15f;       ///< Exponential smoothing (15% new, 85% old)
        static constexpr float DEADZONE_HYSTERESIS = 1.0f; ///< Hysteresis around zero (%)
    };

    /**
     * @brief Gear Transition Control
     * Prevents jerking during gear changes
     */
    struct GearTransition {
        static constexpr unsigned long TRANSITION_TIME_MS = 200; ///< Zero torque hold time during shifts (ms)
        static constexpr float RAMPUP_RATE = 2.0f;              ///< Torque ramp-up rate after shift (%/cycle)
    };

    /**
     * @brief System Timing Parameters
     * Defines control loop and timeout values
     */
    struct Timing {
        static constexpr unsigned long FAST_CYCLE_MS = 50;    ///< Fast loop interval (ms)
        static constexpr unsigned long SLOW_CYCLE_MS = 100;   ///< Slow loop interval (ms)
        static constexpr unsigned long NLG_UNLOCK_TIMEOUT = 3000; ///< Charger unlock timeout (ms)
        static constexpr unsigned long PRECHARGE_TIMEOUT = 5000;  ///< Precharge timeout (ms)
        static constexpr unsigned long BMS_TIMEOUT_MS = 1500;     ///< BMS timeout (ms)
        static constexpr float CONTROL_DT = 0.01f;                ///< PID control loop time step (seconds)
    };
    

    /**
     * @brief Vehicle Speed and Performance Limits
     * Defines vehicle performance boundaries
     */
    struct Limits {
        static constexpr float MAX_SPEED = 120.0f;        ///< Maximum speed (kph)
        static constexpr float MAX_REVERSE_SPEED = 20.0f; ///< Maximum reverse speed (kph)
        static constexpr int MAX_MOTOR_RPM = 8000;        ///< Maximum motor speed (RPM)
        static constexpr float MAX_ACCELERATION = 3.0f;   ///< Maximum acceleration (m/s²)
    };

    // REMOVED UNUSED CONSTANTS:
    // - Control::SPEED_FACTOR (unused)
    // - Control::MIN_PEDAL_THRESHOLD (replaced by new pedal system)  
    // - Control::COAST_POSITION_MIN/MAX (replaced by new pedal zones)
    // - Regen::FADE_START, ZERO_SPEED, MIN_SPEED (not used in current implementation)
    // - OPD constants (will be redesigned later if needed)
};