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
 * - Control system configuration
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
        static constexpr int MAX_DMC_CURRENT = 100; ///< Maximum motor controller current (A)
        static constexpr int MAX_NLG_CURRENT = 62;  ///< Maximum charging current (A)
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
        static constexpr float MOT_HIGH = 80.0f;   ///< Motor high temp limit (°C)
        static constexpr float INV_LOW = 40.0f;    ///< Inverter low temp threshold (°C)
        static constexpr float MOT_LOW = 50.0f;    ///< Motor low temp threshold (°C)
        static constexpr float NLG_MAX = 80.0f;    ///< Maximum charger temperature (°C)
    };

    /**
     * @brief Motor Control Parameters
     * Defines torque limits and control characteristics
     */
    struct Motor {
        static constexpr int MAX_TRQ = 850;         ///< Maximum motor torque (Nm) 0-850
        static constexpr int MAX_REQ_TRQ = 800;     ///< Maximum request torque (Nm)0-850
        static constexpr int MAX_REVERSE_TRQ = 200; ///< Maximum reverse torque (Nm)0-850
        static constexpr int MAX_RPM = 1000;         ///< Maximum motor RPM 0-6000
        static constexpr float MAX_ACCEL_STEP = 8.0f;   ///< Torque ramp-up limit (Nm/cycle)
        static constexpr float MAX_DECEL_STEP = 25.0f;  ///< Torque ramp-down limit (Nm/cycle)
        static constexpr float TORQUE_DEADBAND_HIGH = 25.0f; ///< Upper deadband threshold (Nm)
        static constexpr float TORQUE_DEADBAND_LOW = 18.0f;  ///< Lower deadband threshold (Nm)
    };

    /**
     * @brief Power Management Parameters
     * Defines power limits for various systems
     */
    struct Power {
        static constexpr int DMC_DC_MOT = 450;     ///< Motor power limit (A)0-450
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
        static constexpr float RPM_SHIFT_THRESHOLD = 100.0f; ///< Shift RPM threshold
    };

    /**
     * @brief Vehicle Control Parameters
     * Defines pedal response and control behavior
     */
    struct Control {
        static constexpr float PEDAL_GAMMA = 1.5f;      ///< Pedal response curve exponent
        static constexpr float SPEED_FACTOR = 1.2f;     ///< Speed scaling factor
        static constexpr float MIN_PEDAL_THRESHOLD = 2.0f; ///< Minimum pedal activation (%)
        static constexpr float COAST_POSITION_MIN = 20.0f; ///< Minimum coast pedal position (%)
        static constexpr float COAST_POSITION_MAX = 50.0f; ///< Maximum coast pedal position (%)
        static constexpr float CONTROL_DT = 0.01f;      ///< Control loop time step (seconds)
    };

    /**
     * @brief Regenerative Braking Parameters
     * Defines regen behavior and limits
     */
    struct Regen {
        static constexpr float FADE_START = 400.0f;    ///< Speed for regen fade start (kph)
        static constexpr float ZERO_SPEED = 0.5f;      ///< Zero speed threshold (kph)
        static constexpr float MIN_SPEED = 100.0f;     ///< Minimum regen speed (kph)
        static constexpr float END_POINT = 35.0f;      ///< Regen end pedal position (%)
        static constexpr float COAST_END = 40.0f;      ///< Coast end pedal position (%)
    };

    /**
     * @brief One Pedal Drive Configuration
     * Defines OPD behavior and response curves
     */
    struct OPD {
        static constexpr double MAX_SPEED = 120.0;     ///< Maximum vehicle speed (kph)
        static constexpr double ZERO_SPEED_THRESHOLD = 1.0; ///< Threshold below which the vehicle is considered nearly stopped (kph)
        static constexpr double COAST_RANGE = 10.0;    ///< Pedal coast range (%)
        static constexpr double PHI = 35.0;            ///< Top speed pedal position (%)
        static constexpr double SHAPE_FACTOR = 2.0;    ///< Pedal map curve shape factor
        static constexpr double ROLLBACK_SPEED = 3.0;  ///< Anti-rollback threshold (kph)
        static constexpr double ROLLBACK_TORQUE = 15.0;///< Anti-rollback torque (% of MAX_TRQ)
        static constexpr double MAX_REGEN = 80.0;      ///< Maximum regenerative braking level (% of MAX_TRQ)
        static constexpr double MAX_DRIVE = 100.0;     ///< Maximum drive torque level (% of MAX_TRQ)
        static constexpr double BRAKE_LIGHT_THRESHOLD = 20.0; ///< Brake light activation threshold (%)
        static constexpr double SLOW_SPEED_REGEN_CAP = 200.0; ///< Regen torque cap at slow speeds (Nm)
        static constexpr double REGEN_TORQUE_CAP = 300;
    };

    /**
     * @brief System Timing Parameters
     * Defines control loop and timeout values
     */
    struct Timing {
        static constexpr unsigned long FAST_CYCLE_MS = 50;    ///< Fast loop interval (ms)
        static constexpr unsigned long SLOW_CYCLE_MS = 100;    ///< Slow loop interval (ms)
        static constexpr unsigned long NLG_UNLOCK_TIMEOUT = 3000; ///< Charger unlock timeout (ms)
        static constexpr unsigned long PRECHARGE_TIMEOUT = 5000;  ///< Precharge timeout (ms)
        static constexpr unsigned long BMS_TIMEOUT_MS = 1000;     ///< NLG charging timout when bms fuckes up
    };
    
    /**
     * @brief ADC Configuration Parameters
     * Defines ADC scaling and calibration values
     */
    struct ADC {
        static constexpr int MIN_POT_VALUE = 15568;   ///< Minimum pedal ADC value
        static constexpr int MAX_POT_VALUE = 11200;   ///< Maximum pedal ADC value
        static constexpr float POT_DEADBAND = 0.02f;  ///< Pedal deadband (2%)
    };

    /**
     * @brief Vehicle Speed and Performance Limits
     * Defines vehicle performance boundaries
     */
    struct Limits {
        static constexpr float MAX_SPEED = 120.0f;     ///< Maximum speed (kph)
        static constexpr float MAX_REVERSE_SPEED = 20.0f; ///< Maximum reverse speed (kph)
        static constexpr int MAX_MOTOR_RPM = 8000;     ///< Maximum motor speed (RPM)
        static constexpr float MAX_ACCELERATION = 3.0f; ///< Maximum acceleration (m/s²)
    };
};