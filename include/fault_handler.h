/**
 * @file fault_handler.h
 * @brief Fault detection (with hysteresis) and the reactions to active faults.
 *
 * FaultHandler is the single place that decides when a fault is raised or
 * cleared and what the vehicle does about it. ErrorMonitor only stores faults.
 * Thresholds that already exist in vehicle_parameters.h are used unchanged;
 * the hysteresis margins and the DMC/BSC/NLG timeouts below are new and local
 * to this file on purpose (vehicle_parameters.h is not modified).
 */

#pragma once
#include <Arduino.h>
#include "error_monitor.h"
#include "can_manager.h"
#include "state_manager.h"

namespace FaultLimits {
    // Hysteresis: a fault clears only once the value is back this far inside the limit.
    constexpr float VOLTAGE_HYST_V = 5.0f;     ///< pack volts (under/overvoltage)
    constexpr float TEMP_HYST_C    = 5.0f;     ///< inverter/motor °C

    // Inverter/motor are rated to 100 °C. VehicleParams INV_HIGH/MOT_HIGH (45 °C) are
    // cooling-stage thresholds in StateManager and are NOT used for derating.
    constexpr float TEMP_DERATE_C = 100.0f;      ///< derate (torque x0.5) above this
    constexpr float TEMP_CUTOFF_C = 105.0f;      ///< torque 0 and DMC disabled above this

    // The charger cuts its own power when it runs hot; this only tells the driver why
    // charging slows down. The charger itself is good to Temperature::NLG_MAX (95 °C).
    constexpr float CHARGER_HOT_C = 85.0f;       ///< cooling plate °C, info fault above this

    // CAN receive timeouts. BMS uses VehicleParams::Timing::BMS_TIMEOUT_MS.
    constexpr unsigned long DMC_TIMEOUT_MS = 500;
    constexpr unsigned long BSC_TIMEOUT_MS = 1500;
    constexpr unsigned long NLG_TIMEOUT_MS = 1500;

    // Torque scale (fraction of requested torque, applied to drive and regen).
    constexpr float TORQUE_SCALE_DERATE = 0.5f;  ///< overtemp, undervoltage
    constexpr float TORQUE_SCALE_ZERO   = 0.0f;  ///< BMS timeout, hard overtemp, overvoltage
}

/** What the rest of the firmware must do because of the current faults. */
struct FaultReaction {
    float torqueScale = 1.0f;   ///< multiply requested torque percentage by this (0..1)
    bool disableDMC = false;    ///< true: do not enable the DMC
};

class FaultHandler {
public:
    explicit FaultHandler(ErrorMonitor& monitor) : monitor(monitor), bootMs(millis()) {}

    /** Evaluate all fault conditions and recompute the reaction. Call every CAN-task cycle. */
    void update(const CANManager& can, const StateManager& state);

    /** Reaction computed by the last update(). */
    const FaultReaction& reaction() const { return current; }

private:
    ErrorMonitor& monitor;
    unsigned long bootMs;
    VehicleState lastState = VehicleState::STANDBY;
    unsigned long stateEnteredMs = 0;
    FaultReaction current;
    bool lastFlagsFresh = false;   ///< BMS allow-flag frame freshness in the last update()

    /** Raise when `bad`, clear when `ok`, otherwise keep the current state (hysteresis band). */
    void evaluate(ErrorCode code, bool bad, bool ok, ErrorSeverity sev,
                  const char* text, float value, const char* component);
    /** Age of a node's last frame, measured from `since` if nothing arrived after it. */
    static unsigned long ageMs(unsigned long lastMs, unsigned long since, unsigned long now);
};
