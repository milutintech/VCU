/**
 * @file fault_handler.cpp
 * @brief Fault detection with hysteresis and fault reactions (see README "Fault handling")
 */

#include "fault_handler.h"
#include "vehicle_parameters.h"

void FaultHandler::evaluate(ErrorCode code, bool bad, bool ok, ErrorSeverity sev,
                            const char* text, float value, const char* component) {
    if (bad) {
        monitor.raiseFault(code, sev, text, value, component);
    } else if (ok) {
        monitor.clearFault(code);
    }
}

unsigned long FaultHandler::ageMs(unsigned long lastMs, unsigned long since, unsigned long now) {
    unsigned long ref = lastMs > since ? lastMs : since;
    return now - ref;
}

void FaultHandler::update(const CANManager& can, const StateManager& state) {
    using namespace VehicleParams;
    const unsigned long now = millis();

    const VehicleState vs = state.getCurrentState();
    if (vs != lastState) {
        lastState = vs;
        stateEnteredMs = now;
    }

    // ---- CAN timeouts -------------------------------------------------------
    // A node that never sent anything is timed from boot (BMS, BSC) or from entering
    // the state in which it is needed (DMC in RUN, NLG in CHARGING), so a node that
    // is simply off in standby does not raise a fault.
    const unsigned long bmsAge = ageMs(can.getLastBMSMs(), bootMs, now);
    const bool bmsTimeout = bmsAge > Timing::BMS_TIMEOUT_MS;
    evaluate(ErrorCode::BMS_TIMEOUT, bmsTimeout, !bmsTimeout, ErrorSeverity::ERROR,
             "BMS CAN timeout", (float)bmsAge, "BMS");

    const unsigned long bscAge = ageMs(can.getLastBSCMs(), bootMs, now);
    const bool bscTimeout = bscAge > FaultLimits::BSC_TIMEOUT_MS;
    evaluate(ErrorCode::BSC_TIMEOUT, bscTimeout, !bscTimeout, ErrorSeverity::WARNING,
             "BSC CAN timeout", (float)bscAge, "BSC");

    bool dmcTimeout = false;
    if (vs == VehicleState::RUN) {
        const unsigned long age = ageMs(can.getLastDMCMs(), stateEnteredMs, now);
        dmcTimeout = age > FaultLimits::DMC_TIMEOUT_MS;
        evaluate(ErrorCode::DMC_TIMEOUT, dmcTimeout, !dmcTimeout, ErrorSeverity::CRITICAL,
                 "DMC CAN timeout", (float)age, "DMC");
    } else if (monitor.isFaultActive(ErrorCode::DMC_TIMEOUT)) {
        monitor.clearFault(ErrorCode::DMC_TIMEOUT);   // not needed outside RUN
    }

    if (vs == VehicleState::CHARGING) {
        const unsigned long age = ageMs(can.getLastNLGMs(), stateEnteredMs, now);
        const bool t = age > FaultLimits::NLG_TIMEOUT_MS;
        evaluate(ErrorCode::NLG_TIMEOUT, t, !t, ErrorSeverity::WARNING,
                 "NLG CAN timeout", (float)age, "NLG");
    } else if (monitor.isFaultActive(ErrorCode::NLG_TIMEOUT)) {
        monitor.clearFault(ErrorCode::NLG_TIMEOUT);
    }

    // ---- Battery voltage (only while the BMS value is fresh) ----------------
    const float packV = (float)can.getBMSData().voltage;
    if (can.getLastBMSMs() != 0 && !bmsTimeout) {
        evaluate(ErrorCode::BATTERY_UNDERVOLTAGE,
                 packV < Battery::MIN_VOLTAGE,
                 packV >= Battery::MIN_VOLTAGE + FaultLimits::VOLTAGE_HYST_V,
                 ErrorSeverity::ERROR, "Battery voltage low", packV, "BMS");
        evaluate(ErrorCode::BATTERY_OVERVOLTAGE,
                 packV > Battery::MAX_VOLTAGE,
                 packV <= Battery::MAX_VOLTAGE - FaultLimits::VOLTAGE_HYST_V,
                 ErrorSeverity::CRITICAL, "Battery voltage high", packV, "BMS");
    }

    // ---- Inverter / motor temperature (only once the DMC has reported) ------
    const DMCData& dmc = can.getDMCData();
    bool invCutoff = false, motCutoff = false;
    if (can.getLastDMCMs() != 0) {
        evaluate(ErrorCode::INVERTER_OVERTEMP,
                 dmc.tempInverter > FaultLimits::TEMP_DERATE_C,
                 dmc.tempInverter < FaultLimits::TEMP_DERATE_C - FaultLimits::TEMP_HYST_C,
                 ErrorSeverity::ERROR, "Inverter overtemperature", dmc.tempInverter, "DMC");
        evaluate(ErrorCode::MOTOR_OVERTEMP,
                 dmc.tempMotor > FaultLimits::TEMP_DERATE_C,
                 dmc.tempMotor < FaultLimits::TEMP_DERATE_C - FaultLimits::TEMP_HYST_C,
                 ErrorSeverity::ERROR, "Motor overtemperature", dmc.tempMotor, "DMC");
        invCutoff = dmc.tempInverter > FaultLimits::TEMP_CUTOFF_C;
        motCutoff = dmc.tempMotor > FaultLimits::TEMP_CUTOFF_C;
        if (invCutoff) monitor.raiseFault(ErrorCode::INVERTER_OVERTEMP, ErrorSeverity::CRITICAL,
                                          "Inverter overtemperature", dmc.tempInverter, "DMC");
        if (motCutoff) monitor.raiseFault(ErrorCode::MOTOR_OVERTEMP, ErrorSeverity::CRITICAL,
                                          "Motor overtemperature", dmc.tempMotor, "DMC");
    }

    // ---- Reactions: the only place that maps faults to actions --------------
    FaultReaction r;
    auto limit = [&r](float scale) { if (scale < r.torqueScale) r.torqueScale = scale; };

    if (monitor.isFaultActive(ErrorCode::INVERTER_OVERTEMP) ||
        monitor.isFaultActive(ErrorCode::MOTOR_OVERTEMP))     limit(FaultLimits::TORQUE_SCALE_DERATE);
    if (invCutoff || motCutoff)                               limit(FaultLimits::TORQUE_SCALE_ZERO);
    if (monitor.isFaultActive(ErrorCode::BATTERY_UNDERVOLTAGE)) limit(FaultLimits::TORQUE_SCALE_DERATE);
    if (monitor.isFaultActive(ErrorCode::BATTERY_OVERVOLTAGE))  limit(FaultLimits::TORQUE_SCALE_ZERO);
    if (monitor.isFaultActive(ErrorCode::BMS_TIMEOUT))          limit(FaultLimits::TORQUE_SCALE_ZERO);

    // Any active CRITICAL fault: stop commanding the DMC and request no torque.
    if (monitor.hasActiveFault(ErrorSeverity::CRITICAL)) {
        r.disableDMC = true;
        r.torqueScale = 0.0f;
    }
    current = r;
}
