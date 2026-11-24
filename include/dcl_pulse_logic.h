/**
 * @file dcl_pulse_logic.h
 * @brief Acceleration pulses above the Orion's discharge limit (DCL), for the DMC current limit.
 *
 * Pure logic, no hardware. The DCL is conservative, so the DMC may draw up to PULSE_PERCENT of
 * it, never more than the DMC's own limit, for PULSE_MS of time above the DCL. Over the last
 * TAPER_MS of that budget the limit ramps down to the DCL, so the end of a pulse is no torque
 * step. Time spent clearly below the DCL earns the budget back 1:1, so a short pulse only costs
 * what it used and a full one needs PULSE_MS to recover.
 *
 * The limit scales with the DCL, so a falling DCL (low cells, heat) pulls it down at once, and
 * DCL 0 A means 0 A: the BMS forbidding discharge is never pulsed over.
 */

#pragma once
#include <stdint.h>

namespace DclPulse {
    constexpr uint32_t PULSE_MS = 60000;          ///< time above the DCL per pulse (Orion: 60 s)
    constexpr uint32_t TAPER_MS = 5000;           ///< ramp to the DCL at the end of it
    constexpr uint32_t PULSE_PERCENT = 200;       ///< of the DCL (Orion pulse setting: 200 %)
    constexpr uint32_t RECOVER_BELOW_PERCENT = 90;///< budget returns only below this % of the DCL
}

class DclPulseLogic {
public:
    /**
     * @param now        time in ms
     * @param dclA       BMS discharge limit (A), 0 when discharge is not allowed
     * @param currentA   pack current (A), positive = discharging
     * @param maxA       DMC's own limit (A), a hard cap
     * @return discharge current limit for the DMC (A)
     */
    uint32_t update(uint32_t now, uint32_t dclA, int32_t currentA, uint32_t maxA) {
        const uint32_t dt = started ? now - last : 0;
        started = true;
        last = now;

        if (dclA == 0) return 0;

        if (currentA > static_cast<int32_t>(dclA)) {
            used = (dt >= DclPulse::PULSE_MS - used) ? DclPulse::PULSE_MS : used + dt;
        } else if (currentA * 100 < static_cast<int32_t>(dclA * DclPulse::RECOVER_BELOW_PERCENT)) {
            used = (dt >= used) ? 0 : used - dt;
        }

        const uint32_t base = dclA < maxA ? dclA : maxA;
        const uint32_t pulse = dclA * DclPulse::PULSE_PERCENT / 100;
        const uint32_t ceiling = pulse < maxA ? pulse : maxA;
        if (ceiling <= base) return base;

        const uint32_t remaining = DclPulse::PULSE_MS - used;
        if (remaining >= DclPulse::TAPER_MS) return ceiling;
        return base + (ceiling - base) * remaining / DclPulse::TAPER_MS;
    }

    /** Pulse time still available (ms). */
    uint32_t remainingMs() const { return DclPulse::PULSE_MS - used; }

private:
    bool started = false;
    uint32_t last = 0;
    uint32_t used = 0;    ///< budget spent above the DCL (ms)
};
