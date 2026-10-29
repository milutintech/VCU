/**
 * @file pi_power_logic.h
 * @brief When the infotainment Pi gets power, and the shutdown that must come before it is cut.
 *
 * Pure logic, no hardware: inputs are the time, the ignition and an acknowledgement from the Pi,
 * outputs are whether the Pi has power and whether it is being asked to shut down. It runs
 * outside the control task, so nothing here can delay driving, precharge or the contactor.
 *
 * Sequence:
 *   OFF ----(ignition on, delays passed)----> ON
 *   ON  ----(ignition off for 3 s)----------> REQUESTED   Pi is asked to shut down
 *   REQUESTED --(Pi acknowledges)-----------> ACKED       Pi has started its shutdown
 *   ACKED --(20 s later)--------------------> OFF         power cut
 *   REQUESTED --(no answer in 15 s)---------> OFF         power cut (Pi absent or unreachable)
 *   REQUESTED/ACKED --(60 s in total)-------> OFF         power cut whatever happens
 *
 * Once a shutdown is requested it always runs to the end, also when the ignition comes back in
 * the meantime. The Pi is then powered on again after a short off time, so a quick power cycle
 * of the car shuts the Pi down properly and restarts it.
 */

#pragma once
#include <stdint.h>

namespace PiPowerTiming {
    constexpr uint32_t IGNITION_OFF_DEBOUNCE_MS = 3000;  ///< ignition must stay off this long
    constexpr uint32_t ACK_TIMEOUT_MS = 15000;           ///< wait for the Pi's acknowledgement
    constexpr uint32_t POST_ACK_MS = 50000;              ///< Pi's time to shut down after it
    constexpr uint32_t HARD_MAX_MS = 70000;              ///< longest a shutdown may take (>= ack 15 s + 50 s)
    constexpr uint32_t MIN_OFF_MS = 3000;                ///< power stays cut at least this long
    constexpr uint32_t POWER_ON_DELAY_MS = 1000;         ///< ignition on this long before power
}

enum class PiState : uint8_t {
    OFF,         ///< no power
    ON,          ///< powered
    REQUESTED,   ///< powered, asked to shut down, no answer yet
    ACKED        ///< powered, shutting down
};

class PiPowerLogic {
public:
    struct Out {
        bool powerOn;             ///< the Pi has power
        bool shutdownRequested;   ///< the Pi is asked to shut down
    };

    /** Call often with the current time. `ack` is true when the Pi has acknowledged a request. */
    Out update(uint32_t now, bool ignition, bool ack) {
        using namespace PiPowerTiming;
        if (!started) {
            started = true;
            offSince = now;        // power has been off since the VCU started
        }
        switch (st) {
        case PiState::OFF:
            if (ignition) {
                if (!ignitionOnTiming) { ignitionOnTiming = true; ignitionOnSince = now; }
                if (now - ignitionOnSince >= POWER_ON_DELAY_MS && now - offSince >= MIN_OFF_MS) {
                    st = PiState::ON;
                    ignitionOffTiming = false;
                }
            } else {
                ignitionOnTiming = false;
            }
            break;

        case PiState::ON:
            if (!ignition) {
                if (!ignitionOffTiming) { ignitionOffTiming = true; ignitionOffSince = now; }
                if (now - ignitionOffSince >= IGNITION_OFF_DEBOUNCE_MS) {
                    st = PiState::REQUESTED;
                    requestedAt = now;
                }
            } else {
                ignitionOffTiming = false;
            }
            break;

        case PiState::REQUESTED:
            if (now - requestedAt >= HARD_MAX_MS) { cut(now); break; }
            if (ack) { st = PiState::ACKED; ackedAt = now; break; }
            if (now - requestedAt >= ACK_TIMEOUT_MS) cut(now);
            break;

        case PiState::ACKED:
            if (now - requestedAt >= HARD_MAX_MS || now - ackedAt >= POST_ACK_MS) cut(now);
            break;
        }
        return {st != PiState::OFF,
                st == PiState::REQUESTED || st == PiState::ACKED};
    }

    PiState state() const { return st; }

    /** Deep sleep only once the Pi has no power and no shutdown is pending. */
    bool mayDeepSleep() const { return st == PiState::OFF; }

    /** Time until the power is cut at the latest, 0 when no shutdown is running. */
    uint32_t cutInMs(uint32_t now) const {
        if (st != PiState::REQUESTED && st != PiState::ACKED) return 0;
        const uint32_t used = now - requestedAt;
        return used >= PiPowerTiming::HARD_MAX_MS ? 0 : PiPowerTiming::HARD_MAX_MS - used;
    }

private:
    void cut(uint32_t now) {
        st = PiState::OFF;
        offSince = now;
        ignitionOnTiming = false;
        ignitionOffTiming = false;
    }

    PiState st = PiState::OFF;
    bool started = false;
    bool ignitionOnTiming = false;
    bool ignitionOffTiming = false;
    uint32_t ignitionOnSince = 0;
    uint32_t ignitionOffSince = 0;
    uint32_t requestedAt = 0;
    uint32_t ackedAt = 0;
    uint32_t offSince = 0;
};
