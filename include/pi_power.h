/**
 * @file pi_power.h
 * @brief Hardware side of the Pi power sequence (see pi_power_logic.h): owns IO21 (Pins::LWP6).
 */

#pragma once
#include <Arduino.h>
#include "pi_power_logic.h"

class PiPower {
public:
    /** Takes over IO21 (released from its deep-sleep hold) and leaves the Pi without power. */
    void begin();

    /** Call from the web task. Never blocks. */
    void update();

    /** The Pi acknowledged the shutdown request (POST /api/pi/ack). */
    void acknowledge() { ackFlag = true; }

    /** False while the Pi is powered or a shutdown is running: no deep sleep yet. */
    bool mayDeepSleep() const { return logic.mayDeepSleep(); }

    /** Holds IO21 low through deep sleep. Only to be called right before it. */
    void holdForSleep();

    PiState state() const { return logic.state(); }
    bool shutdownRequested() const { return requested; }
    uint32_t cutInMs() const { return logic.cutInMs(millis()); }
    static const char* name(PiState s);

private:
    void drive(bool on);

    PiPowerLogic logic;
    volatile bool ackFlag = false;
    volatile bool requested = false;
    bool powered = false;
};
