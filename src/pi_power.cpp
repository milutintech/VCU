#include "pi_power.h"
#include <driver/gpio.h>
#include "config.h"

// HIGH switches the Pi's supply on. The pin comes up LOW after every reset, so a VCU that is
// not running its firmware leaves the Pi without power.
static constexpr bool PI_POWER_ACTIVE_HIGH = true;
static constexpr gpio_num_t PI_POWER_GPIO = static_cast<gpio_num_t>(Pins::LWP6);
static constexpr float LV_OFF_V = 11.5f;        // Pi shuts down below this
static constexpr float LV_ON_V = 12.5f;         // and may start again from this
static constexpr float LV_PLAUSIBLE_V = 5.0f;   // below: no real reading, the Pi would not run

void PiPower::begin() {
    gpio_hold_dis(PI_POWER_GPIO);        // it may still be held from before deep sleep
    pinMode(Pins::LWP6, OUTPUT);
    drive(false);
}

void PiPower::drive(bool on) {
    digitalWrite(Pins::LWP6, (on == PI_POWER_ACTIVE_HIGH) ? HIGH : LOW);
    powered = on;
    Serial.printf("[PiPower] Pi power %s\n", on ? "ON" : "OFF");
}

void PiPower::update(float lvVolts, bool lvFresh) {
    // 12 V low: the Pi shuts down the normal way (ignition-off path, 3 s debounce). It stays
    // off until the DC-DC has clearly recovered the rail: a resting battery comes back above
    // the cutoff once the Pi's load is gone, which would boot and stop the Pi in a loop.
    if (lvFresh && lvVolts >= LV_PLAUSIBLE_V) {
        if (!lowVoltage && lvVolts < LV_OFF_V) {
            lowVoltage = true;
            Serial.printf("[PiPower] 12 V low (%.1f V), shutting the Pi down\n", lvVolts);
        } else if (lowVoltage && lvVolts >= LV_ON_V) {
            lowVoltage = false;
            Serial.printf("[PiPower] 12 V back (%.1f V)\n", lvVolts);
        }
    }

    const PiState before = logic.state();
    // Ignition only: charging does not power the Pi (owner, 2026-11-20; screen dark while charging).
    const bool ignition = !lowVoltage && digitalRead(Pins::IGNITION) == HIGH;
    // Only an acknowledgement given during a request counts.
    const bool ack = ackFlag && (before == PiState::REQUESTED);
    const PiPowerLogic::Out out = logic.update(millis(), ignition, ack);
    if (logic.state() != PiState::REQUESTED) ackFlag = false;

    if (out.powerOn != powered) drive(out.powerOn);
    requested = out.shutdownRequested;
    if (logic.state() != before) {
        Serial.printf("[PiPower] %s -> %s\n", name(before), name(logic.state()));
    }
}

void PiPower::holdForSleep() {
    drive(false);                        // must already be off, this keeps it that way
    gpio_hold_en(PI_POWER_GPIO);
    gpio_deep_sleep_hold_en();
}

const char* PiPower::name(PiState s) {
    switch (s) {
        case PiState::OFF: return "OFF";
        case PiState::ON: return "ON";
        case PiState::REQUESTED: return "REQUESTED";
        case PiState::ACKED: return "ACKED";
    }
    return "?";
}
