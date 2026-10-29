#include "pi_power.h"
#include <driver/gpio.h>
#include "config.h"

// HIGH switches the Pi's supply on. The pin comes up LOW after every reset, so a VCU that is
// not running its firmware leaves the Pi without power.
static constexpr bool PI_POWER_ACTIVE_HIGH = true;
static constexpr gpio_num_t PI_POWER_GPIO = static_cast<gpio_num_t>(Pins::LWP6);

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

void PiPower::update() {
    const PiState before = logic.state();
    const bool ignition = digitalRead(Pins::IGNITION) == HIGH;
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
