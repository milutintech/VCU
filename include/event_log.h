/**
 * @file event_log.h
 * @brief Small event log in RTC memory: survives deep sleep and software resets (not power loss).
 *
 * Made to find out afterwards why a charging session ended (VCU asleep when the owner came back).
 * Times are seconds of the ESP32 system clock, which keeps counting through deep sleep and starts
 * at 0 on power-up; `nowS` in the output gives the reference. Read it with `GET /api/log/events`
 * or the serial command `events`.
 */
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

namespace EventLog {

enum class Ev : uint8_t {
    BOOT = 1,           // a = esp_reset_reason(), b = esp_sleep_get_wakeup_cause()
    STATE,              // a = new VehicleState, b = StopReason (for STANDBY)
    DEEP_SLEEP,         // a = pin bits (1 ignition, 2 NLG wake, 4 unlock button)
    PLUG_GLITCH,        // NLG wake line LOW shorter than the unplug debounce, b = ms
    WAIT_UNPLUG,        // STANDBY with the plug in after a finished session: awake until unplugged
    PRECHARGE_DONE,     // b = HV actual (V)
    PRECHARGE_TIMEOUT,  // a = attempt
    PRECHARGE_FAILED,   // a = attempts
    BATTERY_OFF,        // contactor opened / BSC off (armBattery(false) with something on)
    CHARGER_STATE,      // accepted (debounced) NLG stateAct, a = old, b = new
    CHARGER_DEMAND,     // NLG state demand, a = old, b = new
    NLG_TIMEOUT,        // a = 1 lost, 0 back
    SOC_LIMIT,          // a = max SOC
    UNLOCK_BUTTON,      // a = connector locked
};

/** Why the VCU went to STANDBY (Ev::STATE, b). */
enum class StopReason : uint8_t {
    NONE = 0,
    NO_WAKE_SIGNAL,     // woke up, nothing high
    IGNITION_OFF,       // RUN ended
    UNPLUGGED,          // NLG wake line LOW for the unplug debounce
    UNLOCK_DONE,        // unlock wait: connector unlocked
    UNLOCK_TIMEOUT,     // unlock wait: 15 s, still locked
    PERSIST_UNLOCK,     // unlock request (button / SOC limit), connector unlocked
    BUTTON_UNLOCKED,    // unlock button with the connector already unlocked
};

struct Entry {
    uint32_t timeS;     // system clock seconds (continues through deep sleep)
    uint16_t boot;      // boot counter (wake from deep sleep counts)
    uint8_t ev;
    uint8_t a;
    uint16_t b;
    uint16_t packV;     // pack voltage (V) at the time
    uint8_t soc;        // BMS SOC (%) at the time
};

/** Call once in setup(): checks the RTC block (power-up = garbage), counts the boot, logs BOOT. */
void begin();

/** Thread safe (spinlock), callable from any task. Not from an ISR. */
void add(Ev ev, uint8_t a = 0, uint16_t b = 0, uint16_t packV = 0, uint8_t soc = 0);

const char* name(uint8_t ev);
const char* stopReasonName(uint8_t r);

/** Oldest first. */
void toJson(JsonDocument& doc);
void printTo(Print& out);

}  // namespace EventLog
