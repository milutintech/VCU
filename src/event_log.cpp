#include "event_log.h"
#include <sys/time.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>

namespace EventLog {

namespace {

constexpr uint32_t MAGIC = 0x56434C31;  // "VCL1", change when Entry changes
constexpr size_t CAPACITY = 64;

struct Store {
    uint32_t magic;
    uint16_t boot;
    uint16_t head;      // next write position
    uint16_t count;
    Entry entries[CAPACITY];
};

// Not initialised at reset: keeps its contents through deep sleep and software resets.
RTC_NOINIT_ATTR Store store;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t nowS() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<uint32_t>(tv.tv_sec);
}

}  // namespace

void begin() {
    portENTER_CRITICAL(&mux);
    if (store.magic != MAGIC || store.head >= CAPACITY || store.count > CAPACITY) {
        memset(&store, 0, sizeof(store));
        store.magic = MAGIC;
    }
    store.boot++;
    portEXIT_CRITICAL(&mux);
    add(Ev::BOOT, static_cast<uint8_t>(esp_reset_reason()),
        static_cast<uint16_t>(esp_sleep_get_wakeup_cause()));
}

void add(Ev ev, uint8_t a, uint16_t b, uint16_t packV, uint8_t soc) {
    Entry e;
    e.timeS = nowS();
    e.ev = static_cast<uint8_t>(ev);
    e.a = a;
    e.b = b;
    e.packV = packV;
    e.soc = soc;
    portENTER_CRITICAL(&mux);
    e.boot = store.boot;
    store.entries[store.head] = e;
    store.head = (store.head + 1) % CAPACITY;
    if (store.count < CAPACITY) store.count++;
    portEXIT_CRITICAL(&mux);
}

const char* name(uint8_t ev) {
    switch (static_cast<Ev>(ev)) {
        case Ev::BOOT: return "BOOT";
        case Ev::STATE: return "STATE";
        case Ev::DEEP_SLEEP: return "DEEP_SLEEP";
        case Ev::PLUG_GLITCH: return "PLUG_GLITCH";
        case Ev::WAIT_UNPLUG: return "WAIT_UNPLUG";
        case Ev::PRECHARGE_DONE: return "PRECHARGE_DONE";
        case Ev::PRECHARGE_TIMEOUT: return "PRECHARGE_TIMEOUT";
        case Ev::PRECHARGE_FAILED: return "PRECHARGE_FAILED";
        case Ev::BATTERY_OFF: return "BATTERY_OFF";
        case Ev::CHARGER_STATE: return "CHARGER_STATE";
        case Ev::CHARGER_DEMAND: return "CHARGER_DEMAND";
        case Ev::NLG_TIMEOUT: return "NLG_TIMEOUT";
        case Ev::SOC_LIMIT: return "SOC_LIMIT";
        case Ev::UNLOCK_BUTTON: return "UNLOCK_BUTTON";
    }
    return "?";
}

const char* stopReasonName(uint8_t r) {
    switch (static_cast<StopReason>(r)) {
        case StopReason::NONE: return "";
        case StopReason::NO_WAKE_SIGNAL: return "no wake signal";
        case StopReason::IGNITION_OFF: return "ignition off";
        case StopReason::UNPLUGGED: return "unplugged";
        case StopReason::UNLOCK_DONE: return "unlock done";
        case StopReason::UNLOCK_TIMEOUT: return "unlock timeout";
        case StopReason::PERSIST_UNLOCK: return "unlock request done";
        case StopReason::BUTTON_UNLOCKED: return "unlock button, connector unlocked";
    }
    return "?";
}

namespace {
size_t snapshot(Entry* out) {
    portENTER_CRITICAL(&mux);
    const size_t n = store.count;
    const size_t start = (store.head + CAPACITY - n) % CAPACITY;
    for (size_t i = 0; i < n; i++) out[i] = store.entries[(start + i) % CAPACITY];
    portEXIT_CRITICAL(&mux);
    return n;
}
}  // namespace

void toJson(JsonDocument& doc) {
    static Entry copy[CAPACITY];   // callers are the web task only
    const size_t n = snapshot(copy);
    doc["nowS"] = nowS();
    doc["boot"] = store.boot;
    JsonArray arr = doc["events"].to<JsonArray>();
    for (size_t i = 0; i < n; i++) {
        const Entry& e = copy[i];
        JsonObject o = arr.add<JsonObject>();
        o["t"] = e.timeS;
        o["boot"] = e.boot;
        o["ev"] = name(e.ev);
        o["a"] = e.a;
        o["b"] = e.b;
        if (e.ev == static_cast<uint8_t>(Ev::STATE) && e.b) o["reason"] = stopReasonName(e.b);
        o["packV"] = e.packV;
        o["soc"] = e.soc;
    }
}

void printTo(Print& out) {
    static Entry copy[CAPACITY];
    const size_t n = snapshot(copy);
    const uint32_t now = nowS();
    out.printf("Event log: %u entries, boot %u, now %lu s\n", (unsigned)n, store.boot, (unsigned long)now);
    for (size_t i = 0; i < n; i++) {
        const Entry& e = copy[i];
        out.printf("  -%6lus boot %u %-17s a=%u b=%u %s pack %uV soc %u%%\n",
                   (unsigned long)(now - e.timeS), e.boot, name(e.ev), e.a, e.b,
                   e.ev == static_cast<uint8_t>(Ev::STATE) ? stopReasonName(e.b) : "",
                   e.packV, e.soc);
    }
}

}  // namespace EventLog
