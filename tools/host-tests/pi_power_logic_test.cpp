// Host test for the Pi power sequence: c++ -std=c++17 -I include tools/host-tests/pi_power_logic_test.cpp && ./a.out
#include <cassert>
#include <cstdio>
#include "pi_power_logic.h"

using namespace PiPowerTiming;

// Steps the logic every 10 ms like the web task does, until `until` ms.
static PiPowerLogic::Out run(PiPowerLogic& l, uint32_t& t, uint32_t until, bool ign, bool ack = false) {
    PiPowerLogic::Out o{};
    for (; t < until; t += 10) o = l.update(t, ign, ack);
    return o;
}

static void powers_on_after_the_delays() {
    PiPowerLogic l; uint32_t t = 0;
    assert(!run(l, t, 2000, false).powerOn);
    assert(!run(l, t, 2900, true).powerOn);           // off time since boot not yet passed
    assert(run(l, t, 3500, true).powerOn);
    assert(l.state() == PiState::ON);
}

static void a_flicker_of_the_ignition_does_nothing() {
    PiPowerLogic l; uint32_t t = 0;
    run(l, t, 5000, true);
    assert(run(l, t, 7000, false).powerOn);           // 2 s off: still on
    assert(!run(l, t, 7100, true).shutdownRequested);
    assert(!run(l, t, 9000, false).shutdownRequested);
    assert(run(l, t, 9100, true).powerOn);
}

static void normal_shutdown_asks_waits_for_the_ack_and_cuts_20_s_later() {
    PiPowerLogic l; uint32_t t = 0;
    run(l, t, 5000, true);
    auto o = run(l, t, 5000 + IGNITION_OFF_DEBOUNCE_MS + 20, false);
    assert(o.powerOn && o.shutdownRequested && l.state() == PiState::REQUESTED);
    assert(!l.mayDeepSleep());
    uint32_t ackAt = t;
    o = run(l, t, ackAt + 10, false, true);
    assert(l.state() == PiState::ACKED && o.shutdownRequested);
    o = run(l, t, ackAt + POST_ACK_MS - 100, false);
    assert(o.powerOn && !l.mayDeepSleep());
    o = run(l, t, ackAt + POST_ACK_MS + 100, false);
    assert(!o.powerOn && !o.shutdownRequested && l.mayDeepSleep());
}

static void no_ack_means_the_power_is_cut_after_the_timeout() {
    PiPowerLogic l; uint32_t t = 0;
    run(l, t, 5000, true);
    run(l, t, 5000 + IGNITION_OFF_DEBOUNCE_MS + 20, false);
    uint32_t req = t;
    assert(run(l, t, req + ACK_TIMEOUT_MS - 100, false).powerOn);
    assert(!run(l, t, req + ACK_TIMEOUT_MS + 100, false).powerOn);
    assert(l.mayDeepSleep());
}

static void the_hard_limit_cuts_even_when_the_pi_keeps_acknowledging_late() {
    PiPowerLogic l; uint32_t t = 0;
    run(l, t, 5000, true);
    run(l, t, 5000 + IGNITION_OFF_DEBOUNCE_MS + 20, false);
    uint32_t req = t;
    run(l, t, req + 14000, false);                     // ack arrives just before the timeout
    run(l, t, req + 14010, false, true);
    assert(l.state() == PiState::ACKED);
    // POST_ACK_MS (50 s) after 14 s would be 64 s, inside the 70 s limit.
    assert(!run(l, t, req + 14010 + POST_ACK_MS + 100, false).powerOn);
    assert(t - req < HARD_MAX_MS);
}

static void a_quick_power_cycle_shuts_the_pi_down_and_brings_it_back() {
    PiPowerLogic l; uint32_t t = 0;
    run(l, t, 5000, true);
    run(l, t, 5000 + IGNITION_OFF_DEBOUNCE_MS + 20, false);   // key off long enough to request
    assert(l.state() == PiState::REQUESTED);
    auto o = run(l, t, t + 2000, true);                       // key back on during the shutdown
    assert(o.powerOn && o.shutdownRequested);                 // the shutdown still runs through
    uint32_t ackAt = t;
    run(l, t, ackAt + 10, true, true);
    o = run(l, t, ackAt + POST_ACK_MS + 100, true);           // cut ...
    assert(!o.powerOn);
    o = run(l, t, t + MIN_OFF_MS + POWER_ON_DELAY_MS + 200, true);   // ... and on again
    assert(o.powerOn && !o.shutdownRequested && l.state() == PiState::ON);
}

static void power_is_never_back_before_the_minimum_off_time() {
    PiPowerLogic l; uint32_t t = 0;
    run(l, t, 5000, true);
    run(l, t, 5000 + IGNITION_OFF_DEBOUNCE_MS + 20, false);
    run(l, t, t + ACK_TIMEOUT_MS + 100, true);                // cut at about this moment
    uint32_t cutAt = t;
    assert(!run(l, t, cutAt + MIN_OFF_MS - 200, true).powerOn);
    assert(run(l, t, cutAt + MIN_OFF_MS + POWER_ON_DELAY_MS + 100, true).powerOn);
}

static void a_stale_ack_does_not_count_for_the_next_request() {
    PiPowerLogic l; uint32_t t = 0;
    run(l, t, 5000, true);
    run(l, t, t + 100, true, true);                           // ack while ON is ignored
    assert(l.state() == PiState::ON);
}

// Like run(), but by duration, so it can step across the 32-bit clock wrap.
static void run_for(PiPowerLogic& l, uint32_t& t, uint32_t ms, bool ign) {
    for (uint32_t done = 0; done < ms; done += 10, t += 10) l.update(t, ign, false);
}

static void the_clock_wrapping_around_does_not_break_the_timers() {
    PiPowerLogic l; uint32_t t = 0xFFFFFFFFu - 6000;          // 49.7 days of uptime
    run_for(l, t, 5000, true);
    assert(l.state() == PiState::ON);
    run_for(l, t, IGNITION_OFF_DEBOUNCE_MS + 100, false);     // crosses the wrap
    assert(l.state() == PiState::REQUESTED);
}

int main() {
    powers_on_after_the_delays();
    a_flicker_of_the_ignition_does_nothing();
    normal_shutdown_asks_waits_for_the_ack_and_cuts_20_s_later();
    no_ack_means_the_power_is_cut_after_the_timeout();
    the_hard_limit_cuts_even_when_the_pi_keeps_acknowledging_late();
    a_quick_power_cycle_shuts_the_pi_down_and_brings_it_back();
    power_is_never_back_before_the_minimum_off_time();
    a_stale_ack_does_not_count_for_the_next_request();
    the_clock_wrapping_around_does_not_break_the_timers();
    std::puts("pi power logic: all tests passed");
    return 0;
}
