// c++ -std=c++17 -I include tools/host-tests/dcl_pulse_logic_test.cpp && ./a.out
#include <cassert>
#include <cstdio>
#include "dcl_pulse_logic.h"

using namespace DclPulse;

static uint32_t run(DclPulseLogic& l, uint32_t& t, uint32_t ms, uint32_t dcl, int32_t cur, uint32_t maxA) {
    uint32_t out = 0;
    for (uint32_t i = 0; i < ms / 50; i++, t += 50) out = l.update(t, dcl, cur, maxA);
    return out;
}

int main() {
    {   // DCL 300 A, DMC 450 A: pulse to 450 A, 60 s, then down to the DCL
        DclPulseLogic l; uint32_t t = 0;
        assert(l.update(t, 300, 0, 450) == 450);        // pedal down: headroom is already there
        assert(run(l, t, PULSE_MS - TAPER_MS - 100, 300, 440, 450) == 450);
        const uint32_t mid = run(l, t, TAPER_MS / 2 + 100, 300, 440, 450);
        assert(mid > 300 && mid < 450);                 // in the taper
        assert(run(l, t, TAPER_MS, 300, 440, 450) == 300);
    }
    {   // the taper never steps: each sample at most a small drop
        DclPulseLogic l; uint32_t t = 0, prev = 450;
        for (uint32_t i = 0; i < 1400; i++, t += 50) {
            const uint32_t o = l.update(t, 300, 440, 450);
            assert(o <= prev && prev - o <= 5);
            prev = o;
        }
        assert(prev == 300);
    }
    {   // never above the DMC limit, not above 200 % of a low DCL
        DclPulseLogic l; uint32_t t = 0;
        assert(l.update(t, 1000, 0, 450) == 450);
        DclPulseLogic low;
        assert(low.update(t, 100, 0, 450) == 200);
    }
    {   // DCL 0 A = 0 A, also with budget left
        DclPulseLogic l; uint32_t t = 0;
        assert(l.update(t, 0, 100, 450) == 0);
    }
    {   // budget comes back 1:1 below the DCL, not while sitting at the DCL
        DclPulseLogic l; uint32_t t = 0;
        run(l, t, PULSE_MS + 1000, 300, 440, 450);
        assert(l.remainingMs() == 0);
        assert(run(l, t, 30000, 300, 295, 450) == 300);  // at the limit: no recovery
        assert(l.remainingMs() == 0);
        run(l, t, 30000, 300, 100, 450);
        assert(l.remainingMs() >= 29000 && l.remainingMs() <= 31000);
        assert(run(l, t, 100, 300, 100, 450) == 450);
    }
    {   // pulses with a DCL above the DMC limit are simply the DMC limit
        DclPulseLogic l; uint32_t t = 0;
        assert(run(l, t, PULSE_MS * 2, 800, 449, 450) == 450);
    }
    {   // millis() wrap
        DclPulseLogic l; uint32_t t = 0xFFFFF000u;
        assert(run(l, t, 10000, 300, 440, 450) == 450);
        assert(l.remainingMs() < PULSE_MS - 9000);
    }
    puts("dcl pulse logic: all tests passed");
}
