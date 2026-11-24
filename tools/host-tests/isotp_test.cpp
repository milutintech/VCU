// Host test of include/isotp.h: c++ -std=c++17 -I include tools/host-tests/isotp_test.cpp && ./a.out
#include "isotp.h"
#include <cstdio>
#include <vector>
#include <cstdlib>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

struct Bus {
    std::vector<std::vector<uint8_t>> sent;
    std::vector<uint8_t> msg; int msgs = 0; bool lastFunctional = false;
    bool full = false;
};
static bool sendFn(void* c, const uint8_t* d) {
    Bus* b = (Bus*)c; if (b->full) return false;
    b->sent.emplace_back(d, d + 8); return true;
}
static void msgFn(void* c, const uint8_t* d, size_t l, bool f) {
    Bus* b = (Bus*)c; b->msg.assign(d, d + l); b->msgs++; b->lastFunctional = f;
}

int main() {
    Bus bus; IsoTp t({&bus, sendFn, msgFn});

    // single frame in
    uint8_t sf[8] = {3, 0x22, 0xF1, 0x86, 0, 0, 0, 0};
    t.onFrame(sf, 8, false, 0);
    CHECK(bus.msgs == 1 && bus.msg.size() == 3 && bus.msg[0] == 0x22);
    // bad SF lengths
    uint8_t bad[8] = {0, 1}; t.onFrame(bad, 8, false, 0);
    uint8_t bad2[8] = {8, 1}; t.onFrame(bad2, 8, false, 0);
    CHECK(bus.msgs == 1);

    // multi-frame in: 10 bytes
    bus.sent.clear();
    uint8_t ff[8] = {0x10, 10, 1, 2, 3, 4, 5, 6};
    t.onFrame(ff, 8, false, 100);
    CHECK(bus.sent.size() == 1 && bus.sent[0][0] == 0x30 && bus.sent[0][1] == 0 && bus.sent[0][2] >= 2);
    uint8_t cf[8] = {0x21, 7, 8, 9, 10, 0, 0, 0};
    t.onFrame(cf, 8, false, 105);
    CHECK(bus.msgs == 2 && bus.msg.size() == 10 && bus.msg[9] == 10);
    // wrong sequence number aborts
    t.onFrame(ff, 8, false, 200);
    uint8_t cfbad[8] = {0x22, 7, 8, 9, 10};
    t.onFrame(cfbad, 8, false, 201);
    CHECK(!t.rxBusy());
    // RX timeout
    t.onFrame(ff, 8, false, 300);
    t.tick(300 + 1001);
    CHECK(!t.rxBusy());
    // functional first frame ignored
    t.onFrame(ff, 8, true, 400);
    CHECK(!t.rxBusy());
    // too long -> overflow FC
    bus.sent.clear();
    uint8_t big[8] = {0x1F, 0xFF, 0, 0, 0, 0, 0, 0};
    t.onFrame(big, 8, false, 500);
    CHECK(bus.sent.size() == 1 && bus.sent[0][0] == 0x32);

    // single frame out
    bus.sent.clear();
    uint8_t r[3] = {0x62, 0xF1, 0x86};
    CHECK(t.send(r, 3, 0));
    CHECK(bus.sent.size() == 1 && bus.sent[0][0] == 3 && bus.sent[0][4] == IsoTp::PAD);

    // multi-frame out: 20 bytes, STmin 0 asked -> we still wait 2 ms, BS 0
    bus.sent.clear();
    uint8_t m[20]; for (int i = 0; i < 20; i++) m[i] = i;
    CHECK(t.send(m, 20, 1000));
    CHECK(bus.sent.size() == 1 && bus.sent[0][0] == 0x10 && bus.sent[0][1] == 20);
    CHECK(!t.send(m, 20, 1000));             // busy
    t.tick(1005); CHECK(bus.sent.size() == 1);   // still waiting for FC
    uint8_t fc[8] = {0x30, 0, 0};
    t.onFrame(fc, 8, false, 1010);
    t.tick(1011); CHECK(bus.sent.size() == 1);   // STmin floor
    t.tick(1012); CHECK(bus.sent.size() == 2 && bus.sent[1][0] == 0x21 && bus.sent[1][1] == 6);
    t.tick(1013); CHECK(bus.sent.size() == 2);
    t.tick(1014); CHECK(bus.sent.size() == 3 && bus.sent[2][0] == 0x22);
    CHECK(!t.txBusy());                          // 6 + 7 + 7 = 20
    CHECK(bus.sent[2][1] == 13 && bus.sent[2][7] == 19);

    // block size 2, tester STmin 5 ms; TX buffers full once
    bus.sent.clear();
    uint8_t m2[40]; for (int i = 0; i < 40; i++) m2[i] = i;
    CHECK(t.send(m2, 40, 2000));
    uint8_t fc2[8] = {0x30, 2, 5};
    t.onFrame(fc2, 8, false, 2000);
    bus.full = true; t.tick(2005); CHECK(bus.sent.size() == 1);
    bus.full = false; t.tick(2006); CHECK(bus.sent.size() == 2);
    t.tick(2010); CHECK(bus.sent.size() == 2);
    t.tick(2011); CHECK(bus.sent.size() == 3);
    t.tick(2030); CHECK(bus.sent.size() == 3);   // block of 2 done, waits for FC
    t.onFrame(fc2, 8, false, 2031);
    t.tick(2036); CHECK(bus.sent.size() == 4);
    // N_Bs timeout
    bus.sent.clear();
    t.onFrame(fc2, 8, false, 2100);            // stray FC, ignored when not waiting
    uint8_t m3[10] = {0};
    // finish previous send first
    for (uint32_t ms = 2040; ms < 4000 && t.txBusy(); ms++) { t.tick(ms); if (ms % 20 == 0) t.onFrame(fc2, 8, false, ms); }
    CHECK(!t.txBusy());
    CHECK(t.send(m3, 10, 5000));
    t.tick(5000 + 1001); CHECK(!t.txBusy());
    // FC overflow aborts
    CHECK(t.send(m3, 10, 6000));
    uint8_t fco[8] = {0x32, 0, 0};
    t.onFrame(fco, 8, false, 6001); CHECK(!t.txBusy());

    printf(fails ? "FAILED (%d)\n" : "ok\n", fails);
    return fails ? 1 : 0;
}
