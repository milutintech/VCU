/**
 * @file isotp.h
 * @brief ISO 15765-2 (ISO-TP) transport for classic CAN, 11 bit IDs, header only, no Arduino
 *        dependencies (host test: tools/host-tests/isotp_test.cpp).
 *
 * One request/response channel. All frames are padded to 8 bytes. Receiving multi-frame
 * messages sends a flow control with block size 0 and STmin RX_STMIN_MS (no bursts). Sending
 * honours the tester's flow control (block size, STmin) but never faster than MIN_TX_STMIN_MS.
 * Timeouts N_Bs / N_Cr: TIMEOUT_MS. The caller feeds frames with onFrame() and calls tick()
 * regularly (every CAN loop pass); everything runs in that one task, nothing is locked.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

class IsoTp {
public:
    static constexpr size_t   MAX_LEN = 512;           ///< longest message in either direction
    static constexpr uint8_t  PAD = 0xAA;
    static constexpr uint8_t  RX_BS = 0;               ///< block size we ask for (0 = no more FC)
    static constexpr uint8_t  RX_STMIN_MS = 2;         ///< STmin we ask for
    static constexpr uint8_t  MIN_TX_STMIN_MS = 2;     ///< our own floor for consecutive frames
    static constexpr uint32_t TIMEOUT_MS = 1000;
    static constexpr uint8_t  MAX_FC_WAITS = 10;

    struct Io {
        void* ctx;
        /** Send one 8 byte frame. false = no free TX buffer (a consecutive frame is retried). */
        bool (*send)(void* ctx, const uint8_t* data8);
        /** A complete message arrived. May call send() from inside. */
        void (*message)(void* ctx, const uint8_t* data, size_t len, bool functional);
    };

    explicit IsoTp(const Io& io) : io(io) {}

    /** True while a multi-frame message is being sent. */
    bool txBusy() const { return txActive; }
    bool rxBusy() const { return rxActive; }

    /** Feed one received frame (data of a request or flow-control frame). */
    void onFrame(const uint8_t* d, uint8_t len, bool functional, uint32_t nowMs) {
        if (len < 1) return;
        switch (d[0] >> 4) {
        case 0x0: {   // single frame
            const uint8_t l = d[0] & 0x0F;
            if (l == 0 || l > 7 || l > len - 1) return;
            rxActive = false;
            uint8_t tmp[7];
            memcpy(tmp, d + 1, l);
            io.message(io.ctx, tmp, l, functional);
            break;
        }
        case 0x1: {   // first frame
            if (functional || len < 8) return;     // multi-frame is physical only
            const size_t l = (static_cast<size_t>(d[0] & 0x0F) << 8) | d[1];
            if (l < 8) return;
            if (l > MAX_LEN) { sendFc(0x32); return; }   // overflow
            memcpy(rxBuf, d + 2, 6);
            rxLen = l;
            rxPos = 6;
            rxSn = 1;
            rxActive = true;
            rxSinceMs = nowMs;
            sendFc(0x30);
            break;
        }
        case 0x2: {   // consecutive frame
            if (!rxActive) return;
            if ((d[0] & 0x0F) != rxSn) { rxActive = false; return; }
            size_t n = rxLen - rxPos;
            if (n > 7) n = 7;
            if (n > static_cast<size_t>(len - 1)) { rxActive = false; return; }
            memcpy(rxBuf + rxPos, d + 1, n);
            rxPos += n;
            rxSn = (rxSn + 1) & 0x0F;
            rxSinceMs = nowMs;
            if (rxPos >= rxLen) {
                rxActive = false;
                io.message(io.ctx, rxBuf, rxLen, false);
            }
            break;
        }
        case 0x3: {   // flow control
            if (!txActive || !txWaitFc || len < 3) return;
            const uint8_t fs = d[0] & 0x0F;
            if (fs == 0) {
                txBlock = d[1];
                txBlockLeft = d[1];
                const uint8_t st = d[2];
                uint8_t ms = st <= 0x7F ? st : (st >= 0xF1 && st <= 0xF9 ? 1 : 127);
                if (ms < MIN_TX_STMIN_MS) ms = MIN_TX_STMIN_MS;
                txStMs = ms;
                txWaitFc = false;
                txNextMs = nowMs + txStMs;
                fcWaits = 0;
            } else if (fs == 1) {
                if (++fcWaits > MAX_FC_WAITS) txActive = false;
                txSinceMs = nowMs;
            } else {
                txActive = false;                  // overflow or unknown: give up
            }
            break;
        }
        default: break;
        }
    }

    /** Start sending a message. false: a multi-frame send is running, or too long. */
    bool send(const uint8_t* data, size_t len, uint32_t nowMs) {
        if (txActive || len == 0 || len > MAX_LEN) return false;
        uint8_t f[8];
        memset(f, PAD, 8);
        if (len <= 7) {
            f[0] = static_cast<uint8_t>(len);
            memcpy(f + 1, data, len);
            return io.send(io.ctx, f);
        }
        f[0] = 0x10 | static_cast<uint8_t>(len >> 8);
        f[1] = static_cast<uint8_t>(len & 0xFF);
        memcpy(f + 2, data, 6);
        if (!io.send(io.ctx, f)) return false;
        memcpy(txBuf, data, len);
        txLen = len;
        txPos = 6;
        txSn = 1;
        txActive = true;
        txWaitFc = true;
        txSinceMs = nowMs;
        fcWaits = 0;
        return true;
    }

    /** Timeouts and pacing of consecutive frames. Call every loop pass. */
    void tick(uint32_t nowMs) {
        if (rxActive && elapsed(nowMs, rxSinceMs) > TIMEOUT_MS) rxActive = false;
        if (!txActive) return;
        if (txWaitFc) {
            if (elapsed(nowMs, txSinceMs) > TIMEOUT_MS) txActive = false;
            return;
        }
        if (static_cast<int32_t>(nowMs - txNextMs) < 0) return;
        uint8_t f[8];
        memset(f, PAD, 8);
        f[0] = 0x20 | txSn;
        size_t n = txLen - txPos;
        if (n > 7) n = 7;
        memcpy(f + 1, txBuf + txPos, n);
        if (!io.send(io.ctx, f)) return;           // TX buffers full: try again next pass
        txPos += n;
        txSn = (txSn + 1) & 0x0F;
        txNextMs = nowMs + txStMs;
        if (txPos >= txLen) { txActive = false; return; }
        if (txBlock > 0 && --txBlockLeft == 0) {
            txWaitFc = true;
            txSinceMs = nowMs;
        }
    }

private:
    Io io;
    uint8_t rxBuf[MAX_LEN];
    size_t rxLen = 0, rxPos = 0;
    uint8_t rxSn = 0;
    bool rxActive = false;
    uint32_t rxSinceMs = 0;

    uint8_t txBuf[MAX_LEN];
    size_t txLen = 0, txPos = 0;
    uint8_t txSn = 0;
    bool txActive = false, txWaitFc = false;
    uint8_t txBlock = 0, txBlockLeft = 0, txStMs = MIN_TX_STMIN_MS, fcWaits = 0;
    uint32_t txSinceMs = 0, txNextMs = 0;

    static uint32_t elapsed(uint32_t now, uint32_t since) {
        const int32_t d = static_cast<int32_t>(now - since);
        return d < 0 ? 0 : static_cast<uint32_t>(d);
    }

    void sendFc(uint8_t fs) {
        uint8_t f[8];
        memset(f, PAD, 8);
        f[0] = fs;
        f[1] = RX_BS;
        f[2] = RX_STMIN_MS;
        io.send(io.ctx, f);
    }
};
