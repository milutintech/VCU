/**
 * @file uds_server.h
 * @brief Diagnostic interface over CAN: UDS (ISO 14229) on ISO-TP (isotp.h), on request only.
 *
 * For a scanner / diagnostic app on the vehicle CAN bus (same functions as LIVI: read live
 * values, read and reset faults, change parameters). Request 0x7E0 (physical) or 0x7DF
 * (functional, single frames), response 0x7E8. Nothing is sent cyclically except one small
 * "fault flag" frame (DIAG_STATUS, 1 Hz) so the app can show a banner without polling.
 *
 * Services: 0x10 session (default / extended 0x03, S3 5 s), 0x3E tester present,
 * 0x22 read DIDs (several per request), 0x2E write DID (extended session), 0x19 DTC
 * (01 count, 02 list, 04 snapshot, 06 extended data = occurrence counter + severity, 0A all
 * supported), 0x14 clear DTCs (extended session). Everything big endian, fixed scaling.
 * The table of DIDs, DTCs and scaling: docs/diag/vcu_diag_table.json (keep in sync with
 * uds_server.cpp).
 *
 * Tasks: onFrame() and tick() run in the CAN task (the only task that may send on the bus).
 * housekeeping() runs in the web task: it does the flash writes (config save, DTC store) so
 * the CAN task never blocks on NVS.
 */
#pragma once
#include <Arduino.h>
#include <mutex>
#include "isotp.h"
#include "config.h"
#include "error_monitor.h"

class CANManager;
class StateManager;
class VehicleControl;
class Configuration;

class UdsServer {
public:
    UdsServer(CANManager& can, ErrorMonitor& monitor, StateManager& state,
              VehicleControl& vehicle, Configuration& cfg);

    /** Load the stored DTCs from flash (setup, before the tasks run). */
    void begin();
    /** CAN task: a frame to DIAG_REQ / DIAG_FUNC. */
    void onFrame(uint32_t id, const uint8_t* buf, uint8_t len);
    /** CAN task, every loop pass: ISO-TP pacing, S3 timer, fault sync, status frame. */
    void tick();
    /** Web task: flash writes (config after a UDS write, DTC store). */
    void housekeeping();

    static constexpr size_t MAX_DTC = 32;
    static constexpr size_t SNAP_LEN = 13;     ///< freeze frame, see DID 0x0F00
    static constexpr uint32_t DTC_PREFIX = 0xE00000;   ///< DTC = 0xE0 | 16 bit ErrorCode
    static constexpr uint32_t S3_TIMEOUT_MS = 5000;

    struct Out;   // response writer (uds_server.cpp)

private:
    struct DidDef;   // table row (uds_server.cpp)
    static const DidDef* findDid(uint16_t did);

    struct Dtc {
        uint16_t code;            ///< ErrorCode
        uint16_t count;           ///< occurrences since the store was last cleared
        uint8_t severity;         ///< ErrorSeverity at the last occurrence
        uint8_t active;           ///< runtime only
        uint8_t snap[SNAP_LEN];   ///< values at the last occurrence
    };

    CANManager& can;
    ErrorMonitor& monitor;
    StateManager& state;
    VehicleControl& vehicle;
    Configuration& cfg;
    IsoTp isotp;

    uint8_t session = 0;                 ///< 0 default, 3 extended
    uint32_t lastReqMs = 0, lastSyncMs = 0, lastStatusMs = 0;
    uint16_t statusCounter = 0;
    uint8_t resp[IsoTp::MAX_LEN];

    // DTC store: written by the CAN task, copied to flash by the web task
    std::mutex dtcMutex;
    Dtc dtc[MAX_DTC];
    size_t dtcCount = 0;
    volatile bool dtcDirty = false;
    uint32_t lastPersistMs = 0;

    // Config writes: saved by the web task one second after the last write
    volatile uint8_t saveStatus = 0;     ///< 0 never, 1 pending, 2 ok, 3 failed (DID 0x0F10)
    volatile uint32_t lastCfgChangeMs = 0;
    void cfgChanged() { lastCfgChangeMs = millis(); saveStatus = 1; }

    void handle(const uint8_t* req, size_t len, bool functional);
    void reply(const uint8_t* data, size_t len);
    void negative(uint8_t sid, uint8_t nrc);
    uint8_t readDid(uint16_t did, Out& out);
    uint8_t writeDid(uint16_t did, const uint8_t* d, size_t n);
    void serviceDtc(const uint8_t* req, size_t len);
    void serviceClear(const uint8_t* req, size_t len);

    void syncFaults();
    void captureSnapshot(uint8_t* snap);
    void sendStatus();
    Dtc* findDtc(uint16_t code);
    static uint8_t dtcStatus(const Dtc& d) { return d.active ? 0x09 : 0x08; }

    static bool sendThunk(void* ctx, const uint8_t* d8);
    static void messageThunk(void* ctx, const uint8_t* d, size_t len, bool functional);
};
