/**
 * @file uds_server.cpp
 * @brief UDS diagnostic server (see uds_server.h). DID/DTC table: docs/diag/vcu_diag_table.json
 */
#include "uds_server.h"
#include <Preferences.h>
#include <cmath>
#include "can_manager.h"
#include "state_manager.h"
#include "vehicle_control.h"
#include "configuration.h"
#include "vehicle_parameters.h"

namespace {

// UDS negative response codes
constexpr uint8_t NRC_SERVICE_NOT_SUPPORTED = 0x11;
constexpr uint8_t NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12;
constexpr uint8_t NRC_BAD_LENGTH = 0x13;
constexpr uint8_t NRC_RESPONSE_TOO_LONG = 0x14;
constexpr uint8_t NRC_REQUEST_OUT_OF_RANGE = 0x31;
constexpr uint8_t NRC_NOT_IN_SESSION = 0x7F;   // serviceNotSupportedInActiveSession

constexpr uint8_t SESSION_DEFAULT = 0x01;
constexpr uint8_t SESSION_EXTENDED = 0x03;

constexpr uint8_t DTC_AVAILABILITY_MASK = 0x09;   // bit0 testFailed, bit3 confirmedDTC
constexpr uint8_t SNAPSHOT_RECORD = 0x01;
constexpr uint16_t DID_SNAPSHOT = 0x0F00;

constexpr uint16_t DID_RAW_BASE = 0x4000;         // + CAN ID (0..0x7FF): last frame of that ID
constexpr uint16_t DID_RAW_END = 0x47FF;

const char* const SW_VERSION = "VCU " __DATE__ " " __TIME__;

// Every ErrorCode, for 0x19 0A (supported DTCs). Same list as docs/diag/vcu_diag_table.json.
const ErrorCode ALL_CODES[] = {
    ErrorCode::SYSTEM_BOOT, ErrorCode::SYSTEM_SHUTDOWN, ErrorCode::WATCHDOG_RESET, ErrorCode::MEMORY_ERROR,
    ErrorCode::LV_BATTERY_LOW, ErrorCode::CAN_INIT_FAILED, ErrorCode::CAN_BUS_OFF, ErrorCode::BMS_TIMEOUT,
    ErrorCode::DMC_TIMEOUT, ErrorCode::BSC_TIMEOUT, ErrorCode::NLG_TIMEOUT, ErrorCode::BATTERY_UNDERVOLTAGE,
    ErrorCode::BATTERY_OVERVOLTAGE, ErrorCode::BATTERY_OVERCURRENT, ErrorCode::PRECHARGE_FAILED,
    ErrorCode::PRECHARGE_TIMEOUT, ErrorCode::SOC_OUT_OF_RANGE, ErrorCode::BMS_DISCHARGE_NOT_ALLOWED,
    ErrorCode::BMS_CHARGE_NOT_ALLOWED, ErrorCode::BMS_FLAGS_IGNORED, ErrorCode::MOTOR_OVERTEMP,
    ErrorCode::INVERTER_OVERTEMP, ErrorCode::DMC_ERROR, ErrorCode::TORQUE_LIMIT_EXCEEDED,
    ErrorCode::SPEED_LIMIT_EXCEEDED, ErrorCode::CHARGER_ERROR, ErrorCode::CHARGER_OVERTEMP,
    ErrorCode::CONNECTOR_ERROR, ErrorCode::CHARGE_CURRENT_ERROR, ErrorCode::CONFIG_INVALID,
    ErrorCode::CONFIG_SAVE_FAILED, ErrorCode::CONFIG_LOAD_FAILED, ErrorCode::CALIBRATION_ERROR,
    ErrorCode::GEAR_SHIFT_ERROR, ErrorCode::PEDAL_ERROR, ErrorCode::USER_ABORT,
};

// --- scaling helpers: round, saturate, NaN = 0 ---------------------------------------------
uint16_t u16s(float v, float scale = 1.0f) {
    const float x = v * scale;
    if (!(x == x) || x <= 0.0f) return 0;
    if (x >= 65535.0f) return 65535;
    return static_cast<uint16_t>(lroundf(x));
}
int16_t i16s(float v, float scale = 1.0f) {
    const float x = v * scale;
    if (!(x == x)) return 0;
    if (x <= -32768.0f) return -32768;
    if (x >= 32767.0f) return 32767;
    return static_cast<int16_t>(lroundf(x));
}
int8_t i8s(float v) {
    if (!(v == v)) return 0;
    if (v <= -128.0f) return -128;
    if (v >= 127.0f) return 127;
    return static_cast<int8_t>(lroundf(v));
}
uint16_t ageU16(unsigned long lastMs, unsigned long now) {   // 0xFFFF = never / older than 65 s
    if (lastMs == 0) return 0xFFFF;
    const unsigned long a = msSince(lastMs, now);
    return a >= 0xFFFF ? 0xFFFF : static_cast<uint16_t>(a);
}
uint16_t be16(const uint8_t* d) { return (static_cast<uint16_t>(d[0]) << 8) | d[1]; }

}  // namespace

/** Big endian response writer. Overflow is remembered, never written past the buffer. */
struct UdsServer::Out {
    uint8_t* p;
    size_t cap;
    size_t n = 0;
    bool ovf = false;
    Out(uint8_t* p, size_t cap) : p(p), cap(cap) {}
    void u8(uint8_t v) { if (n < cap) p[n++] = v; else ovf = true; }
    void u16(uint16_t v) { u8(v >> 8); u8(v & 0xFF); }
    void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
    void u32(uint32_t v) { u16(v >> 16); u16(v & 0xFFFF); }
    void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
};

/** One DID: reader (always), writer (optional, exact length wlen). Both return 0 or an NRC. */
struct UdsServer::DidDef {
    uint16_t id;
    uint8_t (*read)(UdsServer&, Out&);
    uint8_t wlen;
    uint8_t (*write)(UdsServer&, const uint8_t*);
};

// =============================================================================================
// DID table. Scaling is fixed and listed in docs/diag/vcu_diag_table.json.
// Lambdas are written inside a member function so they may use the private members.
// =============================================================================================
const UdsServer::DidDef* UdsServer::findDid(uint16_t did) {
    using S = UdsServer;
    static const DidDef table[] = {
        // ---- identification ---------------------------------------------------------------
        {0xF186, [](S& s, Out& o) -> uint8_t { o.u8(s.session == 3 ? SESSION_EXTENDED : SESSION_DEFAULT); return 0; }, 0, nullptr},
        {0xF189, [](S&, Out& o) -> uint8_t { for (const char* c = SW_VERSION; *c; c++) o.u8(*c); return 0; }, 0, nullptr},

        // ---- VCU ----------------------------------------------------------------------------
        {0x0100, [](S& s, Out& o) -> uint8_t {   // state, gear, drive mode, flags, uptime
            o.u8(static_cast<uint8_t>(s.state.getCurrentState()));
            o.u8(static_cast<uint8_t>(s.vehicle.getCurrentGear()));
            o.u8(static_cast<uint8_t>(s.cfg.getDriveMode()));
            uint8_t f = 0;
            if (s.state.isBatteryArmed()) f |= 0x01;
            if (s.state.isPreCharged()) f |= 0x02;
            if (s.can.isBSCEnabled()) f |= 0x04;
            if (s.can.isBSCBoost()) f |= 0x08;
            if (s.can.getIgnoreAllowFlags()) f |= 0x10;
            o.u8(f);
            o.u32(millis() / 1000);
            return 0; }, 0, nullptr},
        {0x0101, [](S& s, Out& o) -> uint8_t {   // limits sent to the DMC, BSC HV target
            o.u16(u16s(s.can.getDmcDriveLimitA()));
            o.u16(u16s((s.can.getPulseRemainingMs() + 999) / 1000));
            o.u16(s.can.getHVVoltageDemand());
            return 0; }, 0, nullptr},
        {0x0102, [](S& s, Out& o) -> uint8_t {   // CAN receive statistics
            o.u32(s.can.getRxCount());
            o.u32(s.can.getRxOverflowCount());
            o.u16(u16s(s.can.getRxQueueDrops()));
            o.u16(u16s(s.can.getOtherIdFrames()));
            return 0; }, 0, nullptr},
        {0x0103, [](S& s, Out& o) -> uint8_t {   // age of the last frame per node (ms)
            const unsigned long now = millis();
            o.u16(ageU16(s.can.getLastBMSMs(), now));
            o.u16(ageU16(s.can.getLastBMSFlagsMs(), now));
            o.u16(ageU16(s.can.getLastDMCMs(), now));
            o.u16(ageU16(s.can.getLastBSCMs(), now));
            o.u16(ageU16(s.can.getLastNLGMs(), now));
            return 0; }, 0, nullptr},
        {0x0104, [](S& s, Out& o) -> uint8_t {   // CAN timing maxima of the last 10 s window (us)
            const CANManager::CanTiming& t = s.can.lastTiming();
            o.u16(u16s(t.intLatencyUs)); o.u16(u16s(t.spiWaitUs)); o.u16(u16s(t.txUs));
            o.u16(u16s(t.loopUs)); o.u16(u16s(t.txFails));
            return 0; }, 0, nullptr},

        // ---- BMS (Orion) ---------------------------------------------------------------------
        {0x0200, [](S& s, Out& o) -> uint8_t {   // 0x010: SOC, voltage, current, DCL, CCL
            const BMSData& b = s.can.getBMSData();
            o.u8(b.soc);
            o.u16(b.voltage);
            o.i16(b.current);          // 0.1 A, negative = charging
            o.u16(b.maxDischarge);
            o.u16(b.maxCharge);
            return 0; }, 0, nullptr},
        {0x0201, [](S& s, Out& o) -> uint8_t {   // 0x001: temps, cell min/max, allow flags
            const BMSData& b = s.can.getBMSData();
            o.i8(b.tempHigh); o.i8(b.tempLow);
            o.u16(b.cellLowDeciMv); o.u16(b.cellHighDeciMv);
            uint8_t f = 0;
            if (b.chargeRelay) f |= 0x01;
            if (b.dischargeRelay) f |= 0x02;
            if (s.can.chargeAllowed()) f |= 0x04;
            if (s.can.dischargeAllowed()) f |= 0x08;
            if (s.can.bmsPackFresh()) f |= 0x10;
            if (s.can.bmsFlagsFresh()) f |= 0x20;
            o.u8(f);
            return 0; }, 0, nullptr},

        // ---- DMC (inverter) ------------------------------------------------------------------
        {0x0300, [](S& s, Out& o) -> uint8_t {   // 0x258
            const DMCData& d = s.can.getDMCData();
            o.u8((d.ready ? 0x01 : 0) | (d.running ? 0x02 : 0));
            o.i16(i16s(d.torqueAvailable, 10)); o.i16(i16s(d.torqueActual, 10));
            o.i16(i16s(d.speedActual));
            return 0; }, 0, nullptr},
        {0x0301, [](S& s, Out& o) -> uint8_t {   // 0x259
            const DMCData& d = s.can.getDMCData();
            o.u16(u16s(d.dcVoltageAct, 10)); o.i16(i16s(d.dcCurrentAct, 10));
            o.u16(u16s(d.acCurrentAct, 10));
            o.u32(static_cast<uint32_t>(d.mechPower));
            return 0; }, 0, nullptr},
        {0x0302, [](S& s, Out& o) -> uint8_t {   // 0x458
            const DMCData& d = s.can.getDMCData();
            o.i16(i16s(d.tempInverter, 10)); o.i16(i16s(d.tempMotor, 10));
            o.i8(d.tempSystem);
            return 0; }, 0, nullptr},

        // ---- BSC (DC-DC) ---------------------------------------------------------------------
        {0x0400, [](S& s, Out& o) -> uint8_t {   // 0x26A
            const BSCData& b = s.can.getBSCData();
            o.u16(u16s(b.hvVoltageAct, 10)); o.u16(u16s(b.lvVoltageAct, 10));
            o.i16(i16s(b.hvCurrentAct, 10)); o.i16(i16s(b.lvCurrentAct));
            o.u8(b.mode);
            o.u8((s.can.isBSCEnabled() ? 0x01 : 0) | (s.can.isBSCBoost() ? 0x02 : 0));
            return 0; }, 0, nullptr},

        // ---- NLG (charger) -------------------------------------------------------------------
        {0x0500, [](S& s, Out& o) -> uint8_t {   // 0x728 (voltage/current raw, see table)
            const NLGData& n = s.can.getNLGData();
            o.u8(n.stateAct); o.u8(n.stateActRaw); o.u8(n.stateCtrlPilot);
            o.u8((n.connectorLocked ? 0x01 : 0) | (n.unlockRequest ? 0x02 : 0));
            o.u16(n.dcHvVoltageAct); o.u16(n.dcHvCurrentAct);
            o.u8(n.stateDemand); o.u8(n.ledDemand);
            return 0; }, 0, nullptr},
        {0x0501, [](S& s, Out& o) -> uint8_t {   // 0x739
            const NLGData& n = s.can.getNLGData();
            o.u8(n.coolingRequest); o.i16(i16s(n.tempCoolPlate, 10));
            return 0; }, 0, nullptr},

        // ---- faults / bookkeeping --------------------------------------------------------------
        {0x0800, [](S& s, Out& o) -> uint8_t {   // active count, stored count, highest active severity
            std::lock_guard<std::mutex> lk(s.dtcMutex);
            uint8_t active = 0, sev = 0xFF;
            for (size_t i = 0; i < s.dtcCount; i++) {
                if (!s.dtc[i].active) continue;
                active++;
                if (sev == 0xFF || s.dtc[i].severity > sev) sev = s.dtc[i].severity;
            }
            o.u8(active); o.u8(static_cast<uint8_t>(s.dtcCount)); o.u8(sev);
            return 0; }, 0, nullptr},
        {0x0F10, [](S& s, Out& o) -> uint8_t { o.u8(s.saveStatus); return 0; }, 0, nullptr},   // config save

        // ---- all CAN IDs seen on the bus: count of frames since boot --------------------------
        {0x3F00, [](S& s, Out& o) -> uint8_t {
            size_t n = 0;
            const CANManager::IdStat* st = s.can.idStats(n);
            o.u8(static_cast<uint8_t>(n));
            for (size_t i = 0; i < n; i++) { o.u16(static_cast<uint16_t>(st[i].id)); o.u32(st[i].count); }
            return 0; }, 0, nullptr},

        // ---- configuration (same values and limits as LIVI; writes need the extended session) --
        {0x2001, [](S& s, Out& o) -> uint8_t { o.u8(static_cast<uint8_t>(s.cfg.getDriveMode())); return 0; },
         1, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setDriveModeFromByte(d[0])) return NRC_REQUEST_OUT_OF_RANGE;
            s.vehicle.setDrivingMode(s.cfg.getDriveMode());
            s.cfgChanged(); return 0; }},
        {0x2002, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getMaxTorque())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setMaxTorque(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2003, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getMaxReverseTorque())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setMaxReverseTorque(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2004, [](S& s, Out& o) -> uint8_t { o.u8(s.cfg.getMaxSOC()); return 0; },
         1, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setMaxSOC(d[0])) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2005, [](S& s, Out& o) -> uint8_t { o.u8(s.cfg.getMaxChargingCurrent()); return 0; },
         1, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setMaxChargingCurrent(d[0])) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},

        {0x2010, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getRegenZoneEnd(), 10)); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setRegenZoneEnd(be16(d) / 10.0f)) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2011, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getCoastZoneEnd(), 10)); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setCoastZoneEnd(be16(d) / 10.0f)) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2012, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getRegenProgression(), 100)); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setRegenProgression(be16(d) / 100.0f)) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2013, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getAccelProgression(), 100)); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setAccelProgression(be16(d) / 100.0f)) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},

        {0x2020, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getRegenEngageTime())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setRegenEngageTime(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.vehicle.setRegenEngageTime(s.cfg.getRegenEngageTime());
            s.cfgChanged(); return 0; }},
        {0x2021, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getRegenReleaseTime())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setRegenReleaseTime(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.vehicle.setRegenReleaseTime(s.cfg.getRegenReleaseTime());
            s.cfgChanged(); return 0; }},
        {0x2022, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getPowerEngageTime())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setPowerEngageTime(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.vehicle.setPowerEngageTime(s.cfg.getPowerEngageTime());
            s.cfgChanged(); return 0; }},
        {0x2023, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getPowerReleaseTime())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setPowerReleaseTime(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.vehicle.setPowerReleaseTime(s.cfg.getPowerReleaseTime());
            s.cfgChanged(); return 0; }},
        {0x2024, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getCrossoverTime())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setCrossoverTime(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.vehicle.setCrossoverTime(s.cfg.getCrossoverTime());
            s.cfgChanged(); return 0; }},

        {0x2030, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getThrottleMinADC())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setThrottleMinADC(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2031, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getThrottleMaxADC())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setThrottleMaxADC(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},

        {0x2040, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getBaseSpeed())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setBaseSpeed(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2041, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getDeltaSpeed())); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setDeltaSpeed(be16(d))) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2042, [](S& s, Out& o) -> uint8_t { o.u16(u16s(s.cfg.getNominalPower(), 10)); return 0; },
         2, [](S& s, const uint8_t* d) -> uint8_t {
            if (!s.cfg.setNominalPower(be16(d) / 10.0f)) return NRC_REQUEST_OUT_OF_RANGE;
            s.cfgChanged(); return 0; }},
        {0x2043, [](S& s, Out& o) -> uint8_t {   // 5 zones, 0.1 %
            for (int i = 0; i < 5; i++) o.u16(u16s(s.cfg.getDrivePowerLimits()[i], 10));
            return 0; },
         10, [](S& s, const uint8_t* d) -> uint8_t {
            float old[5];
            memcpy(old, s.cfg.getDrivePowerLimits(), sizeof(old));
            for (int i = 0; i < 5; i++) {
                if (!s.cfg.setDrivePowerLimit(i, be16(d + 2 * i) / 10.0f)) {
                    for (int j = 0; j < i; j++) s.cfg.setDrivePowerLimit(j, old[j]);   // all or nothing
                    return NRC_REQUEST_OUT_OF_RANGE;
                }
            }
            s.cfgChanged(); return 0; }},
        {0x2044, [](S& s, Out& o) -> uint8_t {
            for (int i = 0; i < 5; i++) o.u16(u16s(s.cfg.getRegenPowerLimits()[i], 10));
            return 0; },
         10, [](S& s, const uint8_t* d) -> uint8_t {
            float old[5];
            memcpy(old, s.cfg.getRegenPowerLimits(), sizeof(old));
            for (int i = 0; i < 5; i++) {
                if (!s.cfg.setRegenPowerLimit(i, be16(d + 2 * i) / 10.0f)) {
                    for (int j = 0; j < i; j++) s.cfg.setRegenPowerLimit(j, old[j]);
                    return NRC_REQUEST_OUT_OF_RANGE;
                }
            }
            s.cfgChanged(); return 0; }},

        // Runtime only (never saved), like /api/config/bms: every boot starts with the flags enforced.
        {0x2050, [](S& s, Out& o) -> uint8_t { o.u8(s.can.getIgnoreAllowFlags() ? 1 : 0); return 0; },
         1, [](S& s, const uint8_t* d) -> uint8_t {
            if (d[0] > 1) return NRC_REQUEST_OUT_OF_RANGE;
            const bool ignore = d[0] == 1;
            if (ignore != s.can.getIgnoreAllowFlags()) {
                s.monitor.logWarning(ignore ? "BMS allow flags override ON (UDS)" : "BMS allow flags override OFF (UDS)", "BMS");
            }
            s.can.setIgnoreAllowFlags(ignore);
            return 0; }},
    };
    for (const DidDef& d : table) {
        if (d.id == did) return &d;
    }
    return nullptr;
}

// =============================================================================================

UdsServer::UdsServer(CANManager& can, ErrorMonitor& monitor, StateManager& state,
                     VehicleControl& vehicle, Configuration& cfg)
    : can(can), monitor(monitor), state(state), vehicle(vehicle), cfg(cfg),
      isotp(IsoTp::Io{this, &UdsServer::sendThunk, &UdsServer::messageThunk}) {
    memset(dtc, 0, sizeof(dtc));
}

bool UdsServer::sendThunk(void* ctx, const uint8_t* d8) {
    return static_cast<UdsServer*>(ctx)->can.sendFrame(CANIds::DIAG_RESP, d8, 8);
}

void UdsServer::messageThunk(void* ctx, const uint8_t* d, size_t len, bool functional) {
    static_cast<UdsServer*>(ctx)->handle(d, len, functional);
}

void UdsServer::onFrame(uint32_t id, const uint8_t* buf, uint8_t len) {
    isotp.onFrame(buf, len, id == CANIds::DIAG_FUNC, millis());
}

void UdsServer::reply(const uint8_t* data, size_t len) {
    isotp.send(data, len, millis());
}

void UdsServer::negative(uint8_t sid, uint8_t nrc) {
    const uint8_t r[3] = {0x7F, sid, nrc};
    reply(r, 3);
}

// ---- request handling --------------------------------------------------------------------

void UdsServer::handle(const uint8_t* req, size_t len, bool functional) {
    if (len < 1) return;
    // A multi-frame answer is still going out: a new request cannot be answered now, drop it.
    if (isotp.txBusy()) return;

    const uint32_t now = millis();
    lastReqMs = now;
    const uint8_t sid = req[0];

    // Functional addressing: stay quiet instead of answering "not supported" (many nodes share it).
    auto neg = [&](uint8_t nrc) {
        if (functional && (nrc == NRC_SERVICE_NOT_SUPPORTED || nrc == NRC_SUBFUNCTION_NOT_SUPPORTED ||
                           nrc == NRC_REQUEST_OUT_OF_RANGE || nrc == NRC_NOT_IN_SESSION)) return;
        negative(sid, nrc);
    };

    switch (sid) {
    case 0x10: {   // DiagnosticSessionControl
        if (len != 2) return neg(NRC_BAD_LENGTH);
        const uint8_t sub = req[1] & 0x7F;
        if (sub != SESSION_DEFAULT && sub != SESSION_EXTENDED) return neg(NRC_SUBFUNCTION_NOT_SUPPORTED);
        const uint8_t newSession = sub == SESSION_EXTENDED ? 3 : 0;
        if (newSession != session) {
            session = newSession;
            monitor.logInfo(session ? "UDS extended session started" : "UDS back to default session", "UDS");
        }
        if (req[1] & 0x80) return;
        const uint8_t r[6] = {0x50, sub, 0x00, 0x32, 0x01, 0xF4};   // P2 50 ms, P2* 5000 ms
        reply(r, 6);
        return;
    }
    case 0x3E: {   // TesterPresent (lastReqMs above keeps the session alive)
        if (len != 2) return neg(NRC_BAD_LENGTH);
        if ((req[1] & 0x7F) != 0) return neg(NRC_SUBFUNCTION_NOT_SUPPORTED);
        if (req[1] & 0x80) return;
        const uint8_t r[2] = {0x7E, 0x00};
        reply(r, 2);
        return;
    }
    case 0x22: {   // ReadDataByIdentifier, up to 16 DIDs
        if (len < 3 || (len - 1) % 2 != 0 || len > 1 + 2 * 16) return neg(NRC_BAD_LENGTH);
        Out out(resp, sizeof(resp));
        out.u8(0x62);
        for (size_t i = 1; i + 1 < len; i += 2) {
            const uint16_t did = be16(req + i);
            out.u16(did);
            const uint8_t nrc = readDid(did, out);
            if (nrc) return neg(nrc);
        }
        if (out.ovf) return neg(NRC_RESPONSE_TOO_LONG);
        reply(resp, out.n);
        return;
    }
    case 0x2E: {   // WriteDataByIdentifier
        if (len < 4) return neg(NRC_BAD_LENGTH);
        if (session != 3) return neg(NRC_NOT_IN_SESSION);
        const uint16_t did = be16(req + 1);
        const uint8_t nrc = writeDid(did, req + 3, len - 3);
        if (nrc) return neg(nrc);
        char msg[48];
        snprintf(msg, sizeof(msg), "UDS write DID 0x%04X (%u bytes)", did, (unsigned)(len - 3));
        monitor.logInfo(msg, "UDS");
        const uint8_t r[3] = {0x6E, req[1], req[2]};
        reply(r, 3);
        return;
    }
    case 0x19:
        serviceDtc(req, len);
        return;
    case 0x14:
        serviceClear(req, len);
        return;
    default:
        return neg(NRC_SERVICE_NOT_SUPPORTED);
    }
}

uint8_t UdsServer::readDid(uint16_t did, Out& out) {
    if (did >= DID_RAW_BASE && did <= DID_RAW_END) {   // last frame of CAN ID (did - 0x4000)
        uint8_t data[8] = {0};
        uint8_t l = 0;
        uint32_t age = 0;
        if (!can.rawFrame(did - DID_RAW_BASE, data, l, age)) return NRC_REQUEST_OUT_OF_RANGE;
        out.u16(age >= 0xFFFF ? 0xFFFF : static_cast<uint16_t>(age));
        out.u8(l);
        for (int i = 0; i < 8; i++) out.u8(data[i]);
        return 0;
    }
    const DidDef* d = findDid(did);
    if (!d) return NRC_REQUEST_OUT_OF_RANGE;
    return d->read(*this, out);
}

uint8_t UdsServer::writeDid(uint16_t did, const uint8_t* data, size_t n) {
    const DidDef* d = findDid(did);
    if (!d || !d->write) return NRC_REQUEST_OUT_OF_RANGE;
    if (n != d->wlen) return NRC_BAD_LENGTH;
    return d->write(*this, data);
}

// ---- DTCs ---------------------------------------------------------------------------------

UdsServer::Dtc* UdsServer::findDtc(uint16_t code) {
    for (size_t i = 0; i < dtcCount; i++) {
        if (dtc[i].code == code) return &dtc[i];
    }
    return nullptr;
}

void UdsServer::captureSnapshot(uint8_t* snap) {
    const BMSData& b = can.getBMSData();
    const DMCData& d = can.getDMCData();
    Out o(snap, SNAP_LEN);
    o.u16(b.voltage);
    o.i16(b.current);
    o.u8(b.soc);
    o.i16(i16s(d.speedActual));
    o.i8(i8s(d.tempInverter));
    o.i8(i8s(d.tempMotor));
    o.u16(u16s(can.getBSCData().lvVoltageAct, 10));
    o.u8(static_cast<uint8_t>(state.getCurrentState()));
    o.u8(static_cast<uint8_t>(vehicle.getCurrentGear()));
}

/**
 * Compare the ErrorMonitor fault list with the DTC store. A fault that is active now and was not
 * at the last pass is a new occurrence: count + snapshot. Checked every 100 ms, so an activation
 * shorter than that can be missed (the ErrorMonitor still counts it).
 */
void UdsServer::syncFaults() {
    FaultBrief fb[ErrorMonitor::MAX_FAULTS];
    const size_t n = monitor.snapshotFaults(fb, ErrorMonitor::MAX_FAULTS);

    std::lock_guard<std::mutex> lk(dtcMutex);
    bool seen[MAX_DTC] = {};
    for (size_t i = 0; i < n; i++) {
        if (fb[i].code == ErrorCode::SYSTEM_BOOT) continue;
        const uint16_t code = static_cast<uint16_t>(fb[i].code);
        Dtc* r = findDtc(code);
        if (!r) {
            if (!fb[i].active || dtcCount >= MAX_DTC) continue;   // nothing to store, or store full
            r = &dtc[dtcCount++];
            memset(r, 0, sizeof(*r));
            r->code = code;
        }
        if (fb[i].active && !r->active) {
            if (r->count < UINT16_MAX) r->count++;
            r->severity = static_cast<uint8_t>(fb[i].severity);
            captureSnapshot(r->snap);
            dtcDirty = true;
        }
        r->active = fb[i].active ? 1 : 0;
        seen[r - dtc] = true;
    }
    for (size_t i = 0; i < dtcCount; i++) {
        if (!seen[i]) dtc[i].active = 0;   // no longer in the monitor's list
    }
}

void UdsServer::serviceDtc(const uint8_t* req, size_t len) {
    if (len < 2) return negative(0x19, NRC_BAD_LENGTH);
    const uint8_t sub = req[1] & 0x7F;
    Out out(resp, sizeof(resp));
    out.u8(0x59);
    out.u8(sub);

    auto dtcNo = [](const uint8_t* d) { return (static_cast<uint32_t>(d[0]) << 16) | (d[1] << 8) | d[2]; };
    auto putDtc = [&](const Dtc& d) {
        out.u8(DTC_PREFIX >> 16); out.u16(d.code); out.u8(dtcStatus(d));
    };

    std::lock_guard<std::mutex> lk(dtcMutex);
    switch (sub) {
    case 0x01: {   // number of DTCs by status mask
        if (len != 3) return negative(0x19, NRC_BAD_LENGTH);
        uint16_t count = 0;
        for (size_t i = 0; i < dtcCount; i++) if (dtcStatus(dtc[i]) & req[2]) count++;
        out.u8(DTC_AVAILABILITY_MASK); out.u8(0x01); out.u16(count);   // 0x01 = ISO 14229-1 DTC format
        break;
    }
    case 0x02: {   // DTCs by status mask
        if (len != 3) return negative(0x19, NRC_BAD_LENGTH);
        out.u8(DTC_AVAILABILITY_MASK);
        for (size_t i = 0; i < dtcCount; i++) if (dtcStatus(dtc[i]) & req[2]) putDtc(dtc[i]);
        break;
    }
    case 0x04: {   // snapshot record of one DTC
        if (len != 6) return negative(0x19, NRC_BAD_LENGTH);
        const uint32_t no = dtcNo(req + 2);
        const Dtc* r = (no & 0xFF0000) == DTC_PREFIX ? findDtc(no & 0xFFFF) : nullptr;
        if (!r || (req[5] != SNAPSHOT_RECORD && req[5] != 0xFF)) return negative(0x19, NRC_REQUEST_OUT_OF_RANGE);
        putDtc(*r);
        out.u8(SNAPSHOT_RECORD);
        out.u8(1);                  // one DID follows
        out.u16(DID_SNAPSHOT);
        for (size_t i = 0; i < SNAP_LEN; i++) out.u8(r->snap[i]);
        break;
    }
    case 0x06: {   // extended data: record 1 = occurrences (saturated to 255), record 2 = severity
        if (len != 6) return negative(0x19, NRC_BAD_LENGTH);
        const uint32_t no = dtcNo(req + 2);
        const Dtc* r = (no & 0xFF0000) == DTC_PREFIX ? findDtc(no & 0xFFFF) : nullptr;
        if (!r || !(req[5] == 0x01 || req[5] == 0x02 || req[5] == 0xFF)) return negative(0x19, NRC_REQUEST_OUT_OF_RANGE);
        putDtc(*r);
        if (req[5] == 0x01 || req[5] == 0xFF) { out.u8(0x01); out.u8(r->count > 255 ? 255 : r->count); }
        if (req[5] == 0x02 || req[5] == 0xFF) { out.u8(0x02); out.u8(r->severity); }
        break;
    }
    case 0x0A: {   // all supported DTCs with their current status (0 = never stored)
        if (len != 2) return negative(0x19, NRC_BAD_LENGTH);
        out.u8(DTC_AVAILABILITY_MASK);
        for (const ErrorCode c : ALL_CODES) {
            const Dtc* r = nullptr;
            for (size_t i = 0; i < dtcCount; i++) if (dtc[i].code == static_cast<uint16_t>(c)) r = &dtc[i];
            out.u8(DTC_PREFIX >> 16); out.u16(static_cast<uint16_t>(c)); out.u8(r ? dtcStatus(*r) : 0x00);
        }
        break;
    }
    default:
        return negative(0x19, NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
    if (out.ovf) return negative(0x19, NRC_RESPONSE_TOO_LONG);
    reply(resp, out.n);
}

/** 0x14: clear all DTCs (group 0xFFFFFF) or one (0xE0xxxx). Active faults come back at once. */
void UdsServer::serviceClear(const uint8_t* req, size_t len) {
    if (len != 4) return negative(0x14, NRC_BAD_LENGTH);
    if (session != 3) return negative(0x14, NRC_NOT_IN_SESSION);
    const uint32_t group = (static_cast<uint32_t>(req[1]) << 16) | (req[2] << 8) | req[3];
    if (group == 0xFFFFFF) {
        monitor.clearAllFaults();
        monitor.forgetInactiveFaults();
        std::lock_guard<std::mutex> lk(dtcMutex);
        dtcCount = 0;
        memset(dtc, 0, sizeof(dtc));
        dtcDirty = true;
        monitor.logInfo("UDS: all faults cleared", "UDS");
    } else if ((group & 0xFF0000) == DTC_PREFIX) {
        const uint16_t code = group & 0xFFFF;
        monitor.clearFault(static_cast<ErrorCode>(code));
        std::lock_guard<std::mutex> lk(dtcMutex);
        Dtc* r = findDtc(code);
        if (!r) return negative(0x14, NRC_REQUEST_OUT_OF_RANGE);
        const size_t idx = r - dtc;
        for (size_t i = idx; i + 1 < dtcCount; i++) dtc[i] = dtc[i + 1];
        dtcCount--;
        memset(&dtc[dtcCount], 0, sizeof(Dtc));
        dtcDirty = true;
        monitor.logInfo("UDS: fault " + ErrorMonitor::errorCodeToString(static_cast<ErrorCode>(code)) + " cleared", "UDS");
    } else {
        return negative(0x14, NRC_REQUEST_OUT_OF_RANGE);
    }
    const uint8_t r[1] = {0x54};
    reply(r, 1);
}

// ---- periodic work -------------------------------------------------------------------------

void UdsServer::sendStatus() {
    uint8_t active = 0, sev = 0xFF;
    size_t stored;
    {
        std::lock_guard<std::mutex> lk(dtcMutex);
        stored = dtcCount;
        for (size_t i = 0; i < dtcCount; i++) {
            if (!dtc[i].active) continue;
            active++;
            if (sev == 0xFF || dtc[i].severity > sev) sev = dtc[i].severity;
        }
    }
    uint8_t f[8];
    f[0] = (active ? 0x01 : 0) | (sev == static_cast<uint8_t>(ErrorSeverity::CRITICAL) ? 0x02 : 0) |
           (can.getIgnoreAllowFlags() ? 0x04 : 0) | (session ? 0x08 : 0) | (stored ? 0x10 : 0);
    f[1] = active;
    f[2] = static_cast<uint8_t>(stored);
    f[3] = sev;
    f[4] = static_cast<uint8_t>(state.getCurrentState());
    f[5] = 0;
    f[6] = statusCounter >> 8;
    f[7] = statusCounter & 0xFF;
    statusCounter++;
    can.sendFrame(CANIds::DIAG_STATUS, f, 8);
}

void UdsServer::tick() {
    const uint32_t now = millis();
    isotp.tick(now);
    if (session != 0 && now - lastReqMs > S3_TIMEOUT_MS) {
        session = 0;
        monitor.logInfo("UDS session timed out (S3), back to default", "UDS");
    }
    if (now - lastSyncMs >= 100) {
        lastSyncMs = now;
        syncFaults();
    }
    if (now - lastStatusMs >= 1000) {
        lastStatusMs = now;
        sendStatus();
    }
}

// ---- flash (web task) -----------------------------------------------------------------------

namespace {
constexpr uint8_t DTC_BLOB_VERSION = 2;
}

void UdsServer::begin() {
    Preferences p;
    if (!p.begin("vcu_dtc", true)) return;
    uint8_t buf[2 + MAX_DTC * sizeof(Dtc)];
    const size_t got = p.getBytes("d", buf, sizeof(buf));
    p.end();
    if (got < 2 || buf[0] != DTC_BLOB_VERSION || buf[1] != sizeof(Dtc)) return;   // other layout: start empty
    const size_t n = (got - 2) / sizeof(Dtc);
    std::lock_guard<std::mutex> lk(dtcMutex);
    dtcCount = n > MAX_DTC ? MAX_DTC : n;
    memcpy(dtc, buf + 2, dtcCount * sizeof(Dtc));
    for (size_t i = 0; i < dtcCount; i++) dtc[i].active = 0;
}

void UdsServer::housekeeping() {
    const uint32_t now = millis();

    if (saveStatus == 1 && now - lastCfgChangeMs >= 1000) {
        saveStatus = 4;   // saving (a write during the save sets 1 again and is saved next time)
        const bool ok = cfg.save();
        if (saveStatus == 4) saveStatus = ok ? 2 : 3;
        if (!ok) monitor.logError("UDS: saving the configuration failed", "UDS");
    }

    if (dtcDirty && now - lastPersistMs >= 5000) {
        lastPersistMs = now;
        uint8_t buf[2 + MAX_DTC * sizeof(Dtc)];
        size_t n;
        {
            std::lock_guard<std::mutex> lk(dtcMutex);
            n = dtcCount;
            buf[0] = DTC_BLOB_VERSION;
            buf[1] = sizeof(Dtc);
            memcpy(buf + 2, dtc, n * sizeof(Dtc));
            dtcDirty = false;
        }
        Preferences p;
        if (p.begin("vcu_dtc", false)) {
            p.putBytes("d", buf, 2 + n * sizeof(Dtc));
            p.end();
        } else {
            dtcDirty = true;
        }
    }
}
