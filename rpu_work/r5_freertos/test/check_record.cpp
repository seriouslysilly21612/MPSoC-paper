/* The record of a cycle in this firmware (src/cycle_record.h) must be the record of the timing log of the APU
 * controller (goal-to-torque_MPSoC/indy7/src/TimingLog.h): one tool reads both.  This file only has to compile.
 * From r5_freertos/ (test/check_copies.sh does it):
 *   g++ -std=c++17 -fsyntax-only -I../../goal-to-torque_MPSoC/indy7/src -Isrc test/check_record.cpp */
#include <cstddef>
#include "TimingLog.h"
#include "cycle_record.h"

typedef CTimingLog::Record app_record_t;
#define SAME_FIELD(app, fw) \
    static_assert(offsetof(app_record_t, app) == offsetof(cycle_record_t, fw) && \
                  sizeof(((app_record_t *)0)->app) == sizeof(((cycle_record_t *)0)->fw), "field " #fw " differs from " #app)

static_assert(sizeof(app_record_t) == sizeof(cycle_record_t), "the size of the record differs");
SAME_FIELD(uPeriod, period);   SAME_FIELD(nW, w_ns);          SAME_FIELD(nS, s_ns);          SAME_FIELD(nE, e_ns);
SAME_FIELD(nDyn, dyn_ns);      SAME_FIELD(nHl, hl_ns);        SAME_FIELD(nSim, sim_ns);      SAME_FIELD(uSyncDiff, sync_ns);
SAME_FIELD(usFlags, flags);    SAME_FIELD(usFlags2, reserved);
SAME_FIELD(nRecv, recv_ns);    SAME_FIELD(nSend, send_ns);    SAME_FIELD(nTrajW, traj_w_ns); SAME_FIELD(nTrajE, traj_e_ns);
SAME_FIELD(nDynW, dyn_w_ns);   SAME_FIELD(nDynE, dyn_e_ns);

static_assert((unsigned)RECORD_OP == (unsigned)CTimingLog::TF_OP && (unsigned)RECORD_WKC == (unsigned)CTimingLog::TF_WKC &&
              (unsigned)RECORD_LATE == (unsigned)CTimingLog::TF_LATE && (unsigned)RECORD_MISSED == (unsigned)CTimingLog::TF_MISSED &&
              (unsigned)RECORD_OVERRUN == (unsigned)CTimingLog::TF_OVERRUN && (unsigned)RECORD_STOP == (unsigned)CTimingLog::TF_STOP &&
              (unsigned)RECORD_BUS == (unsigned)CTimingLog::TF_BUS && (unsigned)RECORD_SERVO == (unsigned)CTimingLog::TF_SERVO,
              "a flag bit of the record differs");
