/* host test of one cycle of the EtherCAT task (ecat_cycle() of ecat_task.c) with its timing (loop_timing.c) and its
 * record (cycle_record.c); the bus, the slots of the APU and the axis logic are stand-ins here.  From r5_freertos/:
 *   gcc -std=gnu11 -DHOST_TEST -DRECORD_RING_BYTES=4096 -Isrc/soem -Isrc/osal -Isrc/osal/freertos -Isrc/oshw/freertos \
 *       -Itest/stub -Isrc test/test_ecat_cycle.c src/loop_timing.c src/cycle_record.c -lm -o /tmp/t_cycle && /tmp/t_cycle
 * It checks, cycle by cycle: the release goes into the State slot and to the slots of the APU; one record per cycle
 * with the period of the State and the flags OP, WKC, SERVO, STOP; a late send is counted in the record of its
 * cycle and in the State of the next; a cycle in which the second controller ran ends with it, has its two times
 * and holds the period with it; a stop asked for by Linux, to the last cycle that sends no frame. */
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "../src/ecat_task.c"

/* ---- the bus (SOEM) ---- */
ec_slavet ec_slave[EC_MAXSLAVE];
int ec_slavecount = AXIS_COUNT;
int ecat_expected_wkc = 3 * AXIS_COUNT;
static int next_wkc, frames_sent, drive_state = EC_STATE_OPERATIONAL;
int ec_receive_processdata(int timeout) { (void)timeout; return next_wkc; }
int ec_send_processdata(void) { frames_sent++; return 1; }
int ec_readstate(void) { int i; for (i = 1; i <= ec_slavecount; i++) ec_slave[i].state = (uint16)drive_state; return drive_state; }
int ec_writestate(uint16 slave) { (void)slave; drive_state = EC_STATE_PRE_OP; return 1; }
int ecat_bring_up(void) { return 1; }

/* ---- the drives ---- */
static drive_outputs_t out[AXIS_COUNT]; static drive_inputs_t in[AXIS_COUNT];
drive_outputs_t *drive_out[AXIS_COUNT]; drive_inputs_t *drive_in[AXIS_COUNT];
unsigned short cia402_state_from_statusword(unsigned short s)
{
    if ((s & 0x6F) == 0x27) return CIA402_OPERATION_ENABLED;
    if ((s & 0x4F) == 0x40) return CIA402_SWITCH_ON_DISABLED;
    return CIA402_STATE_UNKNOWN;
}
static int updates, zeros;
void axis_control_servo(int on) { (void)on; }
int  axis_control_servo_requested(void) { return 0; }
void axis_control_torque_input(int fresh, int usable, const double tau[AXIS_COUNT]) { (void)fresh; (void)usable; (void)tau; }
void axis_control_update(void) { updates++; }
void axis_control_torque_zero(void) { zeros++; }

/* ---- the slots of the APU ---- */
static struct { uint64_t release; uint32_t period, flags; link_faults_t faults; int n; } state;
static uint64_t obs_now;
static uint32_t servo_flag;
int      ocm_link_start(void) { return 1; }
uint32_t ocm_link_take_command(void) { return IPC_CMD_NONE; }
int      ocm_link_take_torque(link_torque_t *held) { memset(held, 0, sizeof(*held)); return 0; }
uint32_t ocm_link_publish_state(uint64_t t_release, uint32_t period, uint32_t flags, const link_faults_t *faults)
{
    state.release = t_release; state.period = period; state.flags = flags | servo_flag; state.faults = *faults; state.n++;
    return state.flags;
}
void obs_link_start(void) { }
void obs_link_poll(uint64_t now) { obs_now = now; }
void obs_link_report(uint64_t now) { (void)now; }

/* ---- the second controller: with a payload it runs (the clock: the fourth part of the cycle) ---- */
static int shadow_payload, shadow_steps;
int  shadow_link_start(void) { return 1; }
int  shadow_link_step(int32_t *traj_ns, int32_t *ctrl_ns)
{
    if (!shadow_payload) return 0;
    shadow_steps++;
    *traj_ns = 123456; *ctrl_ns = -1;
    return 1;
}
void shadow_link_report(void) { }

/* ---- events, console, FreeRTOS, caches ---- */
static event_t an_event; static int stop_events;
volatile uint32_t events_dropped; uint32_t events_suppressed; int event_budget;
event_t *event_reserve(uint32_t id, int budgeted) { (void)budgeted; memset(&an_event, 0, sizeof(an_event)); an_event.id = id; return &an_event; }
void event_publish(void) { }
void event_post(uint32_t id, int budgeted, int a, int b, int c) { (void)budgeted; (void)a; (void)b; (void)c; if (id == EVENT_STOP) stop_events++; }
void xil_printf(const char *f, ...) { (void)f; }
uint32_t r5log_boot_id(void) { return 1u; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment) { *previous_wake += increment; }
TickType_t xTaskGetTickCount(void) { return 0; }
void Xil_DCacheFlushRange(INTPTR adr, uint32_t len) { (void)adr; (void)len; }
void Xil_DCacheInvalidateRange(INTPTR adr, uint32_t len) { (void)adr; (void)len; }

/* ---- the clock: the test sets when a cycle starts and how long each of its parts takes [ticks] ---- */
uint8_t host_test_ring[RECORD_RING_BYTES];
static uint64_t clock_now;
static uint32_t wake_ticks, part_ticks[4];     /* how long the receive, the logic, the send and the second controller take */
static int part;
uint64_t host_test_counter(void) { clock_now += part_ticks[part++ % 4]; return clock_now; }    /* every read is one part later */
int  loop_timing_start(void) { return 1; }
void loop_timing_begin(uint32_t planned_tick, cycle_times_t *t)
{
    clock_now = (uint64_t)planned_tick * TICKS_PER_CYCLE + wake_ticks;
    part = 0;
    t->start = (uint32_t)clock_now; t->wake = wake_ticks; t->release = clock_now - wake_ticks;
}

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static cycle_record_t last_record(void)
{
    cycle_record_t r;
    uint32_t count;

    memcpy(&count, host_test_ring + 0x10, 4);
    memcpy(&r, host_test_ring + RECORD_HEAD_BYTES + ((count - 1u) % ((RECORD_RING_BYTES - RECORD_HEAD_BYTES) / 60u)) * 60u, 60);
    return r;
}
static uint32_t record_count(void) { uint32_t count; memcpy(&count, host_test_ring + 0x10, 4); return count; }

int main(void)
{
    cycle_record_t r;
    int i, cycles;

    /* the stop page of ecat_task.c is a fixed address */
    if (mmap((void *)CTL_BASE, 4096, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == MAP_FAILED) {
        printf("NOT RUN: no memory at 0x%lx\n", (unsigned long)CTL_BASE);
        return 2;
    }
    for (i = 0; i < AXIS_COUNT; i++) { drive_out[i] = &out[i]; drive_in[i] = &in[i]; in[i].statusword = 0x0040; }
    ctl_init();
    cycle_record_start();
    memset(&loop_timing, 0, sizeof(loop_timing)); loop_timing_reset_window();

    /* an ordinary cycle: 5 us late, the receive 10 us, the logic 20 us, the send 30 us */
    next_wkc = ecat_expected_wkc; wake_ticks = 500; part_ticks[0] = 1000; part_ticks[1] = 2000; part_ticks[2] = 3000;
    ecat_cycle(1000);
    CHECK(cycle_index == 1 && state.n == 1 && state.period == 1 && state.flags == IPC_STATE_OP);
    CHECK(state.release == 1000ull * TICKS_PER_CYCLE && obs_now == state.release);       /* the tick, not the start */
    CHECK(state.faults.late == 0 && state.faults.missed == 0 && state.faults.overruns == 0 && state.faults.wkc == 0);
    CHECK(updates == 1 && zeros == 0 && frames_sent == 1 && record_count() == 1);
    r = last_record();
    CHECK(r.period == 1 && r.flags == (RECORD_BUS | RECORD_OP) && r.w_ns == 5000);
    CHECK(r.recv_ns == 10000 && r.send_ns == 30000 && r.s_ns == 60001 && r.e_ns == 60001);
    CHECK(r.dyn_ns == -1 && r.hl_ns == -1 && r.dyn_e_ns == -1 && r.sync_ns == 0xFFFFFFFFu && shadow_steps == 0);

    /* a payload of the control task: the second controller runs after the send, 250 us; the cycle ends with it */
    shadow_payload = 1; part_ticks[3] = 25000;
    ecat_cycle(1001);
    r = last_record();
    CHECK(shadow_steps == 1 && r.period == 2 && r.flags == (RECORD_BUS | RECORD_OP) && frames_sent == 2);
    CHECK(r.s_ns == 60001 && r.e_ns == 310003 && r.hl_ns == 123456 && r.dyn_ns == -1);
    part_ticks[3] = TICKS_PER_CYCLE;                       /* it runs into the next release: an overrun, the send in time */
    ecat_cycle(1002);
    r = last_record();
    CHECK(shadow_steps == 2 && r.flags == (RECORD_BUS | RECORD_OP | RECORD_OVERRUN) && loop_timing.missed == 0);
    shadow_payload = 0; part_ticks[3] = 0;
    memset(&loop_timing, 0, sizeof(loop_timing)); loop_timing_reset_window();

    /* every axis enabled: SERVO in the record */
    servo_flag = IPC_STATE_SERVO_ON;
    ecat_cycle(1003);
    r = last_record();
    CHECK(r.period == 4 && r.flags == (RECORD_BUS | RECORD_OP | RECORD_SERVO) && record_count() == 4);
    servo_flag = 0;

    /* a lost frame: no OP, WKC in the record and in the State, torque 0, the frame goes out all the same */
    next_wkc = ecat_expected_wkc - 3;
    ecat_cycle(1004);
    r = last_record();
    CHECK(r.period == 5 && r.flags == (RECORD_BUS | RECORD_WKC) && state.flags == (IPC_STATE_OP | IPC_STATE_WKC_ERR));
    CHECK(zeros == 1 && updates == 4 && frames_sent == 5 && state.faults.wkc == 1);
    next_wkc = ecat_expected_wkc;

    /* a send past the deadline: in the record of its cycle, in the State of the next one */
    part_ticks[2] = 37000;                                 /* release -> send returns: 500 + 1000 + 2000 + 37000 = 405 us */
    ecat_cycle(1005);
    r = last_record();
    CHECK(r.flags == (RECORD_BUS | RECORD_OP | RECORD_MISSED) && state.faults.missed == 0 && loop_timing.missed == 1);
    part_ticks[2] = 3000;
    ecat_cycle(1006);
    r = last_record();
    CHECK(r.flags == (RECORD_BUS | RECORD_OP) && state.faults.missed == 1 && state.period == 7);
    /* a cycle that starts more than a period late and so ends after the next release */
    wake_ticks = TICKS_PER_CYCLE + 1u;
    ecat_cycle(1007);
    r = last_record();
    CHECK(r.flags == (RECORD_BUS | RECORD_OP | RECORD_LATE | RECORD_MISSED | RECORD_OVERRUN));
    wake_ticks = 500;
    ecat_cycle(1008);
    CHECK(state.faults.late == 1 && state.faults.overruns == 1 && state.faults.missed == 2 && state.faults.wkc == 1);
    CHECK(record_count() == 9 && cycle_index == 9 && frames_sent == 9);

    /* Linux asks for a stop: STOP from that cycle on, the working counter falling is no WKC fault, and the last
     * cycle sends no frame; every cycle of the stop has its record */
    CTL_CMD = CTL_CMD_STOP;
    shadow_payload = 1;                                    /* not during a stop */
    ecat_cycle(1009);
    r = last_record();
    CHECK(stop_step == STOP_HOLD && stop_events == 1 && (r.flags & RECORD_STOP) && (state.flags & IPC_STATE_STOP));
    next_wkc = 0;                                          /* the drives leave OP */
    for (cycles = 0; stop_step != STOP_DONE && cycles < 5000; cycles++) {
        ecat_cycle(1010u + (uint32_t)cycles);
        r = last_record();
        if (!(r.flags & RECORD_STOP) || (r.flags & (RECORD_WKC | RECORD_OP))) break;
    }
    CHECK(stop_step == STOP_DONE && state.faults.wkc == 1 && zeros >= (int)STOP_HOLD_MS);
    CHECK(frames_sent == 10 + cycles - 1 && record_count() == 10u + (uint32_t)cycles && shadow_steps == 2);   /* the last cycle: a record, no frame */
    CHECK(CTL_STATE == ECAT_STATUS_STOPPING);

    printf(fails ? "test_ecat_cycle: %d FAILED\n" : "test_ecat_cycle OK\n", fails);
    return fails != 0;
}
