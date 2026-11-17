/* host test of axis_control.c (no board, no FreeRTOS).  From r5_freertos/:
 *   gcc -std=gnu11 -Itest/stub -Isrc test/test_axis_control.c src/axis_control.c src/robot_target.c -o /tmp/t && /tmp/t
 * (add -DROBOT_TARGET=1 for the Indy7 table: the torque test then only checks what holds for both)
 * It checks: servo off keeps the drives off, servo on walks the CiA402 states, the fault reset limit, the torque
 * (only when enabled, usable, servo on; the unit conversion and its limits), and the stale-torque stop, also with
 * a drive that is not enabled. */
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "axis_control.h"
#include "cia402_drive.h"
#include "robot_target.h"
#include "event_log.h"

drive_outputs_t *drive_out[AXIS_COUNT]; drive_inputs_t *drive_in[AXIS_COUNT];
static drive_outputs_t out[AXIS_COUNT]; static drive_inputs_t in[AXIS_COUNT];
unsigned short cia402_state_from_statusword(unsigned short s)
{ if ((s&0x4F)==0x00) return CIA402_NOT_READY_TO_SWITCH_ON; if ((s&0x4F)==0x08) return CIA402_FAULT; if ((s&0x4F)==0x40) return CIA402_SWITCH_ON_DISABLED;
  if ((s&0x6F)==0x27) return CIA402_OPERATION_ENABLED; if ((s&0x6F)==0x23) return CIA402_SWITCHED_ON; if ((s&0x6F)==0x21) return CIA402_READY_TO_SWITCH_ON; return CIA402_STATE_UNKNOWN; }
static int stale_stop_events;
void event_post(uint32_t id, int b, int x, int y, int z) { (void)b;(void)x;(void)y;(void)z; if (id == EVENT_TORQUE_STALE_STOP) stale_stop_events++; }
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void)
{
    int i;
    for (i = 0; i < AXIS_COUNT; i++) { drive_out[i] = &out[i]; drive_in[i] = &in[i]; in[i].statusword = 0x0040; in[i].position_actual = 1000 + i; }

    axis_control_update();                                   /* servo off: disable voltage, target follows actual */
    CHECK(out[0].controlword == CW_DISABLE_VOLTAGE && out[0].target_position == 1000);

    axis_control_servo(1);
    axis_control_update();  CHECK(out[0].controlword == CW_SHUTDOWN);           /* switch on disabled */
    in[0].statusword = 0x0021; axis_control_update(); CHECK(out[0].controlword == CW_SWITCH_ON);
    in[0].statusword = 0x0023; axis_control_update(); CHECK(out[0].controlword == CW_ENABLE_OPERATION);
    in[0].statusword = 0x0027; in[0].position_actual = 5000; axis_control_update();
    CHECK(out[0].target_position == 5000 && out[0].modes_of_operation == MODE_CST);   /* CST: the position target only follows */
    in[0].position_actual = 5100; axis_control_update();
    CHECK(out[0].target_position == 5100);

    axis_control_servo(0);  axis_control_update(); CHECK(out[0].controlword == CW_DISABLE_VOLTAGE);   /* off at any time */
    in[0].statusword = 0x0040; axis_control_update();

    axis_control_servo(1);                                   /* faults: MAX_FAULT_RESETS resets, then none */
    for (i = 0; i < MAX_FAULT_RESETS; i++) {
        in[1].statusword = 0x0008; axis_control_update(); CHECK(out[1].controlword == CW_FAULT_RESET);
        in[1].statusword = 0x0040; axis_control_update(); CHECK(out[1].controlword == CW_SHUTDOWN);
    }
    in[1].statusword = 0x0008; axis_control_update(); CHECK(out[1].controlword == CW_DISABLE_VOLTAGE);
    axis_control_servo(0); axis_control_servo(1);            /* a new servo-on gives the resets back */
    in[1].statusword = 0x0040; axis_control_update(); in[1].statusword = 0x0008; axis_control_update();
    CHECK(out[1].controlword == CW_FAULT_RESET);

    /* ---- torque ---- */
    {
        double tau[AXIS_COUNT]; int k;
        for (i = 0; i < AXIS_COUNT; i++) { in[i].statusword = 0x0027; tau[i] = 0.0; }
#if ROBOT_TARGET == TARGET_L7N_BENCH
        tau[0] = 0.1404;                                           /* N m: inside the limit (0.4), 105 drive units */
#else
        tau[0] = 5.0;                                              /* N m: 0.1404 would be below one drive unit */
#endif
        axis_control_servo(0); axis_control_servo(1);
        axis_control_torque_input(1, 1, tau); axis_control_update();
        CHECK(out[0].target_torque == axis_raw_torque(0, tau[0]) && out[0].target_torque != 0);
        CHECK(out[1].target_torque == 0);
#if ROBOT_TARGET == TARGET_L7N_BENCH
        CHECK(out[0].target_torque == -105);                       /* 0.1404 / 1.337 * 1000, dir -1, fraction dropped */
        CHECK(axis_raw_torque(0, 0.0) == 0);
        CHECK(axis_raw_torque(0, 99.0) == -299 && axis_raw_torque(0, -99.0) == 299);   /* cut at TORQUE_LIMIT_U 0.4 N m */
#endif
        CHECK(axis_raw_torque(0, 0.0 / 0.0) == 0);                 /* not a number */
        CHECK(axis_raw_torque(0, 1.0 / 0.0) == 0);                 /* infinity too: a broken value is not a command */
        CHECK(axis_raw_torque(0, 1e9) == -axis_raw_torque(0, -1e9) && axis_raw_torque(0, 1e9) != 0);   /* a huge one is the limit */
        in[0].statusword = 0x0023;                                 /* not enabled: no torque */
        axis_control_torque_input(1, 1, tau); axis_control_update();
        CHECK(out[0].target_torque == 0);
        in[0].statusword = 0x0027;
        axis_control_torque_input(1, 0, tau); axis_control_update();   /* slot not valid / not enabled */
        CHECK(out[0].target_torque == 0);
        axis_control_torque_input(1, 1, tau); axis_control_update();
        CHECK(out[0].target_torque != 0);
        axis_control_torque_zero();
        CHECK(out[0].target_torque == 0);

        /* ---- stale torque: held for TORQUE_STALE_LIMIT - 1 cycles, then torque 0 and servo off ---- */
        {
            int held; uint32_t total = axis_control_torque_stale_total();
            axis_control_torque_input(1, 1, tau); axis_control_update();
            held = out[0].target_torque;
            for (k = 1; k < TORQUE_STALE_LIMIT; k++) {
                axis_control_torque_input(0, 1, tau); axis_control_update();
                CHECK(out[0].target_torque == held && axis_control_servo_requested());
            }
            axis_control_torque_input(0, 1, tau); axis_control_update();
            CHECK(!axis_control_servo_requested() && out[0].target_torque == 0 && stale_stop_events == 1);
            CHECK(out[0].controlword == CW_DISABLE_VOLTAGE);       /* in the same cycle */
            axis_control_torque_input(0, 1, tau); axis_control_update();
            CHECK(out[0].controlword == CW_DISABLE_VOLTAGE && stale_stop_events == 1);   /* once, not every cycle */
            CHECK(axis_control_torque_stale_total() - total == TORQUE_STALE_LIMIT + 1);
            axis_control_servo(1);                                 /* the APU asks again: torque flows again */
            axis_control_torque_input(1, 1, tau); axis_control_update();
            CHECK(out[0].target_torque == held);
            /* The stop does not wait for every drive (review F1, 2026-11-17: it used to count only while all were
             * enabled).  One drive in FAULT with its resets used up, one that stays switch-on disabled, one that is
             * only switched on: the others are enabled and must not keep the last torque of an APU that is gone. */
            {
                static const unsigned short not_enabled[3] = { 0x0008, 0x0040, 0x0023 };
                int c;
                for (c = 0; c < 3; c++) {
                    total = axis_control_torque_stale_total();
                    axis_control_servo(0); axis_control_servo(1);
                    in[2].statusword = not_enabled[c];
                    for (k = 0; k < 5; k++) { axis_control_torque_input(1, 1, tau); axis_control_update(); }   /* the APU writes */
                    CHECK(axis_control_servo_requested() && out[0].target_torque == held && out[2].target_torque == 0);
                    for (k = 1; k < TORQUE_STALE_LIMIT; k++) {                                                 /* the APU is gone */
                        axis_control_torque_input(0, 1, tau); axis_control_update();
                        CHECK(out[0].target_torque == held && axis_control_servo_requested());
                    }
                    axis_control_torque_input(0, 1, tau); axis_control_update();
                    CHECK(!axis_control_servo_requested() && stale_stop_events == 2 + c);
                    CHECK(out[0].target_torque == 0 && out[0].controlword == CW_DISABLE_VOLTAGE);
                    CHECK(axis_control_torque_stale_total() == total);        /* the State slot counts only cycles with every drive on */
                }
                in[2].statusword = 0x0027;
            }
            /* while the drives are on their way to OPERATION_ENABLED, an APU that writes every cycle is not stopped ... */
            axis_control_servo(0); axis_control_servo(1);
            for (i = 0; i < AXIS_COUNT; i++) in[i].statusword = 0x0040;
            for (k = 0; k < 3 * TORQUE_STALE_LIMIT; k++) { axis_control_torque_input(1, 1, tau); axis_control_update(); }
            CHECK(axis_control_servo_requested() && stale_stop_events == 4);
            /* ... and one that does not write is, before any drive is enabled */
            for (k = 0; k < TORQUE_STALE_LIMIT; k++) { axis_control_torque_input(0, 1, tau); axis_control_update(); }
            CHECK(!axis_control_servo_requested() && stale_stop_events == 5);
            /* servo off: nothing is stopped, however long the slot stands still after a write */
            axis_control_torque_input(1, 1, tau); axis_control_update();
            for (k = 0; k < 3 * TORQUE_STALE_LIMIT; k++) { axis_control_torque_input(0, 1, tau); axis_control_update(); }
            CHECK(stale_stop_events == 5);
        }
    }

    printf(fails ? "%d FAILED\n" : "all passed\n", fails);
    return fails != 0;
}
