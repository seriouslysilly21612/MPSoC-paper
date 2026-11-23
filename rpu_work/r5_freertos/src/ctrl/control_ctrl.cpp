/*
 * control_ctrl.cpp  --  see control_ctrl.h.
 */
#include "control_ctrl.h"

#include <atomic>
#include <cstring>

#include "ControlCycle.h"
#include "ctrl_model.h"
#include "ctrl_numbers_indy7.h"

static_assert(CTRL_NUMBERS_COUNT == SHADOW_NUMBERS, "ctrl_numbers_indy7.h is not of this ShadowControl.h: write it again");
static_assert(CTRL_OBS_NUMBERS_COUNT == OBS_NUMBERS, "ctrl_numbers_indy7.h is not of this ShadowControl.h: write it again");

/* written once, by the task that builds the controller; read by the task of the planner */
static std::atomic<CControlCycle *> controller(NULL);

uint32_t control_ctrl_check_value(void)
{
    return ShadowCheckValue(CTRL_NUMBERS_CRC, ctrl_model_check_value());
}

uint32_t control_ctrl_obs_check_value(void)
{
    return CTRL_OBS_NUMBERS_CRC;
}

void control_ctrl_set_obs_clock(uint64_t (*now)(void))
{
    CControlCycle *const ctl = controller.load();
    if (ctl != NULL)
        ctl->SetObsClock(now);
}

int control_ctrl_obs_cost(obs_cost_t *out)
{
    const CControlCycle *const ctl = controller.load();
    if (ctl == NULL)
        return 0;
    std::memcpy(out, &ctl->ObsCost(), sizeof(*out));     // the control task writes it meanwhile: words of one moment
    return 1;
}

int control_ctrl_build(volatile uint32_t *region, double ticks_per_s)
{
    if (controller.load() != NULL)
        return 1;
    int result = 1;
    try
    {
        CControlCycle *const built = new CControlCycle;
        if (!built->Init("", CTRL_NUMBERS, control_ctrl_check_value()))     // no file: the model is ctrl_model.cpp's
        {
            delete built;
            return 0;
        }
        CControlCycle::ObsLink link;
        link.pBox = (volatile obs_mailbox_t *)(region + OBS_OFF_MAILBOX / 4u);
        link.pStatus = (volatile obs_status_region_t *)(region + OBS_OFF_STATUS / 4u);
        link.pRing = (volatile obs_ring_t *)(region + OBS_OFF_RING / 4u);
        link.dTicksPerS = ticks_per_s;
        link.pNumbers = CTRL_OBS_NUMBERS;
        link.uNumbersCrc = CTRL_OBS_NUMBERS_CRC;
        link.uContractCrc = CTRL_CONTRACT_CRC;
        if (!built->SetObsLink(link))
            result = 2;
        controller.store(built);        // only now: the planner task may call it from here on
    }
    catch (...)         // no memory, or an error of a library: an exception that nobody catches would stop the firmware
    {
        return 0;
    }
    return result;
}

void control_ctrl_step(const control_in_t *in, control_out_t *out, ipc_report_t *report)
{
    try
    {
        CControlCycle::Input stIn;
        stIn.uPeriod = in->period;
        stIn.bState = in->state != 0;
        stIn.bSimArm = in->sim_arm != 0;
        stIn.bServoOn = in->servo_on != 0;
        stIn.uNotReady = in->not_ready;
        stIn.uSimStep = in->sim_step;
        stIn.tRelease = in->release;
        stIn.fResponse = in->response;
        for (int i = 0; i < IPC_CTRL_DOF; i++)
        {
            stIn.adQ[i] = in->q[i];
            stIn.adQd[i] = in->qd[i];
            stIn.adTau[i] = in->tau[i];
        }
        stIn.pCommand = in->command;
        stIn.pGoal = in->goal != NULL ? in->goal->p : NULL;
        CControlCycle::Output stOut;
        controller.load()->Step(stIn, &stOut, report);
        out->torque = stOut.bTorque ? 1 : 0;
        for (int i = 0; i < IPC_CTRL_DOF; i++)
            out->tau[i] = stOut.adTau[i];
        out->servo = stOut.nServo;
    }
    catch (...)         // as above: the EtherCAT cycle must go on, without a torque of this controller
    {
        memset(out, 0, sizeof(*out));
        out->servo = IPC_CMD_SERVO_OFF;
        memset(report, 0, sizeof(*report));
        report->in.period = in->period;
        report->state = IPC_CTL_FAULT;
    }
}

int control_ctrl_serve(void)
{
    CControlCycle *const built = controller.load();
    if (built == NULL)
        return 0;
    try
    {
        return built->ServePlanner() ? 1 : 0;
    }
    catch (...)         // as above
    {
        return 0;
    }
}
