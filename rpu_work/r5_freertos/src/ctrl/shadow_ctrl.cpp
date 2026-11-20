/*
 * shadow_ctrl.cpp  --  see shadow_ctrl.h.
 */
#include "shadow_ctrl.h"

#include <atomic>
#include <cstring>

#include "ShadowControl.h"
#include "ctrl_model.h"
#include "ctrl_numbers_indy7.h"

static_assert(CTRL_NUMBERS_COUNT == SHADOW_NUMBERS, "ctrl_numbers_indy7.h is not of this ShadowControl.h: write it again");

/* written once, by the task that builds the controller; read by the task of the planner */
static std::atomic<CShadowControl *> controller(NULL);

uint32_t shadow_ctrl_check_value(void)
{
    return ShadowCheckValue(CTRL_NUMBERS_CRC, ctrl_model_check_value());
}

int shadow_ctrl_build(void)
{
    if (controller.load() != NULL)
        return 1;
    try
    {
        CShadowControl *const built = new CShadowControl;
        if (!built->Init("", CTRL_NUMBERS, shadow_ctrl_check_value()))      // no file: the model is ctrl_model.cpp's
        {
            delete built;
            return 0;
        }
        controller.store(built);        // only now: the planner task may call it from here on
    }
    catch (...)         // no memory, or an error of a library: an exception that nobody catches would stop the firmware
    {
        return 0;
    }
    return 1;
}

void shadow_ctrl_step(const ipc_shadow_in_t *in, ipc_shadow_out_t *out)
{
    try
    {
        controller.load()->Step(*in, out);
    }
    catch (...)         // as above: the EtherCAT cycle must go on
    {
        memset(out, 0, sizeof(*out));
        out->period = in->period;
        out->flags = IPC_SHADOW_OUT_FAULT;
    }
}

int shadow_ctrl_serve(void)
{
    CShadowControl *const built = controller.load();
    if (built == NULL)
        return 0;
    try
    {
        return built->ServePlanner() ? 1 : 0;
    }
    catch (...)         // as above: an exception that nobody catches would stop the firmware
    {
        return 0;
    }
}
