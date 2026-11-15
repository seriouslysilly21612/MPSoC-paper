/*
 * ecat_task.h  --  the EtherCAT master task: bring-up, then the 1 kHz process-data loop.
 */
#ifndef ECAT_TASK_H
#define ECAT_TASK_H

enum {                                   /* ecat_status; the LED task in main.c shows it */
    ECAT_STATUS_STARTING    =  0,        /* NIC, bus scan, bring-up to OP */
    ECAT_STATUS_RUNNING     =  1,        /* all drives OPERATIONAL, 1 kHz loop running */
    ECAT_STATUS_INIT_FAILED = -1,        /* bring-up failed, the task is parked, nothing moves */
    ECAT_STATUS_BUS_FAULT   = -2         /* working counter incomplete WKC_BUS_FAULT_CYCLES cycles in a row */
};
extern volatile int ecat_status;

void ecat_task(void *arg);               /* FreeRTOS task entry; main.c creates it as "ecat" */

#endif /* ECAT_TASK_H */
