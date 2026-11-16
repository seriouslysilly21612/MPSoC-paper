/*
 * ecat_diag.h  --  EtherCAT diagnostics in words: AL states, AL status codes, SOEM's error list, the slave states.
 */
#ifndef ECAT_DIAG_H
#define ECAT_DIAG_H

#include "ethercat.h"

const char *al_status_code_text(uint16 code);   /* ETG.1000 AL status code -> text */
const char *al_state_name(unsigned state);      /* EC_STATE_* -> "PRE-OP", ... */

void print_soem_errors(int max);                /* empty SOEM's error list, print its oldest `max` entries */
void print_slave_states(const char *why);       /* read and print every slave's AL state and AL status code */
void print_sync_parameters(void);               /* read (CoE SDO) and print every slave's 0x1C32 */

#endif /* ECAT_DIAG_H */
