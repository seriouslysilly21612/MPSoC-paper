/*
 * ecat_bringup.h  --  EtherCAT bring-up: NIC, bus scan, PDO mapping, DC SYNC0 (L7N bench only), SAFE-OP, OPERATIONAL.
 */
#ifndef ECAT_BRINGUP_H
#define ECAT_BRINGUP_H

extern int ecat_expected_wkc;   /* working counter of a complete process-data frame, set by ecat_bring_up() */

int ecat_bring_up(void);        /* 1 = all drives OPERATIONAL, 0 = failed (reason already printed) */

#endif /* ECAT_BRINGUP_H */
