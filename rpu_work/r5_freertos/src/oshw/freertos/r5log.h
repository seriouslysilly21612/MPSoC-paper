/*
 * r5log.h -- R5 console output -> ring buffer in the DDR carveout (readable from Linux)
 *
 * You normally do not include this: just add r5log.c to the project and every
 * xil_printf() / print() / printf() goes into the ring (see r5log.c).
 */
#ifndef R5LOG_H
#define R5LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Optional.  outbyte() initialises the ring on first use anyway.
 * Calling this at the top of main() marks a fresh run (boot_id + 1, head = 0). */
void r5log_init(void);

/* Write a NUL-terminated string into the ring (no newline added). */
void r5log_puts(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* R5LOG_H */
