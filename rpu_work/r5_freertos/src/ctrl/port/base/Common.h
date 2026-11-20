/*
 * base/Common.h of the firmware  --  what the controller classes (copies of the application's, unchanged) take
 * from the application's base/Common.h, without Linux: the types, the clock and the log lines.  The firmware
 * finds this header in place of the application's, which needs RT-POSIX and a terminal.
 *
 *   read_timer     [ns], from the system counter (ctrl_port.c)
 *   DBG_LOG_...    ctrl_port_log(): printed while the firmware starts, counted once the 1 kHz loop runs (the
 *                  loop never prints)
 */
#ifndef G2T_COMMON_H
#define G2T_COMMON_H

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TRUE  1
#define FALSE 0
typedef bool         BOOL;
typedef unsigned int UINT;
typedef uint64_t     RTTIME;
typedef std::string  TSTRING;

enum eLogLevel { eLogTrace = 0, eLogWarn, eLogError, eLogInfo };

extern "C" RTTIME read_timer(void);
/* no format attribute: the classes print uint32_t with %u, which is unsigned long for this compiler (same size) */
extern "C" void ctrl_port_log(int level, const char *format, ...);

#define DBG_LOG_TRACE(...) ctrl_port_log(eLogTrace, __VA_ARGS__)
#define DBG_LOG_WARN(...)  ctrl_port_log(eLogWarn, __VA_ARGS__)
#define DBG_LOG_ERROR(...) ctrl_port_log(eLogError, __VA_ARGS__)
#define DBG_LOG_INFO(...)  ctrl_port_log(eLogInfo, __VA_ARGS__)

#endif /* G2T_COMMON_H */
