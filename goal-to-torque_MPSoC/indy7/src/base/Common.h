/*****************************************************************************
*	Name: Common.h
*	Description: Types, logging and terminal helpers shared by the programs
*	of this repository. The integer and boolean types, RTTIME and the clock
*	(read_timer, CLOCK_MONOTONIC in ns) come from RT-POSIX
*	(third_party/rt-posix: commons.h, posix_rt.h).
*****************************************************************************/
#ifndef G2T_COMMON_H
#define G2T_COMMON_H

// unistd.h before posix_rt.h: RT-POSIX defines gettid() as a macro when
// the C library has not declared it yet
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

#include "posix_rt.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef std::string TSTRING;

/* Log lines: "<date time.ms>[LEVEL] message [file:line]", errors on stderr,
 * the others on stdout. Stack buffers only, no lock: callable from the
 * real-time tasks, whose lines go through base/RtConsole.h. */
enum eLogLevel { eLogTrace = 0, eLogWarn, eLogError, eLogInfo };

/* seconds since 1970-01-01 00:00 -> year, month, day, hour, minute, second
 * (H. Hinnant's civil_from_days; no lock, unlike localtime_r and gmtime_r) */
inline void CivilFromSeconds(long alSec, int* anT)
{
	const long lDays = (alSec >= 0 ? alSec : alSec - 86399) / 86400, lSod = alSec - lDays * 86400;
	const long lZ = lDays + 719468, lEra = (lZ >= 0 ? lZ : lZ - 146096) / 146097;
	const long lDoe = lZ - lEra * 146097;
	const long lYoe = (lDoe - lDoe / 1460 + lDoe / 36524 - lDoe / 146096) / 365;
	const long lDoy = lDoe - (365 * lYoe + lYoe / 4 - lYoe / 100), lMp = (5 * lDoy + 2) / 153;
	anT[2] = (int)(lDoy - (153 * lMp + 2) / 5 + 1);
	anT[1] = (int)(lMp < 10 ? lMp + 3 : lMp - 9);
	anT[0] = (int)(lYoe + lEra * 400 + (anT[1] <= 2 ? 1 : 0));
	anT[3] = (int)(lSod / 3600);
	anT[4] = (int)(lSod / 60 % 60);
	anT[5] = (int)(lSod % 60);
}

inline void LogLine(const char* apcFile, int anLine, eLogLevel aeLevel, const char* apcFormat, ...)
	__attribute__((format(printf, 4, 5)));

inline void
LogLine(const char* apcFile, int anLine, eLogLevel aeLevel, const char* apcFormat, ...)
{
	char acMsg[1024];
	va_list args;
	va_start(args, apcFormat);
	vsnprintf(acMsg, sizeof(acMsg), apcFormat, args);
	va_end(args);

	struct timespec stTs;
	clock_gettime(CLOCK_REALTIME, &stTs);
	// local time without localtime_r, which takes the time-zone lock of glibc
	// (shared with every thread): the offset of the first call (main(), before
	// the tasks; no daylight saving here)
	static const long s_lUtcOff = [] {
		const time_t t = time(NULL);
		struct tm stTm;
		return localtime_r(&t, &stTm) != NULL ? stTm.tm_gmtoff : 0L;
	}();
	int anT[6];
	CivilFromSeconds((long)stTs.tv_sec + s_lUtcOff, anT);
	char acTime[96];
	snprintf(acTime, sizeof(acTime), "%04d-%02d-%02d %02d:%02d:%02d.%03d", anT[0], anT[1], anT[2], anT[3], anT[4],
	         anT[5], (int)(stTs.tv_nsec / 1000000L));

	static const char* const s_apcColor[] = { "\033[0m", "\033[1m\033[33m", "\033[1m\033[31m", "\033[1m\033[32m" };
	static const char* const s_apcLevel[] = { "TRACE", "WARN", "ERROR", "INFO" };
	const int nLevel = (aeLevel >= eLogTrace && aeLevel <= eLogInfo) ? (int)aeLevel : 0;
	char acLine[1200];
	snprintf(acLine, sizeof(acLine), "%s<%s>[%s] %s [%s:%d]\033[0m\n", s_apcColor[nLevel], acTime,
	         s_apcLevel[nLevel], acMsg, apcFile, anLine);
	FILE* fp = (aeLevel == eLogError) ? stderr : stdout;
	fputs(acLine, fp);
}

#define DBG_LOG_TRACE(...)	LogLine(__FILENAME__, __LINE__, eLogTrace, __VA_ARGS__)
#define DBG_LOG_WARN(...)	LogLine(__FILENAME__, __LINE__, eLogWarn, __VA_ARGS__)
#define DBG_LOG_ERROR(...)	LogLine(__FILENAME__, __LINE__, eLogError, __VA_ARGS__)
#define DBG_LOG_INFO(...)	LogLine(__FILENAME__, __LINE__, eLogInfo, __VA_ARGS__)

/* one key from the terminal, without waiting for Enter: getch() without
 * echo, getche() with it */
inline int
ReadKey(bool abEcho)
{
	struct termios stOld, stNew;
	const bool bTty = (0 == tcgetattr(STDIN_FILENO, &stOld));
	if (bTty)
	{
		stNew = stOld;
		stNew.c_lflag &= ~(abEcho ? (tcflag_t)ICANON : (tcflag_t)(ICANON | ECHO));
		tcsetattr(STDIN_FILENO, TCSANOW, &stNew);
	}
	const int nCh = getchar();
	if (bTty)
		tcsetattr(STDIN_FILENO, TCSANOW, &stOld);
	return nCh;
}
inline int getch()  { return ReadKey(false); }
inline int getche() { return ReadKey(true); }

#endif // G2T_COMMON_H
