/*****************************************************************************
*	console_check - the console of the real-time tasks (indy7/src/base/RtConsole.h)
*	and the date of the log lines (CivilFromSeconds, indy7/src/base/Common.h).
*
*	  1. CivilFromSeconds against gmtime_r, 1901..2200, every 3 h 0 min 7 s
*	  2. three threads marked real-time on one core (as the tasks: FIFO
*	     97/96/95 on CPU 0 if the limits allow it, else plain threads) print
*	     numbered lines through the wrapped printf, puts and fputs while a
*	     plain thread prints too; stdout goes into a file, which must hold
*	     every line of every thread once, whole, and in the order of its thread
*	     (or the queue reports the lines it dropped)
*	Linked like indy7_ctrl, with the stdio wrappers (Makefile WRAP_STDIO).
*****************************************************************************/
#include "base/Common.h"
#include "base/RtConsole.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <map>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

static int CheckDates()
{
	int nBad = 0;
	for (long t = -2208988800L; t < 7258118400L; t += 10807)
	{
		const time_t tt = (time_t)t;
		struct tm stTm;
		gmtime_r(&tt, &stTm);
		int anT[6];
		CivilFromSeconds(t, anT);
		if (anT[0] != stTm.tm_year + 1900 || anT[1] != stTm.tm_mon + 1 || anT[2] != stTm.tm_mday ||
		    anT[3] != stTm.tm_hour || anT[4] != stTm.tm_min || anT[5] != stTm.tm_sec)
		{
			if (nBad++ < 5)
				fprintf(stderr, "date of %ld: %04d-%02d-%02d %02d:%02d:%02d, gmtime_r %04d-%02d-%02d %02d:%02d:%02d\n",
				        t, anT[0], anT[1], anT[2], anT[3], anT[4], anT[5], stTm.tm_year + 1900, stTm.tm_mon + 1,
				        stTm.tm_mday, stTm.tm_hour, stTm.tm_min, stTm.tm_sec);
		}
	}
	return nBad;
}

enum { LINES = 3000 };

static void Producer(int anId, bool abRt)
{
	if (abRt)
	{
		cpu_set_t stCpus;
		CPU_ZERO(&stCpus);
		CPU_SET(0, &stCpus);
		pthread_setaffinity_np(pthread_self(), sizeof(stCpus), &stCpus);
		struct sched_param stP;
		stP.sched_priority = 97 - anId;
		pthread_setschedparam(pthread_self(), SCHED_FIFO, &stP);   // may fail without the limits: plain then
		rt_console::MarkRealTime();
	}
	for (int k = 0; k < LINES; k++)
	{
		if (k % 3 == 0)
			printf("P%d %d %s\n", anId, k, "printf");
		else if (k % 3 == 1)
		{
			char acLine[64];
			snprintf(acLine, sizeof(acLine), "P%d %d puts", anId, k);
			puts(acLine);
		}
		else
		{
			char acLine[64];
			snprintf(acLine, sizeof(acLine), "P%d %d fputs\n", anId, k);
			fputs(acLine, stdout);
		}
		if (k % 50 == 49)
			usleep(1000);       // below the rate the writer drains (256 lines per 2 ms)
	}
}

int main()
{
	int nFail = 0;
	const int nBadDates = CheckDates();
	printf("%s dates: %d of 1901..2200 differ from gmtime_r\n", nBadDates ? "FAIL" : "ok  ", nBadDates);
	nFail += nBadDates != 0;
	fflush(stdout);

	char acPath[] = "/tmp/console_check_XXXXXX";
	const int nFd = mkstemp(acPath);
	if (nFd < 0)
	{
		perror("mkstemp");
		return 1;
	}
	const int nOut = dup(STDOUT_FILENO);
	dup2(nFd, STDOUT_FILENO);
	rt_console::Start();
	std::vector<std::thread> vTh;
	for (int i = 0; i < 3; i++)
		vTh.emplace_back(Producer, i, true);
	vTh.emplace_back(Producer, 3, false);
	for (auto& rTh : vTh)
		rTh.join();
	rt_console::Stop();         // writes out the rest; reports drops as "[CONSOLE] N texts ..."
	fflush(stdout);
	dup2(nOut, STDOUT_FILENO);

	FILE* fp = fopen(acPath, "r");
	std::map<int, int> mapNext;
	long nLines = 0, nBad = 0, nDropped = 0;
	char acLine[256];
	while (fp != NULL && fgets(acLine, sizeof(acLine), fp) != NULL)
	{
		int nId, nK;
		char acKind[16];
		if (sscanf(acLine, "[CONSOLE] %ld texts", &nDropped) == 1)
			continue;
		const bool bOk = sscanf(acLine, "P%d %d %15s", &nId, &nK, acKind) == 3 && nId >= 0 && nId < 4 &&
		                 strchr(acLine, '\n') != NULL && nK >= mapNext[nId];
		if (!bOk)
		{
			if (nBad++ < 5)
				fprintf(stderr, "bad or out of order line: %s", acLine);
			continue;
		}
		mapNext[nId] = nK + 1;
		nLines++;
	}
	if (fp != NULL)
		fclose(fp);
	unlink(acPath);
	const long nExpected = 4L * LINES;
	const bool bOk = nBad == 0 && nLines + nDropped == nExpected;
	printf("%s console: %ld of %ld lines whole and in order, %ld dropped by the queue, %ld bad\n",
	       bOk ? "ok  " : "FAIL", nLines, nExpected, nDropped, nBad);
	nFail += !bOk;
	printf("%s: %d check(s) failed\n", nFail ? "FAIL" : "PASS", nFail);
	return nFail ? 1 : 0;
}
