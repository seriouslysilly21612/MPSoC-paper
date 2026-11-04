/*****************************************************************************
*	Name: RtConsole.cpp
*	Description: see RtConsole.h. The queue is the bounded multi-producer
*	queue of D. Vyukov: a producer claims a cell by a compare-and-swap of the
*	head and publishes it with the sequence of the cell; the one consumer is
*	the writer thread. The three real-time tasks share one core and preempt
*	each other, so a claim can be interrupted: the other producers go on with
*	the next cells, and only the writer waits for the interrupted one.
*****************************************************************************/
#include "RtConsole.h"

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <thread>

namespace
{
	enum { CELLS = 256, LEN = 512 };   // 128 kB, locked with the rest of the process

	struct Cell
	{
		std::atomic<unsigned> uSeq;
		bool bErr;                      // stderr, else stdout
		char acText[LEN];
	};

	Cell g_aCells[CELLS];
	std::atomic<unsigned> g_uHead{0};  // producers
	unsigned g_uTail = 0;               // the writer only
	std::atomic<long> g_nDropped{0};
	std::atomic<bool> g_bStop{false};
	std::thread g_th;
	thread_local bool t_bRt = false;

	void Push(bool abErr, const char* apcText, size_t anLen)
	{
		unsigned uPos = g_uHead.load(std::memory_order_relaxed);
		for (;;)
		{
			const int nDif = (int)(g_aCells[uPos % CELLS].uSeq.load(std::memory_order_acquire) - uPos);
			if (nDif == 0)
			{
				if (g_uHead.compare_exchange_weak(uPos, uPos + 1, std::memory_order_relaxed))
					break;
			}
			else if (nDif < 0)
			{
				g_nDropped.fetch_add(1, std::memory_order_relaxed);   // full
				return;
			}
			else
				uPos = g_uHead.load(std::memory_order_relaxed);
		}
		Cell& rCell = g_aCells[uPos % CELLS];
		const bool bNewline = anLen > 0 && apcText[anLen - 1] == '\n';
		if (anLen > LEN - 1)
			anLen = LEN - 1;
		memcpy(rCell.acText, apcText, anLen);
		if (bNewline)                       // a cut line still ends its line
			rCell.acText[anLen - 1] = '\n';
		rCell.acText[anLen] = '\0';
		rCell.bErr = abErr;
		rCell.uSeq.store(uPos + 1, std::memory_order_release);
	}

	void PushV(bool abErr, const char* apcFmt, va_list aArgs)
	{
		char acBuf[LEN];
		int n = vsnprintf(acBuf, sizeof(acBuf), apcFmt, aArgs);
		if (n < 0)
			return;
		if (n >= (int)sizeof(acBuf))   // cut: the line still ends
		{
			n = sizeof(acBuf) - 1;
			acBuf[n - 1] = '\n';
		}
		Push(abErr, acBuf, (size_t)n);
	}

	bool IsStd(FILE* apF) { return apF == stdout || apF == stderr; }

	/* the writer: every published cell in order; true if it wrote one */
	bool Drain()
	{
		bool bAny = false;
		for (;;)
		{
			Cell& rCell = g_aCells[g_uTail % CELLS];
			if ((int)(rCell.uSeq.load(std::memory_order_acquire) - (g_uTail + 1)) != 0)
				break;
			fputs(rCell.acText, rCell.bErr ? stderr : stdout);
			rCell.uSeq.store(g_uTail + CELLS, std::memory_order_release);
			g_uTail++;
			bAny = true;
		}
		if (bAny)
			fflush(stdout);
		return bAny;
	}
}

namespace rt_console
{
	void MarkRealTime() { t_bRt = true; }
	bool IsRealTime() { return t_bRt; }

	void Start()
	{
		for (unsigned i = 0; i < CELLS; i++)
			g_aCells[i].uSeq.store(i, std::memory_order_relaxed);
		g_bStop = false;
		g_th = std::thread([] {
			pthread_setname_np(pthread_self(), "rt-console");
			while (!g_bStop.load(std::memory_order_acquire))
				if (!Drain())
					std::this_thread::sleep_for(std::chrono::milliseconds(2));
			Drain();
		});
	}

	void Stop()
	{
		if (!g_th.joinable())
			return;
		g_bStop.store(true, std::memory_order_release);
		g_th.join();
		const long nDropped = g_nDropped.load();
		if (nDropped > 0)
			printf("[CONSOLE] %ld texts of the real-time tasks dropped (queue full)\n", nDropped);
		const unsigned uLeft = g_uHead.load() - g_uTail;   // claimed but not yet published: a task still printing
		if (uLeft > 0)
			printf("[CONSOLE] %u texts of the real-time tasks not written (a task was still printing)\n", uLeft);
		fflush(stdout);
	}
}

/* the wrapped stdio calls (Makefile WRAP_STDIO: -Wl,--wrap=<name>); with
 * _FORTIFY_SOURCE printf and fprintf are __printf_chk and __fprintf_chk */
extern "C"
{
	int __real_puts(const char*);
	int __real_putchar(int);
	int __real_fputs(const char*, FILE*);
	int __real_fputc(int, FILE*);
	size_t __real_fwrite(const void*, size_t, size_t, FILE*);
	int __real_fflush(FILE*);
	int __vprintf_chk(int, const char*, va_list);
	int __vfprintf_chk(FILE*, int, const char*, va_list);

	int __wrap___printf_chk(int anFlag, const char* apcFmt, ...)
	{
		va_list args;
		va_start(args, apcFmt);
		int n = 0;
		if (t_bRt)
			PushV(false, apcFmt, args);
		else
			n = __vprintf_chk(anFlag, apcFmt, args);
		va_end(args);
		return n;
	}

	int __wrap___fprintf_chk(FILE* apF, int anFlag, const char* apcFmt, ...)
	{
		va_list args;
		va_start(args, apcFmt);
		int n = 0;
		if (t_bRt && IsStd(apF))
			PushV(apF == stderr, apcFmt, args);
		else
			n = __vfprintf_chk(apF, anFlag, apcFmt, args);
		va_end(args);
		return n;
	}

	int __wrap_puts(const char* apcText)
	{
		if (!t_bRt)
			return __real_puts(apcText);
		char acBuf[LEN];
		int n = snprintf(acBuf, sizeof(acBuf), "%s\n", apcText);
		if (n >= (int)sizeof(acBuf))
		{
			n = sizeof(acBuf) - 1;
			acBuf[n - 1] = '\n';
		}
		Push(false, acBuf, (size_t)n);
		return 1;
	}

	int __wrap_putchar(int anC)
	{
		if (!t_bRt)
			return __real_putchar(anC);
		const char c = (char)anC;
		Push(false, &c, 1);
		return anC;
	}

	int __wrap_fputs(const char* apcText, FILE* apF)
	{
		if (!t_bRt || !IsStd(apF))
			return __real_fputs(apcText, apF);
		Push(apF == stderr, apcText, strlen(apcText));
		return 1;
	}

	int __wrap_fputc(int anC, FILE* apF)
	{
		if (!t_bRt || !IsStd(apF))
			return __real_fputc(anC, apF);
		const char c = (char)anC;
		Push(apF == stderr, &c, 1);
		return anC;
	}

	size_t __wrap_fwrite(const void* apData, size_t anSize, size_t anCount, FILE* apF)
	{
		if (!t_bRt || !IsStd(apF))
			return __real_fwrite(apData, anSize, anCount, apF);
		Push(apF == stderr, (const char*)apData, anSize * anCount);
		return anCount;
	}

	int __wrap_fflush(FILE* apF)
	{
		if (t_bRt && (apF == NULL || IsStd(apF)))
			return 0;                   // the writer flushes
		return __real_fflush(apF);
	}
}
