/*****************************************************************************
*	Name: obs_status_check.cpp
*	Description: The test of what the control task tells the camera side
*	(indy7/src/obs/obs_status.h) and of the check value of the contract
*	(indy7/src/obs/ObsContract.h).
*
*	    bin/obs_status_check [writes, default 2000000]
*
*	1. The slot between two threads: a writer that writes one status after
*	   the other, every field made from one counter, and a reader that reads
*	   as fast as it can. No copy mixes two statuses, and no read goes back.
*	2. Before the writer is up there is nothing to read; a writer that
*	   starts again is not read before its first write, and what it wrote
*	   before is never read as new.
*	3. The largest of the newest values: of the last full window and the
*	   window being filled.
*	4. The check value of the contract: that of zlib for the same bytes, and
*	   another one as soon as one constant differs.
*	5. Eq. (7) against five values. The test of the camera side has the same
*	   five (ros2_ws/src/vitis_ai_detector_pkg/test/test_contract.py): the
*	   two sides compute the same bound.
*	The test passes if no check failed.
*****************************************************************************/
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "obs/ObsContract.h"
#include "obs/obs_receiver.h"
#include "obs/obs_status.h"

namespace {

int g_nFailed = 0;

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

/* the status number auCount: every field follows from the number */
obs_status_t StatusOf(uint32_t auCount)
{
	obs_status_t st;
	memset(&st, 0, sizeof(st));
	st.t_release = 1000000ull * auCount;
	st.result_t_e = st.t_release - 180000000ull;
	st.result_t_valid_until = st.result_t_e + 250000000ull;
	st.result_n = auCount;
	st.target_seq = auCount / 1000u;
	st.result_error_bound = 0.005f + 1e-6f * (float)(auCount % 1000u);
	st.tolerance = 0.010f;
	st.response_max = 1e-6f * (float)(auCount % 500u);
	st.contract_crc = ~auCount;
	st.control_period = 0.001f;
	st.method = (uint8_t)(auCount % 5u);
	st.flags = (uint8_t)(auCount % 4u);
	st.result_mode = (uint8_t)(auCount % 2u);
	return st;
}

/* 1 */
void BetweenTwoThreads(uint32_t auWrites)
{
	printf("1. the slot between two threads, %u writes\n", auWrites);
	static obs_status_region_t s_stRegion;
	memset(&s_stRegion, 0, sizeof(s_stRegion));
	obs_status_start(&s_stRegion);

	std::atomic<bool> bDone{false};
	long nRead = 0, nMixed = 0, nBack = 0, nFailedRead = 0;
	std::thread cReader([&]() {
		obs_status_t st;
		uint32_t auStamp[2] = { 0, 0 };
		uint32_t uLast = 0;
		while (!bDone.load(std::memory_order_acquire))
		{
			if (obs_status_read(&s_stRegion, &st, auStamp) != IPC_OK)
			{
				nFailedRead++;
				continue;
			}
			nRead++;
			const obs_status_t stExpected = StatusOf(st.result_n);
			if (0 != memcmp(&st, &stExpected, sizeof(st)))
				nMixed++;
			if (st.result_n < uLast)
				nBack++;
			uLast = st.result_n;
		}
	});
	for (uint32_t uCount = 1; uCount <= auWrites; uCount++)
	{
		const obs_status_t st = StatusOf(uCount);
		obs_status_write(&s_stRegion, &st);
	}
	bDone.store(true, std::memory_order_release);
	cReader.join();
	printf("   %ld reads, %ld attempts during a write\n", nRead, nFailedRead);
	Expect(nRead > 0, "the reader read at least once");
	Expect(nMixed == 0, "no copy mixes two statuses");
	Expect(nBack == 0, "no read is older than the one before");
}

/* 2 */
void StartAndRestart()
{
	printf("2. the start of the writer\n");
	obs_status_region_t stRegion;
	memset(&stRegion, 0xA5, sizeof(stRegion));			// whatever the memory held: the start zeroes nothing
	obs_status_t st = StatusOf(5), stRead = StatusOf(9);
	uint32_t auStamp[2] = { 0, 0 };
	Expect(obs_status_read(&stRegion, &stRead, auStamp) == IPC_NO_PEER && stRead.result_n == 9,
	       "no writer yet: nothing is read and the copy stays");
	obs_status_start(&stRegion);
	Expect(obs_status_read(&stRegion, &stRead, auStamp) == IPC_BUSY, "the writer is up but has not written: nothing is read");
	obs_status_write(&stRegion, &st);
	Expect(obs_status_read(&stRegion, &stRead, auStamp) == IPC_OK && stRead.result_n == 5, "after the first write: read");
	const uint32_t uEpoch = auStamp[0];

	obs_status_start(&stRegion);						// the control task starts again
	Expect(obs_status_read(&stRegion, &stRead, auStamp) == IPC_BUSY && stRead.result_n == 5,
	       "a writer that started again: its old status is not read as new");
	st = StatusOf(6);
	obs_status_write(&stRegion, &st);
	Expect(obs_status_read(&stRegion, &stRead, auStamp) == IPC_OK && stRead.result_n == 6 && auStamp[0] == uEpoch + 1,
	       "its first write is read, with the next epoch");
}

/* 3 */
void RecentMax()
{
	printf("3. the largest of the newest values\n");
	obs_recent_max_t st;
	obs_recent_max_start(&st, 4);
	Expect(obs_recent_max(&st) == 0.0f, "no value yet: 0");
	obs_recent_max_add(&st, 1.0f);
	Expect(obs_recent_max(&st) == 1.0f, "the first value");
	obs_recent_max_add(&st, 3.0f);
	Expect(obs_recent_max(&st) == 3.0f, "a larger one after it");
	obs_recent_max_add(&st, 2.0f);
	obs_recent_max_add(&st, 1.0f);						// the first window is full: 1 3 2 1
	Expect(obs_recent_max(&st) == 3.0f, "a full window: its largest");
	obs_recent_max_add(&st, 9.0f);						// a peak in the second window
	Expect(obs_recent_max(&st) == 9.0f, "a larger value in the window being filled shows at once");
	for (int i = 0; i < 3; i++)
		obs_recent_max_add(&st, 1.0f);					// the second window is full: 9 1 1 1
	for (int i = 0; i < 3; i++)
		obs_recent_max_add(&st, 2.0f);
	Expect(obs_recent_max(&st) == 9.0f, "the peak is still there 6 values later (window 4: for 4 to 7 values)");
	obs_recent_max_add(&st, 2.0f);						// the third window is full: 2 2 2 2
	Expect(obs_recent_max(&st) == 2.0f, "and gone once the window after its own is full");
	obs_recent_max_start(&st, 0);
	obs_recent_max_add(&st, 5.0f);
	obs_recent_max_add(&st, 4.0f);
	Expect(obs_recent_max(&st) == 4.0f, "a window of 0 values is one of 1");
}

/* 4 */
void ContractCheckValue()
{
	printf("4. the check value of the contract\n");
	ST_CONFIG_CONTRACT st;
	st.dDeltaM = 10 * 1e-3;				// as the configuration reads DELTA_MM=10: the number times the scale
	st.dAgeCapS = 250 * 1e-3;
	st.dE0M = 1 * 1e-3;
	st.dG = 1.0;
	st.dEpsV = 0.0;
	st.dAMax = 0.0;
	st.dEpsCalM = 2 * 1e-3;
	st.vEpsM[0] = 2 * 1e-3;
	st.vEpsM[1] = 3 * 1e-3;
	// zlib.crc32(struct.pack('<I', 1) + b''.join(struct.pack('<d', v) for v in the nine constants))
	Expect(ObsContractCrc(st) == 0x2E7B0347u, "the constants of INDY7.cfg: 0x2E7B0347, as zlib");
	const uint32_t uCrc = ObsContractCrc(st);
	double* const apdConstant[9] = { &st.dDeltaM, &st.dAgeCapS, &st.dE0M, &st.dG, &st.dEpsV, &st.dAMax, &st.dEpsCalM,
	                                 &st.vEpsM[0], &st.vEpsM[1] };
	bool bAllDiffer = true;
	for (double* pdConstant : apdConstant)
	{
		const double dKept = *pdConstant;
		*pdConstant = dKept + 1e-6;
		bAllDiffer = bAllDiffer && ObsContractCrc(st) != uCrc;
		*pdConstant = dKept;
	}
	Expect(bAllDiffer, "each of the nine constants changes the check value");
	st.vTagOffset[0] += 0.01;
	Expect(ObsContractCrc(st) == uCrc, "TAG_OFFSET_M does not: only the control task uses it");

	st.dDeltaM = 12.5 * 1e-3;
	st.dAgeCapS = 180 * 1e-3;
	st.dE0M = 0.7 * 1e-3;
	st.dG = 1.5;
	st.dEpsV = 0.05;
	st.dAMax = 0.3;
	st.dEpsCalM = 1.2 * 1e-3;
	st.vEpsM[0] = 2.5 * 1e-3;
	st.vEpsM[1] = 4 * 1e-3;
	Expect(ObsContractCrc(st) == 0x21A36B83u, "other constants: 0x21A36B83, as zlib");
}

/* 5 */
void ErrorBound()
{
	printf("5. eq. (7), the values that the camera side tests too\n");
	obs_settings_t stCfg, stOther;
	memset(&stCfg, 0, sizeof(stCfg));
	stCfg.e0 = 1 * 1e-3;				// INDY7.cfg
	stCfg.g = 1.0;
	stCfg.eps_cal = 2 * 1e-3;
	memset(&stOther, 0, sizeof(stOther));
	stOther.e0 = 0.7 * 1e-3;
	stOther.g = 1.5;
	stOther.eps_v = 0.05;
	stOther.a_max = 0.3;
	stOther.eps_cal = 1.2 * 1e-3;
	// the error of a mode is a 32-bit float in the mailbox
	const struct { const obs_settings_t* pSet; float fSigma; double dAgeS; double dBound; } astCase[5] = {
		{ &stCfg, (float)(2 * 1e-3), 0.18, 0.0050000000949949027 },
		{ &stCfg, (float)(3 * 1e-3), 0.25, 0.0060000000260770322 },
		{ &stOther, (float)(2.5 * 1e-3), 0.2, 0.030249999916180968 },
		{ &stOther, (float)(4 * 1e-3), 0.0, 0.0085000002849847066 },
		{ &stOther, (float)(4 * 1e-3), 0.137, 0.022998025284984711 },
	};
	bool bAll = true;
	for (const auto& stC : astCase)
		bAll = bAll && std::fabs(obs_error_bound(stC.pSet, stC.fSigma, stC.dAgeS) - stC.dBound) < 1e-15;
	Expect(bAll, "the five bounds");
}

}	// namespace

int main(int argc, char** argv)
{
	const long nWrites = argc > 1 ? atol(argv[1]) : 2000000;
	if (nWrites <= 0 || nWrites > 2000000000L)
	{
		printf("usage: obs_status_check [writes, default 2000000]\n");
		return 2;
	}
	BetweenTwoThreads((uint32_t)nWrites);
	StartAndRestart();
	RecentMax();
	ContractCheckValue();
	ErrorBound();
	printf("%s: %d check(s) failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
