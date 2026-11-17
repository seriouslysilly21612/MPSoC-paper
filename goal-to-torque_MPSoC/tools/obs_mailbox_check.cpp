/*****************************************************************************
*	Name: obs_mailbox_check.cpp
*	Description: The test of the mailbox for the camera results
*	(indy7/src/obs/obs_mailbox.h) in one process: the region is a variable,
*	the bridge and the control task are threads.
*
*	    bin/obs_mailbox_check [--results N] [--unsafe]
*
*	1. A result through the mailbox, byte by byte, with the fields that
*	   obs_put sets.
*	2. The faults, step by step: a region full of garbage; nine results
*	   before the reader looks; a result past its deadline; a writer that
*	   dies in WRITING and a reader that dies in READING, and their restarts;
*	   results of an older epoch; a broken n_check, n = 0; another magic or
*	   layout. A refused read must leave the reader's copy as it was.
*	3. One writer puts N results (default 10,000,000) as fast as it can while
*	   one reader looks at the eight slots in turn, never waiting. Result k
*	   carries k in every field the bridge fills: a copy that mixes two
*	   results breaks this. Every result written must be read exactly once,
*	   and the counters of the head must agree with the writer's.
*	--unsafe frees a slot before copying it in 3, to show that the test sees
*	a mixed copy when there is one.
*	The test passes if no check of 1 and 2 failed and 3 found nothing wrong.
*****************************************************************************/
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "obs/obs_mailbox.h"

namespace {

alignas(64) obs_mailbox_t g_stMb;
uint64_t g_uNow = 1000;			// the system counter of the test; set before the writer runs
int g_nFailed = 0;

volatile obs_mailbox_t* Mb() { return &g_stMb; }
uint64_t Now() { return g_uNow; }
uint32_t State(unsigned b) { return ipc_load32(&Mb()->state[b][0]); }
void SetState(unsigned b, uint32_t u) { ipc_store32(&Mb()->state[b][0], u); }

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

/* result k: k in every field the bridge fills */
void Fill(obs_desc_t* ad, uint32_t auK)
{
	memset(ad, 0, sizeof(*ad));
	ad->t_e = (uint64_t)auK * 1000003u;
	ad->t_valid_until = UINT64_MAX - auK;		// never past it
	for (unsigned i = 0; i < 3; i++)
	{
		const uint32_t u = auK * 3u + i;
		memcpy(&ad->z[i], &u, 4);
	}
	const uint32_t uS = ~auK;
	memcpy(&ad->sigma, &uS, 4);
	ad->mode = (uint8_t)auK;
	ad->flags = (uint8_t)(auK >> 8);
	ad->cls = (uint16_t)(auK >> 16);
	ad->target_seq = auK ^ 0xA5A5A5A5u;
}

/* is ad result k, and which k */
bool Matches(const obs_desc_t& ad, uint32_t* auK)
{
	if (ad.t_e % 1000003u != 0)
		return false;
	obs_desc_t stE;
	*auK = (uint32_t)(ad.t_e / 1000003u);
	Fill(&stE, *auK);
	stE.n = ad.n;
	stE.n_check = ad.n;
	stE.epoch = ad.epoch;
	stE.f = ad.f;
	return ad.n != 0 && !memcmp(&stE, &ad, sizeof(stE));
}

/* one look at slot b expected to give anStatus; a refused look must leave the copy as it was */
bool TakeIs(unsigned b, int anStatus, obs_desc_t* adOut = NULL)
{
	obs_desc_t stD, stWas;
	memset(&stD, 0x5A, sizeof(stD));
	stWas = stD;
	const int nStatus = obs_take(Mb(), b, &stD);
	if (nStatus != anStatus)
		return false;
	if (adOut != NULL)
		*adOut = stD;
	return (nStatus == OBS_OK || nStatus == OBS_STALE) || !memcmp(&stD, &stWas, sizeof(stD));
}

/* a descriptor as a garbage slot might hold it: READY with these words */
void PlantReady(unsigned b, uint32_t auN, uint32_t auNCheck, uint32_t auEpoch)
{
	obs_desc_t stD;
	Fill(&stD, 77);
	stD.n = auN;
	stD.n_check = auNCheck;
	stD.epoch = auEpoch;
	uint32_t au[OBS_DESC_WORDS];
	memcpy(au, &stD, sizeof(au));
	for (unsigned i = 0; i < OBS_DESC_WORDS; i++)
		ipc_store32(&Mb()->slot[b][i], au[i]);
	SetState(b, OBS_READY);
}

void Payload()
{
	memset(&g_stMb, 0, sizeof(g_stMb));
	obs_reader_start(Mb());
	const uint32_t uEpoch = obs_writer_start(Mb());
	g_uNow = 1000;

	obs_desc_t stD, stR;
	for (size_t i = 0; i < sizeof(stD); i++)
		((unsigned char*)&stD)[i] = (unsigned char)(7 * i + 1);
	stD.t_valid_until = 5000;
	Expect(obs_put(Mb(), &stD, Now) == OBS_OK && stD.n == 1 && stD.n_check == 1 && stD.epoch == uEpoch &&
	       stD.s == 0 && stD.f == 1000 && State(0) == OBS_READY, "a result into slot 0, with n, epoch, f, s, n_check");
	Expect(TakeIs(1, OBS_EMPTY), "slot 1 is empty");
	Expect(TakeIs(0, OBS_OK, &stR) && !memcmp(&stR, &stD, sizeof(stD)) && State(0) == OBS_FREE,
	       "the result out of the mailbox, byte by byte; the slot FREE again");
	Expect(TakeIs(0, OBS_EMPTY), "a slot is read once");
}

void Faults()
{
	obs_desc_t stD, stR;

	// a region full of garbage. The head's epoch is known, so the planted results can be made to fail one check each
	uint32_t uX = 2463534242u;
	for (uint32_t* pu = (uint32_t*)&g_stMb; pu < (uint32_t*)(&g_stMb + 1); pu++)
	{
		uX ^= uX << 13; uX ^= uX >> 17; uX ^= uX << 5;
		*pu = uX;
	}
	ipc_store32(&Mb()->magic, OBS_MAGIC ^ 0x100u);
	const uint32_t uOld = ipc_load32(&Mb()->epoch), uNew = (uOld + 1u == 0u) ? 1u : uOld + 1u;
	PlantReady(0, 0, 0, uNew);			// n = 0
	SetState(1, OBS_WRITING);
	PlantReady(2, 9, 10, uNew);			// a broken n_check
	SetState(3, OBS_READING);
	SetState(4, 0xDEADBEEFu);
	PlantReady(5, 5, 5, uOld);			// the epoch before
	SetState(6, OBS_FREE);
	SetState(7, 7u);
	bool bOk = true;
	for (unsigned b = 0; b < OBS_SLOTS; b++)
		bOk = bOk && TakeIs(b, OBS_NO_PEER);
	Expect(bOk && State(0) == OBS_READY && State(3) == OBS_READING, "garbage, before the writer started: OBS_NO_PEER, no state changed");
	obs_reader_start(Mb());
	Expect(State(3) == OBS_FREE && State(0) == OBS_READY && State(1) == OBS_WRITING && State(4) == 0xDEADBEEFu,
	       "the reader's start frees READING and nothing else");
	Expect(obs_writer_start(Mb()) == uNew && State(1) == OBS_FREE && State(4) == OBS_FREE && State(7) == OBS_FREE &&
	       State(0) == OBS_READY && State(2) == OBS_READY && State(5) == OBS_READY,
	       "the writer's start frees WRITING and garbage, keeps READY, counts the epoch up");
	Expect(TakeIs(0, OBS_STALE, &stR) && stR.n == 0 && State(0) == OBS_FREE, "n = 0: OBS_STALE, the slot freed");
	Expect(TakeIs(2, OBS_STALE, &stR) && stR.n == 9, "a broken n_check: OBS_STALE");
	Expect(TakeIs(5, OBS_STALE, &stR) && stR.epoch == uOld, "the epoch before: OBS_STALE");

	// all eight slots in use after garbage; nine results before the reader looks: the ninth is dropped and counted
	g_uNow = 2000;
	bOk = true;
	for (uint32_t k = 1; k <= 8; k++)
	{
		Fill(&stD, k);
		bOk = bOk && obs_put(Mb(), &stD, Now) == OBS_OK && stD.n == k;
	}
	Fill(&stD, 9);
	Expect(bOk && obs_put(Mb(), &stD, Now) == OBS_FULL && ipc_load32(&Mb()->dropped_full) == 1 &&
	       ipc_load32(&Mb()->n_last) == 8, "eight results in, the ninth dropped and counted");
	bOk = true;
	for (unsigned b = 0; b < OBS_SLOTS; b++)
	{
		uint32_t uK = 0;
		bOk = bOk && TakeIs(b, OBS_OK, &stR) && Matches(stR, &uK) && uK == b + 1 && stR.n == b + 1;
	}
	Expect(bOk && State(7) == OBS_FREE, "the eight results out, in the order of the slots");

	// a result past its deadline is not written; one at its deadline is
	Fill(&stD, 10);
	stD.t_valid_until = 1999;
	Expect(obs_put(Mb(), &stD, Now) == OBS_EXPIRED && ipc_load32(&Mb()->dropped_expired) == 1 &&
	       ipc_load32(&Mb()->n_last) == 8 && State(0) == OBS_FREE, "past t_valid_until: not written, counted");
	stD.t_valid_until = 2000;
	Expect(obs_put(Mb(), &stD, Now) == OBS_OK && TakeIs(0, OBS_OK, &stR) && stR.n == 9, "at t_valid_until: written");

	// the writer dies in WRITING (half a result in slot 1), after a complete result in slot 0; then it restarts
	Fill(&stD, 11);
	Expect(obs_put(Mb(), &stD, Now) == OBS_OK && State(0) == OBS_READY, "a result in slot 0");
	SetState(1, OBS_WRITING);
	for (unsigned i = 0; i < OBS_DESC_WORDS / 2; i++)
		ipc_store32(&Mb()->slot[1][i], 0xFFFFFFFFu);
	Expect(TakeIs(1, OBS_EMPTY), "a write that never ends: OBS_EMPTY");
	const uint32_t uE2 = obs_writer_start(Mb());
	Expect(uE2 == uNew + 1u && State(1) == OBS_FREE && State(0) == OBS_READY && ipc_load32(&Mb()->n_last) == 0,
	       "the writer restarted: WRITING freed, READY kept, n from 1 again");
	Expect(TakeIs(0, OBS_STALE, &stR) && stR.epoch == uNew, "the result written before the restart: OBS_STALE");
	Fill(&stD, 12);
	Expect(obs_put(Mb(), &stD, Now) == OBS_OK && stD.n == 1 && stD.epoch == uE2 && TakeIs(0, OBS_OK, &stR) &&
	       stR.n == 1, "the first result after the restart");

	// the reader dies in READING: the writer goes around the slot, the reader's restart frees it
	Fill(&stD, 13);
	Expect(obs_put(Mb(), &stD, Now) == OBS_OK && State(0) == OBS_READY, "a result in slot 0");
	SetState(0, OBS_READING);
	Fill(&stD, 14);
	Expect(obs_put(Mb(), &stD, Now) == OBS_OK && State(1) == OBS_READY && State(0) == OBS_READING,
	       "the writer leaves a READING slot alone");
	SetState(2, OBS_READING);
	SetState(3, OBS_READY);
	const uint32_t uE3 = obs_writer_start(Mb());
	Expect(State(0) == OBS_READING && State(2) == OBS_READING && State(1) == OBS_READY,
	       "a writer's restart leaves READING and READY alone");
	obs_reader_start(Mb());
	Expect(State(0) == OBS_FREE && State(2) == OBS_FREE && State(1) == OBS_READY && State(3) == OBS_READY,
	       "the reader's restart frees READING, leaves READY alone");
	Expect(TakeIs(1, OBS_STALE) && TakeIs(3, OBS_STALE), "the READY results of before: OBS_STALE");

	// a writer that is not this layout
	Fill(&stD, 15);
	Expect(obs_put(Mb(), &stD, Now) == OBS_OK && stD.epoch == uE3, "a result");
	ipc_store32(&Mb()->magic, OBS_MAGIC ^ 1u);
	Expect(TakeIs(0, OBS_NO_PEER) && State(0) == OBS_READY, "another magic: OBS_NO_PEER, the slot untouched");
	ipc_store32(&Mb()->magic, OBS_MAGIC);
	ipc_store32(&Mb()->layout, OBS_LAYOUT + 1u);
	Expect(TakeIs(0, OBS_NO_PEER) && State(0) == OBS_READY, "another layout: OBS_NO_PEER, the slot untouched");
	ipc_store32(&Mb()->layout, OBS_LAYOUT);
	uint32_t uK = 0;
	Expect(TakeIs(0, OBS_OK, &stR) && Matches(stR, &uK) && uK == 15, "magic and layout restored");
}

/* the reader of --unsafe: frees the slot before copying it */
int TakeUnsafe(unsigned b, obs_desc_t* ad)
{
	uint32_t au[OBS_DESC_WORDS];
	if (State(b) != OBS_READY)
		return OBS_EMPTY;
	SetState(b, OBS_FREE);
	for (unsigned i = 0; i < OBS_DESC_WORDS; i++)
		au[i] = ipc_load32(&Mb()->slot[b][i]);
	memcpy(ad, au, sizeof(au));
	return OBS_OK;
}

} // namespace

int main(int argc, char** argv)
{
	long nResults = 10000000;
	bool bUnsafe = false;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "--results") && i + 1 < argc) nResults = atol(argv[++i]);
		else if (!strcmp(argv[i], "--unsafe")) bUnsafe = true;
		else
		{
			fprintf(stderr, "usage: %s [--results N] [--unsafe]\n", argv[0]);
			return 2;
		}
	}
	if (nResults < 1 || nResults > 0x7FFFFFFFL)
		return 2;

	printf("layout %u: %u slots of %u B, head %u B, states %u B, %u B in all\n", (unsigned)OBS_LAYOUT,
	       (unsigned)OBS_SLOTS, (unsigned)sizeof(obs_desc_t), (unsigned)offsetof(obs_mailbox_t, state),
	       (unsigned)(offsetof(obs_mailbox_t, slot) - offsetof(obs_mailbox_t, state)), (unsigned)sizeof(obs_mailbox_t));
	Payload();
	Faults();
	printf("payload and faults: %d checks failed\n", g_nFailed);

	// part 3
	memset(&g_stMb, 0, sizeof(g_stMb));
	obs_reader_start(Mb());
	obs_writer_start(Mb());
	g_uNow = 1000;
	std::vector<unsigned char> vWritten((size_t)nResults + 1, 0), vRead((size_t)nResults + 1, 0);
	std::atomic<bool> bDone(false);
	long nOk = 0, nFull = 0, nOther = 0;
	long nRead = 0, nEmpty = 0, nStale = 0, nBad = 0, nTwice = 0, nNotWritten = 0;

	const auto tStart = std::chrono::steady_clock::now();
	std::thread cReader([&]() {
		obs_desc_t stD;
		for (bool bLast = false; !bLast; )
		{
			bLast = bDone.load();			// one more round after the last result
			for (unsigned b = 0; b < OBS_SLOTS; b++)
			{
				const int nStatus = bUnsafe ? TakeUnsafe(b, &stD) : obs_take(Mb(), b, &stD);
				if (nStatus == OBS_EMPTY) { nEmpty++; continue; }
				if (nStatus != OBS_OK) { nStale++; continue; }
				uint32_t uK = 0;
				if (!Matches(stD, &uK) || uK == 0 || uK > (uint32_t)nResults) { nBad++; continue; }
				nRead++;
				if (vRead[uK]) nTwice++;
				vRead[uK] = 1;
			}
		}
	});
	std::thread cWriter([&]() {
		obs_desc_t stD;
		for (long k = 1; k <= nResults; k++)
		{
			Fill(&stD, (uint32_t)k);
			const int nStatus = obs_put(Mb(), &stD, Now);
			if (nStatus == OBS_OK) { nOk++; vWritten[(size_t)k] = 1; }
			else if (nStatus == OBS_FULL) nFull++;
			else nOther++;
		}
		bDone.store(true);
	});
	cWriter.join();
	cReader.join();
	const double dSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();

	long nLost = 0;
	for (long k = 1; k <= nResults; k++)
	{
		if (vWritten[(size_t)k] && !vRead[(size_t)k]) nLost++;
		if (vRead[(size_t)k] && !vWritten[(size_t)k]) nNotWritten++;
	}
	const bool bCounters = ipc_load32(&Mb()->n_last) == (uint32_t)nOk && ipc_load32(&Mb()->dropped_full) == (uint32_t)nFull &&
	                       ipc_load32(&Mb()->dropped_expired) == 0 && nOk + nFull == nResults && nOther == 0;
	printf("%s: %ld results in %.1f s; written %ld, dropped (full) %ld; read %ld, empty looks %ld, stale %ld\n",
	       bUnsafe ? "UNSAFE (freed before the copy)" : "mailbox", nResults, dSeconds, nOk, nFull, nRead, nEmpty, nStale);
	printf("  mixed copies %ld, read twice %ld, written and never read %ld, read but not written %ld; counters %s\n",
	       nBad, nTwice, nLost, nNotWritten, bCounters ? "agree" : "DISAGREE");
	const bool bPass = g_nFailed == 0 && nBad == 0 && nTwice == 0 && nLost == 0 && nNotWritten == 0 && nStale == 0 &&
	                   nRead == nOk && bCounters && nOk > 0;
	printf("%s: %d checks failed, %ld mixed copies, %ld results lost or doubled\n", bPass ? "PASS" : "FAIL", g_nFailed,
	       nBad, nLost + nTwice + nNotWritten);
	return bPass ? 0 : 1;
}
