/*****************************************************************************
*	Name: obs_receiver_check.cpp
*	Description: The test of the control task's side of the camera results
*	(indy7/src/obs/obs_receiver.h) in one process: the mailbox is a variable,
*	the system counter a number that the test sets. A period is 1000 ticks.
*
*	    bin/obs_receiver_check [--periods N] [--results N]
*
*	1. The states: the angles at a capture time against the two states
*	   around it by definition, with times that jitter, periods without a
*	   state, captures outside the record, the wrap of the period number and
*	   a period number that starts again. Eq. (7) against numbers.
*	2. The verdicts, case by case, under both policies: a result in time; one
*	   published after the release; two in a period, and an older capture
*	   that comes later, also after the one used has gone; past the deadline
*	   on arrival and while held, and the cap on the writer's deadline; no
*	   state at the capture, no tag; results that cannot be; an error bound
*	   above the tolerance, and the tolerance of a later period; a ninth held
*	   result, and sixteen verdicts in one period; the two formats, with a
*	   tag that moved since the capture; no writer; settings that cannot be
*	   used.
*	3. N periods (default 200,000) of random results under each policy, with
*	   bursts, a writer that restarts, results planted twice, periods without
*	   a state and a control task that stalls. In every period the receiver
*	   must agree with a second one, written from Alg. 1 with a vector (the
*	   result in use, the held results, the verdicts), and with eq. (3)
*	   itself; at the end every result written has exactly one verdict.
*	4. One writer thread with N results (default 1,000,000) against the
*	   receiver in another: every result written leaves whole and with
*	   exactly one verdict, and eq. (3) holds in every period.
*	The test passes if no check failed.
*****************************************************************************/
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <thread>
#include <tuple>
#include <vector>

#include "obs/obs_receiver.h"

namespace {

const uint64_t T = 1000;				// a period [ticks]
const uint64_t CAP = 250 * T;			// the longest age
const double TOL = 0.010;				// the tolerance of a period [m]
const float SIG_IN = 0.002f;			// a sigma within TOL up to an age of 107 ms
const float SIG_OVER = 0.020f;			// one above TOL at every age
const char* const VERDICT[OBS_VERDICTS] = { "accepted", "superseded", "expired", "state_unrecoverable", "seq_fail",
                                            "error_exceeded", "evicted" };

alignas(64) obs_mailbox_t g_stMb;
obs_receiver_t g_stR;
std::atomic<uint64_t> g_uNow(0);		// the system counter of the test: what obs_put reads
int g_nFailed = 0;

volatile obs_mailbox_t* Mb() { return &g_stMb; }
uint64_t Now() { return g_uNow.load(); }
uint64_t Ms(double adMs) { return (uint64_t)llround(adMs * 1000.0); }
uint64_t Key(const obs_desc_t& ad) { return ((uint64_t)ad.epoch << 32) | ad.n; }

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

obs_settings_t Settings(int anPolicy)
{
	obs_settings_t st;
	memset(&st, 0, sizeof(st));
	st.policy = anPolicy;
	st.period_ticks = T;
	st.age_cap_ticks = CAP;
	st.ticks_per_s = 1e6;
	st.e0 = 0.001;
	st.g = 1.0;
	st.eps_v = 0.02;
	st.a_max = 0.5;
	st.eps_cal = 0.002;
	return st;
}

struct Rng
{
	uint32_t u;
	uint32_t Next() { u ^= u << 13; u ^= u >> 17; u ^= u << 5; return u; }
	uint32_t Below(uint32_t auN) { return Next() % auN; }
	bool Chance(double adP) { return Next() < (uint32_t)(adP * 4294967296.0); }
};

/* ---- the states that the test pushed, and the angles at a time by definition ---- */

struct TestState { bool bSet; uint32_t uK; uint64_t t; float afQ[OBS_DOF]; };
TestState g_astState[1024];				// [k % 1024]
uint32_t g_uKNew = 0;
bool g_bAnyState = false;

void Push(uint32_t auK, uint64_t auT)
{
	TestState& st = g_astState[auK % 1024];
	st.bSet = true;
	st.uK = auK;
	st.t = auT;
	for (int j = 0; j < OBS_DOF; j++)
		st.afQ[j] = (float)sin(0.013 * (double)(auK % 100000u) + (double)j);
	g_uKNew = auK;
	g_bAnyState = true;
	obs_state_push(&g_stR, auK, auT, st.afQ);
}

/* the states of two consecutive periods around auTe, no further apart than 1.5 periods, of the last 256 periods */
bool Truth(uint64_t auTe, uint32_t* auS, double* adQ)
{
	for (uint32_t uBack = 1; g_bAnyState && uBack < OBS_STATES; uBack++)
	{
		const uint32_t uP = g_uKNew - uBack;
		const TestState& a = g_astState[uP % 1024];
		const TestState& b = g_astState[(uP + 1u) % 1024];
		if (!a.bSet || a.uK != uP || !b.bSet || b.uK != uP + 1u || a.t > auTe || auTe >= b.t || b.t - a.t > T + T / 2)
			continue;
		const double dW = (double)(auTe - a.t) / (double)(b.t - a.t);
		for (int j = 0; j < OBS_DOF; j++)
			adQ[j] = (double)a.afQ[j] + dW * ((double)b.afQ[j] - (double)a.afQ[j]);
		*auS = uP;
		return true;
	}
	return false;
}

bool QNear(const float* afQ, const double* adQ)
{
	for (int j = 0; j < OBS_DOF; j++)
		if (!(fabs((double)afQ[j] - adQ[j]) < 1e-5))
			return false;
	return true;
}

/* obs_state_at against the definition; *abFound and *auS: what it gave */
bool StateAgrees(uint64_t auTe, bool* abFound = NULL, uint32_t* auS = NULL)
{
	float afQ[OBS_DOF];
	double adQ[OBS_DOF];
	uint32_t uS = 0, uSTruth = 0;
	const bool bGot = obs_state_at(&g_stR, auTe, afQ, &uS) != 0;
	const bool bTruth = Truth(auTe, &uSTruth, adQ);
	if (abFound != NULL) *abFound = bGot;
	if (auS != NULL) *auS = uS;
	return bGot == bTruth && (!bGot || (uS == uSTruth && QNear(afQ, adQ)));
}

/* the state at auTe is that of period auS (and the next) */
bool StateIs(uint64_t auTe, uint32_t auS)
{
	bool bFound = false;
	uint32_t uS = 0;
	return StateAgrees(auTe, &bFound, &uS) && bFound && uS == auS;
}

bool NoState(uint64_t auTe)
{
	bool bFound = true;
	return StateAgrees(auTe, &bFound) && !bFound;
}

/* ---- the receiver again, written from Alg. 1 with a vector ---- */

struct ModelHeld { obs_desc_t d; double adQ[OBS_DOF]; uint64_t tIn; bool bAccepted, bOver; };
typedef std::tuple<uint64_t, int, uint64_t> ModelRec;		// the result, its verdict, the period that took it
std::vector<ModelHeld> g_vModel;		// the newest capture first
uint64_t g_uModelUsed = 0;				// the capture of the newest result that the model used

/* one period of the model on what the mailbox holds now: before obs_step, which empties it. Returns the index in
 * g_vModel of the result to use, -1: none. */
int ModelStep(int anPolicy, uint64_t atK, double adTol, bool* abFresh, std::vector<ModelRec>* avRec)
{
	const obs_settings_t stSet = Settings(anPolicy);
	avRec->clear();
	*abFresh = false;
	const bool bPeer = ipc_load32(&Mb()->magic) == OBS_MAGIC && ipc_load32(&Mb()->layout) == OBS_LAYOUT;
	for (unsigned b = 0; bPeer && b < OBS_SLOTS; b++)
	{
		if (ipc_load32(&Mb()->state[b][0]) != OBS_READY)
			continue;
		ModelHeld stH;
		memset(&stH, 0, sizeof(stH));
		uint32_t au[OBS_DESC_WORDS];
		for (unsigned i = 0; i < OBS_DESC_WORDS; i++)
			au[i] = ipc_load32(&Mb()->slot[b][i]);
		memcpy(&stH.d, au, sizeof(au));
		stH.tIn = atK;
		const obs_desc_t& d = stH.d;
		bool bBad = d.n == 0 || d.n_check != d.n || d.epoch != ipc_load32(&Mb()->epoch) || d.t_e > d.f || d.t_e > atK ||
		            !(d.sigma >= 0.0f) || ((d.flags & OBS_Z_TAG) != 0) == ((d.flags & OBS_Z_BASE) != 0);
		for (const ModelHeld& h : g_vModel)
			bBad = bBad || Key(h.d) == Key(d);
		if (bBad)
		{
			avRec->push_back(ModelRec(Key(d), OBS_V_SEQ_FAIL, atK));
			continue;
		}
		stH.d.t_valid_until = std::min(d.t_valid_until, d.t_e + CAP);
		if (atK > d.t_valid_until)
		{
			avRec->push_back(ModelRec(Key(d), OBS_V_EXPIRED, atK));
			continue;
		}
		if (d.t_e <= g_uModelUsed)
		{
			avRec->push_back(ModelRec(Key(d), OBS_V_SUPERSEDED, atK));
			continue;
		}
		bool bState = false;
		if (d.flags & OBS_Z_BASE)
			bState = true;
		else if (d.flags & OBS_TAG_LOST)
			;
		else if (anPolicy == OBS_CONTRACT)
			bState = Truth(d.t_e, &stH.d.s, stH.adQ);
		else if (g_bAnyState)
		{
			bState = true;
			stH.d.s = g_uKNew;
			for (int j = 0; j < OBS_DOF; j++)
				stH.adQ[j] = g_astState[g_uKNew % 1024].afQ[j];
		}
		if (!bState)
		{
			avRec->push_back(ModelRec(Key(d), OBS_V_STATE_UNRECOVERABLE, atK));
			continue;
		}
		if (g_vModel.size() == OBS_HELD)
		{
			// the oldest capture of the nine goes; of two equally old, the one that came later
			if (d.t_e <= g_vModel.back().d.t_e)
			{
				avRec->push_back(ModelRec(Key(d), OBS_V_EVICTED, atK));
				continue;
			}
			if (!g_vModel.back().bAccepted)
				avRec->push_back(ModelRec(Key(g_vModel.back().d), OBS_V_EVICTED, g_vModel.back().tIn));
			g_vModel.pop_back();
		}
		std::vector<ModelHeld>::iterator it = g_vModel.begin();
		while (it != g_vModel.end() && it->d.t_e >= d.t_e)
			++it;
		g_vModel.insert(it, stH);
	}

	int nUse = -1;
	std::vector<ModelHeld> vKeep;
	for (ModelHeld& h : g_vModel)
	{
		if (nUse >= 0)
		{
			if (!h.bAccepted)
				avRec->push_back(ModelRec(Key(h.d), OBS_V_SUPERSEDED, h.tIn));
			continue;
		}
		if (atK > h.d.t_valid_until)
		{
			if (!h.bAccepted)
				avRec->push_back(ModelRec(Key(h.d), h.bOver ? OBS_V_ERROR_EXCEEDED : OBS_V_EXPIRED, h.tIn));
			continue;
		}
		if (h.d.f <= atK)
		{
			if (anPolicy != OBS_CONTRACT ||
			    obs_error_bound(&stSet, h.d.sigma, (double)(atK - h.d.t_e) / stSet.ticks_per_s) <= adTol)
				nUse = (int)vKeep.size();
			else
				h.bOver = true;
		}
		vKeep.push_back(h);
	}
	g_vModel.swap(vKeep);
	if (nUse >= 0 && !g_vModel[(size_t)nUse].bAccepted)
	{
		g_vModel[(size_t)nUse].bAccepted = true;
		g_uModelUsed = g_vModel[(size_t)nUse].d.t_e;
		*abFresh = true;
		avRec->push_back(ModelRec(Key(g_vModel[(size_t)nUse].d), OBS_V_ACCEPTED, g_vModel[(size_t)nUse].tIn));
	}
	return nUse;
}

/* the receiver after obs_step against the model after ModelStep */
bool SameAsModel(const obs_held_t* apUse, int anFresh, int anUse, bool abFresh, std::vector<ModelRec> avRec)
{
	if ((apUse == NULL) != (anUse < 0) || (anFresh != 0) != abFresh || g_stR.n_held != g_vModel.size())
		return false;
	if (apUse != NULL && apUse != &g_stR.held[anUse])
		return false;
	for (unsigned i = 0; i < g_stR.n_held; i++)
	{
		const obs_held_t& h = g_stR.held[i];
		const ModelHeld& m = g_vModel[i];
		if (memcmp(&h.d, &m.d, sizeof(h.d)) || h.t_in != m.tIn || (h.accepted != 0) != m.bAccepted ||
		    (h.over != 0) != m.bOver || !QNear(h.q, m.adQ))
			return false;
	}
	std::vector<ModelRec> vGot;
	for (unsigned i = 0; i < g_stR.n_rec; i++)
		vGot.push_back(ModelRec(Key(g_stR.rec[i].h.d), g_stR.rec[i].verdict, g_stR.rec[i].h.t_in));
	std::sort(vGot.begin(), vGot.end());
	std::sort(avRec.begin(), avRec.end());
	return vGot == avRec;
}

/* eq. (3) on what is held after obs_step: the result in use may be used in this period, no newer capture may, and
 * nothing older is left */
bool Eq3Holds(const obs_held_t* apUse, int anPolicy, uint64_t atK, double adTol)
{
	const obs_settings_t stSet = Settings(anPolicy);
	for (unsigned i = 0; i < g_stR.n_held; i++)
	{
		const obs_held_t& h = g_stR.held[i];
		if (i > 0 && g_stR.held[i - 1].d.t_e < h.d.t_e)
			return false;
		const bool bMay = h.d.f <= atK && atK <= h.d.t_valid_until && h.d.t_e <= atK && h.d.t_valid_until <= h.d.t_e + CAP &&
		                  (anPolicy != OBS_CONTRACT ||
		                   obs_error_bound(&stSet, h.d.sigma, (double)(atK - h.d.t_e) / 1e6) <= adTol);
		if (&h == apUse)
			return bMay && i + 1 == g_stR.n_held;
		if (bMay)
			return false;
	}
	return apUse == NULL;
}

/* ---- the cases ---- */

/* a mailbox with both sides up, the receiver, and the states of periods auFirst..auLast at T * k, without auSkip */
void Begin(int anPolicy, uint32_t auFirst = 1, uint32_t auLast = 299, uint32_t auSkip = 0)
{
	memset(&g_stMb, 0, sizeof(g_stMb));
	obs_reader_start(Mb());
	obs_writer_start(Mb());
	const obs_settings_t stSet = Settings(anPolicy);
	if (!obs_receiver_start(&g_stR, &stSet))
		Expect(false, "the receiver starts with the settings of the test");
	memset(g_astState, 0, sizeof(g_astState));
	g_bAnyState = false;
	g_vModel.clear();
	g_uModelUsed = 0;
	for (uint32_t k = auFirst; k <= auLast; k++)
		if (k != auSkip)
			Push(k, T * k);
}

/* a result captured at auTe, published at auF, with the deadline auUntil. Returns its n; 0: not written. */
uint32_t Put(uint64_t auTe, uint64_t auF, uint64_t auUntil, float afSigma = SIG_IN, uint8_t auFlags = OBS_Z_TAG,
             float afZ0 = 0.0f)
{
	obs_desc_t stD;
	memset(&stD, 0, sizeof(stD));
	stD.t_e = auTe;
	stD.t_valid_until = auUntil;
	stD.z[0] = afZ0;
	stD.sigma = afSigma;
	stD.flags = auFlags;
	g_uNow = auF;
	return obs_put(Mb(), &stD, Now) == OBS_OK ? stD.n : 0;
}

/* a READY slot with these words; false: no slot is FREE */
bool Plant(const obs_desc_t& ad)
{
	uint32_t au[OBS_DESC_WORDS];
	memcpy(au, &ad, sizeof(au));
	for (unsigned b = 0; b < OBS_SLOTS; b++)
		if (ipc_load32(&Mb()->state[b][0]) == OBS_FREE)
		{
			for (unsigned i = 0; i < OBS_DESC_WORDS; i++)
				ipc_store32(&Mb()->slot[b][i], au[i]);
			ipc_store32(&Mb()->state[b][0], OBS_READY);
			return true;
		}
	return false;
}

/* period auK: its state at T * auK, then obs_step */
const obs_held_t* Step(uint32_t auK, int* anFresh, double adTol = TOL)
{
	Push(auK, T * auK);
	return obs_step(&g_stR, Mb(), T * auK, adTol, anFresh);
}

/* this period gave result auN of the writer's epoch this verdict; the record has the period that took it and this one */
bool RecIs(uint32_t auN, int anVerdict, uint64_t atIn, uint64_t atOut)
{
	for (unsigned i = 0; i < g_stR.n_rec; i++)
	{
		const obs_record_t& st = g_stR.rec[i];
		if (st.h.d.n == auN && st.h.d.epoch == ipc_load32(&Mb()->epoch))
			return st.verdict == anVerdict && st.h.t_in == atIn && st.t_out == atOut;
	}
	return false;
}

/* every result taken has its verdict, or is held and not used yet */
bool Balanced()
{
	uint32_t uSum = 0;
	for (int v = 0; v < OBS_VERDICTS; v++)
		uSum += g_stR.count[v];
	for (unsigned i = 0; i < g_stR.n_held; i++)
		if (!g_stR.held[i].accepted)
			uSum++;
	return uSum == g_stR.taken;
}

void States()
{
	const obs_settings_t stSet = Settings(OBS_CONTRACT);
	bool bOk = true;

	Begin(OBS_CONTRACT, 1, 0);
	Expect(NoState(T * 5), "no state yet");

	// one state per period
	Begin(OBS_CONTRACT, 1000, 1300);
	Expect(StateIs(T * 1200 + 400, 1200), "a capture between two states: the angles between theirs");
	Expect(StateIs(T * 1250, 1250), "a capture at the time of a state: that state");
	Expect(StateIs(T * 1300 - 1, 1299) && NoState(T * 1300) && NoState(T * 1300 + 5),
	       "up to the newest state, not at it or after it");
	Expect(StateIs(T * 1045 + 1, 1045) && StateIs(T * 1045, 1045) && NoState(T * 1045 - 1),
	       "back to 255 periods before the newest state, not further");
	for (uint64_t t = T * 990; t < T * 1310; t += 37)
		bOk = bOk && StateAgrees(t);
	Expect(bOk, "every capture time: as by definition");

	// times that jitter by a quarter of a period
	Begin(OBS_CONTRACT, 1, 0);
	Rng c = { 2463534242u };
	for (uint32_t k = 2000; k <= 2600; k++)
		Push(k, T * k + c.Below(481) - 240);
	bOk = true;
	bool bAll = true;
	for (uint64_t t = T * 2300; t < T * 2602; t += 13)
	{
		bool bFound = false;
		bOk = bOk && StateAgrees(t, &bFound);
		if (t >= T * 2347 && t < T * 2599)
			bAll = bAll && bFound;
	}
	Expect(bOk && bAll, "times that jitter: as by definition, and every capture inside the record has its state");

	// periods without a state, in the second round of the record: their entries hold the states of 256 periods before
	Begin(OBS_CONTRACT, 1, 0);
	for (uint32_t k = 3000; k <= 3700; k++)
		if (k != 3600 && k != 3601 && k != 3650)
			Push(k, T * k);
	Expect(StateIs(T * 3599 - 1, 3598) && NoState(T * 3599) && NoState(T * 3600 + 500) && NoState(T * 3602 - 1) &&
	       StateIs(T * 3602, 3602), "two periods without a state: no state for a capture between their neighbours");
	Expect(StateIs(T * 3649 - 1, 3648) && NoState(T * 3649) && NoState(T * 3651 - 1) && StateIs(T * 3651, 3651),
	       "one period without a state: its neighbours are 2 periods apart, no state");
	bOk = true;
	for (uint64_t t = T * 3300; t < T * 3702; t += 37)
		bOk = bOk && StateAgrees(t);
	Expect(bOk, "every capture time, also those older than the record: as by definition");

	// a state that comes late: its period and the one before are further apart
	Begin(OBS_CONTRACT, 190, 200);
	Push(201, T * 201 + T / 2);
	for (uint32_t k = 202; k <= 205; k++)
		Push(k, T * k);
	Push(206, T * 206 + T / 2 + T / 10);
	for (uint32_t k = 207; k <= 210; k++)
		Push(k, T * k);
	Expect(StateIs(T * 200 + 700, 200) && StateIs(T * 201 + 300, 200) && StateIs(T * 201 + 700, 201),
	       "two states 1.5 periods apart: still the state of a capture between them");
	Expect(NoState(T * 205 + 800) && NoState(T * 206 + 300) && StateIs(T * 206 + 700, 206),
	       "two states 1.6 periods apart: no state between them");

	// the period number wraps
	Begin(OBS_CONTRACT, 1, 0);
	for (uint32_t i = 0; i <= 300; i++)
		Push(0xFFFFFF00u + i, T * (5000 + i));
	Expect(StateIs(T * 5254 + 300, 0xFFFFFFFEu) && StateIs(T * 5255 + 300, 0xFFFFFFFFu) && StateIs(T * 5256 + 300, 0u) &&
	       StateIs(T * 5299 + 300, 43u), "the period number wraps");

	// the period number starts again while the time goes on
	Begin(OBS_CONTRACT, 7000, 7300);
	for (uint32_t i = 0; i <= 50; i++)
		Push(i, T * (7301 + i));
	Expect(StateIs(T * 7321 + 500, 20) && NoState(T * 7290 + 500) && NoState(T * 7300 + 500),
	       "the period number starts again: the states of before are not taken for those of now");

	// an entry that was never written
	Begin(OBS_CONTRACT, 1, 1);
	Expect(NoState(500) && NoState(999), "the entry of a period before the first is no state");
	Push(2, 2 * T);
	Expect(StateIs(1500, 1), "the first two states");

	// eq. (7)
	Expect(fabs(obs_error_bound(&stSet, 0.004, 0.1) - 0.0115) < 1e-15 && fabs(obs_error_bound(&stSet, 0.004, 0.0) - 0.007) < 1e-15,
	       "eq. (7): 1 + (4 + 2 + 2.5 + 2) mm at 0.1 s, 1 + (4 + 2) mm at 0 s");
	obs_settings_t stG = stSet;
	stG.g = 2.0;
	Expect(fabs(obs_error_bound(&stG, 0.004, 0.1) - 0.022) < 1e-15, "eq. (7): the gain on everything but e0");
}

void InTime()
{
	const obs_settings_t stSet = Settings(OBS_CONTRACT);
	int nFresh = 0;
	double adQ[OBS_DOF];
	uint32_t uS = 0;

	Begin(OBS_CONTRACT);
	uint32_t n = Put(Ms(250.4), Ms(299.5), Ms(450.4));
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == n && nFresh == 1 && Truth(Ms(250.4), &uS, adQ) && uS == 250 && p->d.s == 250 &&
	       QNear(p->q, adQ) && fabs((double)p->ebar - obs_error_bound(&stSet, SIG_IN, 0.0496)) < 1e-8 &&
	       g_stR.n_rec == 1 && RecIs(n, OBS_V_ACCEPTED, Ms(300), Ms(300)),
	       "a result in time: used, with the angles at its capture and its error bound; one verdict");
	p = Step(301, &nFresh);
	Expect(p != NULL && p->d.n == n && nFresh == 0 && g_stR.n_rec == 0, "used again in the next period: not fresh, no second verdict");
	for (uint32_t k = 302; k <= 357; k++)
		p = Step(k, &nFresh);
	Expect(p != NULL && p->d.n == n && p->over == 0, "used while its error bound is within the tolerance (age 106.6 ms)");
	p = Step(358, &nFresh);
	Expect(p == NULL && g_stR.n_held == 1 && g_stR.held[0].over == 1 && g_stR.n_rec == 0,
	       "not used once the bound is above it (age 107.6 ms): held, no verdict");
	for (uint32_t k = 359; k <= 450; k++)
		p = Step(k, &nFresh);
	Expect(p == NULL && g_stR.n_held == 1, "held until its deadline");
	p = Step(451, &nFresh);
	Expect(p == NULL && g_stR.n_held == 0 && g_stR.n_rec == 0 && g_stR.count[OBS_V_ACCEPTED] == 1 && g_stR.taken == 1 &&
	       Balanced(), "gone after its deadline: accepted stays its verdict");

	Begin(OBS_MAX_AGE);
	n = Put(Ms(250.4), Ms(299.5), Ms(450.4));
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == n && nFresh == 1 && p->d.s == 300 &&
	       !memcmp(p->q, g_astState[300].afQ, sizeof(p->q)) && RecIs(n, OBS_V_ACCEPTED, Ms(300), Ms(300)),
	       "OBS_MAX_AGE: used, with the angles of the period of arrival");
	bool bOk = true;
	for (uint32_t k = 301; k <= 450; k++)
		bOk = bOk && Step(k, &nFresh) != NULL && nFresh == 0;
	Expect(bOk && Step(451, &nFresh) == NULL && g_stR.n_held == 0 && Balanced(),
	       "OBS_MAX_AGE: used until its deadline, whatever its error bound");
}

void AfterRelease()
{
	int nFresh = 0;

	Begin(OBS_CONTRACT);
	const uint32_t n = Put(Ms(280.6), Ms(300.2), Ms(480));
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p == NULL && nFresh == 0 && g_stR.n_rec == 0 && g_stR.n_held == 1 && g_stR.taken == 1,
	       "published after the release: taken, not used in this period");
	p = Step(301, &nFresh);
	Expect(p != NULL && p->d.n == n && nFresh == 1 && RecIs(n, OBS_V_ACCEPTED, Ms(300), Ms(301)),
	       "used in the next period; the record has both periods");
	const uint32_t n2 = Put(Ms(290.5), Ms(302), Ms(480));
	p = Step(302, &nFresh);
	Expect(p != NULL && p->d.n == n2 && nFresh == 1 && Balanced(), "published at the release: used in this period");
}

void TwoInAPeriod()
{
	int nFresh = 0;

	Begin(OBS_CONTRACT);
	const uint32_t nB = Put(Ms(270.5), Ms(299.1), Ms(470));		// the newer capture in the first slot
	const uint32_t nA = Put(Ms(240.5), Ms(299.2), Ms(440));
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == nB && nFresh == 1 && g_stR.n_rec == 2 && RecIs(nB, OBS_V_ACCEPTED, Ms(300), Ms(300)) &&
	       RecIs(nA, OBS_V_SUPERSEDED, Ms(300), Ms(300)) && g_stR.n_held == 1,
	       "two results in a period: the newer capture is used, the older superseded");
	const uint32_t nC = Put(Ms(260.5), Ms(300.5), Ms(460));
	p = Step(301, &nFresh);
	Expect(p != NULL && p->d.n == nB && nFresh == 0 && g_stR.n_rec == 1 && RecIs(nC, OBS_V_SUPERSEDED, Ms(301), Ms(301)),
	       "an older capture that comes later is superseded; the one in use stays");
	const uint32_t nD = Put(Ms(285.5), Ms(301.5), Ms(485));
	p = Step(302, &nFresh);
	Expect(p != NULL && p->d.n == nD && nFresh == 1 && g_stR.n_rec == 1 && g_stR.n_held == 1 &&
	       g_stR.count[OBS_V_ACCEPTED] == 2 && g_stR.count[OBS_V_SUPERSEDED] == 2 && Balanced(),
	       "a newer capture replaces the one in use, which has its verdict already");
	obs_writer_start(Mb());
	p = Step(303, &nFresh);
	Expect(p != NULL && p->d.n == nD && nFresh == 0, "the writer restarts: the result in use stays");
	uint32_t nNew = 0;
	for (uint32_t i = 0; i < nD; i++)			// the new epoch counts from 1 again, up to the n of the one in use
		nNew = Put(Ms(295.5 + 0.1 * i), Ms(303.5), Ms(495));
	p = Step(304, &nFresh);
	Expect(nNew == nD && p != NULL && p->d.n == nD && nFresh == 1 && p->d.t_e == Ms(295.5 + 0.1 * (nD - 1)) &&
	       g_stR.count[OBS_V_SEQ_FAIL] == 0 && Balanced(), "a result of the new epoch with the n of the one in use is no copy of it");

	Begin(OBS_CONTRACT);
	const uint32_t nG = Put(Ms(270.5), Ms(299.1), Ms(470));
	const uint32_t nH = Put(Ms(270.5), Ms(299.2), Ms(470));
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == nG && RecIs(nH, OBS_V_SUPERSEDED, Ms(300), Ms(300)) && Balanced(),
	       "two results of one capture time: the one written first is used");

	Begin(OBS_CONTRACT);
	const uint32_t nE = Put(Ms(240.5), Ms(299.1), Ms(440));		// the older capture in the first slot
	const uint32_t nF = Put(Ms(270.5), Ms(299.2), Ms(470));
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == nF && RecIs(nE, OBS_V_SUPERSEDED, Ms(300), Ms(300)) && Balanced(),
	       "the same with the older capture in the first slot");
}

/* the clock of the cost: + 7 ticks every read */
uint64_t g_uCostClock = 0;
uint64_t CostClock() { return g_uCostClock += 7; }

/* B (eq. (6)): the reader looks at the writer's first B slots only; with a clock it measures its work of a period */
void SlotsAndCost()
{
	int nFresh = 0;

	Begin(OBS_CONTRACT);
	obs_writer_start_slots(Mb(), 2);
	obs_desc_t stD;
	memset(&stD, 0, sizeof(stD));
	stD.t_e = Ms(270.5);
	stD.f = Ms(299.1);
	stD.t_valid_until = Ms(470);
	stD.flags = OBS_Z_TAG;
	stD.sigma = SIG_IN;
	stD.n = stD.n_check = 1;
	stD.epoch = ipc_load32(&Mb()->epoch);
	uint32_t au[OBS_DESC_WORDS];
	memcpy(au, &stD, sizeof(au));
	for (unsigned i = 0; i < OBS_DESC_WORDS; i++)		// a READY result in slot 3, outside B = 2
		ipc_store32(&Mb()->slot[3][i], au[i]);
	ipc_store32(&Mb()->state[3][0], OBS_READY);
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p == NULL && g_stR.taken == 0 && ipc_load32(&Mb()->state[3][0]) == OBS_READY && g_stR.cost.periods == 0,
	       "B = 2: slot 3 is not looked at; no clock, no cost");

	obs_receiver_set_clock(&g_stR, CostClock);
	const uint32_t nA = Put(Ms(271.5), Ms(300.2), Ms(471));
	const uint32_t nB = Put(Ms(272.5), Ms(300.3), Ms(472));
	const uint32_t nC = Put(Ms(273.5), Ms(300.4), Ms(473));		// no FREE slot among the two
	p = Step(301, &nFresh);
	Expect(nA != 0 && nB != 0 && nC == 0 && p != NULL && p->d.n == nB &&
	       g_stR.taken == 2 && g_stR.cost.slots == 2 && g_stR.cost.periods == 1 && g_stR.cost.drain_periods[2] == 1 &&
	       g_stR.cost.drain_max[2] == 7 && g_stR.cost.select_max[2] == 7 && g_stR.cost.total_max == 14,
	       "B = 2: two results fill it, the third is dropped; the cost of the period: 2 taken, 2 held at the choice");
	p = Step(302, &nFresh);
	Expect(g_stR.cost.periods == 2 && g_stR.cost.drain_periods[0] == 1 && g_stR.cost.select_max[1] == 7,
	       "a period with nothing taken and one result held");
	obs_receiver_start(&g_stR, &g_stR.set);
	Expect(g_stR.now == NULL && g_stR.cost.periods == 0, "the start clears the clock and the cost");
}

void Deadline()
{
	int nFresh = 0;

	Begin(OBS_CONTRACT);
	const uint32_t n1 = Put(Ms(200.5), Ms(299.5), Ms(299.8));	// before its deadline when written, past it at the release
	const uint32_t n2 = Put(Ms(250.5), Ms(300.2), Ms(300.5));	// published after release 300, deadline before release 301
	const obs_held_t* p = Step(300, &nFresh);
	Expect(n1 != 0 && n2 != 0 && p == NULL && g_stR.n_rec == 1 && RecIs(n1, OBS_V_EXPIRED, Ms(300), Ms(300)) && g_stR.n_held == 1,
	       "past its deadline on arrival: expired");
	p = Step(301, &nFresh);
	Expect(p == NULL && g_stR.n_rec == 1 && RecIs(n2, OBS_V_EXPIRED, Ms(300), Ms(301)) && g_stR.n_held == 0,
	       "past its deadline before a period could use it: expired");
	const uint32_t n3 = Put(Ms(280.5), Ms(301.5), Ms(303));
	p = Step(302, &nFresh);
	Expect(p != NULL && p->d.n == n3 && nFresh == 1, "a result with its deadline at release 303");
	p = Step(303, &nFresh);
	Expect(p != NULL && p->d.n == n3, "used at its deadline");
	p = Step(304, &nFresh);
	Expect(p == NULL && g_stR.n_held == 0 && g_stR.n_rec == 0 && Balanced(), "not after it");
	const uint32_t nOld = Put(Ms(270.5), Ms(304.2), Ms(470));
	const uint32_t nSame = Put(Ms(280.5), Ms(304.3), Ms(470));
	p = Step(305, &nFresh);
	Expect(p == NULL && g_stR.n_rec == 2 && RecIs(nOld, OBS_V_SUPERSEDED, Ms(305), Ms(305)) &&
	       RecIs(nSame, OBS_V_SUPERSEDED, Ms(305), Ms(305)) && g_stR.n_held == 0,
	       "an older capture, and one of the same time, after the one used has gone: superseded, not used");
	const uint32_t n4 = Put(Ms(300.5), Ms(305.5), Ms(700.5));	// 400 ms; the cap is 250 ms
	p = Step(306, &nFresh);
	Expect(p != NULL && p->d.n == n4 && p->d.t_valid_until == Ms(550.5), "a deadline beyond the cap is cut to the cap");

	Begin(OBS_CONTRACT);
	const uint32_t n6 = Put(Ms(280.5), Ms(299.5), Ms(300));
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == n6 && nFresh == 1 && Step(301, &nFresh) == NULL && Balanced(),
	       "a result that arrives at its deadline: used in that period");

	Begin(OBS_CONTRACT, 250, 299);			// no state at the capture either: the deadline is looked at first
	const uint32_t n7 = Put(Ms(200.5), Ms(299.5), Ms(299.8));
	p = Step(300, &nFresh);
	Expect(p == NULL && RecIs(n7, OBS_V_EXPIRED, Ms(300), Ms(300)) && Balanced(),
	       "past its deadline on arrival and no state at its capture: expired");

	Begin(OBS_CONTRACT);
	const uint32_t n8 = Put(Ms(280.5), Ms(300.1), Ms(480));
	const uint32_t n9 = Put(Ms(290.5), Ms(300.2), Ms(300.5));	// the newer capture, its deadline before release 301
	p = Step(300, &nFresh);
	Expect(p == NULL && g_stR.n_held == 2 && g_stR.n_rec == 0, "two results published after the release");
	p = Step(301, &nFresh);
	Expect(p != NULL && p->d.n == n8 && nFresh == 1 && g_stR.n_rec == 2 && RecIs(n9, OBS_V_EXPIRED, Ms(300), Ms(301)) &&
	       RecIs(n8, OBS_V_ACCEPTED, Ms(300), Ms(301)) && g_stR.n_held == 1,
	       "the newer capture is past its deadline and goes; the older one is used");
	p = Step(302, &nFresh);
	Expect(p != NULL && p->d.n == n8 && nFresh == 0 && g_stR.count[OBS_V_ACCEPTED] == 1 && Balanced(),
	       "and stays in use, with its one verdict");

	Begin(OBS_MAX_AGE);
	const uint32_t n5 = Put(Ms(290.5), Ms(299.5), Ms(690.5));
	bool bOk = true;
	for (uint32_t k = 300; k <= 540; k++)
	{
		p = Step(k, &nFresh);
		bOk = bOk && p != NULL && p->d.n == n5;
	}
	Expect(bOk && Step(541, &nFresh) == NULL && g_stR.n_held == 0 && Balanced(),
	       "OBS_MAX_AGE: used until the cap, not until the writer's deadline");
}

void StateOfCapture()
{
	int nFresh = 0;

	Begin(OBS_CONTRACT, 250, 299);			// the control task started after the capture
	uint32_t n = Put(Ms(200.5), Ms(299.5), Ms(400));
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p == NULL && g_stR.n_rec == 1 && RecIs(n, OBS_V_STATE_UNRECOVERABLE, Ms(300), Ms(300)) && g_stR.n_held == 0,
	       "no state at the capture: state_unrecoverable");
	Begin(OBS_MAX_AGE, 250, 299);
	n = Put(Ms(200.5), Ms(299.5), Ms(400));
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == n && p->d.s == 300, "OBS_MAX_AGE: the same result is used");

	Begin(OBS_CONTRACT, 1, 299, 270);		// period 270 has no state
	const uint32_t n1 = Put(Ms(270.5), Ms(299.1), Ms(470));
	const uint32_t n2 = Put(Ms(269.5), Ms(299.2), Ms(469));
	const uint32_t n3 = Put(Ms(268.5), Ms(299.3), Ms(468));
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == n3 && p->d.s == 268 && g_stR.n_rec == 3 &&
	       RecIs(n1, OBS_V_STATE_UNRECOVERABLE, Ms(300), Ms(300)) && RecIs(n2, OBS_V_STATE_UNRECOVERABLE, Ms(300), Ms(300)) &&
	       Balanced(), "a period without a state: the captures next to it are state_unrecoverable, the one before is used");

	for (int nPolicy = OBS_CONTRACT; nPolicy <= OBS_MAX_AGE; nPolicy++)
	{
		Begin(nPolicy);
		n = Put(Ms(280.5), Ms(299.5), Ms(480), SIG_IN, OBS_Z_TAG | OBS_TAG_LOST);
		p = Step(300, &nFresh);
		Expect(p == NULL && RecIs(n, OBS_V_STATE_UNRECOVERABLE, Ms(300), Ms(300)) && Balanced(),
		       "the tag was not seen: state_unrecoverable under both policies");
	}

	Begin(OBS_MAX_AGE, 1, 0);
	n = Put(Ms(280.5), Ms(299.5), Ms(480));
	p = obs_step(&g_stR, Mb(), Ms(300), TOL, &nFresh);
	Expect(p == NULL && RecIs(n, OBS_V_STATE_UNRECOVERABLE, Ms(300), Ms(300)), "OBS_MAX_AGE before the first state: state_unrecoverable");
}

void CannotBe()
{
	int nFresh = 0;
	obs_desc_t stD;

	Begin(OBS_CONTRACT);
	uint32_t n = Put(Ms(280.5), Ms(299.5), Ms(480));
	obs_writer_start(Mb());
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p == NULL && g_stR.n_rec == 1 && g_stR.rec[0].verdict == OBS_V_SEQ_FAIL && g_stR.rec[0].h.d.n == n &&
	       g_stR.rec[0].h.d.epoch + 1u == ipc_load32(&Mb()->epoch), "a result of the writer's earlier epoch: seq_fail");

	memset(&stD, 0, sizeof(stD));
	stD.epoch = ipc_load32(&Mb()->epoch);
	stD.t_e = Ms(280.5);
	stD.f = Ms(300.5);
	stD.t_valid_until = Ms(480);
	stD.sigma = SIG_IN;
	stD.flags = OBS_Z_TAG;
	stD.n = 9;
	stD.n_check = 10;
	bool bOk = Plant(stD);
	stD.n = 0;
	stD.n_check = 0;
	bOk = bOk && Plant(stD);
	p = Step(301, &nFresh);
	Expect(bOk && p == NULL && g_stR.n_rec == 2 && g_stR.count[OBS_V_SEQ_FAIL] == 3, "a check word that does not fit, n = 0: seq_fail");

	n = Put(Ms(290.5), Ms(301.5), Ms(490));
	p = Step(302, &nFresh);
	Expect(p != NULL && p->d.n == n && nFresh == 1, "a result in use");
	bOk = Plant(g_stR.held[0].d);
	p = Step(303, &nFresh);
	Expect(bOk && p != NULL && p->d.n == n && nFresh == 0 && g_stR.n_rec == 1 && g_stR.rec[0].verdict == OBS_V_SEQ_FAIL &&
	       g_stR.n_held == 1, "the same result again: seq_fail, the one in use stays");

	const uint32_t n1 = Put(Ms(303.9), Ms(303.5), Ms(490));
	p = Step(304, &nFresh);
	Expect(n1 != 0 && RecIs(n1, OBS_V_SEQ_FAIL, Ms(304), Ms(304)), "captured after its publication: seq_fail");
	const uint32_t n2 = Put(Ms(305.5), Ms(305.6), Ms(490));
	p = Step(305, &nFresh);
	Expect(n2 != 0 && RecIs(n2, OBS_V_SEQ_FAIL, Ms(305), Ms(305)), "captured after the release of the period that takes it: seq_fail");
	const uint32_t n3 = Put(Ms(300.5), Ms(305.7), Ms(490), -0.001f);
	const uint32_t n4 = Put(Ms(300.6), Ms(305.8), Ms(490), NAN);
	p = Step(306, &nFresh);
	Expect(n3 != 0 && n4 != 0 && RecIs(n3, OBS_V_SEQ_FAIL, Ms(306), Ms(306)) && RecIs(n4, OBS_V_SEQ_FAIL, Ms(306), Ms(306)),
	       "a sigma below 0, a sigma that is no number: seq_fail");
	Expect(p != NULL && p->d.n == n && g_stR.count[OBS_V_SEQ_FAIL] == 8 && g_stR.taken == 9 && Balanced(),
	       "eight refused, the ninth still in use");
}

void Tolerance()
{
	int nFresh = 0;

	Begin(OBS_CONTRACT);
	const uint32_t nGood = Put(Ms(260.5), Ms(299.1), Ms(360.5));
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == nGood, "a result in use");
	const uint32_t nOver = Put(Ms(290.5), Ms(300.5), Ms(320.5), SIG_OVER);
	p = Step(301, &nFresh);
	Expect(p != NULL && p->d.n == nGood && nFresh == 0 && g_stR.n_held == 2 && g_stR.held[0].d.n == nOver &&
	       g_stR.held[0].over == 1 && g_stR.n_rec == 0,
	       "a newer capture with its error bound above the tolerance: held, the older one stays in use");
	for (uint32_t k = 302; k <= 320; k++)
		p = Step(k, &nFresh);
	Expect(p != NULL && p->d.n == nGood && g_stR.n_held == 2, "until its deadline");
	p = Step(321, &nFresh);
	Expect(p != NULL && p->d.n == nGood && g_stR.n_rec == 1 && RecIs(nOver, OBS_V_ERROR_EXCEEDED, Ms(301), Ms(321)) &&
	       g_stR.rec[0].h.ebar > TOL, "then error_exceeded");

	const uint32_t nOver2 = Put(Ms(310.5), Ms(321.5), Ms(400), SIG_OVER);
	p = Step(322, &nFresh);
	Expect(p != NULL && p->d.n == nGood, "another one above the tolerance");
	p = Step(323, &nFresh, 0.030);
	Expect(p != NULL && p->d.n == nOver2 && nFresh == 1 && g_stR.n_rec == 1 && RecIs(nOver2, OBS_V_ACCEPTED, Ms(322), Ms(323)) &&
	       g_stR.n_held == 1, "a period with a wider tolerance uses it");
	p = Step(324, &nFresh);
	Expect(p == NULL && g_stR.n_held == 1 && g_stR.n_rec == 0 && Balanced(), "the next period, with the tolerance of before, does not");

	Begin(OBS_MAX_AGE);
	const uint32_t n = Put(Ms(290.5), Ms(299.5), Ms(320.5), SIG_OVER);
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == n && nFresh == 1 && p->ebar > TOL && p->over == 0,
	       "OBS_MAX_AGE: used; the error bound is computed, not applied");
}

void NineHeld()
{
	int nFresh = 0;
	uint32_t an[8], am[8];

	Begin(OBS_CONTRACT);
	for (int i = 0; i < 8; i++)
		an[i] = Put(Ms(280.5 + i), Ms(299.5), Ms(500), SIG_OVER);
	const obs_held_t* p = Step(300, &nFresh);
	Expect(p == NULL && g_stR.n_held == 8 && g_stR.n_rec == 0, "eight results above the tolerance: held");
	const uint32_t n9 = Put(Ms(290.5), Ms(300.5), Ms(500), SIG_OVER);
	p = Step(301, &nFresh);
	Expect(p == NULL && g_stR.n_held == 8 && g_stR.n_rec == 1 && RecIs(an[0], OBS_V_EVICTED, Ms(300), Ms(301)) &&
	       g_stR.held[0].d.n == n9 && g_stR.held[7].d.n == an[1], "a ninth: the oldest capture is evicted");
	const uint32_t n10 = Put(Ms(270.5), Ms(301.5), Ms(500), SIG_OVER);
	p = Step(302, &nFresh);
	Expect(p == NULL && g_stR.n_held == 8 && g_stR.n_rec == 1 && RecIs(n10, OBS_V_EVICTED, Ms(302), Ms(302)) &&
	       g_stR.held[7].d.n == an[1], "a ninth that is itself the oldest capture: evicted on arrival");
	for (int i = 0; i < 8; i++)
		am[i] = Put(Ms(292.5 + i), Ms(302.5), Ms(500));
	p = Step(303, &nFresh);
	bool bOk = p != NULL && p->d.n == am[7] && nFresh == 1 && g_stR.n_rec == OBS_RECORDS && g_stR.n_held == 1 &&
	           RecIs(am[7], OBS_V_ACCEPTED, Ms(303), Ms(303)) && RecIs(n9, OBS_V_EVICTED, Ms(301), Ms(303));
	for (int i = 0; i < 7; i++)
		bOk = bOk && RecIs(an[i + 1], OBS_V_EVICTED, Ms(300), Ms(303)) && RecIs(am[i], OBS_V_SUPERSEDED, Ms(303), Ms(303));
	Expect(bOk && g_stR.count[OBS_V_EVICTED] == 10 && Balanced(),
	       "eight held and eight newer in one period: 8 evicted, 7 superseded, 1 accepted - sixteen verdicts");

	Begin(OBS_CONTRACT);
	const uint32_t nA = Put(Ms(260.5), Ms(299.5), Ms(500));
	p = Step(300, &nFresh);
	Expect(p != NULL && p->d.n == nA, "a result in use");
	for (int i = 0; i < 8; i++)
		Put(Ms(280.5 + i), Ms(300.5), Ms(500), SIG_OVER);
	p = Step(301, &nFresh);
	Expect(p == NULL && g_stR.n_held == 8 && g_stR.n_rec == 0 && g_stR.count[OBS_V_EVICTED] == 0 && Balanced(),
	       "eight newer captures above the tolerance push the one in use out: no second verdict for it");
}

/* a tag at (q[0], 0, 0); *apCalls counts the calls */
void TagAtQ0(void* apCalls, const float* afQ, double* adP)
{
	(*(int*)apCalls)++;
	adP[0] = afQ[0];
	adP[1] = adP[2] = 0.0;
}

void Formats()
{
	int nFresh = 0, nCalls = 0;
	double adP[3] = { 0.0, 0.0, 0.0 };
	const float afNone[OBS_DOF] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	Begin(OBS_CONTRACT, 250, 299);			// no state at the capture
	uint32_t n = Put(Ms(200.5), Ms(299.5), Ms(400), SIG_IN, OBS_Z_BASE, 0.612f);
	const obs_held_t* p = Step(300, &nFresh);
	if (p != NULL)
		obs_object(p, TagAtQ0, &nCalls, adP);
	Expect(p != NULL && p->d.n == n && nFresh == 1 && p->d.s == 0 && !memcmp(p->q, afNone, sizeof(afNone)) && nCalls == 0 &&
	       adP[0] == (double)0.612f && adP[1] == 0.0 && adP[2] == 0.0,
	       "the object in the base frame: used without a state, and the object is z itself");
	n = Put(Ms(290.5), Ms(300.5), Ms(490), SIG_IN, OBS_Z_BASE | OBS_TAG_LOST);
	p = Step(301, &nFresh);
	Expect(p != NULL && p->d.n == n && nFresh == 1, "in the base frame no tag is needed");
	const uint32_t n1 = Put(Ms(291.5), Ms(301.1), Ms(490), SIG_IN, 0);
	const uint32_t n2 = Put(Ms(292.5), Ms(301.2), Ms(490), SIG_IN, OBS_Z_TAG | OBS_Z_BASE);
	p = Step(302, &nFresh);
	Expect(n1 != 0 && n2 != 0 && p != NULL && p->d.n == n && RecIs(n1, OBS_V_SEQ_FAIL, Ms(302), Ms(302)) &&
	       RecIs(n2, OBS_V_SEQ_FAIL, Ms(302), Ms(302)) && Balanced(), "neither format, both formats: seq_fail");

	Begin(OBS_MAX_AGE, 1, 0);
	n = Put(Ms(280.5), Ms(299.5), Ms(480), SIG_IN, OBS_Z_BASE);
	p = obs_step(&g_stR, Mb(), Ms(300), TOL, &nFresh);
	Expect(p != NULL && p->d.n == n, "OBS_MAX_AGE before the first state: a result in the base frame is used");

	// the tag moves at 0.2 m/s and is at 0.500 m at the capture; the object is 0.100 m beyond it; the result
	// arrives 120 ms later, with the tag at 0.524 m
	for (int nPolicy = OBS_CONTRACT; nPolicy <= OBS_MAX_AGE; nPolicy++)
	{
		Begin(nPolicy, 1, 0);
		for (uint32_t k = 1; k <= 300; k++)
		{
			float afQ[OBS_DOF] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
			afQ[0] = (float)(0.5 + 0.2 * ((double)(T * k) - (double)Ms(180)) / 1e6);
			obs_state_push(&g_stR, k, T * k, afQ);
		}
		n = Put(Ms(180), Ms(299.5), Ms(400), SIG_IN, OBS_Z_TAG, 0.1f);
		p = obs_step(&g_stR, Mb(), Ms(300), 0.020, &nFresh);
		nCalls = 0;
		if (p != NULL)
			obs_object(p, TagAtQ0, &nCalls, adP);
		Expect(p != NULL && p->d.n == n && nCalls == 1 && fabs(adP[0] - (nPolicy == OBS_CONTRACT ? 0.600 : 0.624)) < 1e-6 &&
		       adP[1] == 0.0 && adP[2] == 0.0,
		       nPolicy == OBS_CONTRACT ? "the object seen from a tag that moved: OBS_CONTRACT has it where it is, 0.600 m"
		                               : "OBS_MAX_AGE has it 24 mm off, at 0.624 m");
	}
}

void NoWriterAndSettings()
{
	int nFresh = 0;
	obs_desc_t stD;

	memset(&g_stMb, 0, sizeof(g_stMb));
	obs_settings_t stSet = Settings(OBS_CONTRACT);
	Expect(obs_receiver_start(&g_stR, &stSet) == 1, "the settings of the test");
	memset(&stD, 0, sizeof(stD));
	stD.n = stD.n_check = 1;
	stD.t_e = Ms(280.5);
	stD.f = Ms(299.5);
	stD.t_valid_until = Ms(480);
	stD.flags = OBS_Z_TAG;
	Expect(Plant(stD) && obs_step(&g_stR, Mb(), Ms(300), TOL, &nFresh) == NULL && g_stR.taken == 0 && g_stR.n_rec == 0 &&
	       ipc_load32(&Mb()->state[0][0]) == OBS_READY, "no writer: nothing is taken");

	g_stR.n_held = 3;
	g_stR.taken = 7;
	Expect(obs_receiver_start(&g_stR, &stSet) == 1 && g_stR.n_held == 0 && g_stR.taken == 0 && g_stR.have_state == 0,
	       "a start empties the receiver");
	stSet.period_ticks = 0;
	Expect(obs_receiver_start(&g_stR, &stSet) == 0, "a period of 0 ticks: refused");
	stSet = Settings(OBS_CONTRACT);
	stSet.ticks_per_s = 0.0;
	bool bOk = obs_receiver_start(&g_stR, &stSet) == 0;
	stSet.ticks_per_s = NAN;
	Expect(bOk && obs_receiver_start(&g_stR, &stSet) == 0, "0 ticks in a second, no number: refused");
	stSet = Settings(OBS_CONTRACT);
	stSet.policy = 2;
	Expect(obs_receiver_start(&g_stR, &stSet) == 0, "a policy that is none: refused");
	stSet = Settings(OBS_CONTRACT);
	stSet.age_cap_ticks = 253 * T;
	bOk = obs_receiver_start(&g_stR, &stSet) == 0;
	stSet.age_cap_ticks = 253 * T - 1;
	Expect(bOk && obs_receiver_start(&g_stR, &stSet) == 1, "a cap that the states do not reach back to: refused");
}

/* N periods of random results under one policy */
void Random(int anPolicy, long anPeriods)
{
	const char* const pcPolicy = anPolicy == OBS_CONTRACT ? "OBS_CONTRACT" : "OBS_MAX_AGE";
	Rng c = { anPolicy == OBS_CONTRACT ? 88172645u : 362436069u };
	std::map<uint64_t, int> mapOut;			// a result written -> the verdicts it got
	std::map<uint64_t, int> mapPlanted;		// a result planted again -> copies not refused yet
	long anCount[OBS_VERDICTS] = { 0 };
	long nWritten = 0, nPlanted = 0, nNotModel = 0, nNotEq3 = 0, nUnknown = 0, nNotInOrder = 0;
	unsigned uMostRec = 0;
	uint64_t uLastTe = 0;
	int nStall = 0;

	Begin(anPolicy, 1, 0);
	uint32_t k = 1000;
	for (long i = 0; i < anPeriods + 600; i++, k++)		// the last 600 periods bring nothing new: what is held leaves
	{
		const uint64_t tK = T * k;
		if (i < anPeriods)
		{
			// the writer, in the period before this release: now and then a burst of recent captures, half of the
			// bursts above the tolerance
			const bool bBurst = c.Chance(0.002), bBurstOver = bBurst && c.Chance(0.5);
			for (int nNew = bBurst ? 12 : (c.Chance(0.1) ? 1 + (int)c.Below(2) : 0); nNew > 0; nNew--)
			{
				obs_desc_t stD;
				memset(&stD, 0, sizeof(stD));
				stD.t_e = tK - T * (2 + c.Below(bBurst ? 40 : 280)) - c.Below((uint32_t)T);
				stD.t_valid_until = stD.t_e + T * (50 + c.Below(251));
				stD.sigma = (bBurstOver || c.Chance(0.2)) ? SIG_OVER : SIG_IN;
				stD.flags = (uint8_t)((c.Chance(0.1) ? OBS_Z_BASE : OBS_Z_TAG) | (c.Chance(0.02) ? OBS_TAG_LOST : 0));
				if (c.Chance(0.005))
					stD.flags = c.Chance(0.5) ? 0 : (OBS_Z_TAG | OBS_Z_BASE);
				g_uNow = c.Chance(0.1) ? tK + 1 + c.Below((uint32_t)T / 2) : tK - c.Below((uint32_t)T);
				if (obs_put(Mb(), &stD, Now) == OBS_OK)
				{
					mapOut[Key(stD)] = 0;
					nWritten++;
				}
			}
			if (c.Chance(0.001))
				obs_writer_start(Mb());
			if (c.Chance(0.002) && g_stR.n_held > 0)
			{
				const obs_desc_t& d = g_stR.held[c.Below(g_stR.n_held)].d;
				if (Plant(d))
				{
					mapPlanted[Key(d)]++;
					nPlanted++;
				}
			}
			if (c.Chance(0.002))
				nStall = 1 + (int)c.Below(40);
			if (nStall > 0)							// the control task stalls: no state, no step
			{
				nStall--;
				continue;
			}
		}
		if (!c.Chance(0.005))						// else a period without a state
			Push(k, tK - 500 + c.Below(401));
		const double dTol = c.Chance(0.02) ? 0.030 : TOL;

		bool bFreshModel = false;
		std::vector<ModelRec> vRec;
		const int nUse = ModelStep(anPolicy, tK, dTol, &bFreshModel, &vRec);
		int nFresh = 0;
		const obs_held_t* pUse = obs_step(&g_stR, Mb(), tK, dTol, &nFresh);
		if (!SameAsModel(pUse, nFresh, nUse, bFreshModel, vRec))
			nNotModel++;
		if (!Eq3Holds(pUse, anPolicy, tK, dTol))
			nNotEq3++;
		if (pUse != NULL && nFresh)
		{
			if (pUse->d.t_e <= uLastTe)
				nNotInOrder++;
			uLastTe = pUse->d.t_e;
		}
		uMostRec = std::max(uMostRec, g_stR.n_rec);
		for (unsigned j = 0; j < g_stR.n_rec; j++)
		{
			const obs_record_t& st = g_stR.rec[j];
			anCount[st.verdict]++;
			std::map<uint64_t, int>::iterator it = mapPlanted.find(Key(st.h.d));
			if (st.verdict == OBS_V_SEQ_FAIL && it != mapPlanted.end() && it->second > 0)
			{
				it->second--;
				continue;
			}
			it = mapOut.find(Key(st.h.d));
			if (it == mapOut.end())
				nUnknown++;
			else
				it->second++;
		}
	}

	long nNone = 0, nTwice = 0, nLeft = 0;
	for (std::map<uint64_t, int>::const_iterator it = mapOut.begin(); it != mapOut.end(); ++it)
	{
		if (it->second == 0) nNone++;
		if (it->second > 1) nTwice++;
	}
	for (std::map<uint64_t, int>::const_iterator it = mapPlanted.begin(); it != mapPlanted.end(); ++it)
		nLeft += it->second;
	bool bCounts = g_stR.taken == (uint32_t)(nWritten + nPlanted) && g_stR.n_held == 0;
	for (int v = 0; v < OBS_VERDICTS; v++)
		bCounts = bCounts && g_stR.count[v] == (uint32_t)anCount[v];

	printf("random, %s: %ld periods, %ld results written, %ld planted again;", pcPolicy, anPeriods, nWritten, nPlanted);
	for (int v = 0; v < OBS_VERDICTS; v++)
		printf(" %s %ld%s", VERDICT[v], anCount[v], v + 1 < OBS_VERDICTS ? "," : "\n");
	printf("  periods unlike the model %ld, unlike eq. (3) %ld; results without a verdict %ld, with two %ld; "
	       "most verdicts in a period %u\n", nNotModel, nNotEq3, nNone + nLeft, nTwice + nUnknown, uMostRec);
	Expect(nNotModel == 0, "random: the receiver agrees with the model in every period");
	Expect(nNotEq3 == 0, "random: eq. (3) holds in every period");
	Expect(nNone == 0 && nTwice == 0 && nUnknown == 0 && nLeft == 0 && bCounts,
	       "random: every result written has exactly one verdict, and the counters agree");
	Expect(nNotInOrder == 0, "random: the results used come in the order of their captures");
	bool bAll = true;				// OBS_MAX_AGE holds no result for long: none above a tolerance, hardly ever nine
	for (int v = 0; v < OBS_VERDICTS; v++)
		bAll = bAll && (anCount[v] > 0 || (anPolicy == OBS_MAX_AGE && (v == OBS_V_ERROR_EXCEEDED || v == OBS_V_EVICTED)));
	Expect(bAll && (anPolicy == OBS_CONTRACT || anCount[OBS_V_ERROR_EXCEEDED] == 0),
	       "random: every verdict came up (error_exceeded under OBS_CONTRACT only)");
}

/* result k of the writer thread: k in every field that the receiver does not read */
void FillK(obs_desc_t* ad, uint32_t auK)
{
	const uint32_t au[3] = { auK, ~auK, auK * 3u };
	memset(ad, 0, sizeof(*ad));
	memcpy(ad->z, au, sizeof(au));
	ad->mode = (uint8_t)auK;
	ad->cls = (uint16_t)(auK >> 8);
	ad->target_seq = auK ^ 0xA5A5A5A5u;
	ad->sigma = (auK % 5u == 0) ? SIG_OVER : SIG_IN;
	ad->flags = OBS_Z_TAG;
}

bool Whole(const obs_desc_t& ad, uint32_t* auK)
{
	obs_desc_t stE;
	memcpy(auK, &ad.z[0], 4);
	FillK(&stE, *auK);
	return !memcmp(stE.z, ad.z, sizeof(stE.z)) && stE.mode == ad.mode && stE.cls == ad.cls && stE.target_seq == ad.target_seq &&
	       stE.sigma == ad.sigma && stE.flags == ad.flags && ad.n != 0 && ad.n_check == ad.n;
}

/* one writer thread against the receiver in another */
void Threads(long anResults)
{
	std::vector<unsigned char> vWritten((size_t)anResults + 1, 0), vOut((size_t)anResults + 1, 0);
	std::atomic<bool> bDone(false);
	long nOk = 0, nFull = 0, nLate = 0;
	long nBad = 0, nTwice = 0, nNotEq3 = 0, nPeriods = 0;
	long anCount[OBS_VERDICTS] = { 0 };

	Begin(OBS_CONTRACT, 1, 0);
	g_uNow = T * 1000;
	const std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();
	std::thread cReceiver([&]() {
		int nIdle = 0;									// periods after the last result: what is held leaves
		for (uint32_t k = 1000; nIdle < 700; k++, nPeriods++)
		{
			if (bDone.load())
				nIdle++;
			const uint64_t tK = T * k;
			g_uNow = tK + 1;							// a result written in this period is published after its release
			Push(k, tK - 300);
			int nFresh = 0;
			const obs_held_t* pUse = obs_step(&g_stR, Mb(), tK, TOL, &nFresh);
			if (!Eq3Holds(pUse, OBS_CONTRACT, tK, TOL))
				nNotEq3++;
			for (unsigned i = 0; i < g_stR.n_rec; i++)
			{
				uint32_t uK = 0;
				anCount[g_stR.rec[i].verdict]++;
				if (!Whole(g_stR.rec[i].h.d, &uK) || uK == 0 || uK > (uint32_t)anResults) { nBad++; continue; }
				if (vOut[uK]) nTwice++;
				vOut[uK] = 1;
			}
		}
	});
	std::thread cWriter([&]() {
		Rng c = { 521288629u };
		obs_desc_t stD;
		for (long k = 1; k <= anResults; k++)
		{
			const uint64_t uNow = Now();
			FillK(&stD, (uint32_t)k);
			stD.t_e = uNow - T * (1 + c.Below(8)) - c.Below((uint32_t)T);
			stD.t_valid_until = stD.t_e + CAP + T * c.Below(50);
			const int nStatus = obs_put(Mb(), &stD, Now);
			if (nStatus == OBS_OK) { nOk++; vWritten[(size_t)k] = 1; }
			else if (nStatus == OBS_FULL) nFull++;
			else nLate++;								// the thread slept for 240 periods before obs_put
			// two results, then the next period; a receiver that does not run is not waited for
			for (int i = 0; (k & 1) == 0 && i < 1000 && Now() == uNow; i++)
				;
		}
		bDone.store(true);
	});
	cWriter.join();
	cReceiver.join();
	const double dSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();

	long nLost = 0, nNotWritten = 0;
	for (long k = 1; k <= anResults; k++)
	{
		if (vWritten[(size_t)k] && !vOut[(size_t)k]) nLost++;
		if (vOut[(size_t)k] && !vWritten[(size_t)k]) nNotWritten++;
	}
	bool bCounts = g_stR.taken == (uint32_t)nOk && g_stR.n_held == 0 && ipc_load32(&Mb()->n_last) == (uint32_t)nOk &&
	               ipc_load32(&Mb()->dropped_full) == (uint32_t)nFull && ipc_load32(&Mb()->dropped_expired) == (uint32_t)nLate &&
	               nOk + nFull + nLate == anResults && nOk > 0;
	for (int v = 0; v < OBS_VERDICTS; v++)
		bCounts = bCounts && g_stR.count[v] == (uint32_t)anCount[v];
	printf("threads: %ld results in %.1f s, %ld periods; written %ld, dropped (full) %ld, (too late) %ld;", anResults,
	       dSeconds, nPeriods, nOk, nFull, nLate);
	for (int v = 0; v < OBS_VERDICTS; v++)
		printf(" %s %ld%s", VERDICT[v], anCount[v], v + 1 < OBS_VERDICTS ? "," : "\n");
	printf("  mixed copies %ld, two verdicts %ld, written and without a verdict %ld, a verdict but not written %ld; "
	       "periods unlike eq. (3) %ld\n", nBad, nTwice, nLost, nNotWritten, nNotEq3);
	Expect(nBad == 0 && nTwice == 0 && nLost == 0 && nNotWritten == 0 && bCounts,
	       "threads: every result written leaves whole and with exactly one verdict, and the counters agree");
	Expect(nNotEq3 == 0, "threads: eq. (3) holds in every period");
	Expect(anCount[OBS_V_ACCEPTED] > 0 && anCount[OBS_V_SUPERSEDED] > 0, "threads: results were used");
}

} // namespace

int main(int argc, char** argv)
{
	long nPeriods = 200000, nResults = 1000000;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "--periods") && i + 1 < argc) nPeriods = atol(argv[++i]);
		else if (!strcmp(argv[i], "--results") && i + 1 < argc) nResults = atol(argv[++i]);
		else
		{
			fprintf(stderr, "usage: %s [--periods N] [--results N]\n", argv[0]);
			return 2;
		}
	}
	if (nPeriods < 1 || nPeriods > 100000000L || nResults < 1 || nResults > 0x7FFFFFFFL)
		return 2;

	printf("%u states of %u B, %u held results of %u B, %u records; the receiver is %u B\n", (unsigned)OBS_STATES,
	       (unsigned)sizeof(obs_state_t), (unsigned)OBS_HELD, (unsigned)sizeof(obs_held_t), (unsigned)OBS_RECORDS,
	       (unsigned)sizeof(obs_receiver_t));
	States();
	InTime();
	AfterRelease();
	TwoInAPeriod();
	Deadline();
	StateOfCapture();
	CannotBe();
	Tolerance();
	NineHeld();
	Formats();
	NoWriterAndSettings();
	SlotsAndCost();
	printf("states and verdicts: %d checks failed\n", g_nFailed);
	Random(OBS_CONTRACT, nPeriods);
	Random(OBS_MAX_AGE, nPeriods);
	Threads(nResults);
	printf("%s: %d checks failed\n", g_nFailed == 0 ? "PASS" : "FAIL", g_nFailed);
	return g_nFailed == 0 ? 0 : 1;
}
