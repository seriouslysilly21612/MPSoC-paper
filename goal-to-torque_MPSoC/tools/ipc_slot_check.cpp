/*****************************************************************************
*	Name: ipc_slot_check.cpp
*	Description: The test of the slots between the APU and the R5
*	(indy7/src/ipc/ipc_layout.h, ipc_slot.h) in one process: the region is an
*	array, the sides are threads.
*
*	    bin/ipc_slot_check [--snapshots N] [--readers R] [--unsafe]
*
*	1. The three payloads through their slots, byte by byte.
*	2. The faults, step by step: a region full of garbage; a writer that dies
*	   in a write; a writer that restarts, and both sides restarting; a value
*	   of an older epoch; the wrap of the sequence number; a peer with
*	   another magic or layout; a payload too large. After every refused read
*	   the reader's last copy must be as it was.
*	3. One writer publishes N snapshots (default 10,000,000) of the largest
*	   payload while R readers (default 2) read with one attempt each; the
*	   sequence number wraps in the first 200 snapshots. Snapshot k carries k
*	   in its first and its last word and k * (i + 1) between: a copy that
*	   mixes two snapshots breaks this. Every accepted copy is checked; k
*	   must not decrease for a reader, its stamp (epoch, sequence number)
*	   must change exactly when k does, and the read after the last write
*	   must give snapshot N.
*	--unsafe copies the payload in 3 without the check of the sequence
*	number, to show that the test sees a mixed copy when there is one.
*	The test passes if no check of 1 and 2 failed and no inconsistent copy
*	was accepted in 3.
*****************************************************************************/
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "ipc/ipc_slot.h"

namespace {

enum { WORDS = IPC_WORDS_MAX };

alignas(64) uint32_t g_auRegion[IPC_OFF_END / 4];
int g_nFailed = 0;

volatile ipc_head_t* Head() { return reinterpret_cast<volatile ipc_head_t*>(g_auRegion); }
volatile uint32_t* Slot(unsigned auOff) { return g_auRegion + auOff / 4; }

void Expect(bool abOk, const char* apcWhat)
{
	if (abOk)
		return;
	g_nFailed++;
	printf("  FAILED: %s\n", apcWhat);
}

/* snapshot k of part 3 */
void Fill(uint32_t* au, uint32_t auK)
{
	for (unsigned i = 0; i + 1 < WORDS; i++)
		au[i] = auK * (i + 1);
	au[WORDS - 1] = auK;
}

bool Consistent(const uint32_t* au)
{
	if (au[WORDS - 1] != au[0])
		return false;
	for (unsigned i = 1; i + 1 < WORDS; i++)
		if (au[i] != au[0] * (i + 1))
			return false;
	return true;
}

/* the loads of ipc_slot_read() without its checks */
void RawCopy(const volatile uint32_t* apSlot, uint32_t* au)
{
	for (unsigned i = 0; i < WORDS; i++)
		au[i] = ipc_load32(apSlot + IPC_SLOT_HEAD_WORDS + i);
}

/* one read of snapshot auK expected; a refused read must leave the copy and the stamp as they were */
bool ReadIs(int anStatus, uint32_t auK, uint32_t* auStamp)
{
	uint32_t au[WORDS], auWas[WORDS], auS[2] = { 0xAAAAAAAAu, 0xBBBBBBBBu };
	Fill(au, 0x5A5A5A5Au);
	memcpy(auWas, au, sizeof(au));
	const int nStatus = ipc_slot_read(Slot(IPC_OFF_STATE), &Head()->r5, au, WORDS, auS);
	if (nStatus != anStatus)
		return false;
	if (nStatus != IPC_OK)
		return !memcmp(au, auWas, sizeof(au)) && auS[0] == 0xAAAAAAAAu && auS[1] == 0xBBBBBBBBu;
	if (auStamp != NULL)
		memcpy(auStamp, auS, sizeof(auS));
	return Consistent(au) && au[0] == auK;
}

void Payloads()
{
	volatile uint32_t* apR5[] = { Slot(IPC_OFF_STATE) };
	volatile uint32_t* apApu[] = { Slot(IPC_OFF_TORQUE), Slot(IPC_OFF_COMMAND) };
	memset(g_auRegion, 0, sizeof(g_auRegion));
	const uint32_t uR5 = ipc_side_start(&Head()->r5, apR5, 1), uApu = ipc_side_start(&Head()->apu, apApu, 2);
	uint32_t auS[2];

	ipc_state_t stS, stS2;
	for (size_t i = 0; i < sizeof(stS); i++)
		((unsigned char*)&stS)[i] = (unsigned char)(7 * i + 1);
	IPC_WRITE(Slot(IPC_OFF_STATE), uR5, &stS);
	Expect(IPC_READ(Slot(IPC_OFF_STATE), &Head()->r5, &stS2, auS) == IPC_OK && !memcmp(&stS, &stS2, sizeof(stS)) &&
	       auS[0] == uR5, "the State payload through its slot");

	ipc_torque_t stT, stT2;
	for (size_t i = 0; i < sizeof(stT); i++)
		((unsigned char*)&stT)[i] = (unsigned char)(11 * i + 3);
	IPC_WRITE(Slot(IPC_OFF_TORQUE), uApu, &stT);
	Expect(IPC_READ(Slot(IPC_OFF_TORQUE), &Head()->apu, &stT2, auS) == IPC_OK && !memcmp(&stT, &stT2, sizeof(stT)) &&
	       auS[0] == uApu, "the Torque payload through its slot");

	ipc_command_t stC = { 1u, IPC_CMD_SERVO_ON }, stC2 = { 0u, 0u };
	IPC_WRITE(Slot(IPC_OFF_COMMAND), uApu, &stC);
	Expect(IPC_READ(Slot(IPC_OFF_COMMAND), &Head()->apu, &stC2, auS) == IPC_OK && stC2.seq == 1u &&
	       stC2.cmd == IPC_CMD_SERVO_ON, "the Command payload through its slot");
	// the neighbours are untouched: every slot still reads what was written to it
	Expect(IPC_READ(Slot(IPC_OFF_STATE), &Head()->r5, &stS2, auS) == IPC_OK && !memcmp(&stS, &stS2, sizeof(stS)) &&
	       IPC_READ(Slot(IPC_OFF_TORQUE), &Head()->apu, &stT2, auS) == IPC_OK && !memcmp(&stT, &stT2, sizeof(stT)),
	       "a write reached another slot");
}

void Faults()
{
	volatile uint32_t* const pSlot = Slot(IPC_OFF_STATE);
	volatile uint32_t* const pCmd = Slot(IPC_OFF_COMMAND);
	volatile uint32_t* apR5[] = { pSlot };
	volatile uint32_t* apApu[] = { Slot(IPC_OFF_TORQUE), pCmd };
	uint32_t au[WORDS], auS[2], auWas[2], auC[2], auCWas[2];

	// a region full of garbage: nothing is read before the writer started, and nothing but "busy" before it wrote
	uint32_t uX = 2463534242u;
	for (uint32_t& u : g_auRegion)
	{
		uX ^= uX << 13; uX ^= uX >> 17; uX ^= uX << 5;
		u = uX & ~1u;					// the worst garbage: every sequence number even
	}
	Expect(ReadIs(IPC_NO_PEER, 0, NULL), "garbage, before the writer started: IPC_NO_PEER");
	uint32_t uEpoch = ipc_side_start(&Head()->r5, apR5, 1);
	Expect(uEpoch != 0u && ReadIs(IPC_BUSY, 0, NULL), "garbage, started and not written: IPC_BUSY");
	Fill(au, 1);
	ipc_slot_write(pSlot, uEpoch, au, WORDS);
	Expect(ReadIs(IPC_OK, 1, auS) && auS[0] == uEpoch, "the first write after garbage");

	// the writer dies in a write of snapshot 2: sequence number odd, half of the payload stored
	Fill(au, 2);
	ipc_store32(pSlot + IPC_SLOT_GEN, ipc_load32(pSlot + IPC_SLOT_GEN) | 1u);
	for (unsigned i = 0; i < WORDS / 2; i++)
		ipc_store32(pSlot + IPC_SLOT_HEAD_WORDS + i, au[i]);
	RawCopy(pSlot, au);
	Expect(!Consistent(au), "the half-written payload should be a mixed copy (the check of part 3)");
	Expect(ReadIs(IPC_BUSY, 0, NULL), "a write that never ends: IPC_BUSY");
	// ... and restarts: still nothing until its first write; then that write, with the next epoch and the
	// sequence number going on from where it was
	memcpy(auWas, auS, sizeof(auS));
	uEpoch = ipc_side_start(&Head()->r5, apR5, 1);
	Expect(uEpoch == auWas[0] + 1u && ReadIs(IPC_BUSY, 0, NULL), "restarted after a broken write: IPC_BUSY, epoch + 1");
	Fill(au, 3);
	ipc_slot_write(pSlot, uEpoch, au, WORDS);
	Expect(ReadIs(IPC_OK, 3, auS) && auS[0] == uEpoch && auS[1] == auWas[1] + 2u, "the first write after the restart");

	// both sides restart with complete values in their slots: the values of before are not read again, and a
	// command with the number of the last one before the restart is a new command by its stamp
	ipc_command_t stC = { 1u, IPC_CMD_SERVO_ON }, stC2 = { 0u, 0u };
	uint32_t uApu = ipc_side_start(&Head()->apu, apApu, 2);
	IPC_WRITE(pCmd, uApu, &stC);
	Expect(IPC_READ(pCmd, &Head()->apu, &stC2, auCWas) == IPC_OK && auCWas[0] == uApu, "a command");
	memcpy(auWas, auS, sizeof(auS));
	uEpoch = ipc_side_start(&Head()->r5, apR5, 1);
	uApu = ipc_side_start(&Head()->apu, apApu, 2);
	Expect(ReadIs(IPC_BUSY, 0, NULL) && IPC_READ(pCmd, &Head()->apu, &stC2, auC) == IPC_BUSY,
	       "both sides restarted: the values of before are not read");
	IPC_WRITE(pCmd, uApu, &stC);
	Expect(IPC_READ(pCmd, &Head()->apu, &stC2, auC) == IPC_OK && stC2.seq == 1u && auC[0] == auCWas[0] + 1u &&
	       auC[1] == auCWas[1] + 2u, "the same command number after a restart: the next epoch, the sequence number going on");
	Fill(au, 4);
	ipc_slot_write(pSlot, uEpoch, au, WORDS);
	Expect(ReadIs(IPC_OK, 4, auS) && auS[0] == auWas[0] + 1u && auS[1] == auWas[1] + 2u, "the first write after both restarted");

	// a complete value of another epoch than its writer's now (not reachable through ipc_side_start: the last
	// line of defence)
	ipc_store32(&Head()->r5.epoch, uEpoch + 5u);
	Expect(ReadIs(IPC_OLD_EPOCH, 0, NULL), "a value of another epoch: IPC_OLD_EPOCH");
	ipc_store32(&Head()->r5.epoch, uEpoch);

	// the wrap of the sequence number: 0xFFFFFFFC -> 0xFFFFFFFE -> 2 -> 4, never 0
	ipc_store32(pSlot + IPC_SLOT_GEN, 0xFFFFFFFCu);
	Fill(au, 5);
	ipc_slot_write(pSlot, uEpoch, au, WORDS);
	Expect(ReadIs(IPC_OK, 5, auS) && auS[1] == 0xFFFFFFFEu, "the write before the wrap");
	Fill(au, 6);
	ipc_slot_write(pSlot, uEpoch, au, WORDS);
	Expect(ReadIs(IPC_OK, 6, auS) && auS[1] == 2u, "the write across the wrap: sequence number 2");
	Fill(au, 7);
	ipc_slot_write(pSlot, uEpoch, au, WORDS);
	Expect(ReadIs(IPC_OK, 7, auS) && auS[1] == 4u, "the write after the wrap");

	// a peer that is not this layout, and a payload that does not fit the reader
	ipc_store32(&Head()->r5.magic, IPC_MAGIC ^ 1u);
	Expect(ReadIs(IPC_NO_PEER, 0, NULL), "another magic: IPC_NO_PEER");
	ipc_store32(&Head()->r5.magic, IPC_MAGIC);
	ipc_store32(&Head()->r5.layout, IPC_LAYOUT + 1u);
	Expect(ReadIs(IPC_NO_PEER, 0, NULL), "another layout: IPC_NO_PEER");
	ipc_store32(&Head()->r5.layout, IPC_LAYOUT);
	Expect(ReadIs(IPC_OK, 7, NULL), "magic and layout restored");
	uint32_t auBig[WORDS + 1] = { 0 };
	Expect(ipc_slot_read(pSlot, &Head()->r5, auBig, WORDS + 1, auS) == IPC_BAD_SIZE && auBig[0] == 0u,
	       "a payload larger than the reader's buffer: IPC_BAD_SIZE");
}

struct ReaderResult
{
	long nOk = 0, nBusy = 0, nTorn = 0, nOther = 0, nBad = 0, nOrder = 0, nStamp = 0;
	uint32_t uLastK = 0;
};

} // namespace

int main(int argc, char** argv)
{
	long nSnapshots = 10000000;
	int nReaders = 2;
	bool bUnsafe = false;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "--snapshots") && i + 1 < argc) nSnapshots = atol(argv[++i]);
		else if (!strcmp(argv[i], "--readers") && i + 1 < argc) nReaders = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--unsafe")) bUnsafe = true;
		else
		{
			fprintf(stderr, "usage: %s [--snapshots N] [--readers R] [--unsafe]\n", argv[0]);
			return 2;
		}
	}
	if (nReaders < 1 || nSnapshots < 1 || nSnapshots > 0x7FFFFFFFL)
		return 2;

	printf("layout %u: head %u B; State 0x%03x, %u B; Torque 0x%03x, %u B; Command 0x%03x, %u B; + %u B per slot\n",
	       (unsigned)IPC_LAYOUT, (unsigned)sizeof(ipc_head_t), (unsigned)IPC_OFF_STATE, (unsigned)sizeof(ipc_state_t),
	       (unsigned)IPC_OFF_TORQUE, (unsigned)sizeof(ipc_torque_t), (unsigned)IPC_OFF_COMMAND,
	       (unsigned)sizeof(ipc_command_t), 4u * IPC_SLOT_HEAD_WORDS);
	Payloads();
	Faults();
	printf("payloads and faults: %d checks failed\n", g_nFailed);

	// part 3: the sequence number starts 256 below the wrap
	volatile uint32_t* const pSlot = Slot(IPC_OFF_STATE);
	volatile uint32_t* apR5[] = { pSlot };
	memset(g_auRegion, 0, sizeof(g_auRegion));
	ipc_store32(pSlot + IPC_SLOT_GEN, 0xFFFFFF00u);
	const uint32_t uEpoch = ipc_side_start(&Head()->r5, apR5, 1);

	std::atomic<bool> bDone(false);
	std::vector<ReaderResult> vRes((size_t)nReaders);
	std::vector<std::thread> vThreads;
	for (int r = 0; r < nReaders; r++)
		vThreads.emplace_back([&, r]() {
			ReaderResult& res = vRes[(size_t)r];
			uint32_t au[WORDS] = { 0 }, auS[2] = { 0, 0 }, uLastGen = 0;
			for (bool bLast = false; !bLast; )
			{
				bLast = bDone.load();		// one more read after the last write
				int nStatus = IPC_OK;
				if (bUnsafe)
					RawCopy(pSlot, au);
				else
					nStatus = ipc_slot_read(pSlot, &Head()->r5, au, WORDS, auS);
				if (nStatus == IPC_BUSY) { res.nBusy++; continue; }
				if (nStatus == IPC_TORN) { res.nTorn++; continue; }
				if (nStatus != IPC_OK) { res.nOther++; continue; }
				res.nOk++;
				if (!Consistent(au))
				{
					res.nBad++;
					continue;
				}
				if (au[0] < res.uLastK)
					res.nOrder++;
				if (!bUnsafe && (auS[0] != uEpoch || (auS[1] & 1u) || auS[1] == 0u ||
				                 (auS[1] == uLastGen) != (au[0] == res.uLastK)))
					res.nStamp++;
				res.uLastK = au[0];
				uLastGen = auS[1];
			}
		});

	uint32_t auW[WORDS];
	for (long k = 1; k <= nSnapshots; k++)
	{
		Fill(auW, (uint32_t)k);
		ipc_slot_write(pSlot, uEpoch, auW, WORDS);
	}
	bDone.store(true);
	for (auto& t : vThreads)
		t.join();

	long nOk = 0, nBad = 0, nOrder = 0, nStamp = 0, nOther = 0, nNotLast = 0;
	printf("%s: 1 writer, %ld snapshots of %d words; %d readers, one attempt per read\n",
	       bUnsafe ? "UNSAFE (no check)" : "slot", nSnapshots, (int)WORDS, nReaders);
	for (int r = 0; r < nReaders; r++)
	{
		const ReaderResult& res = vRes[(size_t)r];
		printf("  reader %d: %ld copies accepted, %ld busy, %ld torn, %ld other; %ld inconsistent, %ld out of order, "
		       "%ld with a wrong stamp\n", r, res.nOk, res.nBusy, res.nTorn, res.nOther, res.nBad, res.nOrder, res.nStamp);
		nOk += res.nOk;
		nBad += res.nBad;
		nOrder += res.nOrder;
		nStamp += res.nStamp;
		nOther += res.nOther;
		if (!bUnsafe && res.uLastK != (uint32_t)nSnapshots)
			nNotLast++;
	}
	const bool bPass = (g_nFailed == 0 && nOk > 0 && nBad == 0 && nOrder == 0 && nStamp == 0 && nOther == 0 && nNotLast == 0);
	printf("%s: %d checks failed, %ld inconsistent copies accepted, %ld readers without the last snapshot\n",
	       bPass ? "PASS" : "FAIL", g_nFailed, nBad, nNotLast);
	return bPass ? 0 : 1;
}
