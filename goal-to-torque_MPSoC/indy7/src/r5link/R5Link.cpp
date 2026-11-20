/*****************************************************************************
*	Name: R5Link.cpp
*	Description: The APU side of the exchange with the R5. See R5Link.h.
*****************************************************************************/
#include "r5link/R5Link.h"

#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <sys/mman.h>

namespace {

/* the first line of a small text file (sysfs), without its newline; FALSE: not readable */
BOOL ReadLine(const char* apcPath, char* apcOut, size_t auLen)
{
	FILE* fp = fopen(apcPath, "r");
	if (fp == NULL)
		return FALSE;
	const BOOL bOk = (fgets(apcOut, (int)auLen, fp) != NULL) ? TRUE : FALSE;
	fclose(fp);
	if (bOk)
		apcOut[strcspn(apcOut, "\r\n")] = '\0';
	return bOk;
}

const char* const READ_RESULT[IPC_BAD_SIZE + 1] = { "ok", "the R5 side not up", "being written", "torn", "old epoch",
                                                    "size" };

} // namespace

CR5Link::CR5Link()
	: m_pRegion(NULL), m_nFd(-1), m_uMapBytes(0), m_uApuEpoch(0), m_uDof(0), m_uCfgCrc(0), m_tPeriod(0),
	  m_tWakeDelay(0), m_bHaveState(FALSE), m_bLastGridKept(FALSE), m_uR5Epoch(0), m_tStateRelease(0), m_nGridShift(0),
	  m_uPending(IPC_CMD_NONE), m_bPendingAgain(FALSE), m_uLastCommand(IPC_CMD_NONE), m_uCommandSeq(0),
	  m_bToldBadState(FALSE), m_bToldBadTime(FALSE), m_bToldFollowing(FALSE)
{
	memset(m_acWhere, 0, sizeof(m_acWhere));
	memset(&m_stState, 0, sizeof(m_stState));
	memset(&m_stCounts, 0, sizeof(m_stCounts));
	m_auShadowStamp[0] = m_auShadowStamp[1] = 0;
	m_auReportStamp[0] = m_auReportStamp[1] = 0;
	m_auSimStamp[0] = m_auSimStamp[1] = 0;
	m_stFollow.Start(1, 0);
}

CR5Link::~CR5Link()
{
	if (m_nFd >= 0)
	{
		munmap((void*)m_pRegion, m_uMapBytes);
		close(m_nFd);
	}
}

// the UIO device whose name is that of the device tree node: its first map is the region
BOOL
CR5Link::OpenDevice()
{
	char acPath[96], acText[64];
	for (int n = 0; n < 32; n++)
	{
		snprintf(acPath, sizeof(acPath), "/sys/class/uio/uio%d/name", n);
		if (FALSE == ReadLine(acPath, acText, sizeof(acText)) || 0 != strcmp(acText, R5_LINK_UIO_NAME))
			continue;
		unsigned long long uAddr = 0, uSize = 0;
		snprintf(acPath, sizeof(acPath), "/sys/class/uio/uio%d/maps/map0/addr", n);
		if (ReadLine(acPath, acText, sizeof(acText)))
			uAddr = strtoull(acText, NULL, 0);
		snprintf(acPath, sizeof(acPath), "/sys/class/uio/uio%d/maps/map0/size", n);
		if (ReadLine(acPath, acText, sizeof(acText)))
			uSize = strtoull(acText, NULL, 0);
		if (uAddr != R5_LINK_OCM_ADDRESS || uSize < IPC_OFF_END2)
		{
			DBG_LOG_ERROR("(CR5Link) uio%d (%s): map0 at 0x%llx, %llu bytes; the region of the R5 is at 0x%llx, "
			              "at least %u bytes", n, R5_LINK_UIO_NAME, uAddr, uSize, R5_LINK_OCM_ADDRESS,
			              (unsigned)IPC_OFF_END2);
			return FALSE;
		}
		char acDevice[24];
		snprintf(acDevice, sizeof(acDevice), "/dev/uio%d", n);
		m_nFd = open(acDevice, O_RDWR | O_SYNC);
		if (m_nFd < 0)
		{
			DBG_LOG_ERROR("(CR5Link) %s: %s (the udev rule of rpu_work/README_S1.md gives it to the user)", acDevice,
			              strerror(errno));
			return FALSE;
		}
		void* const pMap = mmap(NULL, (size_t)uSize, PROT_READ | PROT_WRITE, MAP_SHARED, m_nFd, 0);
		if (pMap == MAP_FAILED)
		{
			DBG_LOG_ERROR("(CR5Link) %s: mmap of %llu bytes: %s", acDevice, uSize, strerror(errno));
			close(m_nFd);
			m_nFd = -1;
			return FALSE;
		}
		m_pRegion = (volatile uint32_t*)pMap;
		m_uMapBytes = (size_t)uSize;
		snprintf(m_acWhere, sizeof(m_acWhere), "%s (OCM 0x%llx)", acDevice, uAddr);
		return TRUE;
	}
	DBG_LOG_ERROR("(CR5Link) no UIO device named %s: the device tree of this boot has no node for the region of "
	              "the R5 (rpu_work/README_S1.md)", R5_LINK_UIO_NAME);
	return FALSE;
}

void
CR5Link::UseRegion(volatile uint32_t* apRegion, const char* apcWhat)
{
	m_pRegion = apRegion;
	snprintf(m_acWhere, sizeof(m_acWhere), "%s", apcWhat);
}

BOOL
CR5Link::Start(uint32_t auDof, uint32_t auCfgCrc, RTTIME atPeriod, RTTIME atWakeDelay)
{
	if (m_pRegion == NULL || auDof == 0 || auDof > (uint32_t)IPC_MAX_DOF || atWakeDelay >= atPeriod)
	{
		DBG_LOG_ERROR("(CR5Link) no region, %u joints, or a delay outside the period", (unsigned)auDof);
		return FALSE;
	}
	m_uDof = auDof;
	m_uCfgCrc = auCfgCrc;
	m_tPeriod = atPeriod;
	m_tWakeDelay = atWakeDelay;
	m_stFollow.Start((int64_t)atPeriod, (int64_t)atWakeDelay);
	volatile uint32_t* apSlots[5] = { Slot(IPC_OFF_TORQUE), Slot(IPC_OFF_COMMAND), Slot(IPC_OFF_SHADOW_IN), Slot(IPC_OFF_GOAL),
	                                  Slot(IPC_OFF_SIM_STATE) };
	m_uApuEpoch = ipc_side_start(&Head()->apu, apSlots, 5);
	// what the Shadow out slot holds now answers a run of this side before this one: not taken (no read: nothing
	// there). The same for the report and the torque of a simulated arm
	ipc_shadow_out_t stOld;
	(void)IPC_READ(Slot(IPC_OFF_SHADOW_OUT), &Head()->r5, &stOld, m_auShadowStamp);
	ipc_report_t stOldReport;
	(void)IPC_READ(Slot(IPC_OFF_REPORT), &Head()->r5, &stOldReport, m_auReportStamp);
	ipc_sim_torque_t stOldTorque;
	(void)IPC_READ(Slot(IPC_OFF_SIM_TORQUE), &Head()->r5, &stOldTorque, m_auSimStamp);
	// a first offset of the counter while nothing is timed yet: a cycle whose pair of reads is refused keeps the last one
	for (int nTry = 0; nTry < 100 && !m_cClock.Sample(); nTry++)
		;
	// whatever the R5 was told before this start: the drives are off until the operator asks
	RequestCommand(IPC_CMD_SERVO_OFF);
	const BOOL bR5Up = (ipc_load32(&Head()->r5.magic) == IPC_MAGIC) ? TRUE : FALSE;
	printf("[R5] link at %s: APU epoch %u; the R5 side is %s; the tasks follow its period, released %.0f us after "
	       "it; check value of this configuration 0x%08X; counter %llu ticks/s%s\n", m_acWhere,
	       (unsigned)m_uApuEpoch, bR5Up ? "up" : "not up yet (no State slot until it is)",
	       (double)atWakeDelay * 1e-3, (unsigned)auCfgCrc, (unsigned long long)m_cClock.TicksPerSecond(),
	       link_clock::SAME_AS_MONOTONIC ? " (this host: the clock of the tasks)" : "");
	return TRUE;
}

BOOL
CR5Link::NoState()
{
	m_stCounts.nNoState++;
	if (m_stFollow.bFollowing)
		m_stCounts.nNoStateFollowing++;
	return FALSE;
}

BOOL
CR5Link::Receive(RTTIME atPlannedRelease)
{
	m_nGridShift = 0;
	const RTTIME tRead = read_timer();
	ipc_state_t stRead;
	uint32_t auStamp[2];
	const int nRead = IPC_READ(Slot(IPC_OFF_STATE), &Head()->r5, &stRead, auStamp);
	if (nRead != IPC_OK)
	{
		m_stCounts.anReadFailed[nRead]++;
		return NoState();
	}
	if (stRead.dof != m_uDof)
	{
		m_stCounts.nBadState++;
		if (!m_bToldBadState)
			printf("[R5] WARNING: the State slot has %u joints, the bus of this configuration %u: not taken "
			       "(another firmware target?)\n", (unsigned)stRead.dof, (unsigned)m_uDof);
		m_bToldBadState = TRUE;
		return NoState();
	}
	const BOOL bRestart = (m_bHaveState && auStamp[0] != m_uR5Epoch) ? TRUE : FALSE;
	if (m_bHaveState && !bRestart && stRead.period == m_stState.period)
		return NoState();

	/* a new period */
	uint32_t uPeriods = 0;		// since the last State taken; 0: none to count from
	if (bRestart)
	{
		m_stCounts.nR5Restarts++;
		m_stFollow.Restart();
		printf("[R5] the R5 started again (epoch %u -> %u): its drives are off, its periods start anew\n",
		       (unsigned)m_uR5Epoch, (unsigned)auStamp[0]);
	}
	else if (m_bHaveState)
	{
		uPeriods = stRead.period - m_stState.period;
		// counted from a State that was read on the grid: the read that sets the grid (the start, a restart of
		// the R5) comes at any time in the period, and the cycle after it up to a period late
		if (m_bLastGridKept)
		{
			m_stCounts.nPeriodsMissed += (long)(uPeriods - 1u);
			// the torque computed from the State taken before has that period's number; an R5 that stops
			// takes no torque, nor does one that has the control task
			if (stRead.torque_period != m_stState.period && !(stRead.flags & (IPC_STATE_STOP | IPC_STATE_CONTROL)))
				m_stCounts.nTorqueLate++;
		}
	}

	/* its release on the clock of the tasks: the counter must have gone on by the periods that passed */
	if (!m_cClock.Sample())
		m_stCounts.nClockPairsRefused++;
	BOOL bTimeOk = m_cClock.IsValid() ? TRUE : FALSE;
	if (bTimeOk && uPeriods > 0)
	{
		const int64_t nExpected = (int64_t)uPeriods * (int64_t)m_tPeriod;
		const int64_t nPassed = (stRead.t_release > m_stState.t_release) ?
			(int64_t)link_clock::NsOfTicks(stRead.t_release - m_stState.t_release, m_cClock.TicksPerSecond()) : 0;
		if (nPassed < nExpected - (int64_t)m_tPeriod / 2 || nPassed > nExpected + (int64_t)m_tPeriod / 2)
			bTimeOk = FALSE;
	}
	const long nJumpsBefore = m_stFollow.nJumps;
	if (bTimeOk)
	{
		m_tStateRelease = m_cClock.MonotonicOf(stRead.t_release);
		m_nGridShift = m_stFollow.Step(atPlannedRelease, m_tStateRelease);
		if (!m_bToldFollowing)
			printf("[R5] following the period of the R5 from its period %u on\n", (unsigned)stRead.period);
		m_bToldFollowing = TRUE;
	}
	else
	{
		m_stCounts.nBadTime++;
		if (m_cClock.IsValid() && !m_bToldBadTime)
		{
			printf("[R5] WARNING: the release time of the State slot (period %u) did not go on by the periods that "
			       "passed: the grid is not moved, the release is estimated (a firmware without the counter?)\n",
			       (unsigned)stRead.period);
			m_bToldBadTime = TRUE;
		}
		m_tStateRelease = (uPeriods > 0) ? m_tStateRelease + (RTTIME)uPeriods * m_tPeriod :
		                                   atPlannedRelease - m_tWakeDelay;
	}
	m_stState = stRead;
	m_uR5Epoch = auStamp[0];
	m_bHaveState = TRUE;
	m_bLastGridKept = (m_stFollow.nJumps == nJumpsBefore) ? TRUE : FALSE;
	m_stCounts.nStates++;
	if (m_bLastGridKept && tRead > m_tStateRelease)
	{
		const RTTIME tAge = tRead - m_tStateRelease;
		if (m_stCounts.nAges == 0 || tAge < m_stCounts.tAgeMin) m_stCounts.tAgeMin = tAge;
		if (tAge > m_stCounts.tAgeMax) m_stCounts.tAgeMax = tAge;
		m_stCounts.dAgeSumNs += (double)tAge;
		m_stCounts.nAges++;
	}
	return TRUE;
}

void
CR5Link::RequestCommand(uint32_t auCommand, BOOL abAgain)
{
	m_uPending = auCommand;
	m_bPendingAgain = abAgain;
}

BOOL
CR5Link::SendCommand()
{
	const uint32_t uCommand = m_uPending;
	const BOOL bAgain = m_bPendingAgain;
	m_uPending = IPC_CMD_NONE;
	if (uCommand == IPC_CMD_NONE)
		return TRUE;
	if (uCommand == IPC_CMD_SERVO_ON)
	{
		if (!IsServoOnAllowed())
		{
			m_stCounts.nServoOnRefused++;
			return FALSE;
		}
	}
	// servo off and stop are asked for in every cycle of a stop: written once
	else if (uCommand == m_uLastCommand && !bAgain)
		return TRUE;
	ipc_command_t stCommand;
	memset(&stCommand, 0, sizeof(stCommand));
	stCommand.seq = ++m_uCommandSeq;
	stCommand.cmd = uCommand;
	IPC_WRITE(Slot(IPC_OFF_COMMAND), m_uApuEpoch, &stCommand);
	m_uLastCommand = uCommand;
	m_stCounts.nCommands++;
	return TRUE;
}

void
CR5Link::SendTorque(uint32_t auStatePeriod, BOOL abValid, BOOL abEnabled, const double* adTau, unsigned auDof)
{
	ipc_torque_t stTorque;
	memset(&stTorque, 0, sizeof(stTorque));
	stTorque.period = auStatePeriod;
	stTorque.flags = (abValid ? (uint32_t)IPC_TORQUE_VALID : 0u) | (abEnabled ? (uint32_t)IPC_TORQUE_ENABLED : 0u);
	for (unsigned i = 0; i < auDof && i < (unsigned)IPC_MAX_DOF; i++)
		stTorque.tau[i] = adTau[i];
	IPC_WRITE(Slot(IPC_OFF_TORQUE), m_uApuEpoch, &stTorque);
	m_stCounts.nTorquesWritten++;
}

void
CR5Link::SendShadow(const ipc_shadow_in_t& astIn)
{
	IPC_WRITE(Slot(IPC_OFF_SHADOW_IN), m_uApuEpoch, &astIn);
}

BOOL
CR5Link::TakeShadow(ipc_shadow_out_t* apOut)
{
	ipc_shadow_out_t stOut;
	uint32_t auStamp[2];
	if (IPC_READ(Slot(IPC_OFF_SHADOW_OUT), &Head()->r5, &stOut, auStamp) != IPC_OK ||
	    (auStamp[0] == m_auShadowStamp[0] && auStamp[1] == m_auShadowStamp[1]))
		return FALSE;
	m_auShadowStamp[0] = auStamp[0];
	m_auShadowStamp[1] = auStamp[1];
	*apOut = stOut;
	return TRUE;
}

uint32_t
CR5Link::SendControlCommand(const ipc_command_t& astCommand)
{
	ipc_command_t stCommand = astCommand;
	// the run of this side goes into the number: the report can still have the answer to a command of the run
	// before this one, which counted from 1 too (two numbers meet after 65536 commands of a run or 65536 runs)
	stCommand.seq = (m_uApuEpoch << 16) + ++m_uCommandSeq;
	stCommand.reserved = 0;
	IPC_WRITE(Slot(IPC_OFF_COMMAND), m_uApuEpoch, &stCommand);
	m_stCounts.nCommands++;
	return stCommand.seq;
}

void
CR5Link::SendGoal(const double* adP)
{
	ipc_goal_t stGoal;
	for (int k = 0; k < 3; k++)
		stGoal.p[k] = adP[k];
	IPC_WRITE(Slot(IPC_OFF_GOAL), m_uApuEpoch, &stGoal);
}

BOOL
CR5Link::TakeReport(ipc_report_t* apReport)
{
	ipc_report_t stReport;
	uint32_t auStamp[2];
	if (IPC_READ(Slot(IPC_OFF_REPORT), &Head()->r5, &stReport, auStamp) != IPC_OK ||
	    (auStamp[0] == m_auReportStamp[0] && auStamp[1] == m_auReportStamp[1]))
		return FALSE;
	m_auReportStamp[0] = auStamp[0];
	m_auReportStamp[1] = auStamp[1];
	*apReport = stReport;
	return TRUE;
}

void
CR5Link::SendSimState(const ipc_sim_state_t& astSim)
{
	IPC_WRITE(Slot(IPC_OFF_SIM_STATE), m_uApuEpoch, &astSim);
}

BOOL
CR5Link::TakeSimTorque(ipc_sim_torque_t* apTorque)
{
	ipc_sim_torque_t stTorque;
	uint32_t auStamp[2];
	if (IPC_READ(Slot(IPC_OFF_SIM_TORQUE), &Head()->r5, &stTorque, auStamp) != IPC_OK ||
	    (auStamp[0] == m_auSimStamp[0] && auStamp[1] == m_auSimStamp[1]))
		return FALSE;
	m_auSimStamp[0] = auStamp[0];
	m_auSimStamp[1] = auStamp[1];
	*apTorque = stTorque;
	return TRUE;
}

void
CR5Link::PrintSummary() const
{
	const Counts& c = m_stCounts;
	long nFailed = 0;
	char acFailed[160] = "";
	int nLen = 0;
	for (int n = IPC_OK + 1; n <= IPC_BAD_SIZE; n++)
	{
		nFailed += c.anReadFailed[n];
		if (c.anReadFailed[n] > 0 && nLen < (int)sizeof(acFailed) - 40)
			nLen += snprintf(acFailed + nLen, sizeof(acFailed) - (size_t)nLen, "%s%ld %s", nLen > 0 ? ", " : ": ",
			                 c.anReadFailed[n], READ_RESULT[n]);
	}
	const double dN = c.nAges > 0 ? (double)c.nAges : 1.0;
	printf("[R5] State slot: %ld periods taken, read %.1f us after their release (min %.1f, max %.1f us); %ld cycles "
	       "without a new State (%ld while following); %ld periods of the R5 that no cycle took; %ld reads failed%s\n",
	       c.nStates, c.dAgeSumNs / dN * 1e-3, (double)c.tAgeMin * 1e-3, (double)c.tAgeMax * 1e-3, c.nNoState,
	       c.nNoStateFollowing, c.nPeriodsMissed, nFailed, acFailed);
	const R5Follow& f = m_stFollow;
	printf("[R5] following: phase error mean %.1f us, min %.1f, max %.1f us in %ld periods (the tasks %.0f us behind "
	       "the R5); the grid was set at once %ld time%s; %ld States with another number of joints, %ld with a "
	       "release time that did not fit, %ld clock offsets not taken; restarts of the R5: %ld\n",
	       f.nSamples > 0 ? f.dErrorSum / (double)f.nSamples * 1e-3 : 0.0, (double)f.nErrorMin * 1e-3,
	       (double)f.nErrorMax * 1e-3, f.nSamples, (double)m_tWakeDelay * 1e-3, f.nJumps, f.nJumps == 1 ? "" : "s",
	       c.nBadState, c.nBadTime, c.nClockPairsRefused, c.nR5Restarts);
	printf("[R5] Torque slot: %ld written; %ld late (the R5 began the period of a State without the torque of the "
	       "State taken before it); the R5 counts %u periods without a new torque while its drives were on\n",
	       c.nTorquesWritten,
	       c.nTorqueLate, m_bHaveState ? (unsigned)m_stState.n_torque_stale : 0u);
	uint32_t uDone = m_bHaveState ? m_stState.cmd_done : 0u;
	if (HasControlTask())		// the number of its commands has the run in it (SendControlCommand); 0: none of this run
		uDone = (uDone - (m_uApuEpoch << 16) <= m_uCommandSeq) ? uDone - (m_uApuEpoch << 16) : 0u;
	printf("[R5] Command slot: %ld written, the R5 carried out number %u of %u; %ld servo on refused (check value of "
	       "the firmware 0x%08X, of this configuration 0x%08X)\n", c.nCommands,
	       (unsigned)uDone, (unsigned)m_uCommandSeq, c.nServoOnRefused,
	       m_bHaveState ? (unsigned)m_stState.cfg_crc : 0u, (unsigned)m_uCfgCrc);
	if (m_bHaveState)
		printf("[R5] faults the R5 reports: %u late releases, %u overruns, %u missed send deadlines, %u incomplete "
		       "working counters\n", (unsigned)m_stState.n_late, (unsigned)m_stState.n_overrun,
		       (unsigned)m_stState.n_missed, (unsigned)m_stState.n_wkc);
}
