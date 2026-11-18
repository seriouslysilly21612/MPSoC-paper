/*****************************************************************************
*	Name: R5Link.h
*	Description: The APU side of the exchange with the R5 (--arch S1; D-09).
*	The EtherCAT task runs on the R5 (rpu_work/r5_freertos); the tasks here
*	take its State slot and write the Torque and the Command slot of
*	ipc/ipc_layout.h, by the rules of ipc/ipc_slot.h.
*
*	  where    on the board the first 16 KB of the OCM, which Linux gives as
*	           the UIO device "ocm-link" (OpenDevice;
*	           rpu_work/ocm_link_uio.dtso); in the simulation the region of
*	           the simulated R5 (UseRegion; sim/SimR5.h)
*	  State    Receive(), once per cycle of the EtherCAT task of the
*	           application: one read, never a wait. A new period brings the
*	           state, the release of that period on the clock of the tasks
*	           (LinkClock.h), and the move of the release grid that keeps
*	           the tasks WAKE_DELAY_US behind the R5 (R5Follow.h)
*	  Torque   SendTorque(), by the control task right after the control
*	           law: the R5 takes it at the start of its next period. A
*	           State whose period the R5 began without the torque of the
*	           State taken before it is counted here as a late torque
*	           (torque_period of the State slot)
*	  Command  servo on, servo off, stop: asked for with RequestCommand(),
*	           written once per cycle by SendCommand(). Servo off and stop
*	           are sent once, not in every cycle that asks again. A servo on
*	           leaves only when the firmware of the R5 was built from this
*	           configuration: the check value in its State slot equals the
*	           one of the configuration here (D-15, ipc/FwCfg.h)
*
*	What the R5 sends is checked before it is used: the number of joints,
*	and the release time, which must go on by the periods that passed (a
*	firmware without the counter leaves it 0); with a time that fails, the
*	state is taken, the grid is not moved, and the release is estimated.
*
*	One thread per slot: Receive, RequestCommand and SendCommand belong to
*	the EtherCAT task of the application, SendTorque to the control task.
*	Nothing here allocates, locks or waits after Start().
*****************************************************************************/
#ifndef G2T_R5_LINK_H
#define G2T_R5_LINK_H

#include <cstdint>

#include "base/Common.h"
#include "ipc/ipc_slot.h"
#include "r5link/LinkClock.h"
#include "r5link/R5Follow.h"

/* the device tree node of the region (rpu_work/ocm_link_uio.dtso) and where the firmware has it (ocm_link.h) */
#define R5_LINK_UIO_NAME		"ocm-link"
#define R5_LINK_OCM_ADDRESS		0xFFFC0000ull

class CR5Link
{
public:
	struct Counts
	{
		long	nStates;			// periods of the R5 taken
		long	nNoState;			// cycles without a new State
		long	nNoStateFollowing;	// of them, once the grid followed the R5
		long	nPeriodsMissed;		// periods of the R5 that no cycle took
		long	anReadFailed[IPC_BAD_SIZE + 1];	// reads of the State slot, by the result of ipc_slot_read
		long	nBadState;			// States with another number of joints
		long	nBadTime;			// States whose release time did not go on by the periods that passed
		long	nClockPairsRefused;	// offsets of the counter not taken (LinkClock.h)
		long	nR5Restarts;
		long	nTorqueLate;		// States whose period the R5 began without the torque of the State taken before
		long	nTorquesWritten;	// control task
		long	nCommands;			// Command slots written
		long	nServoOnRefused;	// servo ons that did not leave: another check value
		/* a new State was read this long after the release of its period [ns] */
		long	nAges;
		double	dAgeSumNs;
		RTTIME	tAgeMin, tAgeMax;
	};

	CR5Link();
	~CR5Link();

	/* where the region is, before Start(): the UIO device of the board, or memory of this process */
	BOOL	OpenDevice		(	);
	void	UseRegion		(volatile uint32_t* apRegion, const char* apcWhat);
	/* the APU side comes up: auDof joint drives on the bus of the R5, the check value of the configuration
	 * (FwCfgCrc), the period, and the delay of the tasks behind the R5 [ns] */
	BOOL	Start			(uint32_t auDof, uint32_t auCfgCrc, RTTIME atPeriod, RTTIME atWakeDelay);

	/* --- the EtherCAT task of the application --- */
	/* the State slot, once per cycle: TRUE with the State of a new period */
	BOOL	Receive			(RTTIME atPlannedRelease);
	const ipc_state_t& State(	) const { return m_stState; }
	/* the release of that period on the clock of the tasks [ns] */
	RTTIME	StateRelease	(	) const { return m_tStateRelease; }
	/* by how much the release grid moves after the last Receive [ns] (CRtTask::ShiftGrid) */
	int64_t	GridShift		(	) const { return m_nGridShift; }
	BOOL	IsFollowing		(	) const { return m_stFollow.bFollowing ? TRUE : FALSE; }
	/* the firmware of the R5 was built from this configuration */
	BOOL	IsServoOnAllowed(	) const { return (m_bHaveState && m_stState.cfg_crc == m_uCfgCrc) ? TRUE : FALSE; }
	uint32_t ConfigCrc		(	) const { return m_uCfgCrc; }
	/* a command for the next SendCommand (IPC_CMD_*); abAgain: also when it was the last one sent */
	void	RequestCommand	(uint32_t auCommand, BOOL abAgain = FALSE);
	/* FALSE: a servo on was refused */
	BOOL	SendCommand		(	);

	/* --- the control task --- */
	/* the torque computed from the State of period auStatePeriod [N m] */
	void	SendTorque		(uint32_t auStatePeriod, BOOL abValid, BOOL abEnabled, const double* adTau, unsigned auDof);

	/* after the tasks */
	const Counts&	GetCounts	(	) const { return m_stCounts; }
	const R5Follow&	GetFollow	(	) const { return m_stFollow; }
	void	PrintSummary	(	) const;

private:
	volatile ipc_head_t*	Head	(	) const { return (volatile ipc_head_t*)m_pRegion; }
	volatile uint32_t*		Slot	(unsigned auOffset) const { return m_pRegion + auOffset / 4u; }
	BOOL	NoState			(	);

	volatile uint32_t*	m_pRegion;
	int					m_nFd;				// the UIO device, -1 without it
	size_t				m_uMapBytes;
	char				m_acWhere[64];
	uint32_t			m_uApuEpoch;
	uint32_t			m_uDof, m_uCfgCrc;
	RTTIME				m_tPeriod, m_tWakeDelay;

	/* the EtherCAT task of the application */
	ipc_state_t			m_stState;			// the last State taken
	BOOL				m_bHaveState;
	uint32_t			m_uR5Epoch;
	RTTIME				m_tStateRelease;
	int64_t				m_nGridShift;
	link_clock::CTickClock m_cClock;
	R5Follow			m_stFollow;
	uint32_t			m_uPending;			// the command asked for
	BOOL				m_bPendingAgain;
	uint32_t			m_uLastCommand;		// the command last written
	uint32_t			m_uCommandSeq;
	BOOL				m_bToldBadState, m_bToldBadTime, m_bToldFollowing;
	Counts				m_stCounts;
};

#endif // G2T_R5_LINK_H
