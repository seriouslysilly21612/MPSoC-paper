/*****************************************************************************
*	Name: SimR5.h
*	Description: The R5 of the simulation (-s with --arch S1): a thread that
*	does in every period what the EtherCAT task of the firmware does
*	(rpu_work/r5_freertos: ecat_task.c), with the simulated arm in place of
*	the bus:
*
*	  receive  the state that the simulator returns (CSimArm::GetMeasured)
*	           and the status of every simulated drive as it is now
*	  slots    the Command and the Torque slot of the APU, and the
*	           stale-torque rule (SimR5Slots.h)
*	  drives   while the servo is asked for, every drive is walked on in
*	           every period, and an enabled drive gets the torque of the
*	           Torque slot if the APU marked it valid and enabled, otherwise 0
*	  State    written with the status and the measurement of the receive
*	  send     one step of the simulator
*
*	The region is memory of this process with the layout of the OCM window
*	(ipc/ipc_layout.h); the APU side reaches it with CR5Link::UseRegion.
*	The periods lie on a grid of this thread, apart from the grid of the
*	tasks, which therefore follow it as they follow the R5. The release time
*	of the State slot is that of the grid: the R5 starts on its tick, and
*	the delay with which a host wakes this thread is not the R5's. A release
*	that the host let pass is not run afterwards, and its period number is
*	left out (a burst of late periods would only count as lost torques).
*
*	A model of what the APU sees of the R5, written from the sources of the
*	firmware; it shares no code with it, and it does not model the bus (the
*	working counters, the CiA 402 walk of a real drive, its fault resets).
*
*	With --arch S2 the R5 has the control task (SetControl): the thread then
*	runs the period of sim/SimR5Control.h, a firmware built with
*	CONTROL_TASK on a bus without the arm. The arm stays with the
*	application, which steps it from the Sim torque slot.
*****************************************************************************/
#ifndef G2T_SIM_R5_H
#define G2T_SIM_R5_H

#include <atomic>
#include <vector>

#include "../base/RtTask.h"
#include "../r5link/LinkClock.h"
#include "SimArm.h"
#include "SimJoint.h"
#include "SimR5Control.h"
#include "SimR5Slots.h"

class CShadowControl;

class CSimR5
{
public:
	struct Spec
	{
		RTTIME		tPeriod;		// [ns]
		uint32_t	uCfgCrc;		// the check value its State slot carries (FwCfgCrc)
		int			nStaleLimit;	// SIM_STALE_LIMIT
		int			nPriority;		// SCHED_FIFO of the thread
		int			nCpu;			// -1: the CPUs of the process
		unsigned	uDrives;		// with the control task (SetControl): the drives of its bus, which are not the arm
	};
	struct Counts
	{
		long	nPeriods;			// periods run
		long	nLeftOut;			// periods whose release the host let pass
		long	nOverruns;			// periods that ended after the next release
		long	nStaleStops;		// servo offs for want of a new torque
		RTTIME	tCycleMax;			// longest period, release to the end of the step [ns]
	};

	CSimR5();

	/* the region, as the OCM window of the board */
	volatile uint32_t*	Region	(	) { return m_auRegion; }
	/* the R5 side comes up and the thread starts: the arm and its joints belong to it from here on. With the
	 * control task (SetControl) it has no arm: NULL and no joints */
	BOOL	Start			(const Spec& astSpec, CSimArm* apArm, const std::vector<CSimJoint*>& avJoints);
	/* --arch S2, before Start(): the control task that the R5 runs (built; it stays the caller's) */
	void	SetControl		(CControlCycle* apControl) { m_pControl = apControl; }
	/* --shadow, before Start(): the controller that the R5 runs after the send of every period on what the
	 * control task told (the firmware: shadow_link.c). It stays the caller's. */
	void	SetShadow		(CShadowControl* apShadow) { m_pShadow = apShadow; }
	/* the thread ends: the servo off first */
	void	Stop			(	);
	/* the fault tests of the simulation ('n'), taken at the next receive: a joint (1-based) leaves Operation
	 * enabled; the power of every drive is cut (1) or back (2), and what the drives report meanwhile */
	void	Dropout			(int anJoint) { m_nDropout.store(anJoint, std::memory_order_release); }
	void	PowerCut		(int anCutOrRestore, int anReports);

	const Counts&	GetCounts	(	) const { return m_stCounts; }
	void	PrintSummary	(	) const;

private:
	static void	Entry		(void* apThis);
	void	Run				(	);
	void	Cycle			(uint64_t auReleaseTicks, uint32_t auPeriod);

	alignas(64) volatile uint32_t m_auRegion[IPC_OFF_END2 / 4];
	Spec					m_stSpec;
	CShadowControl*			m_pShadow{NULL};
	CControlCycle*			m_pControl{NULL};
	CSimR5Control			m_cControlR5;		// the thread, with m_pControl
	CSimArm*				m_pArm;
	std::vector<CSimJoint*>	m_vJoints;
	CRtTask					m_cTask;
	std::atomic<bool>		m_bStop{false}, m_bEnded{true};
	std::atomic<int>		m_nDropout{0}, m_nPowerCmd{0}, m_nPowerReports{0};

	/* the thread */
	CSimR5Slots				m_cSlots;
	CSimR5Slots::Drives		m_stDrives;
	link_clock::CTickClock	m_cClock;
	Counts					m_stCounts;
};

#endif // G2T_SIM_R5_H
