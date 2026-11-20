/*****************************************************************************
*	Name: main.cpp
*	Description: The controller of the Indy7 (paper, Fig. 1).
*
*	    indy7_ctrl [-f FILE] [-s | -b [-B FILE]] [-t DIR] [--method NAME] [--arch NAME] [--shadow]
*
*	  -f FILE  the configuration; without it ./INDY7.cfg, then INDY7.cfg in
*	           the directory above the program (bin/..)
*	  -s       simulation: MuJoCo in place of the bus (paper, Sec. IV, V)
*	  -b       the bus of the robot in OP with its drives kept off, beside the
*	           simulated arm (the host-timing test, paper, Sec. III-C)
*	  -B FILE  with -b: the slaves of the bus from FILE instead of the
*	           configuration ([ECAT_MASTER] SLAVENUMBER and [AXIS0].., e.g.
*	           bench/L7N3_bus.cfg); the arm stays that of the configuration
*	  -t DIR   the timing of every period into DIR (TimingLog.h,
*	           tools/timing_summary.py)
*	  --method NAME (-m)  M1, M2, A2, A3 or FULL: what the camera application
*	           does with the results of the camera (obs/ObsMethod.h; paper,
*	           Table III). Default M1: the goal slot, as before
*	  --arch NAME (-a)  P0, S1 or S2: where the EtherCAT task and the
*	           control task run (Arch.h). Default P0: here, with the IgH
*	           master, as before. S1: the EtherCAT task on the R5; with -s
*	           the R5 is simulated too, with -b it runs its bus with the
*	           drives off beside the arm simulated here. S2: the control
*	           task on the R5 as well (TaskControlS2.cpp); the arm of -s
*	           and -b stays here, and the R5 computes its torque. With
*	           --method M1 only: the part of the other methods that runs in
*	           the control task is still here
*	  --shadow  with --arch S1: the control task tells the R5 every period
*	           what it did with its controller, and the R5 computes the same
*	           with a controller of its own (shadow/ShadowTask.h). Its torque
*	           is compared, not used: plan stage 3, whether the control
*	           computation fits the period of the R5. With --arch S2 the
*	           roles are the other way round: a controller here repeats
*	           every period of the R5's control task from its report
*	           (r5link/ControlFollower.h)
*
*	The keys of the operator are listed at the start and with '?'
*	(Indy7Ctrl.cpp, PrintHelp). 'q', SIGINT or SIGTERM end the controller:
*	the drives are switched off while the exchange goes on for 1 s, then the
*	master is released.
*****************************************************************************/
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <getopt.h>
#include <sys/mman.h>

#include "Arch.h"
#include "Indy7Ctrl.h"
#include "base/RtConsole.h"
#include "obs/ObsMethod.h"

namespace {

CRobotIndy7* volatile g_pRobot = NULL;

// a signal asks the tasks to stop; the EtherCAT task then switches the
// drives off, and main() ends the application as after 'q'
void SignalHandler(int)
{
	CRobotIndy7* const pRobot = g_pRobot;
	if (pRobot != NULL)
		pRobot->StopTasks();
}

void Usage(const char* apcProg)
{
	fprintf(stderr, "usage: %s [-f FILE] [-s | -b [-B FILE]] [-t DIR] [--method M1|M2|A2|A3|FULL] [--arch P0|S1|S2] [--shadow]\n"
	        "  -f FILE  configuration (default: ./INDY7.cfg, then <program dir>/../INDY7.cfg)\n"
	        "  -s       simulation: MuJoCo in place of the bus\n"
	        "  -b       the bus of the robot, drives off, beside the simulated arm (host-timing test)\n"
	        "  -B FILE  with -b: the slaves of the bus from FILE (e.g. bench/L7N3_bus.cfg)\n"
	        "  -t DIR   timing of every period into DIR\n"
	        "  --method NAME (-m)  what the camera application does with the results of the camera (obs/ObsMethod.h);\n"
	        "           default M1, the goal slot as before\n"
	        "  --arch NAME (-a)  where the EtherCAT task and the control task run (Arch.h): P0 both here (default),\n"
	        "                    S1 the EtherCAT task on the R5, S2 the control task on the R5 as well\n"
	        "  --shadow          with --arch S1: the R5 computes the control of every period as well (its torque is\n"
	        "                    compared, not used), to tell whether the control computation fits its period;\n"
	        "                    with --arch S2: a controller here repeats every period of the R5 from its report\n", apcProg);
}

} // namespace

int main(int argc, char** argv)
{
	// the terminal mode, restored at the end (the keyboard task changes it)
	struct termios stTermios;
	const bool bTermios = (0 == tcgetattr(STDIN_FILENO, &stTermios));

	TSTRING strCfg, strTimingDir, strBusCfg;
	BOOL bSim = FALSE, bHybrid = FALSE;
	int nMethod = obs_method::eM1;
	int nArch = arch::eP0;
	BOOL bShadow = FALSE;
	static const struct option s_astLong[] = { {"method", required_argument, NULL, 'm'},
	                                           {"arch", required_argument, NULL, 'a'},
	                                           {"shadow", no_argument, NULL, 'S'}, {NULL, 0, NULL, 0} };
	int nOpt;
	while ((nOpt = getopt_long(argc, argv, "f:sbB:t:m:a:h", s_astLong, NULL)) != -1)
	{
		switch (nOpt)
		{
			case 'f': strCfg = optarg; break;
			case 's': bSim = TRUE; break;
			case 'b': bHybrid = TRUE; bSim = TRUE; break;   // the arm is simulated as with -s
			case 'B': strBusCfg = optarg; break;
			case 't': strTimingDir = optarg; break;
			case 'm':
				nMethod = obs_method::Parse(optarg);
				if (nMethod < 0)
				{
					fprintf(stderr, "--method %s: no such method\n", optarg);
					Usage(argv[0]);
					return 1;
				}
				break;
			case 'S': bShadow = TRUE; break;
			case 'a':
				nArch = arch::Parse(optarg);
				if (nArch < 0)
				{
					fprintf(stderr, "--arch %s: no such architecture\n", optarg);
					Usage(argv[0]);
					return 1;
				}
				break;
			default:
				Usage(argv[0]);
				return 1;
		}
	}

	if (bShadow && nArch == arch::eP0)
	{
		fprintf(stderr, "--shadow: with --arch S1 (the second controller is on the R5) or S2 (it is here)\n");
		Usage(argv[0]);
		return 1;
	}
	if (nArch == arch::eS2 && bShadow && obs_method::UsesMailbox(nMethod))
	{
		fprintf(stderr, "--arch S2 --shadow: with --method M1 only (the goals of the receiver of the R5 are not in its "
		                "report, from which the controller here repeats its periods)\n");
		return 1;
	}

	strCfg = CConfig::Find(strCfg, argv[0]);
	CConfig cConfig;
	if (FALSE == cConfig.Load(strCfg, bSim))
	{
		Usage(argv[0]);
		return 1;
	}
	// -B FILE: another bus beside the simulated arm, so only with -b
	if (!strBusCfg.empty())
	{
		if (!bHybrid)
		{
			DBG_LOG_ERROR("-B %s: only with -b", strBusCfg.c_str());
			Usage(argv[0]);
			return 1;
		}
		if (FALSE == cConfig.LoadBus(strBusCfg))
		{
			Usage(argv[0]);
			return 1;
		}
	}
	DBG_LOG_INFO("configuration %s: %s%s%s; method %s; architecture %s", cConfig.Path().c_str(),
	             bHybrid ? "-b, the bus (drives off) beside the simulated arm" :
	             cConfig.System().bSim ? "simulation, MuJoCo in place of the bus" : "the robot",
	             cConfig.BusPath().empty() ? "" : ", the bus from ", cConfig.BusPath().c_str(),
	             obs_method::Name(nMethod), arch::Name(nArch));

	// SIGINT and SIGTERM stop the tasks (the real-time tasks block both)
	struct sigaction stSa;
	memset(&stSa, 0, sizeof(stSa));
	stSa.sa_handler = SignalHandler;
	sigemptyset(&stSa.sa_mask);
	sigaction(SIGINT, &stSa, NULL);
	sigaction(SIGTERM, &stSa, NULL);

	// every page locked, before any task is created: without the lock the
	// real-time tasks can take page faults
	if (0 != mlockall(MCL_CURRENT | MCL_FUTURE))
	{
		DBG_LOG_ERROR("mlockall failed: %s (the memlock limit must be unlimited, see run.sh)", strerror(errno));
		return 1;
	}
	// the writer of the console output of the real-time tasks (base/RtConsole.h)
	rt_console::Start();

	CRobotIndy7* pRobot = new CRobotIndy7(&cConfig);
	pRobot->SetHybridBus(bHybrid);
	pRobot->SetMethod(nMethod);
	pRobot->SetArch(nArch);
	pRobot->SetShadow(bShadow);
	if (!strTimingDir.empty())
		pRobot->SetTimingLogDir(strTimingDir);
	g_pRobot = pRobot;

	int nRet = 0;
	if (FALSE == pRobot->Init())
	{
		DBG_LOG_ERROR("the controller cannot start");
		pRobot->StopTasks();
		nRet = 1;
	}
	else if (FALSE == pRobot->CheckSyncMode())
	{
		pRobot->StopTasks();
		nRet = 1;
	}

	while (!pRobot->CheckStopTask())
		usleep(1000);
	// after a stop the EtherCAT task switches the drives off for 1 s while
	// the exchange goes on, releases the master, and ends; the Trajectory
	// and the Dynamics task end with it, and the summaries are printed
	if (pRobot->AreTasksStarted())
	{
		for (int nWait = 0; nWait < 5000 && FALSE == pRobot->IsControlTaskDone(); nWait++)
			usleep(1000);
		if (FALSE == pRobot->IsControlTaskDone())
			DBG_LOG_WARN("the real-time tasks have not ended within 5 s");
	}
	rt_console::Stop();             // the last texts of the tasks, before the summaries
	pRobot->StopServices();
	DBG_LOG_INFO("the controller has ended");

	// the keyboard task may still wait for a key and hold the lock of stdin,
	// which exit() would wait for: restore the terminal, flush, and end the
	// process without the teardown of stdio
	if (bTermios)
		tcsetattr(STDIN_FILENO, TCSANOW, &stTermios);
	fflush(stdout);
	fflush(stderr);
	_exit(nRet);
}
