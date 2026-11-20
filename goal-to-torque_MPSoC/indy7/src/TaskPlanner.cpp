/*****************************************************************************
*	Name: TaskPlanner.cpp
*	Description: The planner task: the IK of a goal and the plans of the
*	high-level controller (HighLevelRT.h), beside the 1 ms tasks. It has the
*	CPU and the release grid of the control task and the priority below it
*	(Indy7Ctrl.cpp, InitRTTasks), so within a period it runs when the
*	EtherCAT task and the control task have ended, and they interrupt it.
*	What the trajectory part asked for in a period is calculated in that
*	period when it fits, and goes on in the next when it does not
*	(HighLevelLink.h). Its counts are in the summary of the tasks.
*****************************************************************************/
#include "Indy7Ctrl.h"

#include "base/RtConsole.h"

void
proc_planner_task(void* apRobot)
{
	rt_console::MarkRealTime();
	auto* pRobot = static_cast<CRobotIndy7*>(apRobot);
	CRtTask* const pTask = CRtTask::Self();
	if (pTask == NULL)
		return;
	while (!pRobot->CheckStopTask())
	{
		pTask->WaitNextRelease();
		while (pRobot->m_cTraj.ServePlanner())
			;
	}
	DBG_LOG_INFO("(proc_planner_task) task ended");
}
