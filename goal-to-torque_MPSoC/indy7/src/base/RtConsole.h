/*****************************************************************************
*	Name: RtConsole.h
*	Description: The console output of the real-time tasks without the lock
*	of stdio (KV260 port).
*
*	printf, puts, fputs ... take the lock of stdout or stderr. It has no
*	priority inheritance and is shared with the threads on the other cores.
*	On the KV260 the camera pipeline keeps those three cores busy, and a
*	thread that was preempted while it held the lock stopped the Dynamics
*	task for 2 ms and the EtherCAT task for 1 ms (an overrun, two periods
*	without a fresh torque) at the start of TRACK.
*
*	The link of indy7_ctrl wraps the stdio calls of the controller
*	(Makefile, WRAP_STDIO). In a thread marked with MarkRealTime() the text
*	is formatted on the stack and goes into a lock-free queue (no lock, no
*	allocation, no system call); a thread on the other cores writes it out
*	within a few milliseconds. In every other thread the call goes to glibc
*	unchanged. A full queue drops the text and counts it.
*****************************************************************************/
#ifndef G2T_RT_CONSOLE_H
#define G2T_RT_CONSOLE_H

namespace rt_console
{
	/* the calling thread is a real-time task (its first statement) */
	void MarkRealTime();
	bool IsRealTime();
	/* main(): the writer thread, before the tasks; Stop() writes out what is
	 * left after the tasks have ended and reports the dropped texts */
	void Start();
	void Stop();
}

#endif // G2T_RT_CONSOLE_H
