/*
 * CallTiming.h of the firmware  --  the application times single library calls with it when asked to
 * (G2T_CALL_TIMING).  Here it is off for good: the sites of the controller classes compile to nothing and read
 * no clock.  The names are those of the application's CallTiming.h.
 */
#ifndef G2T_CALL_TIMING_H
#define G2T_CALL_TIMING_H

#include "base/Common.h"

namespace call_timing
{
enum eSlot { eIkKin = 0, eIkFk, eIkJac, eIkSolve, eIkIter, eOtg, eOtgAt, eCtcKin, eCtcCrba, eCtcNle, eControl, eN };
inline RTTIME Now() { return 0; }
inline RTTIME Lap(eSlot, RTTIME) { return 0; }
} // namespace call_timing

#endif /* G2T_CALL_TIMING_H */
