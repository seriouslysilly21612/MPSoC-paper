// Pure logic shared by the window and test_logic — NO Qt, NO ROS includes.
//
// State mirrors /robot_state, the bridge's 20 Hz JSON snapshot of the exact
// flags CRobotIndy7::DoInput's refusal guards read. keyEnabled() must stay a
// REFLECTION of DoInput — if a guard changes there, change it here to match,
// never the other way round. The robot app still refuses on its own; gating
// only stops the operator pressing buttons that would be refused anyway.
#ifndef RAON_GUI_STATE_HPP
#define RAON_GUI_STATE_HPP

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace raon_gui
{

struct State
{
  std::vector<double> q;
  bool servo{false};   // every axis IsServoOn()
  bool ctrl{false};    // controller enabled ('r') && IsEnabled()
  bool grav{false};    // mode == eGravityCompensation
  bool track{false};   // tracking servo active
  bool busy{false};    // approach/ISO/RECT running
  bool home{false};    // user HOME recorded || ready-seed available
  bool calib{false};   // 's' writes the calibration dataset right now
  bool lock{false};    // bridge has a fresh target lock
  bool menu{false};    // 'p' menu open (digits are consumed only now)
  // Open menu's classes in MENU NUMBER ORDER: entry i selects with digit
  // '1'+i. The bridge builds this from the same m_vMenu that numbers the
  // printed menu, so position → digit cannot diverge from HandleDigit.
  std::vector<std::string> menu_items;
  // Controller eControlMode as int: 0 grav / 1 fulldyn / 2 CT / 3 adaptive /
  // 4 IK / 5 IK6dof / 6 tracking servo. -1 = not reported (old binary or no
  // RT controller) — the jog UI must stay dead then.
  int mode{-1};
  // Live controller gains, the admin tab's readback. EMPTY when the robot app
  // predates /operator_gains or runs without the RT controller — the tab must
  // treat empty as "don't send" (no readback = no proof the channel exists).
  std::vector<double> kp, kd, kf;
};

// Keys the console may offer. 'q' and 'z' are deliberately absent: the
// terminal's proc_keyboard_control eats them before DoInput, so DoInput's 'q'
// case is the unreachable-but-armed 10-minute ISO cube hardware run — a remote
// "quit" would start it. The bridge whitelist blocks them too (belt & braces).
inline bool keyEnabled(char k, const State & s)
{
  switch (k) {
    // always valid (and still valid while tracking — DoInput's whitelist)
    case 'e': case 'x': case 'j': case 'g': case 'p':
      return true;
    // swallowed by the tracking guard, otherwise unguarded
    case 'r': case 't': case 'f': case 'c': case 'i': case 'k':
    case 's': case 'l': case 'd': case 'w': case 'a': case 'm': case 'n':
      return !s.track;
    case 'h':   // servo-on interlock: controller enabled + grav-comp first
      return !s.track && s.ctrl && s.grav;
    case 'b':   // home return: controller + servo + a posture to return to
      return !s.track && s.ctrl && s.servo && s.home;
    case 'o':   // tracking: as a STOP always valid; as a START fully gated
      return s.track || (s.ctrl && s.servo && !s.busy && s.lock);
    case 'v':   // approach pops through the bridge gate, which needs a lock
      return !s.track && s.lock;
    default:
      if (k >= '0' && k <= '9') {return s.menu;}
      return false;
  }
}

// ---- /robot_state JSON ----
// The producer is the bridge's fixed snprintf (ROS2PickBridge.cpp) — nine
// known keys, no nesting, no strings. A ~40-line extractor beats a JSON
// library dependency for a schema we own on both ends.

inline bool jsonBool(const std::string & j, const char * key, bool def)
{
  const std::string pat = std::string("\"") + key + "\":";
  const size_t at = j.find(pat);
  if (at == std::string::npos) {return def;}
  return j.compare(at + pat.size(), 4, "true") == 0;
}

inline int jsonInt(const std::string & j, const char * key, int def)
{
  const std::string pat = std::string("\"") + key + "\":";
  const size_t at = j.find(pat);
  if (at == std::string::npos) {return def;}
  return static_cast<int>(std::strtol(j.c_str() + at + pat.size(), nullptr, 10));
}

inline bool jsonDoubles(
  const std::string & j, const char * key, std::vector<double> & out)
{
  out.clear();
  const std::string pat = std::string("\"") + key + "\":[";
  const size_t at = j.find(pat);
  if (at == std::string::npos) {return false;}
  const size_t ae = j.find(']', at);
  if (ae == std::string::npos) {return false;}
  const char * p = j.c_str() + at + pat.size();
  const char * end = j.c_str() + ae;
  while (p < end) {
    char * next = nullptr;
    const double v = std::strtod(p, &next);
    if (next == p) {break;}
    out.push_back(v);
    p = (*next == ',') ? next + 1 : next;
  }
  return !out.empty();
}

inline bool parseState(const std::string & j, State & out)
{
  if (!jsonDoubles(j, "q", out.q)) {return false;}
  out.servo = jsonBool(j, "servo", false);
  out.ctrl = jsonBool(j, "ctrl", false);
  out.grav = jsonBool(j, "grav", false);
  out.track = jsonBool(j, "track", false);
  out.busy = jsonBool(j, "busy", false);
  out.home = jsonBool(j, "home", false);
  out.calib = jsonBool(j, "calib", false);
  out.lock = jsonBool(j, "lock", false);
  out.menu = jsonBool(j, "menu", false);
  out.mode = jsonInt(j, "mode", -1);
  // optional (older robot binaries don't send them) — empty on absence
  jsonDoubles(j, "kp", out.kp);
  jsonDoubles(j, "kd", out.kd);
  jsonDoubles(j, "kf", out.kf);

  out.menu_items.clear();
  const size_t ma = j.find("\"menu_items\":[");
  if (ma != std::string::npos) {
    const size_t me = j.find(']', ma);
    size_t p = ma + 14;
    while (me != std::string::npos && p < me) {
      const size_t open = j.find('"', p);
      if (open == std::string::npos || open >= me) {break;}
      const size_t close = j.find('"', open + 1);
      if (close == std::string::npos || close > me) {break;}
      out.menu_items.push_back(j.substr(open + 1, close - open - 1));
      p = close + 1;
    }
  }
  return true;
}

}  // namespace raon_gui

#endif  // RAON_GUI_STATE_HPP
