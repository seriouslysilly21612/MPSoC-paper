// Gating + JSON parse — the pure logic in state.hpp, no Qt, no ROS.
// Run:  ./build/raon_operator_gui/test_logic
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>

#include "../src/state.hpp"

using raon_gui::State;
using raon_gui::keyEnabled;
using raon_gui::parseState;

static State full()
{
  State s;
  s.q = {0, 0, 0, 0, 0, 0};
  s.servo = s.ctrl = s.grav = s.home = s.lock = s.menu = true;
  return s;
}

int main()
{
  const char * all_keys = "rhjgfcitkbpvoxensmadlw0123456789";

  // fully permissive → every offered key enabled; 'q'/'z' never offered
  for (const char * k = all_keys; *k; ++k) {
    assert(keyEnabled(*k, full()));
  }
  assert(!keyEnabled('q', full()) && !keyEnabled('z', full()));

  // tracking → exactly DoInput's whitelist survives
  State tr = full();
  tr.track = true;
  const char * track_ok = "ojexgp0123456789";
  for (const char * k = all_keys; *k; ++k) {
    assert(keyEnabled(*k, tr) == (std::strchr(track_ok, *k) != nullptr));
  }

  // individual guards
  State s = full(); s.grav = false; assert(!keyEnabled('h', s));
  s = full(); s.ctrl = false; assert(!keyEnabled('h', s) && !keyEnabled('b', s));
  s = full(); s.servo = false; assert(!keyEnabled('b', s) && !keyEnabled('o', s));
  s = full(); s.home = false; assert(!keyEnabled('b', s));
  s = full(); s.lock = false; assert(!keyEnabled('v', s) && !keyEnabled('o', s));
  s = full(); s.busy = true; assert(!keyEnabled('o', s));
  s = full(); s.menu = false; assert(!keyEnabled('5', s));
  s = full(); s.track = true; s.lock = false;
  assert(keyEnabled('o', s));   // 'o' while tracking = stop, needs nothing

  // JSON — the bridge's exact snprintf shape
  const std::string j =
    "{\"q\":[0.0000,-1.4908,0.0000,0.0000,0.0000,0.0000],"
    "\"servo\":true,\"ctrl\":true,\"grav\":true,\"track\":false,"
    "\"busy\":false,\"home\":true,\"calib\":false,\"lock\":false,\"menu\":true,"
    "\"menu_items\":[\"apple\",\"tennis_ball\"]}";
  State p;
  assert(parseState(j, p));
  assert(p.q.size() == 6 && p.q[1] < -1.49 && p.q[1] > -1.50);
  assert(p.servo && p.ctrl && p.grav && !p.track && !p.busy && p.home &&
    !p.calib && !p.lock && p.menu);
  assert((p.menu_items == std::vector<std::string>{"apple", "tennis_ball"}));
  // menu closed → empty array; old producer without the key → also empty
  State p2;
  assert(parseState(
      "{\"q\":[0.1],\"menu\":false,\"menu_items\":[]}", p2));
  assert(p2.menu_items.empty());
  State p3;
  assert(parseState("{\"q\":[0.1],\"menu\":true}", p3));
  assert(p3.menu_items.empty());
  // gains readback (admin tab): all three arrays parsed when present…
  State pg;
  assert(parseState(
      "{\"q\":[0.1],\"servo\":true,\"mode\":2,"
      "\"kp\":[800.0,600.0,400.0,200.0,150.0,100.0],"
      "\"kd\":[80.0,60.0,40.0,20.0,15.0,10.0],"
      "\"kf\":[8.00,6.00,4.00,2.00,1.50,1.00],\"menu\":false}", pg));
  assert(pg.kp.size() == 6 && pg.kp[0] == 800.0 && pg.kd[1] == 60.0 &&
    pg.kf[5] == 1.0);
  assert(pg.mode == 2);
  // …and EMPTY/-1 from an old binary that doesn't send them (jog + gains
  // UIs must stay dead)
  State png;
  assert(parseState("{\"q\":[0.1],\"menu\":false}", png));
  assert(png.kp.empty() && png.kd.empty() && png.kf.empty());
  assert(png.mode == -1);

  // garbage / truncation must fail, not crash
  assert(!parseState("", p));
  assert(!parseState("{\"servo\":true}", p));
  assert(!parseState("{\"q\":[", p));

  printf("PASS: gating mirrors DoInput (full/track/individual); JSON round-trip "
    "ok incl. kp/kd/kf present+absent\n");
  return 0;
}
