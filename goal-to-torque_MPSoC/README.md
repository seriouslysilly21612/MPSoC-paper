# Goal-to-Torque

Reference software for torque-level robot control from asynchronous goals: a camera, a planner or an
operator gives Cartesian goals at its own rate, and a real-time controller on PREEMPT_RT Linux turns them
into joint torques that it sends to the drives over EtherCAT every millisecond.

This is the source code of the article

> J. H. Yoon, K. W. Kim, J. E. Kim, and R. Delgado, "From High-Level Goals to Joint Torques: A Reference
> Software Architecture for Real-Time Robot Control Using Open-Source Components," submitted to IEEE Access.

It contains the controller of the Neuromeka Indy7 used in the experiments of the paper, its simulation in
MuJoCo, the ROS 2 camera pipeline of the camera application, the computations reused on a humanoid model,
and the tools that produce the tables and figures of the paper from the logs of the controller.

## This copy: the Kria KV260 (MPSoC)

This copy replaces the Indy7 controller of `~/RAON-RT-Revision` on a Kria KV260 (4 x Cortex-A53, Ubuntu
22.04, PREEMPT_RT 5.15) that also runs the camera pipeline of `~/ros2_ws` (ROS 2 Humble, the DPU). The
`ros2/` directory of the paper is not part of it: the bridge talks to that pipeline. What differs from
the host of the paper:

| | the paper (NUC) | the KV260 |
|---|---|---|
| real-time core | CPU 6 | CPU 3 (`isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2`), `[TASKn] CPU=3`; IRQs, camera pipeline and the other threads on CPU 0-2 |
| ROS 2, messages | Jazzy, `ros2/` | Humble, `~/ros2_ws/install/my_interfaces`; `run.sh` sources both |
| MuJoCo, Ruckig | `/opt/mujoco`, `/usr/local` | `~/opt/mujoco` (3.14.0 aarch64), `~/opt/ruckig` (0.19.4), no root needed |
| RBDL, IgH | 3.3.1, 1.6.9 with `igc` | 3.1.3, 1.6.10 with the generic driver (`model_check`: RBDL = MuJoCo to 1e-10) |
| console of the real-time tasks | stdio | a lock-free queue (`indy7/src/base/RtConsole.h`): the stdio lock stopped the tasks for 1-2 ms under the camera load |
| camera application | box, floor and hover of the paper | those of the earlier controller: floor 0.25 m, z1 0.50 m, hover 0.20 m; no "any class" entry; `desired_class` cleared at the start and the end |
| build | `-O2` | also `-mfix-cortex-a53-835769` (the A53 r0p4 of the board) for the objects of the controller; the libraries are built without it, as for the earlier controller |

The host configuration of Table 3, item by item for this board: `isolcpus=domain,managed_irq,3`, `nohlt` for
`idle=poll`, `kernel.timer_migration=0` (with 1 the Dynamics task was now and then woken before the EtherCAT task: a
period without a fresh torque), NTP stopped (kept on here: Claude Code needs the clock), rtprio 99; the clock is fixed
already (no cpufreq driver, 1333 MHz measured on CPU 3). `bash tools/host/kv260_rt_setup.sh` checks it,
`sudo bash tools/host/kv260_rt_setup.sh --apply` applies it (then reboot).

Build and checks (all passed on the board, 2026-11-02):

    cd indy7 && make && make tools        # [indy7] MUJOCO=1 ROS=1 MALLOC_COUNT=0
    ./bin/slot_check; ./bin/goal_filter_check; ./bin/hl_check models/indy7.urdf; ./bin/console_check
    ./bin/model_check models/indy7.urdf models/indy7_mujoco.xml B=6.07917,6.68933,1.99649,0.87306,0.86394,0.84747
    # B as in INDY7.cfg; until 2026-11-13 the controller bounded B at 2.0 (J1, J2), and this check used 2.0, 2.0

The analysis tools need the `mujoco` Python package; it is in `~/opt/g2t_venv`, apart from the numpy of ROS 2:
`~/opt/g2t_venv/bin/python tools/jt_run_sim.py ...`. On the board: `MALLOC_COUNT=1` 0 allocations, Fig. 4
(`jt_run_sim.py`, five identical runs, ratios 0.90-0.99), `hl_run_sim.py` 10 + 10 goals within 0.05 mm, 60 s of
`-t` timing with every criterion of `timing_summary.py` met, and with the camera pipeline running 3 min of
camera tracking (187,000 periods) without a late release, an overrun or a period without a fresh torque. Under
that load one `-s` send exceeded `SEND_DEADLINE` by 12 us: in `-s` the send is the MuJoCo step, so the deadline
of the bus is to be judged with `-b`.

**Simulation, seen on the desktop.** Board: `cd indy7 && ./run.sh -s`, keys `r`, `g`, `h`, then `y`/`u`/`p`.
The arm is integrated on CPU 3; the desktop only shows it (TCP 7430, 50 Hz). The desktop needs the folder
`~/g2t_view` of the board (scene, meshes, `sim_view.py`; `bash tools/make_view_bundle.sh ~/g2t_view
indy7/models/meshes`, the visual meshes of the Indy7 from `indy_description`) and the `mujoco` package in a venv:

    scp -r ubuntu@192.168.120.50:g2t_view . && cd g2t_view
    python3 -m venv ~/g2t_venv && ~/g2t_venv/bin/pip install mujoco numpy pillow
    ~/g2t_venv/bin/python sim_view.py live indy7_scene.xml --host 192.168.120.50

**Camera application in the simulation.** Start the pipeline first (`ros2 launch system_bringup_pkg
pick_place_vitis_ai.launch.py`), then `./run.sh -s`, keys `r`, `g`, `h`, `p` and the digit of an object: the
simulated arm goes to `q_r` and follows the real object 0.20 m above it. Objects beyond 0.92 m from the base
axis are refused, as by the earlier controller.

**Before the real robot** (none of this is needed for the simulation): `[AXISn]` (encoder zeros), `KF_n` and
`HL_READY_Q` are the latest values measured on this robot and replace those of the earlier controller. The
camera calibration of `~/ros2_ws` (`base_to_camera_tf`, with the yaw +5.14 deg re-base of E21) was made with the
earlier encoder zeros, whose J1 zero differs by -5.35 deg (J4 +4.25 deg, J5/J6 one turn): check it, or redo it
with `calib/`, before the arm follows camera goals. A `-b` run (bus in OP, drives off) comes first; the earlier
controller (`Indy7Ctrl.out`) must not run at the same time (`run.sh` refuses).

## The architecture in brief (paper, Sec. II, Fig. 1)

The controller is one process with two periodic tasks on one isolated core, released on a common 1-ms
grid (`indy7/src/Indy7Ctrl.h`):

| Task | Priority | In every period |
|---|---|---|
| EtherCAT (`TaskEtherCAT.cpp`) | 97 | receive the frame, publish the state (State slot), send the torque of the last period (Torque slot) |
| Control (`TaskTrajectory.cpp`, then `TaskDynamics.cpp`) | 96 | the trajectory part: take the newest goal (Goal slot), goal admission, bounded IK, jerk-limited reference (Ruckig), publish it (Reference slot); then the dynamics part: computed-torque law (RBDL) on the reference and the state, publish the torque (Torque slot) |

The paper (Table 3) has the two parts as two tasks, Trajectory (96) and Dynamics (95), which ran in the same order
on one core; since 2026-11-14 they are one task (the three tasks: commit `4abac4b`).

The tasks exchange data only through single-writer atomic slots (`indy7/src/base/AtomicSlot.h`, eq. (1));
every payload carries the period in which it was produced. The torque computed in period k is sent in
period k + 1. The ROS 2 bridge (`ROS2PickBridge.h`) writes camera goals into the Goal slot outside the
periodic path. With `-s` a MuJoCo model of the arm takes the place of the bus; with `-b` the bus of the robot
runs in OP with its drives kept off while the simulated arm is controlled (the host-timing test).

**The boundary to the R5 (2026-11-17; nothing uses it yet).** When the EtherCAT task moves to the R5, the two
processors exchange what `TaskSlots.h` has today, through a shared region: `indy7/src/ipc/ipc_layout.h` (C, one
file for both sides) has a head block and three slots, State (R5 to APU, 328 B), Torque (APU to R5, 72 B) and
Command (APU to R5, 8 B). `ipc_slot.h` is the slot: the sequence-number slot of `base/AtomicSlot.h` with 32-bit
words, one attempt per read, barriers, and an epoch per side.
- A side that starts leaves its slots "being written" and counts its epoch up. So the region is never zeroed,
  and nothing written before a restart is read as new.
- A reader takes a slot only while the writer's half of the head has the magic and the layout number of its
  own build.
- A slot does not tell that its writer has died. The reader has to see that from a value that no longer changes.

`bin/ipc_slot_check`: the three payloads, the faults (garbage in the region, a writer that dies in a write,
restarts of one and of both sides, an old epoch, the wrap of the sequence number, another magic or layout), then
one writer with 10,000,000 snapshots and two readers. `--unsafe` shows that it sees mixed copies. It ran on
x86-64, and as aarch64 and 32-bit ARM builds under QEMU; twelve of thirteen deliberately broken versions of
`ipc_slot.h` fail it. The thirteenth, without the barriers, passes there: x86 and QEMU keep the order of the
stores anyway. On the board's Cortex-A53 the test passes, and without the hardware barrier it fails (11 mixed
copies accepted in 10,000,000 snapshots): it does see a missing barrier between A53 cores. Between the A53 and the
R5 it has not run yet.

`indy7/src/ipc/FwCfg.h` is the check value of the constants that the firmware is built with: the CRC-32 of
`[ECAT_MASTER]` and of the slaves of the bus, in a fixed order and fixed widths. `bin/fw_cfg_header [-f FILE]
[-B FILE] -o fw_cfg.h` writes these constants and the check value as a C header for the firmware. It reads the
configuration with the parser of the controller, so the two cannot read a value differently. The firmware is to
write the value into the State slot, and the controller is to send "servo on" only when it equals its own;
neither is wired yet. `bin/fw_cfg_header --check`: the CRC-32 against its check value and against a record packed
and summed by another program; every value, the number of slaves and their order change the check value, a name
does not. All 23 broken versions of `FwCfg.h` fail it. For `INDY7.cfg` and for the bench (`-B bench/L7N4_bus.cfg`)
a second reader of the files gives the same check values, and the headers compile for the R5 without a warning.

**The camera results (2026-11-17; the methods below use them).** `indy7/src/obs/obs_mailbox.h` (C) carries a result from
the bridge to the control task: the 64-byte descriptor of eq. (5) and the 4-state mailbox of Alg. 1, eight slots.
- A slot goes FREE, WRITING, READY (the writer), then READING, FREE (the reader). Every state has one side that may
  change it, so there is no CAS, and the writer never takes a READY slot back.
- The writer drops a result when no slot is FREE and does not write one past its deadline; both are counted in
  the head. The reader looks at each slot once and never waits.
- Each side restarts by putting back its own half-done states. A result of the writer's earlier epoch is refused.

`indy7/src/obs/obs_receiver.h` (C) is the control task's side (eq. (3), Alg. 1): in every period it takes what the
mailbox holds and gives the one result that the period may use.
- It keeps the joint angles of the last 256 periods. A result brings its capture time, and the angles at that time
  come from the two states around it. Without two such states the result is refused.
- It holds up to eight results, the newest capture first. A period uses the newest one that was published before
  its release, is before its deadline and has its error bound (eq. (7)) within the tolerance of the period. What is
  older goes, so the results used come in the order of their captures.
- Every result taken leaves with one of seven verdicts: accepted, superseded, expired, state_unrecoverable,
  seq_fail, error_exceeded, evicted.
- What the writer says is checked again: its deadline counts up to a cap only, and a result that cannot be
  (captured after its publication or after the release, a sigma that is no error) is refused.
- The policy `OBS_MAX_AGE` is the comparison without the contract: the deadline alone decides, and the angles are
  those of the period in which the result arrives.
- A result is the object seen from the tag on the tool, or the object in the base frame, which needs no state.
  `obs_object` gives the object in the base frame for both: z itself, or the tag at the angles of the result plus z.

`indy7/src/obs/ObsEstimate.h` is the object that the control task makes of the results it used: the moving
average of the bridge (`GoalFilter.h`), now on the receiving side and by capture time, with a velocity for an
object that moves (`beta`, 0 for one that stands still). It restarts when the operator chooses another object,
and after a gap of a second between two captures. A period without a result in use keeps the object it has; the
target is lost after a second without a result.

`bin/obs_mailbox_check`: the result byte by byte, the faults (garbage in the region, nine results before the
reader looks, a result past its deadline, a writer dead in WRITING, a reader dead in READING, old epochs, broken
check words, another magic or layout), then one writer with 10,000,000 results against one reader (3 s on
x86-64). `--unsafe` frees a slot before copying it and fails. It ran on x86-64, and as aarch64 and 32-bit ARM
builds under QEMU; fourteen of seventeen broken versions of `obs_mailbox.h` fail it. Of the three that pass, two
leave out the WRITING or the READING mark, which changes nothing when no slot is taken back, and one leaves out
the barriers. On the board's Cortex-A53 the test passes (2.6 s; 98.7 % of the results written, each read once),
and without the hardware barrier it failed in each of four runs (1 or 2 mixed copies).

`bin/obs_receiver_check`: the angles at a capture time against their definition (times that jitter, periods
without a state, the wrap of the period number); the verdicts case by case under both policies, with a tag that
moves 24 mm while a result is on its way (the object is right with the angles at the capture, 24 mm off with
those of the arrival); then 200,000 periods of random results under each policy, in which the receiver must agree
in every period with a second one written from Alg. 1 and with eq. (3) itself, and every result must leave with
exactly one verdict; then one writer thread with 1,000,000 results against the receiver (1 s on x86-64 in all).
It ran on x86-64, and as aarch64 and 32-bit ARM builds under QEMU, with the same counts on all three; 73 of 74
broken versions of `obs_receiver.h` fail it. The one that passes leaves out a check that a later check repeats.

`bin/obs_estimate_check`: with `beta` = 0 the estimate is the moving average of the bridge bit for bit, with its
restarts; the first result of another target is the estimate (it does not slide over); an object that moves at
0.1 m/s is followed to 0.1 mm with `beta` > 0 and is more than 12 mm behind without it. All 22 broken versions of
`ObsEstimate.h` fail it.

**The methods of the camera application (2026-11-18).** `--method M1|M2|A2|A3|FULL` (default M1) selects how a
camera result becomes the goal of the arm; the section `[CONTRACT]` of the cfg holds the constants of the others.

| Method | Path of a result | The control task uses it |
|---|---|---|
| M1 | the bridge checks and averages the object and writes the goal slot (as before) | as it comes |
| M2 | every result goes into the mailbox as it is | until `AGE_CAP_MS` after its capture, with the joint angles of its arrival |
| A2 | the same | by the contract: the joint angles of its capture, and its error bound (eq. (7)) within `DELTA_MM` |
| A3 | as M2 | as M2 |
| FULL | as A2 | as A2 |

The camera pipeline is started with the same method (`method:=` of `pick_place_vitis_ai.launch.py`), and its
detector decides before an inference what to do with an image:

| Method | The detector |
|---|---|
| M1 | its fixed 30 ms gate (as before) |
| M2, A2 | every image it can take; the mode by its score, the measurement error plus the way of the tool at a reference speed during the latency |
| A3, FULL | the admission rule (eq. (8)): run a mode, skip the image, or reject it, from the status of the control task |

- The camera side is `indy7/src/obs/ObsWriter.h`: the capture stamp of the camera, moved from the system clock to
  the clock of the tasks, the deadline (capture plus `AGE_CAP_MS`) and the error of the mode (`EPS_M_MM`). A result
  older than the cap, or from the future, is not written. The bridge sends the object in the base frame until the
  camera pipeline sends it relative to the tool tag.
- The control task's side is `indy7/src/obs/ObsGoal.h`, once per period before the high-level controller: the
  joint angles of the period into the record, the result that the period may use (`obs_receiver.h`), its object in
  the base frame (with a result relative to the tag: the tag at the joint angles of the result, by the model of the
  high-level controller and `TAG_OFFSET_M`), the workspace checks that the bridge does for M1, the estimate
  (`ObsEstimate.h`), and the goal slot that the high-level controller reads as before.
- The joint angles that a period receives are `SIM_STATE_DELAY_STEPS` periods old at its release, and go into the
  record with that time. In the simulation this is exact; on the robot it is the same model of the drives, whose
  acquisition age has not been measured on its own.
- A run of the application writes `obs_<method>_..._rNN` beside `hl_app_..._rNN`: a row for every result that left
  the receiver, with its verdict, its times, its error bound and, if it was used, its object and the workspace
  check; the counts of both sides are in the `.json`. Every high-level log has the method in its `.json`.
- The camera of the simulation (`indy7/src/sim/SimCamera.h`; cfg `SIM_CAM=1`, refused outside `-s`) stands in for
  the bridge, so that every method runs without the camera pipeline: a thread that sees an object at rest and the
  tool tag `SIM_CAM_HZ` times a second, at the joint angles of the newest state and with their time as the capture
  time, and has its result `SIM_CAM_LATENCY_MS` later, with noise. M1 gets the object in the base frame, with an
  error of the calibration (`SIM_CAM_BIAS_MM`), through the checks and the average of the bridge; the others get
  it relative to the tag or in the base frame (`SIM_CAM_FORMAT`). `y`, `8` starts the application on it.

- The status (`indy7/src/obs/obs_status.h`): with a method other than M1 the control task writes in every period
  what the camera side needs for its decision, into a sequence-number slot: the tolerance of the period, the
  result it used last with its deadline, its mode and its error bound, whether this period had a result to use,
  the lost target, its own response (release to torque in the slot, the largest of the last one to two seconds),
  its period, its method, and the check value of the contract (`obs/ObsContract.h`: the CRC-32 of the nine
  constants of `[CONTRACT]` that both sides compute with). The bridge publishes it every 20 ms as
  `/contract_status` (`my_interfaces/msg/ContractStatus.msg`), its times on the clock of the image stamps.
- The admission rule (`ros2_ws/src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/`): `contract.py` reads
  `[CONTRACT]` from the cfg of the controller and has eq. (7) and the check value; `admission.py` is the rule; and
  `frame_gate.py` puts it at the place of the detector's gate. For every mode that can run: the result reaches the
  control task before its deadline (capture plus `AGE_CAP_MS`), its error bound at the age of its first use is
  within the tolerance, and the control task keeps its deadline while the inference runs (a table of measured
  responses; without one this check is off). Skipping is a candidate while the result in use stays good until the
  next image could replace it. The cheapest candidate wins; with equal cost no inference runs. There is no
  decision without a status to trust: none, one older than 100 ms, one of other contract constants, or one of
  another method. The control task checks every result again when it arrives.
- `ros2_ws/src/system_bringup_pkg/config/mode_profile.yaml` holds the numbers of the camera side. They are first
  values from one run, not a characterization: the latency of a mode to the mailbox is set to 180 ms (416) and
  170 ms (320), and the energy and the DDR traffic of a mode to 0. With them and `AGE_CAP_MS=250` an image passes
  the time check only when the detector has it within 69 ms of its capture, and skipping never passes. One model is
  loaded, so one mode can run: the choice between 416 and 320 is a tested function and waits for the second runner.

`bin/obs_status_check`: no copy of the status mixes two periods and no read goes back (2,000,000 writes against
a reader in another thread); a control task that starts again is not read before its first write; the largest of
the newest values; the check value against zlib; and eq. (7) against five values that the test of the camera side
has too. All 59 broken versions of `admission.py` and `contract.py` fail their tests.

`bin/obs_goal_check`: with results in the base frame the goal slot holds, bit for bit, what the bridge of M1
writes for the same 600 samples (rejections and restarts included); with results relative to a tag that moves at
0.2 m/s and angles that are two periods old, the object is right by the contract, 23.7 mm off with the angles of
the arrival, and 0.4 mm off when the age of the angles is ignored; another target, the entries of a period, the
lost target, settings that are refused, the status of a period, and the camera side on the clock of the machine. All 91 broken versions
of `ObsGoal.h` and `ObsWriter.h` fail it.

## Repository layout

| Directory | Content |
|---|---|
| `indy7/` | the controller: `src/` (tasks, control law, high-level controller, EtherCAT layer, simulation), `INDY7.cfg` (all parameters, with the values of the experiments), `models/` (URDF, MuJoCo model), `bench/` (the bus of the L7N bench for `-b -B`), `Makefile`, `run.sh` |
| `tools/` | offline checks, analysis of the logs (the tables and figures of the paper), identification, test runners, the viewer |
| `tools/host/` | host setup and measurements: cyclictest (Table 4), the operating point of the CPU, the priority of the IgH master thread, the bus inventory |
| `calib/` | the hand-eye calibration of the camera (eq. (10)) |
| `ros2/` | the camera pipeline (ROS 2 Jazzy): RealSense, YOLOv5n with OpenVINO, target selection, 3-D target in the base frame (`ros2/README.md`) |
| `humanoid/` | the computations of the controller on a fixed-base Unitree G1 in MuJoCo (Sec. VI) |
| `third_party/rt-posix/` | RT-POSIX, the real-time task library, unmodified but for the aarch64 stack-size clamp of `~/RT-POSIX` (KV260) |

## Dependencies (paper, Table 1)

| Component | Version of the experiments | Used by |
|---|---|---|
| Linux with PREEMPT_RT | 6.12.66-rt15 (Ubuntu Server 24.04) | the controller (any Linux runs the simulation) |
| IgH EtherCAT master | 1.6.9, native `igc` driver | the controller (`IGH_DIR`, default `/opt/etherlab`) |
| RBDL with its URDF reader | 3.3.1 | the controller, the humanoid (`RBDL_DIR`, default `/usr/local`) |
| Ruckig | 0.19.4 | the controller, the humanoid (`RUCKIG_DIR`) |
| Eigen 3 | of the distribution | (`EIGEN_DIR`, default `/usr/include/eigen3`) |
| MuJoCo (C library) | 3.14.0 | `-s`, `-b`, the humanoid, `model_check` (`MUJOCO_DIR`, default `/opt/mujoco`) |
| ROS 2 | Jazzy | the bridge and the camera pipeline (optional) |
| Python 3 | with numpy; mujoco, matplotlib for some tools | the tools |

## Build

    cd indy7
    make                       # bin/indy7_ctrl; prints [indy7] MUJOCO=.. ROS=.. MALLOC_COUNT=..
    make tools                 # bin/hl_check, bin/goal_filter_check, bin/slot_check, bin/ipc_slot_check,
                               # bin/obs_mailbox_check, bin/obs_receiver_check, bin/obs_estimate_check,
                               # bin/obs_goal_check, bin/obs_status_check, bin/fw_cfg_header,
                               # bin/model_check (MuJoCo)

`make MUJOCO=0` builds without the simulation, `make ROS=0` without the bridge (the default is ROS=1 when ROS 2
and the messages of `ros2/` are found: build `ros2/` first and source both environments, `ros2/README.md`).
`make MALLOC_COUNT=1` (after `make clean`) is the test build that counts the heap allocations of the real-time
tasks, which must stay at zero. The humanoid: `cd humanoid && make`.

## Host configuration (paper, Sec. III-A, Table 3)

The experiments ran on an Intel NUC 13 Pro (Core i7-1360P). Its configuration:

- kernel: PREEMPT_RT, high-resolution timers, `HZ_1000`, `NO_HZ_FULL`, `RCU_NOCB_CPU`; no CPU-frequency or
  idle-state driver;
- boot: `isolcpus=domain,managed_irq,6-7 nohz_full=6-7 rcu_nocbs=6-7 irqaffinity=0-5,8-15 idle=poll`; CPU 7 (the
  sibling of the real-time core 6) offline: `echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online`;
- CPU 6 at its largest non-turbo frequency, turbo off: `tools/host/cpu-noturbo.sh` (and its systemd unit);
- runtime: `sudo sysctl kernel.timer_migration=0`; NTP stopped (`sudo timedatectl set-ntp false`); Wi-Fi off;
- limits of the user: rtprio 99 and memlock unlimited (`indy7/run.sh` checks them);
- the thread of the IgH master at SCHED_FIFO 40: `sudo bash tools/host/install_ecat_op_fifo.sh` once, then
  `indy7/run.sh` lifts it at every start.

`[RT_TASKS]` of `INDY7.cfg` places the two periodic tasks on CPU 6; the controller reads back the policy, the priority
and the affinity of every task at its start and refuses to run otherwise.

## Running

    cd indy7
    ./run.sh -s                # simulation: MuJoCo in place of the bus
    ./run.sh                   # the robot (the bus must be up: ethercat.service)
    ./run.sh -b -t DIR         # the bus of the robot in OP, drives off, beside the simulated arm; timing into DIR
    ./run.sh -b -B bench/L7N4_bus.cfg -t DIR   # the same with another bus: the four L7N drives of the bench
    ./run.sh --method A2       # the camera application by another method (M1, M2, A2, A3, FULL; see above)
    ./run.sh -s --arch S1      # the EtherCAT task on the R5 (below); with -s the R5 is simulated as well

The keys of the operator are printed at the start and with `?`:

| Key | Action |
|---|---|
| `r` | controller on/off (it starts in gravity compensation; off also clears a latched fault) |
| `g` | gravity compensation |
| `h` / `j` | servo on (brakes released, gravity compensation holds) / servo off (brakes) |
| `x` / `e` | stop the motion and hold / e-stop: stop, servo off, brakes |
| `k` | the hold of the gravity compensation on/off |
| `m` | joint move: absolute joint angles in degrees (refused beyond the URDF limits minus the margin) |
| `t` | task move: tool position x y z in mm (refused when the IK cannot reach it) |
| `p` | camera application: the objects in view; a digit makes the arm follow one (needs the bridge) |
| `u` | joint tests and identification: 1 fast PD+g, 2 fast CTC, 3 slow CTC, 4 slow CTC+FF, 5 torque step, 6 breakaway, 7 sine |
| `y` | high-level tests: 1 random goals, 2 goal jumps, 3 goal noise with delta, 4 with delta = 0, 5 reference check, 6 HOME, 7 record q_r, 8 camera application |
| `s` / `w` | print the tool pose / arm the pose capture of the hand-eye calibration (then `s` writes the pose) |
| `q` | quit (the drives are switched off before the master is released) |

On the robot, start with `r`, `g`, `h` and keep the emergency stop at hand. The constants of `[AXIS0]`..`[AXIS5]`
(encoder offsets `HOME_POSITION_OFFSET`, directions `MOVE_CCW`) belong to the robot of the experiments, and the
drive scales of Table 5 were not measured independently: check them on another robot with the arm braked
before any motion. The software responses to faults (a latched controller fault, a drive dropout) are
supervisory behavior, not certified protective functions.

`SYNC_MODE` of `[ECAT_MASTER]` selects how the drives are synchronized: `FREE_RUN` (the experiments of the
paper; nothing is written to the drives), `SM` (synchronous with the frame) or `DC` (distributed clocks,
SYNC0 `DC_SYNC0_SHIFT` after the release). With `SM` or `DC` the controller checks that every drive supports
the mode and runs in it once the bus is in OP, and ends with a warning otherwise.

Logs: the joint and high-level tests write a CSV and a JSON per run into `JOINT_TEST_LOG_DIR` (default
`indy7/jt_logs`); `-t DIR` writes the timing of every period into `DIR/timing_<stamp>.bin`.

**`--arch`: where the EtherCAT task runs.** `P0` (default) is everything above: the EtherCAT task here, with the IgH
master. With `S1` the R5 of the board runs the EtherCAT exchange and the drives (`../rpu_work/r5_freertos`), and
the application keeps the control task: TASK0 takes the State slot of the R5 in place of the receive, the control
task writes its Torque slot right after the control law, and what the application decides about the drives goes
to the R5 as a command (`indy7/src/r5link/R5Link.h`, the head of `TaskEtherCAT.cpp`). The period of the R5 is the
period of the loop: the release grid of the two tasks follows it, `WAKE_DELAY_US` of `[R5_LINK]` behind
(`r5link/R5Follow.h`). `--arch` and `--method` are independent. `-s --arch S1` simulates the R5 as well, a thread
on another CPU that owns the simulated arm and keeps the firmware's rules for the slots (`sim/SimR5.h`): it does
not wait for the tasks, so a host that is not real-time shows in the results (a State that the tasks did not take
moves the arm a period ahead of the reference; such runs are marked `TIMING FAULTS`). On the board, the device tree,
the firmware and what to look at: `../rpu_work/README_S1.md`. Not run with the real R5 yet (2026-11-18).

## The paper and the code

| Paper | Code | Reproduction |
|---|---|---|
| Fig. 1, Sec. II | `indy7/src/Indy7Ctrl.h`, `TaskEtherCAT.cpp`, `TaskTrajectory.cpp`, `TaskDynamics.cpp`, `TaskSlots.h` | |
| eq. (1), Sec. II-A | `indy7/src/base/AtomicSlot.h` | `indy7/bin/slot_check` (1 writer, 3 readers, 150,000 snapshots; `--unsafe` shows that the test sees torn copies) |
| Table 3 | `indy7/src/base/RtTask.h`, `INDY7.cfg` `[RT_TASKS]`, `indy7/run.sh` | `tools/host/` |
| Table 4 | | `tools/host/run_cyclictest.sh`, then `tools/host/cyclictest_summary.py` |
| eq. (2), Table 5 | `indy7/src/ecat/Indy7Joint.cpp`, `INDY7.cfg` `[AXISn]` | `tools/host/bus_inventory.sh` (PDO mapping, sync objects) |
| eq. (3), Fig. 2 | `indy7/src/TaskEtherCAT.cpp`, `SEND_DEADLINE` | |
| Table 6, Fig. 3 | `indy7/src/TimingLog.h` (`-t`) | `tools/run_timing.py --condition idle|stress|perception|pressure`, `tools/timing_summary.py`; all conditions unattended: `tools/run_batch.py` |
| eqs. (4), (5), Table 7 | `indy7/src/FullDynControllerRT.cpp` | `indy7/bin/model_check` (RBDL vs MuJoCo), `tools/phase_margin.py` (PM rows) |
| eq. (6), Table 7 (B) | JOINT TEST 7 | `tools/sine_cfg.py`, `tools/identify_rotor.py` |
| eq. (7), Table 7 (tau_s, K_f) | JOINT TESTs 6 and 4 | `tools/jt_breakaway.py`, `tools/kf_step.py` |
| eq. (8), Table 8 | | `tools/jt_predict.py`, `tools/phase_margin.py` |
| eq. (9), Table 9, Sec. IV-C | `indy7/src/HighLevelRT.h`, `TrajectoryRT.h` | `indy7/bin/hl_check` (the reference contract C1-C5); delta: `tools/vision_probe.py`, `tools/goal_noise_file.py` |
| Table 10, Figs. 4 and 5 | JOINT TESTs 3 and 4 | `tools/jt_run_sim.py` (simulation), `tools/jt_analyze.py` |
| Table 11, Figs. 6-8 | high-level tests 1 and 2 | `tools/hl_run_sim.py` (simulation), `tools/hl_analyze.py --static-err ...`, `tools/sim_view.py` |
| eq. (10), Figs. 9 and 10 | `calib/`, `ros2/`, `indy7/src/ROS2PickBridge.h`, `GoalFilter.h` | `indy7/bin/goal_filter_check`, `tools/app_analyze.py`, `tools/place_check.py`, `tools/snap_view.py` |
| Table 12, Fig. 11 | `humanoid/src/HumanoidSim.cpp` | `humanoid/bin/humanoid_sim`, `humanoid/tools/hum_analyze.py` |

### Checks without the robot

    cd indy7 && make && make tools
    ./bin/slot_check                                      # PASS: 0 inconsistent snapshots accepted
    ./bin/ipc_slot_check                                  # the slots to the R5, PASS
    ./bin/fw_cfg_header --check                           # the check value of the firmware's constants, PASS
    ./bin/obs_mailbox_check                               # the mailbox of the camera results, PASS
    ./bin/obs_receiver_check                              # the control task's side of them, PASS
    ./bin/obs_estimate_check                              # the object estimate from them, PASS
    ./bin/obs_goal_check                                  # the two ends of the mailbox in the application, PASS
    ./bin/obs_status_check                                # the status for the camera side, the contract check value, PASS
    ./bin/r5_link_check                                   # --arch S1: the exchange with the R5 and following its period, PASS
    ./bin/hl_check models/indy7.urdf                      # the reference contract, PASS
    ./bin/goal_filter_check                               # the goal checks of the bridge, PASS
    ./bin/model_check models/indy7.urdf models/indy7_mujoco.xml B=6.07917,6.68933,1.99649,0.87306,0.86394,0.84747  # B as in INDY7.cfg
    python3 ../tools/phase_margin.py                      # Tables 7 (PM) and 8
    make clean && make MALLOC_COUNT=1 && ./run.sh -s      # 'r', 'g', 'h', 'u' 3: no allocation ([MALLOC] at the end)

The two periodic tasks run on CPU 6 (`CPU` of `[TASK0]` and `[TASK1]`), and `run.sh` needs the limits of the host
configuration. On a host whose real-time core is another one, give the tasks that core in a copy of the cfg in
`indy7/` (its relative paths stay valid; the commands from here on run in the top directory):

    cp indy7/INDY7.cfg indy7/sim.cfg
    python3 tools/cfgset.py indy7/sim.cfg TASK0.CPU=2 TASK1.CPU=2

and pass the copy to the tools with `--cfg indy7/sim.cfg` (by hand, in `indy7/`: `./run.sh -s -f sim.cfg`).

### Joint tracking (Sec. V-A)

On the robot: `r`, `g`, `h`, then `u` `4` five times (CTC + FF) and, for the comparison, the same with all
`KF_n` set to 0 (`python3 tools/cfgset.py indy7/INDY7.cfg KF_0=0 ... KF_5=0`, restart). Then

    python3 tools/jt_analyze.py indy7/jt_logs            # Table 10, e_bar for Table 11

The simulation of Fig. 4 runs without friction and rotor inertia, with J6 held, on a copy of the cfg (on
another host with the `TASKn.CPU` values above as well):

    cp indy7/INDY7.cfg indy7/fig4.cfg
    python3 tools/cfgset.py indy7/fig4.cfg ROTOR_INERTIA_0=0 ROTOR_INERTIA_1=0 ROTOR_INERTIA_2=0 ROTOR_INERTIA_3=0 \
        ROTOR_INERTIA_4=0 ROTOR_INERTIA_5=0 "JOINT_TEST_SLOW_DQ=0.9 0.6 0.9 0.9 0.9 0" JOINT_TEST_LOG_DIR=jt_fig4
    python3 tools/jt_run_sim.py --cfg indy7/fig4.cfg --tests slow-ctc -n 5 --analyze    # PASS: within 1.5 x eq. (8)

The simulator steps in the EtherCAT task, once per period, so the simulated loop has the timing of the host:
the values of the paper, and five identical runs, need a host configured as in Table 3. The summary at the end
of a run counts the late releases, overruns, and missed send deadlines; with any of them, the simulated loop
has seen the delay.

### Task-space goals (Sec. V-B)

    python3 tools/hl_run_sim.py --tests hl-fixed,hl-jump --analyze        # simulation, the goals of the robot (seed 1)
    python3 tools/hl_analyze.py indy7/jt_logs --static-err "2.75 0.83 1.52 7.16 6.42 0.66"   # Table 11 with Est.

On the robot: `y` `1` runs the ten goals of `HL_RAND_*` once (five rounds for Table 11), `y` `2` the goal jumps.

### Host timing (Sec. III-C)

With the bus of the robot (drives in OP, power stages off), 10 min per condition:

    python3 tools/run_timing.py --condition idle
    python3 tools/run_timing.py --condition stress
    python3 tools/run_timing.py --condition perception       # the camera pipeline (ros2/README.md) starts and stops with the run

Each run writes the row of Table 6 into `timing_summary.txt` and `timing_table.csv` of its result directory, and
the PS temperatures and the board power, every second, into `temp_power.csv`.

The MPSoC measurements run every condition unattended: idle, stress, perception, and the memory pressure
(perception + `stress-ng --memrate` at about 25, 50, 75, 100 % of its unbounded rate beside the camera
pipeline), 30 min x 3, about 11.8 h (`--quick`: about 1.4 h). On the L7N bench (`indy7/bench/L7N4_bus.cfg`), with the
IgH master running and NTP off (`sudo timedatectl set-ntp false`; it refuses to start otherwise), inside tmux:

    python3 tools/run_batch.py --out ~/timing_logs/p0_bench_batch_<date>    # --dry-run lists the runs; --from N goes on from run N

`batch.csv` has a line per run (its exit code, the memory rate reached, the highest PS temperature) and
`timing_tables.csv` the row of Table 6 of every run. The runs do not log the high-level test (`HL_LOG=0`).

With the EtherCAT task on the R5 (`../rpu_work/README_S1.md`): `--arch S1` for `run_timing.py` and `run_batch.py`;
the send columns of the row then come from the record of the R5, taken during the run (`OUT/r5`).
No IgH master then: the R5 runs the bus and must have it in OP, and the device tree must have the node of its
slots. The reference check ends with the arm at rest at the accepted goal (it used to end in motion, into gravity
compensation); the time to rest is outside the TRACK window of the timing log.

### Camera application (Sec. V-C)

Calibration (in `calib/`, the controller running with `r`, `g`, `h` and the capture armed with `w`):
`calib_grab.py` (images; `s` in the controller writes the matching pose), `calib_cpo.py --tag-mm <side>`
(tag poses), `eye_to_hand_calib.py` (rMc; move ambiguous or outlying pairs to `data/excluded/` and run it
again), `calib_to_tf.py --live --write` (the transform of the launch file). Then start the pipeline
(`ros2/README.md`), the controller with its bridge, and choose the object with `p`. `tools/app_analyze.py`
evaluates the runs (`hl_app_*` logs).

The methods (`--method`) in the simulation, on the camera of the simulation instead of the pipeline:

    cp indy7/INDY7.cfg indy7/sim_app.cfg
    python3 tools/cfgset.py indy7/sim_app.cfg SIM_CAM=1 APP_DURATION_S=6
    for m in M1 M2 A2; do python3 tools/hl_run_sim.py --tests hl-app --cfg indy7/sim_app.cfg --method $m; done
    python3 tools/app_methods.py --cfg indy7/sim_app.cfg --check 0.5     # one line per run; PASS for the contract

The admission rule and its two ends are tested without ROS, in the workspace of the camera pipeline (sourced) and
here:

    cd ~/ros2_ws && python3 -m pytest src/vitis_ai_detector_pkg/test/test_contract.py \
        src/vitis_ai_detector_pkg/test/test_admission.py src/vitis_ai_detector_pkg/test/test_frame_gate.py
    python3 -m pytest tools/test_admission_log.py

With the camera pipeline, both sides get the method, and the detector writes its decisions when it stops:

    ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py method:=FULL decision_log:=~/decisions_full.csv
    ./run.sh -s --method FULL                              # keys r, g, h, p and the digit of an object; x, j, q
    python3 tools/admission_log.py ~/decisions_full.csv indy7/jt_logs/obs_FULL_sim_r01_<stamp>.csv

`tools/admission_log.py` counts the images run, skipped and rejected, the rejections by their reason and the
check that failed, and with the log of the controller joins the images that were run to their results by the
capture time: how many reached the control task, their verdicts, and the period that took a result against the
first use that the rule predicted.

`tools/app_methods.py` puts the runs side by side: the results used, the goals that the high-level controller
accepted, the time of the first goal, the time at rest after the last one and, against the place of the object, the error of the objects, of the goal and of the
tool at the end. On a PC (x86-64, no real-time kernel; the object 0.55 m in front of the base, a result 120 ms
after its capture, no noise; two runs of each method): with M1 and with A2 the arm takes one goal, rests after
1.6 s and ends 0.01 mm from the hover point. With M2 the object is up to 61 mm off while the arm moves (1.05
times the way of the tool since the capture), the high-level controller takes 25 to 28 goals, and the arm rests
after 2.8 to 3.3 s, 0.4 to 5.5 mm from the hover point: a wrong goal taken during the motion stays when the right
one is within delta (6 mm) of it. With an error of the calibration of 5 mm and noise of 1 mm (one run each), M1
and A2 on results in the base frame end 5.7 mm off, A2 on results relative to the tag 1.0 mm. That PC is not a
real-time system: the control task misses the state of some periods, and A2 then refuses the result whose
capture falls there (state_unrecoverable, 1 of 181 in each run with the tag). `bin/obs_goal_check` also ran as
aarch64 and 32-bit ARM builds under QEMU.

On the KV260 (2026-11-18, one run of each method, `-s`): M1 and A2 take one goal and end 0.009 mm from the hover
point, M2 takes 25 goals with the object up to 61.6 mm off and ends 5.5 mm away; A2 accepts all 180 results
(no state_unrecoverable); no late release and no overrun. With the camera pipeline in place of the camera of the
simulation (YOLOv3-tiny 416, A2, 82 s): 904 of 909 results accepted. A result was 176 ms old when it was used
(median; 9 of 10 below 227 ms), and the camera side did not write 164 of 1,079 because they were older than
`AGE_CAP_MS` (250 ms). Those come in runs: 24 gaps of more than 0.25 s between two results, the longest 0.95 s.
The first object of that session was reached after 6.9 s without a result, because a node of the pipeline loaded
a library on its first valid target (`pick_target_3d_node`, since moved to its start); with a pipeline that had
had a target before, the first result came after 0.03 s.

### Humanoid (Sec. VI)

    cd humanoid && make
    ./bin/humanoid_sim --models models --mode check                              # RBDL vs MuJoCo, PASS
    ./bin/humanoid_sim --models models --mode joint --cpu 6 --out /tmp/hum_joint  # 6000 periods, no load
    python3 tools/hum_analyze.py /tmp/hum_joint models/g1_mujoco.xml             # Fig. 11 (e)
    ./bin/humanoid_sim --models models --mode hl --goals 5000 --duration 3600 --cpu 6 --out /tmp/hum_hl
                                                                                 # under stress-ng: Table 12

The summary of each run (`summary_<mode>.txt`) holds the columns of Table 12; `hl_goals.csv` the final
distances of Fig. 11 (f).

## Third-party content

- `third_party/rt-posix/`: RT-POSIX, unmodified except one clamp for aarch64 (KV260): a stack below
  `PTHREAD_STACK_MIN` (128 KiB there, above the default 64 KiB) is raised to it, as in `~/RT-POSIX`.
- `indy7/models/indy7.urdf`: from `neuromeka-robotics/indy-ros2` (variant indy7, commit ca0353f); the changes
  are described at its head. Its mesh paths refer to the ROS 2 package `indy_description` of that repository.
- `indy7/models/meshes/`: the visual meshes `Indy7_0.stl` ... `Indy7_6.stl` of that `indy_description`, for the viewer.
- `humanoid/models/`: derived from `unitreerobotics/unitree_ros` (`g1_29dof.urdf`), see `humanoid/models/README.md`.
- The YOLOv5n model of the camera pipeline is not included (`ros2/README.md`).
