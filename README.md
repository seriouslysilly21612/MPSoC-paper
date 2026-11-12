# MPSoC-paper

The software of the Indy7 camera application on a Kria KV260 (Zynq UltraScale+ MPSoC): the robot control, the
vision pipeline and the RPU, with their calibration and analysis tools.

| Directory | Content | Runs on |
|---|---|---|
| `goal-to-torque_MPSoC/` | the real-time controller of the Neuromeka Indy7: three 1 kHz tasks on the isolated CPU 3 (EtherCAT, trajectory, computed torque), the MuJoCo simulation, the ROS 2 bridge to the camera pipeline; `calib/` the hand-eye calibration, `tools/` the checks and the analysis of the logs. Its `README.md`, section "This copy: the Kria KV260", has the build, the host configuration and the runs. | APU (Cortex-A53), PREEMPT_RT |
| `ros2_ws/` | the camera pipeline (ROS 2 Humble): RealSense D435i, YOLOv3-tiny on the DPU, the choice of the target, its 3-D position in `base_link` (`/pick_target_base`); the detector training, the desktop viewer, the documents and measurements of the development (`docs/`, `evidence/`) | APU + DPU (PL) |
| `rpu_work/` | the FreeRTOS firmware of the R5F-0 and the device tree it boots with (`user-override.dtb` = `base.dtb` + `rpu_v2.dtbo`, the first test: GEM3 still in Linux); `gem3_off.dtso` the overlay that leaves GEM3 to the R5; `r5_freertos/` the EtherCAT master on GEM3 (SOEM): its source, build settings, binary and the device tree it boots with | RPU |

Start:

    # the camera pipeline
    cd ros2_ws && colcon build && source install/setup.bash && launch_script/launch.sh
    # the controller, simulated arm (the pipeline running: keys r, g, h, then p and the digit of an object)
    cd goal-to-torque_MPSoC/indy7 && make && ./run.sh -s
    # the RPU: user-override.dtb into /boot/firmware/, r5_freertos.elf into /lib/firmware/, then
    echo r5_freertos.elf | sudo tee /sys/class/remoteproc/remoteproc0/firmware
    echo start | sudo tee /sys/class/remoteproc/remoteproc0/state

Not part of this repository: the humanoid computations of the paper, the kernel packages and board backups of
`ros2_ws/recovery/`, and the RealSense driver sources (the camera node comes from the ROS 2 packages).
