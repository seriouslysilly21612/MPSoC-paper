"""The single bringup for the perception pipeline — and its desktop viewer.

Brings up the full pick pipeline:
    camera -> detector -> pick_logic -> pick_target_3d -> (static TF) -> pick_target_base

This is also the ONLY launch you need to watch detections as a bbox overlay on
the desktop. There is no separate "viewing" launch (2026-07-20: the old
viewing.launch.py was merged into this file), because watching does not need a
different bringup — it needs a subscriber:

  * The camera advertises /camera/camera/color/image_raw/compressed via
    image_transport (plugin: ros-humble-compressed-image-transport). That JPEG
    stream is encoded LAZILY — image_transport skips the encode while nothing
    subscribes to it, so it costs the board ZERO until a viewer attaches.
  * To watch: run detection_viewer_pkg/detection_viewer_node on the DESKTOP. It
    subscribes to that compressed topic + /detections and draws. The board only
    compresses; all drawing is on the desktop.
    Full procedure/gates: docs/vision/desktop_viewer_plan.md

Color stays at 30 fps here (the production pick path wants fresh frames — see
realsense_pick_place.yaml). The detector caps at 15 Hz, but the viewer draws
EVERY color frame (~30 fps) carrying the newest boxes, so none of the JPEG
encodes go to waste; pass --sync to the viewer for the old 15 Hz frame-exact
join instead. The board sends the same stream either way, and that cost is zero
the moment you close the viewer. Board-side drawing does not exist: it cost
~44 ms/frame on top of a ~37.6 ms detect and was removed (2026-09-03).

METRICS (off by default). Add `metrics:=true` to record everything in one shot:
    ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py metrics:=true
That injects the performance/yield CSV paths into all 4 nodes AND starts the
standalone resource_sampler (the sampler must be a separate process — it reads
/proc for all 7 pipeline processes incl. the VART worker subprocess, which no
ROS node can enumerate). Without the flag nothing is recorded and behaviour is
unchanged. pick_logic (v3) follows only the class the operator selects, so a
standalone run (no robot app) must name one or every target stays invalid:
    ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py \
        metrics:=true desired_class:=apple
Each run writes to its OWN dir evidence/metrics/runs/<timestamp>/ (so
history is never overwritten) and evidence/metrics/latest is repointed at it.
Args: metrics_dir (base), metrics_run (run label, '' = timestamp), metrics_duration
(s, auto-save then Ctrl-C; 0 = until shutdown), sampler_script.
See evidence/metrics/README.md
"""
import os
import re
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _point_latest_at(run_dir, metrics_dir):
    """Repoint <metrics_dir>/latest -> runs/<label> so tools can say
    `--base <metrics_dir>/latest` and always get the newest run.

    The link target is RELATIVE so the whole metrics tree survives being copied
    or scp'd elsewhere. Replacement is atomic (symlink to temp name + rename) so
    a crash mid-update can never leave `latest` dangling. Best-effort: a metrics
    run must never fail because of a convenience link, so all errors are
    swallowed after printing (launch has no logger available here).
    """
    latest = os.path.join(metrics_dir, 'latest')
    try:
        os.makedirs(run_dir, exist_ok=True)
        if os.path.isdir(latest) and not os.path.islink(latest):
            print(f'[metrics] NOTE: {latest} is a real directory, not a link -- '
                  f'leaving it alone. Newest run: {run_dir}')
            return
        tmp = latest + '.tmp'
        if os.path.lexists(tmp):
            os.remove(tmp)
        os.symlink(os.path.relpath(run_dir, metrics_dir), tmp)
        os.replace(tmp, latest)          # atomic swap over any existing link
        print(f'[metrics] this run -> {run_dir}  (also reachable as {latest})')
    except OSError as exc:
        print(f'[metrics] WARNING: could not update {latest}: {exc}')


def launch_setup(context, *args, **kwargs):
    # DDS transport (2026-07-15: moved here from ~/.bashrc so the repo is
    # clone-and-run). Every node below ships 848x480 Images (1.16 MB) to the
    # next one; without this profile FastDDS uses its default 512 KB SHM
    # segment, silently falls back to UDP loopback for anything bigger, and
    # the pipeline costs +6.6%p CPU with no error to tell you why.
    # See config/fastdds_shm_profile.xml for the segment sizing.
    dds_profile = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'fastdds_shm_profile.xml',
    ])

    realsense_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py',
    ])

    realsense_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'realsense_pick_place.yaml',
    ])

    vitis_ai_detector_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'vitis_ai_detector.yaml',
    ])

    # The model ships inside vitis_ai_detector_pkg, so resolve it at runtime
    # instead of hard-coding /home/<user>/... in the YAML (ROS parameter YAML
    # expands neither ~ nor environment variables). decode_meta.json sits beside
    # the xmodel in the same share/ dir — the worker finds it from there.
    model_path = PathJoinSubstitution([
        FindPackageShare('vitis_ai_detector_pkg'),
        'models',
        'yolov3_tiny_7class.xmodel',
    ])

    pick_logic_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'pick_logic.yaml',
    ])

    target_3d_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'target_3d.yaml',
    ])

    target_base_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'target_base.yaml',
    ])

    # ----- metrics wiring (off unless metrics:=true) -----
    metrics_on = LaunchConfiguration('metrics').perform(context).lower() in (
        'true', '1', 'yes', 'on'
    )
    metrics_dir = LaunchConfiguration('metrics_dir').perform(context)
    metrics_duration = LaunchConfiguration('metrics_duration').perform(context)

    # Every run gets its OWN directory under <metrics_dir>/runs/<label>, so a new
    # run never overwrites the previous one. The file NAMES stay fixed
    # (performance/detector.csv, ...) on purpose: join_perf.py and
    # plot_metrics*.py take a --base dir and expect exactly that layout, so
    # per-run dirs preserve history AND keep every tool working unchanged
    # (a timestamped FILENAME would have broken all of them).
    run_dir = metrics_dir
    if metrics_on:
        label = LaunchConfiguration('metrics_run').perform(context).strip()
        if not label:
            label = datetime.now().strftime('%Y%m%d_%H%M%S')
        # sanitize: a label reaches the filesystem, so no path traversal/spaces
        label = re.sub(r'[^A-Za-z0-9._-]', '_', label) or 'run'
        run_dir = os.path.join(metrics_dir, 'runs', label)
        _point_latest_at(run_dir, metrics_dir)

    def metrics_params(node_key):
        """Override params that turn a node's recording on, or [] when off.
        Appended AFTER the node's YAML so it overrides the '' defaults."""
        if not metrics_on:
            return []
        return [{
            'metrics_csv_path': os.path.join(
                run_dir, 'performance', f'{node_key}.csv'),
            'yield_csv_path': os.path.join(run_dir, 'yield', f'{node_key}.csv'),
            'metrics_duration_sec': float(metrics_duration),
        }]

    actions = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', dds_profile),

        # Isolate the realsense include: rs_launch.py iterates over
        # context.launch_configurations and prints a yellow "Parameter 'X' is
        # not supported" warning for every config it doesn't recognize. Without
        # this scope our metrics_* launch args leak in and trigger 4 spurious
        # warnings on EVERY launch. scoped=True + forwarding=False = parent
        # launch configs are NOT visible inside; config_file is still passed
        # explicitly, and the SetEnvironmentVariable above is process-level so
        # realsense still gets the DDS profile.
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(realsense_launch),
                    launch_arguments={
                        'config_file': realsense_config,
                    }.items(),
                ),
            ],
            scoped=True,
            forwarding=False,
        ),

        Node(
            package='vitis_ai_detector_pkg',
            executable='vitis_ai_detector_node',
            name='vitis_ai_detector_node',
            # model_path comes last so it overrides the YAML: it is a resolved
            # path, not a tunable, and must not be edited per machine.
            parameters=[vitis_ai_detector_config, {'model_path': model_path}]
            + metrics_params('detector'),
            output='screen',
        ),

        # NOTE(2026-07-14): merging these three nodes into one process
        # (target_3d_pkg pick_post_stack) was tried and MEASURED SLOWER
        # (+5.4pt: rclpy's executor rebuilds the wait-set over all entities
        # per callback, so a 3-node process raises every callback's dispatch
        # cost more than it saves in per-process/DDS overhead). Keep them
        # as separate processes unless moving to rclcpp composition.
        Node(
            package='pick_logic_pkg',
            executable='pick_logic',
            name='pick_logic_node',
            # desired_class is normally set LIVE by the robot app's operator
            # menu. The launch arg (default '') exists for standalone runs
            # (metrics without the robot app), where '' would leave every
            # target invalid. Last so it overrides the YAML.
            parameters=[pick_logic_config,
                        {'desired_class': LaunchConfiguration('desired_class')}]
            + metrics_params('pick_logic'),
            output='screen',
        ),

        Node(
            package='target_3d_pkg',
            executable='pick_target_3d_node',
            name='pick_target_3d_node',
            parameters=[target_3d_config] + metrics_params('target_3d'),
            output='screen',
        ),

        # Measured by eye-to-hand calibration (2026-07-27, 16 poses, AX=ZB,
        # residual 1.8 cm / 1.7 deg): rMc(base->color_optical) composed with
        # the live realsense camera_link->color_optical TF. Quaternion form
        # on purpose — the equivalent RPY sits near the pitch=90 deg gimbal
        # singularity. Raw data + solver: RAON-RT-Revision/App/CalibUtils/kv260/
        # 2026-07-27 E21 correction: the arm incidents shoved the ROBOT BASE
        # (planar rigid: yaw +5.14 deg, t (+4.5, +10.4) cm — hand-probe fit,
        # apple+tennis residual 0.2 mm, banana check 2.1 cm ~ its own shape
        # ambiguity). Composed here via tools/base_shift_fit.py (RAON repo);
        # the camera itself did not move.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.811666',
                '--y', '0.087684',
                '--z', '0.924974',
                '--qx', '0.72462829',
                '--qy', '0.00008362',
                '--qz', '-0.68706199',
                '--qw', '0.05347582',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        Node(
            package='target_3d_pkg',
            executable='pick_target_base_node',
            name='pick_target_base_node',
            parameters=[target_base_config] + metrics_params('base'),
            output='screen',
        ),
    ]

    if metrics_on:
        # The resource sampler is NOT a ROS node: only an external process can
        # read /proc for all 7 pipeline processes (incl. the VART worker
        # subprocess). launch starts it here so you never type its CLI args.
        # Ctrl-C sends it SIGINT and it writes the CSV; --duration self-stops.
        sampler_script = LaunchConfiguration('sampler_script').perform(context)
        actions.append(ExecuteProcess(
            cmd=[
                'python3', sampler_script,
                '--output', os.path.join(run_dir, 'resource', 'run.csv'),
                '--duration', metrics_duration,
                '--interval', '1.0',
            ],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'metrics', default_value='false',
            description='true = record performance+yield CSVs (all 4 nodes) and '
                        'start the resource sampler. Default false = no recording.'),
        DeclareLaunchArgument(
            'metrics_dir', default_value='/home/ubuntu/ros2_ws/evidence/metrics',
            description='absolute base dir; each run lands in runs/<label>/ under '
                        'it with performance/ resource/ yield/ subdirs'),
        DeclareLaunchArgument(
            'metrics_run', default_value='',
            description="label for this run's subdir under runs/ ('' = timestamp "
                        'YYYYmmdd_HHMMSS). Reusing a label overwrites that run.'),
        DeclareLaunchArgument(
            'metrics_duration', default_value='300.0',
            description='seconds collected from the first frame, then auto-save; '
                        '0 = collect until shutdown'),
        DeclareLaunchArgument(
            'desired_class', default_value='',
            description='class pick_logic follows (empty = none; the robot-app '
                        'operator menu sets it live). Name one for standalone '
                        'runs, e.g. desired_class:=apple'),
        DeclareLaunchArgument(
            'sampler_script',
            default_value='/home/ubuntu/ros2_ws/tools/metrics/resource_sampler.py',
            description='path to tools/metrics/resource_sampler.py'),
        OpaqueFunction(function=launch_setup),
    ])
