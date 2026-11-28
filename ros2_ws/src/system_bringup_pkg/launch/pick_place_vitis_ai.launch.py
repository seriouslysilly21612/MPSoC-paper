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

MODEL. `model_path:=/abs/path/model.xmodel` runs another xmodel instead of the
packaged YOLOv3-tiny one; its decode_meta.json must sit in the same directory
(the worker reads it from there). Empty (the default) = the packaged model.
`model_path_2:=` a second xmodel of the other input size (its own directory
with its decode_meta.json): the worker loads both, and the method (M2, A2, A3,
FULL) picks the mode of each image (D-23). M1 runs model_path only.

TOOL TAG. `tool_tag:=true` makes the detector look for the tool tag (AprilTag
36h11 tag_id:=0, black square tag_size_m:=0.065) in every image that it runs,
in a thread beside the inference (I-122), and pick_target_3d_node pair it with
the object of the same capture (waiting at most tag_wait_ms:=40 for it), so that
the control application can take the object from it (APP_RESULT_FORMAT=tag;
D-22). tag_position:=depth|pnp: its distance from the depth image (default) or
from its corners. tag_refine:=apriltag|subpix: the corners in the window around
the tag; by default subpix with depth (cheaper, the same centre pixel) and
apriltag with pnp. The whole image always gets the plain detector.
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _find_ros_ws():
    """The ros2_ws this launch file belongs to, from its own path.

    It runs from install/system_bringup_pkg/share/system_bringup_pkg/launch/ (or,
    with --symlink-install, may resolve to src/system_bringup_pkg/launch/): the
    workspace is the first directory above it that holds src/system_bringup_pkg.
    Else ~/ros2_ws, as before.
    """
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(d, 'src', 'system_bringup_pkg')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return os.path.expanduser('~/ros2_ws')


ROS_WS = _find_ros_ws()
REPO = os.path.dirname(ROS_WS)      # the MPSoC-paper checkout: goal-to-torque_MPSoC/ is beside ros2_ws/


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
    # The method of the run (paper, Table III): the same value as --method of
    # the control application. M1 needs none of the three files below.
    method = LaunchConfiguration('method').perform(context).strip()
    # check 3 reads the measured table that ships here unless told otherwise
    # (I-132 B2: an empty default left it off in every board run)
    response_table = LaunchConfiguration('response_table').perform(context).strip()
    if response_table.lower() == 'off':
        response_table = ''
    elif not response_table and method.upper() in ('A3', 'FULL'):
        response_table = os.path.join(FindPackageShare('system_bringup_pkg').perform(context),
                                      'config', 'response_v5n.csv')
    method_params = {
        'method': method,
        'contract_cfg_path': os.path.abspath(os.path.expanduser(
            LaunchConfiguration('contract_cfg').perform(context).strip())),
        'mode_profile_path': PathJoinSubstitution([
            FindPackageShare('system_bringup_pkg'), 'config', 'mode_profile.yaml']),
        'response_table_path': response_table,
        'decision_log_path': LaunchConfiguration('decision_log').perform(context).strip(),
        'dynamic_latency': LaunchConfiguration('dynamic_latency').perform(context).strip().lower()
        in ('true', '1', 'yes', 'on'),
    }
    for key in ('response_table_path', 'decision_log_path'):
        if method_params[key]:
            method_params[key] = os.path.abspath(os.path.expanduser(method_params[key]))

    def resolved_model(arg):
        """the xmodel of a launch argument, its decode_meta.json beside it; '' when empty"""
        given = LaunchConfiguration(arg).perform(context).strip()
        if not given:
            return ''
        # `model_path:=~/...` reaches us unexpanded (the shell expands ~ only
        # at the start of a word)
        path = os.path.abspath(os.path.expanduser(given))
        # fail before the camera starts: the worker would report a missing
        # file only after the whole pipeline is up
        for f in (path, os.path.join(os.path.dirname(path), 'decode_meta.json')):
            if not os.path.isfile(f):
                raise RuntimeError(f'{arg}: no such file {f}')
        return path

    model_override = resolved_model('model_path')
    if model_override:
        model_path = model_override
    model_path_2 = resolved_model('model_path_2')
    if model_path_2 and model_override and \
            os.path.dirname(model_path_2) == os.path.dirname(model_override):
        raise RuntimeError('model_path_2: in the directory of model_path; each xmodel needs '
                           'its own decode_meta.json')

    def flag(arg):
        value = LaunchConfiguration(arg).perform(context).strip().lower()
        return value in ('true', '1', 'yes', 'on')

    # the tag: searched by the detector, paired with the object by pick_target_3d_node (I-122)
    tag_position = LaunchConfiguration('tag_position').perform(context).strip()
    tag_refine = LaunchConfiguration('tag_refine').perform(context).strip() or (
        'subpix' if tag_position == 'depth' else 'apriltag')
    # the energy of the modes (tools/mode_energy.py): M1 with a long period leaves the pipeline up
    # without inference, the base of the difference
    detect_period = LaunchConfiguration('detect_period_s').perform(context).strip()
    detector_period_params = {'process_period_sec': float(detect_period)} if detect_period else {}
    detector_tag_params = {
        'tool_tag': flag('tool_tag'),
        'tag_id': int(LaunchConfiguration('tag_id').perform(context)),
        'tag_size_m': float(LaunchConfiguration('tag_size_m').perform(context)),
        'tag_refine': tag_refine,
        'tag_whole_scale': float(LaunchConfiguration('tag_whole_scale').perform(context)),
    }
    target_3d_tag_params = {
        'tool_tag': flag('tool_tag'),
        'tag_position': tag_position,
        'tag_wait_ms': float(LaunchConfiguration('tag_wait_ms').perform(context)),
    }

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
            # camera:=false (HIL, tools/sim/hil_camera.py): another machine publishes the
            # camera topics, camera_info, the depth->color extrinsics and the camera_link TFs
            condition=IfCondition(LaunchConfiguration('camera')),
        ),

        Node(
            package='vitis_ai_detector_pkg',
            executable='vitis_ai_detector_node',
            name='vitis_ai_detector_node',
            # model_path comes last so it overrides the YAML: it is a resolved
            # path, not a tunable, and must not be edited per machine.
            parameters=[vitis_ai_detector_config,
                        {'model_path': model_path, 'model_path_2': model_path_2}, method_params,
                        detector_tag_params, detector_period_params]
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
            parameters=[target_3d_config, target_3d_tag_params] + metrics_params('target_3d'),
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
            'metrics_dir', default_value=os.path.join(ROS_WS, 'evidence', 'metrics'),
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
            default_value=os.path.join(ROS_WS, 'tools', 'metrics', 'resource_sampler.py'),
            description='path to tools/metrics/resource_sampler.py'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='xmodel to run instead of the packaged YOLOv3-tiny '
                        '(empty = packaged); its decode_meta.json must be in '
                        'the same directory'),
        DeclareLaunchArgument(
            'model_path_2', default_value='',
            description='a second xmodel, of the other input size, in a directory of '
                        'its own with its decode_meta.json: the methods other than '
                        'M1 pick the mode of each image (empty = one mode)'),
        DeclareLaunchArgument(
            'tool_tag', default_value='false',
            description='true: the detector finds the tool tag in every image it '
                        'runs, pick_target_3d_node pairs it with the target of the '
                        'same capture (APP_RESULT_FORMAT=tag of the control '
                        'application)'),
        DeclareLaunchArgument(
            'tag_id', default_value='0',
            description='the id of the tool tag (AprilTag 36h11)'),
        DeclareLaunchArgument(
            'tag_size_m', default_value='0.065',
            description='the side of the black square of the tool tag [m]: '
                        'measure it (the calibration tag: 0.065)'),
        DeclareLaunchArgument(
            'tag_position', default_value='depth',
            description="depth: the tag's distance from the depth image, as for the "
                        'object; pnp: from its four corners'),
        DeclareLaunchArgument(
            'tag_wait_ms', default_value='40',
            description='how long pick_target_3d_node waits for the tag of a '
                        "target's capture [ms]; then the target goes without it"),
        DeclareLaunchArgument(
            'detect_period_s', default_value='',
            description='process_period_sec of the detector, the gate of M1 (empty: its YAML, '
                        '0.030; the rule of the other methods does not use it); M1 with 1000: the '
                        'pipeline without inference (tools/mode_energy.py)'),
        DeclareLaunchArgument(
            'tag_whole_scale', default_value='0.5',
            description='the whole image is searched for the tag at this size (0.25 to 1; '
                        '1: as before I-134), the corners then at full size in a window'),
        DeclareLaunchArgument(
            'tag_refine', default_value='',
            description='the tag corners in the window around it: apriltag (as the '
                        'calibration) or subpix (cheaper); empty: subpix with '
                        'tag_position:=depth, apriltag with pnp'),
        DeclareLaunchArgument(
            'method', default_value='M1',
            description='M1, M2, A2, A3 or FULL: what the detector does with an '
                        'image before the inference. Start the control '
                        'application with the same --method'),
        DeclareLaunchArgument(
            'contract_cfg',
            default_value=os.path.join(REPO, 'goal-to-torque_MPSoC', 'indy7', 'INDY7.cfg'),
            description='configuration of the control application: its section '
                        '[CONTRACT] holds the constants both sides compute with '
                        '(methods other than M1)'),
        DeclareLaunchArgument(
            'response_table', default_value='',
            description='A3, FULL: CSV mode,level,response_us of the measured '
                        'responses of the control task (empty = '
                        'config/response_v5n.csv of this package, off = that '
                        'check is off)'),
        DeclareLaunchArgument(
            'dynamic_latency', default_value='false',
            description='A3, FULL: true = L_m of every mode from the latencies of its results '
                        'in this run (config/mode_profile.yaml dynamic_latency); false = the '
                        'fixed L_m of the profile'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='true = the RealSense node; false = no camera here: the camera topics come '
                        'from another machine (tools/sim/hil_camera.py, hardware in the loop)'),
        DeclareLaunchArgument(
            'decision_log', default_value='',
            description='A3, FULL: CSV file for the decision on every image, '
                        'written when the detector stops (empty = not written)'),
        OpaqueFunction(function=launch_setup),
    ])
