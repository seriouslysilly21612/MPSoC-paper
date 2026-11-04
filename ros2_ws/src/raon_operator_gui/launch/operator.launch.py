#!/usr/bin/env python3
"""Desktop operator station — ONE window (the C++ console with embedded RViz).

  ros2 launch raon_operator_gui operator.launch.py
  ros2 launch raon_operator_gui operator.launch.py urdf:=/path/to/real_mesh.urdf

Three processes, one window:
  - operator_console (Qt + embedded RViz — the only thing you see)
  - robot_state_publisher: /joint_states (from the robot app's bridge) → TF.
    Without it the embedded RobotModel has no TF and renders NOTHING.
  - static TF world→base_link: the robot URDF roots at 'world', the perception
    pipeline publishes under 'base_link' — same physical frame, two names.

Default URDF is indy7_mesh.urdf — real Indy7 shells from the vendored STLs
(meshes/indy7/visual/, neuromeka-robotics/indy-ros2). indy7_viz.urdf (mesh-free
cylinders) remains as `urdf:=` fallback if the meshes ever go missing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    urdf = LaunchConfiguration('urdf').perform(context)
    with open(urdf, 'r') as f:
        robot_description = f.read()
    console_args = ['--image-topic',
                    LaunchConfiguration('image_topic').perform(context)]
    if LaunchConfiguration('admin').perform(context).lower() == 'true':
        console_args.append('--admin')
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        ),
        Node(
            package='raon_operator_gui',
            executable='operator_console',
            output='screen',
            arguments=console_args,
            # The console IS the station: closing its window (X or the
            # 콘솔 종료 button) must also stop rsp + static TF, or Ctrl-C in
            # the terminal stays mandatory — which is what this replaces.
            on_exit=Shutdown(),
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('raon_operator_gui')
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf',
            default_value=os.path.join(share, 'urdf', 'indy7_mesh.urdf'),
            description='URDF for the RViz RobotModel (kinematics must stay '
                        'identical to the robot app\'s indy7.urdf). '
                        'indy7_viz.urdf = mesh-free fallback.'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw/compressed',
            description='Compressed color stream from the board.'),
        DeclareLaunchArgument(
            'admin', default_value='false',
            description='true = show the Kp/Kd/KF tab (--admin).'),
        OpaqueFunction(function=_setup),
    ])
