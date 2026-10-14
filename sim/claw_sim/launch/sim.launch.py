"""Start Gazebo with the arm and claw, the ROS bridge, and the teleop node.

  ros2 launch claw_sim sim.launch.py              # Gazebo GUI
  ros2 launch claw_sim sim.launch.py gui:=false   # headless server only
  ros2 launch claw_sim sim.launch.py gpu:=false   # leave the GL driver alone
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from claw_sim.urdf_chain import revolute_limits


def generate_launch_description():
    share = get_package_share_directory('claw_sim')
    world = os.path.join(share, 'worlds', 'pick.sdf')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'claw_arm.urdf.xacro')).toxml()
    description = {'robot_description': urdf}

    gui = LaunchConfiguration('gui')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression(
                ["'-r ' + ('' if '", gui, "'.lower() == 'true' else '-s ') + '", world, "'"]),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Under WSL2, Mesa falls back to software rendering (llvmpipe) unless told to
    # use the D3D12 driver, which is what reaches the real GPU through /dev/dxg.
    gpu_env = []
    if os.path.exists('/dev/dxg') and 'GALLIUM_DRIVER' not in os.environ:
        gpu_env.append(SetEnvironmentVariable(
            'GALLIUM_DRIVER', 'd3d12', condition=IfCondition(LaunchConfiguration('gpu'))))

    # One command topic per joint, named as in urdf/common.xacro.
    bridges = [f'/claw/{joint}/cmd@std_msgs/msg/Float64]gz.msgs.Double'
               for joint in revolute_limits(urdf)]
    bridges.append('/claw/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('gpu', default_value='true',
                              description='Render on the GPU when running under WSL2'),
        DeclareLaunchArgument('port', default_value='8765',
                              description='TCP port the hand tracker connects to'),
        *gpu_env,
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], remappings=[('joint_states', '/claw/joint_states')]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'pick', '-topic', 'robot_description', '-name', 'claw_arm'],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridges,
             output='screen'),
        Node(package='claw_sim', executable='teleop', output='screen',
             parameters=[description, {'port': LaunchConfiguration('port')}]),
    ])
