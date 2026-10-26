"""启动 Ignition Fortress 仿真 + ROS/Gazebo 桥接。

默认只跑服务端（-s），因为本机没有可用显示；有显示时加 gui:=true 连界面一起起。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# Gazebo 话题 -> ROS 话题。方向符号：] = ROS->Gazebo，[ = Gazebo->ROS。
BRIDGE_ARGS = [
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    '/model/smart_car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    '/model/smart_car/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
    '/world/smart_car_world/model/smart_car/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
    '/model/smart_car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    '/model/smart_car/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
]

BRIDGE_REMAPPINGS = [
    ('/model/smart_car/cmd_vel', '/cmd_vel'),
    ('/model/smart_car/odometry', '/odom'),
    ('/world/smart_car_world/model/smart_car/joint_state', '/joint_states'),
    ('/model/smart_car/tf', '/tf'),
    ('/model/smart_car/imu', '/imu/data'),
]


def _gazebo(context):
    # 有显示时 gui:=true 会连 GUI 一起起；否则只跑服务端（-s）
    cmd = ['ign', 'gazebo']
    if LaunchConfiguration('gui').perform(context).lower() not in ('true', '1', 'yes'):
        cmd.append('-s')
    cmd += ['-r', LaunchConfiguration('world').perform(context)]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    package_share = get_package_share_directory('smartcar_sim')
    default_world = os.path.join(package_share, 'worlds', 'smart_car_world.sdf')
    models_dir = os.path.join(package_share, 'models')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='要加载的 SDF 世界文件'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='是否连 Gazebo GUI 一起启动（需要可用的 DISPLAY）'),
        # model://smart_car 靠这个环境变量解析，缺了就加载不出小车
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            [models_dir, ':', EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='')]),
        OpaqueFunction(function=_gazebo),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
            arguments=BRIDGE_ARGS,
            remappings=BRIDGE_REMAPPINGS,
            output='screen'),
    ])
