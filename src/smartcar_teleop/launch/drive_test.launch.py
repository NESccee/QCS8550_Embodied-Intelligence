"""跑一段预设轨迹，用来验证底盘话题通不通（仿真和实车都能用）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = {
    'pattern': ('square', str, '轨迹：straight / square / circle / spin / stop'),
    'cmd_vel_topic': ('/cmd_vel', str, '底盘速度话题'),
    'linear': ('0.3', float, '直线速度 [m/s]'),
    'angular': ('0.8', float, '转向角速度 [rad/s]'),
    'leg_time': ('2.0', float, '每个直线段时长 [s]'),
    'turn_time': ('0.0', float, '转弯时长 [s]，0 表示按转 90 度自动计算'),
    'publish_rate': ('20.0', float, '发布频率 [Hz]'),
    'repeat': ('false', bool, '轨迹跑完后是否循环'),
    'use_sim_time': (
        'false', bool,
        '用仿真时钟计时：接 Gazebo 时置 true，各段时长按仿真秒算，不受实时率 RTF 影响'),
}


def generate_launch_description():
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default, description=description)
            for name, (default, _, description) in ARGUMENTS.items()
        ],
        Node(
            package='smartcar_teleop', executable='drive_test', name='drive_test',
            output='screen',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, (_, value_type, _) in ARGUMENTS.items()
            }],
        ),
    ])
