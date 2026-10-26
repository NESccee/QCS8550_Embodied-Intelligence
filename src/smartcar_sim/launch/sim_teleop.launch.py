"""仿真 + 手柄遥控一起启动：ign gazebo + 桥接 + joy_node + joy_teleop。

默认用实测好的挡位按键：LT（button 8）使能，RT（button 9）高速档。
默认无界面，有显示时加 gui:=true。
支持透传全部仿真与遥控参数（如 world:=... max_linear:=0.3 等）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# teleop 参数 (name: (default, description))
# 在仿真中默认启用手柄挡位：使能键 8 (LT)，高速键 9 (RT)
TELEOP_ARGUMENTS = {
    'joy_device_id': ('0', 'joy_node 的 device_id'),
    'joy_device_name': ('', '按设备名选择手柄，留空表示使用 device_id'),
    'joy_deadzone': ('0.05', 'joy_node 的摇杆死区'),
    'joy_autorepeat_rate': ('20.0', 'joy_node 静止时的重发频率 [Hz]，0 表示只在变化时发'),
    'joy_topic': ('/joy', '手柄话题 sensor_msgs/msg/Joy'),
    'cmd_vel_topic': ('/cmd_vel', '底盘速度话题 geometry_msgs/msg/Twist'),
    'axis_linear': ('1', '前进/后退轴索引（本手柄左摇杆纵向为 1）'),
    'axis_angular': ('0', '转向轴索引（本手柄左摇杆横向为 0）'),
    'invert_linear': ('false', '反转速度轴，前进后退相反时置 true'),
    'invert_angular': ('false', '反转转向轴，左右转向相反时置 true'),
    'deadzone': ('0.05', '遥控节点的摇杆死区'),
    'button_deadman': ('8', '必须按住的使能键索引，-1 表示不检查'),
    'button_turbo': ('9', '高速档按键索引，-1 表示关闭'),
    'turbo_scale': ('1.5', '高速档倍率'),
    'max_linear': ('0.6', '最大线速度 [m/s]'),
    'max_angular': ('1.5', '最大角速度 [rad/s]'),
    'publish_rate': ('20.0', '/cmd_vel 发布频率 [Hz]'),
    'joy_timeout': ('0.5', '超过该时间没收到 Joy 就发零速度 [s]'),
}


def generate_launch_description():
    pkg_share_sim = get_package_share_directory('smartcar_sim')
    pkg_share_teleop = get_package_share_directory('smartcar_teleop')
    default_world = os.path.join(pkg_share_sim, 'worlds', 'smart_car_world.sdf')

    sim_launch = os.path.join(pkg_share_sim, 'launch', 'sim.launch.py')
    teleop_launch = os.path.join(pkg_share_teleop, 'launch', 'teleop.launch.py')

    declarations = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='要加载的 SDF 世界文件'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='是否连 Gazebo GUI 一起启动（需要可用的 DISPLAY）'),
    ]

    for name, (default, description) in TELEOP_ARGUMENTS.items():
        declarations.append(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )

    sim_args = {
        'world': LaunchConfiguration('world'),
        'gui': LaunchConfiguration('gui'),
    }

    teleop_args = {
        name: LaunchConfiguration(name)
        for name in TELEOP_ARGUMENTS.keys()
    }

    return LaunchDescription([
        *declarations,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments=sim_args.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
            launch_arguments=teleop_args.items()),
    ])
