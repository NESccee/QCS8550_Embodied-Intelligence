"""启动 joy_node 与手柄遥控节点（/joy -> /cmd_vel）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 启动参数名: (默认值, 类型, 说明, joy_node 的参数名)
# 注意：第四项必须写 joy_node 真正声明的名字，否则参数下发了也没人用。
JOY_ARGUMENTS = {
    'joy_device_id': ('0', int, 'joy_node 的 device_id', 'device_id'),
    'joy_device_name': ('', str, '按设备名选择手柄，留空表示使用 device_id', 'device_name'),
    'joy_deadzone': ('0.05', float, 'joy_node 的摇杆死区', 'deadzone'),
    'joy_autorepeat_rate': (
        '20.0', float, 'joy_node 静止时的重发频率 [Hz]，0 表示只在变化时发', 'autorepeat_rate'),
}

# name: (默认值, 类型, 说明)，下发给 joy_teleop
TELEOP_ARGUMENTS = {
    'joy_topic': ('/joy', str, '手柄话题 sensor_msgs/msg/Joy'),
    'cmd_vel_topic': ('/cmd_vel', str, '底盘速度话题 geometry_msgs/msg/Twist'),
    'axis_linear': ('1', int, '前进/后退轴索引（本手柄左摇杆纵向为 1）'),
    'axis_angular': ('0', int, '转向轴索引（本手柄左摇杆横向为 0）'),
    'invert_linear': ('false', bool, '反转速度轴，前进后退相反时置 true'),
    'invert_angular': ('false', bool, '反转转向轴，左右转向相反时置 true'),
    'deadzone': ('0.05', float, '遥控节点的摇杆死区'),
    'button_deadman': ('-1', int, '必须按住的使能键索引，-1 表示不检查'),
    'button_turbo': ('-1', int, '高速档按键索引，-1 表示关闭'),
    'turbo_scale': ('1.5', float, '高速档倍率'),
    'max_linear': ('0.6', float, '最大线速度 [m/s]'),
    'max_angular': ('1.5', float, '最大角速度 [rad/s]'),
    'publish_rate': ('20.0', float, '/cmd_vel 发布频率 [Hz]'),
    'joy_timeout': ('0.5', float, '超过该时间没收到 Joy 就发零速度 [s]'),
}


def _declarations(arguments):
    return [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, _, description, *__) in arguments.items()
    ]


def _teleop_parameters(arguments):
    # 必须显式声明类型，否则 LaunchConfiguration 会以字符串下发，
    # rclcpp 在 declare_parameter 时抛 InvalidParameterTypeException。
    return {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, (_, value_type, _) in arguments.items()
    }


def _joy_parameters(arguments):
    return {
        parameter: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, (_, value_type, _, parameter) in arguments.items()
    }


def generate_launch_description():
    return LaunchDescription([
        *_declarations(JOY_ARGUMENTS),
        *_declarations(TELEOP_ARGUMENTS),
        Node(
            package='joy', executable='joy_node', name='joy_node', output='screen',
            parameters=[_joy_parameters(JOY_ARGUMENTS)],
        ),
        Node(
            package='smartcar_teleop', executable='joy_teleop', name='joy_teleop',
            output='screen',
            parameters=[_teleop_parameters(TELEOP_ARGUMENTS)],
        ),
    ])
