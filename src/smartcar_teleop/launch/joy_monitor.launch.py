"""启动 joy_node 与 joy_monitor，用于确认手柄的轴 / 按键映射。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joy_topic', default_value='/joy',
                              description='手柄话题 sensor_msgs/msg/Joy'),
        DeclareLaunchArgument('joy_device_id', default_value='0',
                              description='joy_node 的 device_id'),
        DeclareLaunchArgument('axis_threshold', default_value='0.1',
                              description='轴变化超过该值才打印'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='每帧打印全部轴值'),
        Node(
            package='joy', executable='joy_node', name='joy_node', output='screen',
            parameters=[{'device_id': ParameterValue(
                LaunchConfiguration('joy_device_id'), value_type=int)}],
        ),
        Node(
            package='smartcar_teleop', executable='joy_monitor', name='joy_monitor',
            output='screen',
            parameters=[{
                'joy_topic': ParameterValue(
                    LaunchConfiguration('joy_topic'), value_type=str),
                'axis_threshold': ParameterValue(
                    LaunchConfiguration('axis_threshold'), value_type=float),
                'verbose': ParameterValue(
                    LaunchConfiguration('verbose'), value_type=bool),
            }],
        ),
    ])
