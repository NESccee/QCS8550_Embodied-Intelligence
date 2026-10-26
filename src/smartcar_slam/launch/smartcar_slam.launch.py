"""启动多传感器融合 SLAM 与语义自主导航系统。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data',
            description='高频 IMU 话题'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='底盘/轮速计话题'),
        DeclareLaunchArgument(
            'degeneracy_threshold', default_value='25.0',
            description='退化检测信息矩阵最小特征值阈值'),

        Node(
            package='smartcar_slam',
            executable='lio_sam_node',
            name='lio_sam_node',
            output='screen',
            parameters=[{
                'imu_topic': ParameterValue(LaunchConfiguration('imu_topic'), value_type=str),
                'odom_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
                'degeneracy_threshold': ParameterValue(LaunchConfiguration('degeneracy_threshold'), value_type=float),
            }],
        ),

        Node(
            package='smartcar_slam',
            executable='semantic_projection_node',
            name='semantic_projection_node',
            output='screen',
        ),

        Node(
            package='smartcar_slam',
            executable='semantic_planner_node',
            name='semantic_planner_node',
            output='screen',
        ),
    ])

