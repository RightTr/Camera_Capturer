from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("camera_capturer"), "rviz_cfg", "stereo.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with the packaged stereo.rviz config.",
        ),
        Node(
            package="camera_capturer",
            executable="guidestereo_node",
            name="camera_stereo_node",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="camera_stereo_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
