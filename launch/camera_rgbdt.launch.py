from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sync_enable = LaunchConfiguration("sync_enable")
    if_save = LaunchConfiguration("if_save")
    temp_incre_detect = LaunchConfiguration("temp_incre_detect")
    output_dir = LaunchConfiguration("output_dir")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("camera_capturer"), "rviz_cfg", "rgbdt.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "sync_enable",
            default_value="1",
            description="Enable external sync when non-zero.",
        ),
        DeclareLaunchArgument(
            "if_save",
            default_value="0",
            description="Save captured data to disk when non-zero.",
        ),
        DeclareLaunchArgument(
            "temp_incre_detect",
            default_value="0",
            description="Enable temperature-increase-triggered save mode when non-zero.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="/data/home/pi/Cap",
            description="Output directory used when if_save is enabled.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with the packaged rgbdt.rviz config.",
        ),
        Node(
            package="camera_capturer",
            executable="camera_RGBDT_node",
            name="camera_rgbdt_node",
            output="screen",
            parameters=[{
                "sync_enable": sync_enable,
                "if_save": if_save,
                "temp_incre_detect": temp_incre_detect,
                "output_dir": output_dir,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="camera_rgbdt_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
