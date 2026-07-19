from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    guide_query_ms = LaunchConfiguration("guide_query_ms")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    pwm_line = LaunchConfiguration("pwm_line")
    if_save = LaunchConfiguration("if_save")
    output_dir = LaunchConfiguration("output_dir")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("camera_capturer"), "rviz_cfg", "stereo.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with the packaged stereo.rviz config.",
        ),
        DeclareLaunchArgument(
            "guide_query_ms",
            default_value="100",
            description="Guide camera serial status query interval in milliseconds.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyCH341USB0",
            description="Serial port that receives board-provided PWM edge Unix timestamps in ns.",
        ),
        DeclareLaunchArgument(
            "serial_baud",
            default_value="115200",
            description="Baud rate for the board timestamp serial port.",
        ),
        DeclareLaunchArgument(
            "pwm_line",
            default_value="PAA.00",
            description="GPIO line on the Orin that receives the PWM sync signal.",
        ),
        DeclareLaunchArgument(
            "if_save",
            default_value="0",
            description="Save synchronized guide images and stereo timestamp CSV when nonzero.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="./capture",
            description="Directory used when if_save is enabled.",
        ),
        Node(
            package="camera_capturer",
            executable="guidestereo_sync_node",
            name="camera_stereo_sync_node",
            output="screen",
            parameters=[{
                "guide_query_ms": guide_query_ms,
                "serial_port": serial_port,
                "serial_baud": serial_baud,
                "pwm_line": pwm_line,
                "if_save": if_save,
                "output_dir": output_dir,
            }],
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
