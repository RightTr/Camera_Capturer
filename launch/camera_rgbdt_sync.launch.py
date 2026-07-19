from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rs_sync_mode = LaunchConfiguration("rs_sync_mode")
    if_save = LaunchConfiguration("if_save")
    temp_incre_detect = LaunchConfiguration("temp_incre_detect")
    output_dir = LaunchConfiguration("output_dir")
    guide_query_ms = LaunchConfiguration("guide_query_ms")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    pwm_line = LaunchConfiguration("pwm_line")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("camera_capturer"), "rviz_cfg", "rgbdt.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rs_sync_mode",
            default_value="3",
            description="RealSense inter-cam sync mode. Set 0 to disable.",
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
            default_value="/home/pi/Cap_ws",
            description="Output directory used when if_save is enabled.",
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
            "sync_queue_size",
            default_value="4096",
            description="Maximum unmatched GPIO/serial sync events kept for FIFO pairing.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with the packaged rgbdt.rviz config.",
        ),
        Node(
            package="camera_capturer",
            executable="camera_RGBDT_sync_node",
            name="camera_rgbdt_sync_node",
            output="screen",
            parameters=[{
                "rs_sync_mode": rs_sync_mode,
                "if_save": if_save,
                "temp_incre_detect": temp_incre_detect,
                "output_dir": output_dir,
                "guide_query_ms": guide_query_ms,
                "serial_port": serial_port,
                "serial_baud": serial_baud,
                "pwm_line": pwm_line,
                "sync_queue_size": sync_queue_size,
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
