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
    use_pwm_sync = LaunchConfiguration("use_pwm_sync")
    use_pwm_trigger_stamp = LaunchConfiguration("use_pwm_trigger_stamp")
    pwm_line = LaunchConfiguration("pwm_line")
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
            default_value="1",
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
            "use_pwm_sync",
            default_value="false",
            description="Enable external PWM trigger timestamp capture.",
        ),
        DeclareLaunchArgument(
            "use_pwm_trigger_stamp",
            default_value="false",
            description="Use PWM trigger time as the unified ROS image timestamp. When false, each image uses its own sensor time.",
        ),
        DeclareLaunchArgument(
            "pwm_line",
            default_value="PAA.00",
            description="libgpiod line name used to capture the external PWM trigger edge.",
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
                "rs_sync_mode": rs_sync_mode,
                "if_save": if_save,
                "temp_incre_detect": temp_incre_detect,
                "output_dir": output_dir,
                "use_pwm_sync": use_pwm_sync,
                "use_pwm_trigger_stamp": use_pwm_trigger_stamp,
                "pwm_line": pwm_line,
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
