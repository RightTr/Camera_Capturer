from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rs_sync_mode = LaunchConfiguration("rs_sync_mode")
    fps = LaunchConfiguration("fps")
    enable_imu = LaunchConfiguration("enable_imu")
    imu_fps = LaunchConfiguration("imu_fps")
    enable_align = LaunchConfiguration("enable_align")
    enable_filter = LaunchConfiguration("enable_filter")
    rgb_queue_size = LaunchConfiguration("rgb_queue_size")
    imu_queue_size = LaunchConfiguration("imu_queue_size")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("camera_capturer"), "rviz_cfg", "rgbdt.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rs_sync_mode",
            default_value="0",
            description="RealSense inter-cam sync mode. Set 0 to disable.",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="30",
            description="RGB and depth frame rate.",
        ),
        DeclareLaunchArgument(
            "enable_imu",
            default_value="true",
            description="Enable RealSense accel and gyro publishing.",
        ),
        DeclareLaunchArgument(
            "imu_fps",
            default_value="200",
            description="RealSense accel and gyro frame rate.",
        ),
        DeclareLaunchArgument(
            "enable_align",
            default_value="true",
            description="Align depth to color before publishing.",
        ),
        DeclareLaunchArgument(
            "enable_filter",
            default_value="true",
            description="Apply spatial and temporal filters to depth.",
        ),
        DeclareLaunchArgument(
            "rgb_queue_size",
            default_value="30",
            description="Internal RGBD producer queue size.",
        ),
        DeclareLaunchArgument(
            "imu_queue_size",
            default_value="400",
            description="Internal IMU producer queue size.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz with the packaged rgbdt.rviz config.",
        ),
        Node(
            package="camera_capturer",
            executable="realsense_node",
            name="realsense_node",
            output="screen",
            parameters=[{
                "rs_sync_mode": ParameterValue(rs_sync_mode, value_type=int),
                "fps": ParameterValue(fps, value_type=int),
                "enable_imu": ParameterValue(enable_imu, value_type=bool),
                "imu_fps": ParameterValue(imu_fps, value_type=int),
                "enable_align": ParameterValue(enable_align, value_type=bool),
                "enable_filter": ParameterValue(enable_filter, value_type=bool),
                "rgb_queue_size": ParameterValue(rgb_queue_size, value_type=int),
                "imu_queue_size": ParameterValue(imu_queue_size, value_type=int),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="realsense_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
