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
    if_save = LaunchConfiguration("if_save")
    if_save_img = LaunchConfiguration("if_save_img")
    output_dir = LaunchConfiguration("output_dir")
    guide_query_ms = LaunchConfiguration("guide_query_ms")
    imu_fps = LaunchConfiguration("imu_fps")
    imu_queue_size = LaunchConfiguration("imu_queue_size")
    publish_combined_imu = LaunchConfiguration("publish_combined_imu")
    sync_imu_to_trigger = LaunchConfiguration("sync_imu_to_trigger")
    ros_stamp_host_clock = LaunchConfiguration("ros_stamp_host_clock")
    warmup = LaunchConfiguration("warmup")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    trigger_line = LaunchConfiguration("trigger_line")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    trigger_frequency = LaunchConfiguration("trigger_frequency")
    trigger_tolerance_ns = LaunchConfiguration("trigger_tolerance_ns")
    stereo_pair_tolerance_ns = LaunchConfiguration("stereo_pair_tolerance_ns")
    stereo_trigger_tolerance_ns = LaunchConfiguration("stereo_trigger_tolerance_ns")
    realsense_trigger_max_latency_ns = LaunchConfiguration("realsense_trigger_max_latency_ns")
    stereo_pair_wait_ms = LaunchConfiguration("stereo_pair_wait_ms")
    enable_guide_temperature = LaunchConfiguration("enable_guide_temperature")
    depth_stream_enable = LaunchConfiguration("depth_stream_enable")
    depth_processing_enable = LaunchConfiguration("depth_processing_enable")
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
            "if_save_img",
            default_value="0",
            description="Save left/right/RGBD image PNG files when if_save is enabled.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="./capture",
            description="Output directory used when if_save is enabled.",
        ),
        DeclareLaunchArgument(
            "guide_query_ms",
            default_value="100",
            description="Guide camera serial status query interval (ms).",
        ),
        DeclareLaunchArgument(
            "imu_fps",
            default_value="200",
            description="RealSense accel and gyro frame rate.",
        ),
        DeclareLaunchArgument(
            "imu_queue_size",
            default_value="2000",
            description="Internal IMU producer queue size.",
        ),
        DeclareLaunchArgument(
            "publish_combined_imu",
            default_value="true",
            description="Publish gyro-rate /realsense/imu/data with interpolated acceleration.",
        ),
        DeclareLaunchArgument(
            "sync_imu_to_trigger",
            default_value="true",
            description="Interpolate IMU timestamps from depth hardware time to trigger Unix time.",
        ),
        DeclareLaunchArgument(
            "ros_stamp_host_clock",
            default_value="false",
            description="Stamp camera images at GPIO capture time; mapped IMU uses the same trigger time axis.",
        ),
        DeclareLaunchArgument(
            "warmup",
            default_value="10",
            description="Seconds to wait after RealSense is ready before output starts.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port that receives board-provided PWM edge Unix timestamps in ns.",
        ),
        DeclareLaunchArgument(
            "serial_baud",
            default_value="115200",
            description="Baud rate for the board timestamp serial port.",
        ),
        DeclareLaunchArgument(
            "trigger_line",
            default_value="PAA.00",
            description="GPIO line on the Orin that receives the trigger sync signal.",
        ),
        DeclareLaunchArgument(
            "sync_queue_size",
            default_value="4096",
            description="Maximum unmatched GPIO/serial sync events kept for FIFO pairing.",
        ),
        DeclareLaunchArgument(
            "trigger_frequency",
            default_value="30.0",
            description="Expected trigger frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "trigger_tolerance_ns",
            default_value="5000000",
            description="Allowed trigger period error in nanoseconds.",
        ),
        DeclareLaunchArgument(
            "stereo_pair_tolerance_ns",
            default_value="10000000",
            description="Maximum left/right V4L2 sensor timestamp difference in nanoseconds.",
        ),
        DeclareLaunchArgument(
            "stereo_trigger_tolerance_ns",
            default_value="5000000",
            description="Maximum paired Guide host timestamp to trigger capture difference in nanoseconds.",
        ),
        DeclareLaunchArgument(
            "realsense_trigger_max_latency_ns",
            default_value="25000000",
            description="Maximum delay from trigger capture to RealSense frame host time; must be below one trigger period.",
        ),
        DeclareLaunchArgument(
            "stereo_pair_wait_ms",
            default_value="120",
            description="Maximum time to retain an unmatched Guide image.",
        ),
        DeclareLaunchArgument(
            "enable_guide_temperature",
            default_value="false",
            description="Enable Guide temperature conversion and temperature topic publishing.",
        ),
        DeclareLaunchArgument(
            "depth_stream_enable",
            default_value="true",
            description="Enable the RealSense depth stream.",
        ),
        DeclareLaunchArgument(
            "depth_processing_enable",
            default_value="true",
            description="Process, publish, and save depth after trigger matching.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz with the packaged rgbdt.rviz config.",
        ),
        Node(
            package="camera_capturer",
            executable="rgbdt_trigger_node",
            name="rgbdt_trigger_node",
            output="screen",
            parameters=[{
                "rs_sync_mode": rs_sync_mode,
                "if_save": if_save,
                "if_save_img": if_save_img,
                "output_dir": output_dir,
                "guide_query_ms": guide_query_ms,
                "imu_fps": imu_fps,
                "imu_queue_size": imu_queue_size,
                "publish_combined_imu": ParameterValue(publish_combined_imu, value_type=bool),
                "sync_imu_to_trigger": ParameterValue(sync_imu_to_trigger, value_type=bool),
                "ros_stamp_host_clock": ParameterValue(ros_stamp_host_clock, value_type=bool),
                "warmup": warmup,
                "serial_port": serial_port,
                "serial_baud": serial_baud,
                "trigger_line": trigger_line,
                "sync_queue_size": sync_queue_size,
                "trigger_frequency": trigger_frequency,
                "trigger_tolerance_ns": trigger_tolerance_ns,
                "stereo_pair_tolerance_ns": stereo_pair_tolerance_ns,
                "stereo_trigger_tolerance_ns": stereo_trigger_tolerance_ns,
                "realsense_trigger_max_latency_ns": realsense_trigger_max_latency_ns,
                "stereo_pair_wait_ms": stereo_pair_wait_ms,
                "enable_guide_temperature": ParameterValue(enable_guide_temperature, value_type=bool),
                "depth_stream_enable": ParameterValue(depth_stream_enable, value_type=bool),
                "depth_processing_enable": ParameterValue(depth_processing_enable, value_type=bool),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rgbdt_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
