# RGBDT Camera Capturer

Camera capturer implementation using v4l2.

* Guide Thermal Infrared camera (Support external trigger synchronization)

Guide official SDK and documents: Please refer to *./Linux_USB3.0_V2_0_0-x86_64-linux-gnu-gcc-9_4_0_20251201*

* Realsense D455f RGBD camera (Support external trigger synchronization)

## 1. Prerequisites

### 1.1 Bind video devices to fixed USB ports

Bind the video devices of two camera heads to fixed physical USB ports.

Ensure each camera head is bound to a fixed physical USB port.

For example,

```bash
ls -l /dev/v4l/by-path/

lrwxrwxrwx 1 root root 12 1月  15 18:24 pci-0000:00:14.0-usb-0:2:1.0-video-index0 -> ../../video4
lrwxrwxrwx 1 root root 12 1月  15 18:24 pci-0000:00:14.0-usb-0:2:1.0-video-index1 -> ../../video5
lrwxrwxrwx 1 root root 12 1月  15 18:24 pci-0000:00:14.0-usb-0:8:1.0-video-index0 -> ../../video2
lrwxrwxrwx 1 root root 12 1月  15 18:24 pci-0000:00:14.0-usb-0:8:1.0-video-index1 -> ../../video3

```

* e.g.,pci-0000:00:14.0-usb-0:8:1.0-video-index0 -> camera 1

```bash
# show video stream 1
ffplay -f v4l2   -pixel_format yuyv422   -video_size 1280x513   -framerate 30   /dev/v4l/by-path/pci-0000:00:8.0-usb-0:8:1.0-video-index0
```

* e.g., pci-0000:00:14.0-usb-0:3:1.0-video-index0 -> camera 2

```bash
# show video stream 2
ffplay -f v4l2   -pixel_format yuyv422   -video_size 1280x513   -framerate 30   /dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0
```

### 1.2 Bind serial devices to fixed USB ports

Bind the serial devices of two camera heads to fixed physical USB ports.

Make sure each camera head is bound to a fixed physical USB port.

```bash
ls /dev/ttyACM*
# There are two serial devices
/dev/ttyACM0 /dev/ttyACM1
```

Query the physical device path (DECPATH) of /dev/ttyACM* using udev.

For example,

```bash
udevadm info -n /dev/ttyACM0 | grep DEVPATH
E: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb2/2-2/2-8:1.2/tty/ttyACM0 # physical port: 2-8 -> right camera
udevadm info -n /dev/ttyACM1 | grep DEVPATH
E: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb2/2-8/2-2:1.2/tty/ttyACM1 # physical port: 2-2 -> left camera
```

Make sure the camera head–USB port mapping is correct.
Then, bind each camera head to its own USB port.

```bash
sudo touch /etc/udev/rules.d/99-guide.rules

echo 'SUBSYSTEM=="tty", KERNEL=="ttyACM*", DEVPATH=="*/2-2/*", SYMLINK+="guide_left"' | sudo tee /etc/udev/rules.d/99-guide.rules

echo 'SUBSYSTEM=="tty", KERNEL=="ttyACM*", DEVPATH=="*/2-8/*", SYMLINK+="guide_right"' | sudo tee -a /etc/udev/rules.d/99-guide.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Now, /dev/guide_left and /dev/guide_right refer to fixed USB ports.

### 1.3 Check realSense serial number

```bash
rs-enumerate-devices
Device info: 
    Name                          : 	Intel RealSense D455F
    Serial Number                 : 	253822301280 # Serial Number
    Firmware Version              : 	5.15.1.55
    Recommended Firmware Version  : 	5.16.0.1
    Physical Port                 : 	/sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1:1.0/video4linux/video8
```

Please modify the variable *dev_rs* to the number above in the source code.

## 2. Build

### 2.1 Direct build

```bash
git clone https://github.com/RightTr/Camera_Capturer.git

cd Camera_Capturer
mkdir build && cd build

cmake ..
make
```

### 2.2 Run with ROS

```bash
mkdir cap_ws && cd cap_ws
mkdir src && cd src

git clone https://github.com/RightTr/Camera_Capturer.git

cd Camera_Capturer

# ROS1
./build.sh ROS1

# ROS2 Humble
./build.sh humble
```

## 3. Usage

### 3.1 Direct run

* Guide Mono

```bash
./build/guidemono <camera_id> (<if_save>) (<output_dir>) (<serial_port_id>) (<guide_query_ms>)
```

* Guide Stereo

Please follow the instuctions above to check video streams and serial devices of camera heads, and modify the source code accordingly.

```bash
./build/guidestereo (<if_save>) (<tempIncre_detect>) (<output_dir>) (<guide_query_ms>)
```

Supports external trigger input (1.8 voltage 30Hz 50% duty-cycle PWM) which must be provided before enabling synchronization mode.

tempIncre_detect (default: false), if the focal temperature of the guide thermal infrared camera varies by more than 0.1°C, it will automatically capture 30 frames, and then stop saving to avoid excessive data.

```bash
External sync on (1) or off (0): 1
Sync on command sent.
Port 0 Sync on
Port 1 Sync on
```

* RGBDT Capturer with Guide Stereo and RealSense Camera

```bash
./build/camera_RGBDT (<realsense_sync>) (<if_save>) (<tempIncre_detect>) (<output_dir>) (<guide_query_ms>)
```

Following the instructions above to turn on Guide Stereo external trigger synchronization.

tempIncre_detect (default: false), same as the description above. Especially, when the focal temperature of the dev_camera[0] (default thermal_left) varies by more than 0.1°C, the Realsense consumer thread will start capturing 30 frames according to the temperature increase signal from dev_camera[0].

### 3.2 Run with ROS

* Guide Stereo node

```bash
cd cap_ws

# ROS1
source devel/setup.bash
rosrun camera_capturer guidestereo_node

# ROS 2
source install/setup.bash
ros2 run camera_capturer guidestereo_node
```

Enable external trigger synchronization via the */guidecam/sync* topic

```bash
# ROS1
rostopic pub -1 /guidecam/sync std_msgs/Int32 "{data: 1}" # Sync on
rostopic pub -1 /guidecam/sync std_msgs/Int32 "{data: 0}" # Sync off

# ROS2
ros2 topic pub --once /guidecam/sync std_msgs/Int32 "{data: '1'}" # Sync on
ros2 topic pub --once /guidecam/sync std_msgs/Int32 "{data: '0'}" # Sync off
```

* Guide Stereo launch

```bash
ros2 launch camera_capturer guidestereo.launch.py

ros2 launch camera_capturer guidestereo_trigger.launch.py 
```

Use this launch file (*guidestereo_trigger.launch.py*) when Guide stereo is driven by the external PWM trigger board. The board timestamp serial port provides the PWM output time, and the GPIO line captures the PWM generation time on MCU.

* RGBDT launch

```bash
ros2 launch camera_capturer camera_rgbdt.launch.py

ros2 launch camera_capturer rgbdt_trigger.launch.py 
```

Use this launch file (*rgbdt_trigger.launch.py*) when Guide stereo and RealSense RGBD are driven by the external PWM trigger board. The board timestamp serial port provides the PWM output time, and the GPIO line captures the PWM generation time on MCU.

The two trigger nodes use different synchronization paths. `guidestereo_trigger_node` retains its existing timestamp matcher. `rgbdt_trigger_node` uses the board trigger as the camera slot clock: a thermal stereo pair, RGB, and depth assigned to slot `k` all receive its board output timestamp `T_k`.

In `rgbdt_trigger_node`, Guide left/right frames are paired by V4L2 sensor timestamps. GPIO capture and host arrival times calibrate the initial slot only; `calibration_max_latency_ns` must be a measured upper bound smaller than one trigger period. Forward camera sequence gaps skip the missing trigger slots without recalibrating or clearing the IMU buffer. Missing trigger telemetry similarly preserves physical slot positions when the elapsed board time is an integral number of periods. The Guide producer passes every dequeued frame without software frame-rate throttling.

The coordinator publishes gyro-rate `realsense/imu/data` independently of image completeness. Depth hardware timestamps provide clock anchors even when a Guide frame or RGB-D processing result is missing. Between valid anchors, every received gyro sample is mapped and acceleration is interpolated at that sample's hardware time. Nonadjacent depth anchors are allowed only when their sensor/trigger elapsed times agree with the skipped slot count and IMU continuity remains valid. No IMU samples or timestamps are synthesized. Normally this adds one trigger period of latency; an isolated depth frame skip can extend it to two periods while preserving approximately 5 ms IMU header intervals at 200 Hz.

Guide stereo and RGB-D image pairs publish independently, after the IMU timestamp has crossed their trigger time. Thus Air-VINS (`/guide_left/image`, `/guide_right/image`, `/realsense/imu/data`) does not pause for a missing RGB-D image. Incomplete image slots expire after three periods; timestamps of subsequent images are not compressed or renumbered. A saved trigger slot can contain only thermal or only RGB-D rows in `times.csv`.

Before IMU publication begins, invalid startup segments automatically recalibrate and warn periodically. After IMU publication begins, actual IMU loss, a backward/invalid clock, or loss of the depth clock for three periods stops capture explicitly: resuming a new time epoch into an existing VIO integrator would create an unsafe large integration dt. Reset the downstream estimator before starting a new session in that case. Air-VINS exposes `/stir_slam/restart`, but this driver does not automatically call that unacknowledged interface. Recording failures and unrecoverable producer failures also remain fatal. `recovery_events.csv` records startup recalibration events; `[vio_continuity]` reports the published IMU count, maximum IMU header dt, image pair counts, and incomplete image slots at shutdown. Both hardware IMU streams must have continuous per-stream frame numbers and timestamps; gyro uses `imu_fps` and accel uses the nearest supported rate.

For Cyclone DDS receivers, also check `nstat -az UdpRcvbufErrors` before and after a capture. On this host, the 212992-byte UDP receive buffer caused receiver-side image loss even while the saved Guide timestamps were continuous. Increase the receive buffer on the receiving host, then restart the ROS processes so newly created sockets use it:

```bash
sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608
```

This command changes the running system only; persistence across reboot requires a sysctl configuration managed by the operator. Validate the receiver's image and IMU header intervals, not just the driver's publication count. Air-VINS defaults to best-effort sensor subscriptions, so transport loss can still create a downstream gap independently of capture-side continuity.

When saving is enabled, a separate writer thread handles PNG and CSV output. A full writer queue or write failure stops the capture. `rgbdt_trigger.launch.py` always enables and publishes depth; this mode no longer offers `depth_stream_enable` or `depth_processing_enable` overrides. Independent accel and gyro ROS topics are not published by this trigger node; accepted raw samples remain available in its CSV files.

In strict trigger mode, a short RealSense SDK callback queue buffers framesets before the producer consumes them. A genuine RGB or depth frame-number gap still stops output, and the log shows both previous and current frame numbers. The four camera image publishers use reliable ROS 2 QoS so reliable subscribers can connect.

The first 10 seconds after startup are a warm-up period. A continuous startup segment must be established within 10 seconds after warm-up; otherwise the node exits with an error.

* RealSense launch

```bash
ros2 launch camera_capturer realsense.launch.py
```

RealSense launch files expose two independent depth controls:

- `depth_stream_enable:=false` disables the depth stream completely.
- `depth_stream_enable:=true depth_processing_enable:=false` keeps depth metadata
  and hardware synchronization active, but skips alignment, filtering, depth image
  publication, and depth PNG output.

## 4. Output

When `if_save=1`, data is written under `output_dir`. Image folders are written only when `if_save_img=1`; CSV and text metadata are still saved when `if_save_img=0`.

```text
output_dir/
├── times.csv
├── left/
│   ├── times.csv
│   ├── params.txt
│   ├── focal_temperature.txt
│   ├── image/
│   │   └── <timestamp>.png
│   └── temperature/
│       └── <timestamp>.png
├── right/
│   ├── times.csv
│   ├── params.txt
│   ├── focal_temperature.txt
│   ├── image/
│   │   └── <timestamp>.png
│   └── temperature/
│       └── <timestamp>.png
└── realsense/
    ├── times.csv
    ├── realsense_intrinsics.txt
    ├── depth_scale.txt
    ├── rgb/
    │   └── <timestamp>.png
    ├── depth_raw/
    │   └── <timestamp>.png
    └── imu/
        ├── accel.csv
        └── gyro.csv
```

`times.csv` in the top-level `output_dir` is diagnostic only. Each accepted camera image writes one event row:

```text
trigger_id,trigger_time,source,sensor_time
```

The file does not control matching or publication.

`left/` and `right/` contain Guide stereo output:

- `times.csv`: `sensor_time,host_time`.
- `params.txt`: Guide frame parameters, starting with host timestamp, then humidity, distance, emissivity, reflected temperature, shutter flag, hot/cold/mark points, and region average temperature.
- `focal_temperature.txt`: periodic Guide focal temperature query result.
- `image/<timestamp>.png`: Guide image.
- `temperature/<timestamp>.png`: Guide temperature image.

`realsense/` contains RealSense RGBD and IMU output:

- `times.csv`: `color_sensor_time,color_host_time,depth_sensor_time,depth_host_time`.
- `realsense_intrinsics.txt`: saved RealSense RGB/depth camera intrinsics.
- `depth_scale.txt`: RealSense depth scale.
- `rgb/<timestamp>.png`: RealSense color image.
- `depth_raw/<timestamp>.png`: RealSense raw depth image.
- `imu/accel.csv`: accelerometer samples as `host_time,sensor_time,ax,ay,az,trigger_time`.
- `imu/gyro.csv`: gyroscope samples as `host_time,sensor_time,gx,gy,gz,trigger_time`.

The trigger node preserves raw `sensor_time` in these files and records the mapped trigger-clock time for accepted IMU samples. The combined sample is published on `realsense/imu/data`.

In trigger mode, Guide and RealSense image filenames use the PWM output time. In non-trigger mode, image filenames use the camera sensor time. All saved timestamps use `sec.nsec` format with 9 digits after the decimal point.
