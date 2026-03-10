# Camera Capturer

Camera capturer implementation using v4l2.

* Guide Thermal Infrared Camera (Support external trigger synchronization)

Guide Official SDK: Please refer to *./Linux_USB3.0_V2_0_0-x86_64-linux-gnu-gcc-9_4_0_20251201*

* Realsense D455f RGBD Camera (Support external trigger synchronization)

## 1. Prerequisites

### 1.1 Bind Video Devices to Fixed USB Ports

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
  
``` bash
# show video stream 1
ffplay -f v4l2   -pixel_format yuyv422   -video_size 1280x513   -framerate 30   /dev/v4l/by-path/pci-0000:00:8.0-usb-0:8:1.0-video-index0
```

* e.g., pci-0000:00:14.0-usb-0:3:1.0-video-index0 -> camera 2

``` bash
# show video stream 2
ffplay -f v4l2   -pixel_format yuyv422   -video_size 1280x513   -framerate 30   /dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0
```

### 1.2 Bind Serial Devices to Fixed USB Ports

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

``` bash
sudo touch /etc/udev/rules.d/99-guide.rules

echo 'SUBSYSTEM=="tty", KERNEL=="ttyACM*", DEVPATH=="*/2-2/*", SYMLINK+="guide_left"' | sudo tee /etc/udev/rules.d/99-guide.rules

echo 'SUBSYSTEM=="tty", KERNEL=="ttyACM*", DEVPATH=="*/2-8/*", SYMLINK+="guide_right"' | sudo tee -a /etc/udev/rules.d/99-guide.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Now, /dev/guide_left and /dev/guide_right refer to fixed USB ports.

### 1.3 Check RealSense Serial Number

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

### 2.1 Direct Build

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

### 3.1 Direct Run

* Guide Mono

```bash
./build/guidemono <camera_id> <max_fps> (<if_save>) (<output_dir>) (<serial_port_id>)
```

* Guide Stereo

Please follow the instuctions above to check video streams and serial devices of camera heads, and modify the source code accordingly.

```bash
./build/guidestereo (<if_save>) (<tempIncre_detect>) (<output_dir>)
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
./build/camera_RGBDT (<realsense_sync>) (<if_save>) (<tempIncre_detect>) (<output_dir>)
```

Following the instructions above to turn on Guide Stereo external trigger synchronization.

tempIncre_detect (default: false), same as the description above. Especially, when the focal temperature of the dev_camera[0] (default thermal_left) varies by more than 0.1°C, the Realsense consumer thread will start capturing 30 frames according to the temperature increase signal from dev_camera[0].

### 3.2 Run with ROS

* Guide Stereo Node

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

## TODO

* ARM64 Adaption Test
* ROS1/ROS2 Adaption
* RealSense Color Image Visualization Fault
* RealSense External Trigger Synchronization Test
