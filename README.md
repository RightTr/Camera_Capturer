# Guide Thermal Infrared Camera

## Stereo Camera

1.Bind the video devices of two camera heads to fixed physical USB ports.

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

2.Bind the serial devices of two camera heads to fixed physical USB ports.

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

## Official SDK

Please refer to *Linux_USB3.0_V2_0_0-x86_64-linux-gnu-gcc-9_4_0_20251201*

## Usage

* Mono

```bash
./build/guidemono <camera_id> <max_fps> (if_save) (<output_dir>) (<serial_port_id>)
```

* Stereo

Please follow the instuctions above to check video streams and serial devices of camera heads, and modify the source code accordingly.

```bash
./build/guidestereo (<if_save>) (<output_dir>)
```

Supports external trigger input (1.8 voltage 30Hz 50% duty-cycle PWM), and turn on synchronization mode.

```bash
External sync on (1) or off (0): 1
Sync on command sent.
```

## TODO

* To capsulate functionality in a class
* ARM64 Architecture Adaption
