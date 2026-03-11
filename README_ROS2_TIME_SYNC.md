# ROS2 时间对齐版运行说明（`camera_RGBDT` + `pwm_capture_node`）

本文档用于在当前工程中运行“ROS2 时间对齐版”采集链路：
- `time_sync` 工作空间发布 PWM 上升沿硬件时间戳
- `camera_RGBDT` 订阅该时间戳并将图像时间对齐到 PWM 时间线

## 1. 功能拓扑

1. `pwm_capture_node`（包：`pwm_capture`）发布：
   - `pwm_capture/rising_edge_time_ns`
   - `pwm_capture/period_ns`
2. `camera_RGBDT`（包：`camera_capturer`）内部 ROS2 节点 `camera_rgbdt_pwm_sync` 订阅：
   - `pwm_capture/rising_edge_time_ns`
3. 保存数据时（`if_save=1`），`times.csv` 中包含：
   - `aligned_time,aligned_ns,sensor_time,sensor_ns,host_time,host_ns,host_minus_aligned_ns`

## 2. 运行前检查

1. 先确认 ROS2 环境可用（示例以 `humble` 为准）：
```bash
source /opt/ros/humble/setup.bash
```

2. 检查 Guide 双目视频设备（代码当前固定为以下路径）：
```bash
ls -l /dev/v4l/by-path/platform-3610000.usb-usb-0:3.3:1.0-video-index0
ls -l /dev/v4l/by-path/platform-3610000.usb-usb-0:3.4:1.0-video-index0
```

3. 检查 Guide 串口设备（当前代码优先使用以下路径）：
```bash
ls -l /dev/guide_left /dev/guide_right
ls -l /dev/serial/by-path/platform-3610000.usb-usb-0:3.3:1.2
ls -l /dev/serial/by-path/platform-3610000.usb-usb-0:3.4:1.2
```

4. 检查 RealSense 是否存在（代码里默认序列号是 `253822301280`）：
```bash
rs-enumerate-devices | grep -A2 "Serial Number"
```

## 3. 编译

建议分别在两个工作空间编译对应包。

### 3.1 编译 `pwm_capture`
```bash
cd /home/pi/workspace/time_sync
source /opt/ros/humble/setup.bash
colcon build --packages-select pwm_capture
```

### 3.2 编译 `camera_capturer`
```bash
cd /home/pi/workspace/cap_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_capturer
```

## 4. 启动顺序（推荐 3 个终端）

### 终端 A：启动 PWM 采集节点
```bash
cd /home/pi/workspace/time_sync
source /opt/ros/humble/setup.bash
source install/setup.bash

# 默认 GPIO 线名为 PAA.00
ros2 run pwm_capture pwm_capture_node
```

如果报“无法请求边沿事件/权限不足”，用 sudo 启动：
```bash
sudo -E bash -lc 'source /opt/ros/humble/setup.bash && source /home/pi/workspace/time_sync/install/setup.bash && ros2 run pwm_capture pwm_capture_node'
```

### 终端 B：启动 RGBDT 采集主程序
```bash
cd /home/pi/workspace/cap_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 参数: (<realsense_sync>) (<if_save>) (<output_dir>)
/home/pi/workspace/cap_ws/build/camera_capturer/camera_RGBDT 1 1 /home/pi/workspace/cap_ws/capture_ros_sync
```

参数说明：
- `realsense_sync`: `1` 开启 RealSense 外触发同步配置；`0` 关闭
- `if_save`: `1` 保存数据；`0` 仅显示不落盘
- `output_dir`: 输出目录（仅 `if_save=1` 生效）

### 终端 C：验证 ROS2 时间话题
```bash
source /opt/ros/humble/setup.bash
source /home/pi/workspace/time_sync/install/setup.bash

ros2 topic list | grep pwm_capture
ros2 topic hz /pwm_capture/rising_edge_time_ns
ros2 topic echo /pwm_capture/period_ns
```

## 5. 结果检查

当 `if_save=1` 时，检查：
```bash
ls -R /home/pi/workspace/cap_ws/capture_ros_sync
head -n 5 /home/pi/workspace/cap_ws/capture_ros_sync/left/times.csv
head -n 5 /home/pi/workspace/cap_ws/capture_ros_sync/right/times.csv
head -n 5 /home/pi/workspace/cap_ws/capture_ros_sync/realsense/times.csv
```

`host_minus_aligned_ns` 越接近 0，表示主机时间与 PWM 时间线对齐越好。

## 6. 常见问题

1. `Open serial error ...`
- 先检查 `/dev/guide_left`、`/dev/guide_right` 或 `/dev/serial/by-path/...:3.3:1.2 / ...:3.4:1.2` 是否存在。
- 检查串口权限：用户是否在 `dialout` 组。

2. `找不到 GPIO line: PAA.00`
- 板卡引脚名不同，需改 `pwm_capture_node` 参数 `line_name`，例如：
```bash
ros2 run pwm_capture pwm_capture_node --ros-args -p line_name:=<你的GPIO线名>
```

3. 看不到 `pwm_capture/rising_edge_time_ns`
- 检查外部 PWM 信号是否已经输入（频率/占空比符合预期）。
- 检查 `pwm_capture_node` 是否在运行且无报错。

4. RealSense 未启动
- 当前 `camera_RGBDT` 里 RealSense 序列号写死为 `253822301280`，不一致时需要修改源码后重编译。
