#!/bin/bash

# 配置路径
PROGRAM="/home/pi/workspace/cap_ws/build/camera_capturer/camera_RGBDT"
PTZ_SCRIPT="/home/pi/workspace/cap_ws/src/Camera_Capturer/8_rotation.py"
RESET_SCRIPT="/home/pi/workspace/cap_ws/src/Camera_Capturer/8_reset.py"

# 时间配置
PAUSE_TIME=90    # 两次大循环（每轮12点）之间的休息时间
STABLE_TIME=3   # 云台转动后的稳定时间
CAPTURE_TIME=1  # 每个点位拍摄的时长

trap "echo 'User interrupted, exiting'; exit 0" INT

python3 $RESET_SCRIPT

while true; do
    # 1. 在循环外生成当前批次的总路径
    BATCH_TIME=$(date +"%Y%m%d_%H%M%S")
    BATCH_PATH="./capture/sequence1/${BATCH_TIME}"
    
    echo "============================================"
    echo "Starting Batch: ${BATCH_TIME} (All 12 points will be saved here)"
    
    # 如果文件夹不存在，可以先创建（取决于你的程序是否支持自动创建）
    mkdir -p "$BATCH_PATH"m

    for i in {1..12}; do
        echo "Point [$i/12]: Moving PTZ..."
        
        # 2. 运行云台脚本
        python3 $PTZ_SCRIPT
        
        # 3. 等待云台稳定
        sleep $STABLE_TIME
        
        # 4. 运行采集程序
        echo "Point [$i/12]: Capturing for $CAPTURE_TIME s..."
        $PROGRAM "0" "1" "$BATCH_PATH" < /dev/tty &
        PID=$!
        
        # 5. 采集 1 秒
        sleep $CAPTURE_TIME
        
        # 6. 停止程序
        kill -TERM $PID
        wait $PID
        echo "Point [$i/12]: Done."
    done

    echo "Batch complete. 12 points saved in $BATCH_PATH"
    echo "Sleeping for $PAUSE_TIME seconds..."
    echo "============================================"
    
    sleep $PAUSE_TIME
done