#!/usr/bin/env python3
"""
AGX Orin - ESP32 Gimbal Control Module
双向串口通讯：发送控制命令 + 接收云台反馈数据
"""

import serial
import json
import time
import threading
from datetime import datetime
import sys
import os

class GimbalController:
    def __init__(self, port='/dev/ttyTHS0', baudrate=115200, timeout=1):
        """
        初始化 AGX Orin 与 ESP32 的串口通讯
        
        Args:
            port: 串口号 (AGX Orin 通常是 /dev/ttyTHS0, /dev/ttyTHS1 等)
            baudrate: 波特率 (默认 115200)
            timeout: 串口读超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.last_feedback = {}
        self.lock = threading.Lock()
        
        # 命令类型定义
        self.CMD_MODULE_TYPE = 4  # [NEW] 模块类型切换: 0=无, 1=机械臂, 2=云台
        self.CMD_TIME_SYNC = 150
        self.CMD_GIMBAL_CTRL_SIMPLE = 133
        self.CMD_GIMBAL_STEADY = 137
        self.CMD_GIMBAL_FIGURE8 = 850
        self.FEEDBACK_GIMBAL_AGX = 1007
        
        # 增量式八字绕行状态
        self.fig8_waypoints = []  # 八字轨迹关键点列表
        self.fig8_current_index = 0  # 当前在轨迹中的索引
        self.fig8_pause_until = 0  # 暂停截止时间
        self.fig8_params = {}  # 保存八字参数配置
        self.fig8_state_file = os.path.expanduser('~/.fig8_state.json')  # 状态持久化文件
        
    def connect(self):
        """连接到 ESP32"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # 等待 ESP32 初始化
            print(f"[INFO] Connected to ESP32 at {self.port}")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] Disconnected from ESP32")
    
    def send_command(self, cmd_dict):
        """
        发送 JSON 命令到 ESP32
        
        Args:
            cmd_dict: 命令字典，例如 {"T": 133, "X": 45, "Y": 30, "SPD": 0, "ACC": 0}
        
        Returns:
            True 如果发送成功，否则 False
        """
        if not self.ser or not self.ser.is_open:
            print("[ERROR] Serial port not open")
            return False
        
        try:
            cmd_json = json.dumps(cmd_dict) + '\n'
            self.ser.write(cmd_json.encode())
            print(f"[TX] {cmd_json.strip()}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")
            return False
    
    def sync_time(self):
        """
        同步 ESP32 时间戳到 AGX Orin 当前时间
        这保证了云台反馈数据中的时间戳与 AGX Orin 同步
        """
        timestamp_ms = int(time.time() * 1000)  # 转换为毫秒
        cmd = {
            "T": self.CMD_TIME_SYNC,
            "ts": timestamp_ms
        }
        return self.send_command(cmd)

    def set_module_type(self, module_type=2):
        """
        [NEW 2026-01-06] 设置模块类型（0: 无, 1: 机械臂, 2: 云台）。
        注意: ESP32固件已改为所有模式都推送1007反馈，此命令主要用于:
        - 控制云台自稳环路的启用/禁用
        - 切换不同硬件模块的控制逻辑
        
        Args:
            module_type: 0=无模块, 1=机械臂, 2=云台
        """
        cmd = {
            "T": self.CMD_MODULE_TYPE,
            "cmd": module_type,
        }
        return self.send_command(cmd)
    
    def control_start_8_rotation(self, duration=30):
        # Start: {"T":850,"cmd":1,"dur":30}  Stop: {"T":850,"cmd":0}
        cmd = {
            "T": self.CMD_GIMBAL_FIGURE8,
            "cmd": 1,
            "dur": duration,
        }
        return self.send_command(cmd)
    
    def control_stop_8_rotation(self):
        # Start: {"T":850,"cmd":1,"dur":30}  Stop: {"T":850,"cmd":0}
        cmd = {
            "T": self.CMD_GIMBAL_FIGURE8,
            "cmd": 0,
        }
        return self.send_command(cmd)

    

    def control_gimbal(self, pan_angle, tilt_angle, speed=0, acc=0):
        """
        控制云台位置
        
        Args:
            pan_angle: Pan 角度 (-180 to 180)
            tilt_angle: Tilt 角度 (-30 to 90)
            speed: 速度 (0-360)
            acc: 加速度 (0-360)
        
        Returns:
            True 如果发送成功
        """
        cmd = {
            "T": self.CMD_GIMBAL_CTRL_SIMPLE,
            "X": pan_angle,
            "Y": tilt_angle,
            "SPD": speed,
            "ACC": acc
        }
        return self.send_command(cmd)

    def ctrl_gimbal_steady(self, state=0, angle=0):
        """
        开启/关闭云台自稳定

        Args:
            state: 1 开启 / 0 关闭
            angle: 目标俯仰基准角（-45 ~ 90，超出范围在固件内被限幅）
        """
        cmd = {
            "T": self.CMD_GIMBAL_STEADY,  # 137
            "s": state,
            "y": angle,
        }
        return self.send_command(cmd)
    
    def figure8_pattern_with_pause(self, horizontal_amp=50, vertical_amp=30, pause_duration=3, 
                                   phase_pause_deg=30, speed=100, acc=0):
        """
        执行真正的八字绕行模式（Lissajous曲线），参考C++实现
        使用 Pan = A*sin(φ), Tilt = B*sin(2φ) 形成八字形轨迹
        
        Args:
            horizontal_amp: Pan幅度（默认70度）
            vertical_amp: Tilt幅度（默认30度）
            pause_duration: 每次暂停时长（秒，默认3秒）
            phase_pause_deg: 每隔多少度相位暂停一次（默认30度）
            speed: 云台移动速度（默认100）
            acc: 云台加速度（默认0）
        """
        import math
        
        print(f"[INFO] 开始八字绕行，Pan幅度={horizontal_amp}°，Tilt幅度={vertical_amp}°")
        print(f"[INFO] 每隔{phase_pause_deg}度暂停{pause_duration}秒")
        
        # 八字形参数（Lissajous曲线）
        fig8_center_pan = 0
        fig8_center_tilt = vertical_amp / 2  # 中心在Tilt的中点
        fig8_A_pan = horizontal_amp
        fig8_A_tilt = vertical_amp / 2
        
        # 相位参数
        phase = 0.0  # φ，从0到2π
        phase_step = math.radians(2)  # 每次增加2度相位
        phase_pause_rad = math.radians(phase_pause_deg)
        prev_phase_index = -1
        
        # 计时
        import time as time_module
        start_time = time_module.time()
        pause_until_time = 0
        
        try:
            print("[八字绕行] 开始轨迹...")
            # while phase < 2 * math.pi:
            while True:
                current_time = time_module.time() - start_time
                
                # 计算当前目标（Lissajous曲线）
                pan_cmd = fig8_center_pan + fig8_A_pan * math.sin(phase)
                tilt_cmd = fig8_center_tilt + fig8_A_tilt * math.sin(2 * phase)
                
                # 检查是否需要暂停（基于相位）
                if phase_pause_rad > 0:
                    current_phase_index = int(phase / phase_pause_rad)
                    
                    # 如果相位刚跨越暂停边界
                    if current_phase_index != prev_phase_index:
                        pause_until_time = current_time + pause_duration
                        prev_phase_index = current_phase_index
                        print(f"[八字绕行] 相位 {math.degrees(phase):.1f}° -> 暂停 {pause_duration}秒，目标: Pan={pan_cmd:.1f}°, Tilt={tilt_cmd:.1f}°")
                
                # 如果在暂停期间，保持当前目标不动
                if current_time < pause_until_time:
                    self.control_gimbal(pan_angle=pan_cmd, tilt_angle=tilt_cmd, speed=speed, acc=acc)
                else:
                    # 正常运动
                    self.control_gimbal(pan_angle=pan_cmd, tilt_angle=tilt_cmd, speed=speed, acc=acc)
                    phase += phase_step
                
                time_module.sleep(0.05)  # 50ms更新一次
            
            # 回到起点
            print("[八字绕行] 返回起点...")
            self.control_gimbal(pan_angle=0, tilt_angle=vertical_amp / 2, speed=speed, acc=acc)
            time_module.sleep(1)
            
            print("[INFO] 八字绕行完成")
            
        except Exception as e:
            print(f"[ERROR] 八字绕行出错: {e}")
    
    def init_figure8_incremental(self, horizontal_amp=50, vertical_amp=30, phase_pause_deg=30, 
                                 pause_duration=3, speed=100, acc=0, tolerance=5.0):
        """
        初始化增量式八字绕行（生成12个关键点：360 / 30 = 12）
        每次调用 figure8_incremental_step() 从一个关键点移动到下一个关键点
        
        Args:
            horizontal_amp: Pan幅度（默认50度）
            vertical_amp: Tilt幅度（默认30度）
            phase_pause_deg: 关键点间隔（度数，默认30度）- 用于生成12个点
            pause_duration: 每个关键点停留时长（秒，默认3秒）
            speed: 云台移动速度（默认100）
            acc: 云台加速度（默认0）
            tolerance: 匹配当前位置时的误差容限（度数，默认5.0度）
        """
        import math
        
        print(f"[INFO] 初始化增量式八字绕行")
        print(f"[INFO] Pan幅度={horizontal_amp}°，Tilt幅度={vertical_amp}°")
        print(f"[INFO] 关键点间隔={phase_pause_deg}°（共 {360/phase_pause_deg:.0f} 个点），每点停留={pause_duration}秒")
        
        # 保存参数
        self.fig8_params = {
            'horizontal_amp': horizontal_amp,
            'vertical_amp': vertical_amp,
            'phase_pause_deg': phase_pause_deg,
            'pause_duration': pause_duration,
            'speed': speed,
            'acc': acc,
            'tolerance': tolerance
        }
        
        # 八字形参数（Lissajous曲线）
        fig8_center_pan = 0
        fig8_center_tilt = vertical_amp / 2
        fig8_A_pan = horizontal_amp
        fig8_A_tilt = vertical_amp / 2
        
        # 生成关键点：每隔 phase_pause_deg 度生成一个点
        self.fig8_waypoints = []
        phase_step_rad = math.radians(phase_pause_deg)
        phase = 0.0
        
        while phase < 2 * math.pi:
            pan_cmd = fig8_center_pan + fig8_A_pan * math.sin(phase)
            tilt_cmd = fig8_center_tilt + fig8_A_tilt * math.sin(2 * phase)
            
            self.fig8_waypoints.append({
                'pan': round(pan_cmd, 1),
                'tilt': round(tilt_cmd, 1),
                'phase': phase,
                'phase_deg': round(math.degrees(phase), 1)
            })
            
            phase += phase_step_rad
        
        self.fig8_current_index = -1  # -1 表示还未初始化匹配
        self.fig8_pause_until = 0
        print(f"[INFO] 生成了 {len(self.fig8_waypoints)} 个关键点")
        
        # 加载持久化状态
        self._load_fig8_state()
    
    def _save_fig8_state(self):
        """保存八字绕行状态到文件"""
        try:
            state = {
                'current_index': self.fig8_current_index,
                'params': self.fig8_params,
                'timestamp': time.time()
            }
            with open(self.fig8_state_file, 'w') as f:
                json.dump(state, f, indent=2)
            print(f"[INFO] 状态已保存: 点{self.fig8_current_index + 1}/{len(self.fig8_waypoints)}")
        except Exception as e:
            print(f"[WARNING] 保存状态失败: {e}")
    
    def _load_fig8_state(self):
        """从文件加载八字绕行状态"""
        try:
            if os.path.exists(self.fig8_state_file):
                with open(self.fig8_state_file, 'r') as f:
                    state = json.load(f)
                    # 验证索引有效性
                    if 0 <= state.get('current_index', 0) < len(self.fig8_waypoints):
                        self.fig8_current_index = state.get('current_index', 0)
                        print(f"[INFO] 从状态文件恢复: 当前点 {self.fig8_current_index + 1}/{len(self.fig8_waypoints)}")
                    else:
                        self.fig8_current_index = 0
                        print(f"[INFO] 状态索引无效，重置为起点")
        except Exception as e:
            print(f"[WARNING] 加载状态失败: {e}")
            self.fig8_current_index = 0
    
    def figure8_incremental_step(self):
        """
        增量式执行八字绕行，每调用一次移动到下一个关键点
        需要先调用 init_figure8_incremental() 进行初始化
        
        **重要：** 停留时间由调用者通过 sleep() 控制，不由本函数内部处理
        
        使用示例：
            controller.init_figure8_incremental(...)
            for i in range(24):  # 执行两圈（12个点 × 2）
                result = controller.figure8_incremental_step()
                if result is False:
                    break
                # 调用者控制停留时间
                time.sleep(controller.fig8_params.get('pause_duration', 3))
        
        Returns:
            dict: 包含当前关键点信息，如 {'point_no': 1, 'pan': 0.0, 'tilt': 15.0, ...}
            False 如果已完成一圈（返回到起点）
        """
        import math
        
        if not self.fig8_waypoints:
            print("[ERROR] 请先调用 init_figure8_incremental() 初始化八字轨迹")
            return False
        
        # 第一次调用：初始化，找到最接近当前位置的关键点
        if self.fig8_current_index == -1:
            feedback = self.get_last_feedback()
            if not feedback:
                current_pan = 0
                current_tilt = 0
            else:
                current_pan = feedback.get('pan', 0)
                current_tilt = feedback.get('tilt', 0)
            
            tolerance = self.fig8_params.get('tolerance', 5.0)
            min_distance = float('inf')
            closest_idx = 0
            
            # 找最接近的点
            for idx, point in enumerate(self.fig8_waypoints):
                distance = math.sqrt((point['pan'] - current_pan)**2 + (point['tilt'] - current_tilt)**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_idx = idx
            
            # 初始化：从最接近的点开始（设为当前索引）
            self.fig8_current_index = closest_idx
            
            if min_distance <= tolerance:
                print(f"[八字绕行] 初始化成功：当前位置 Pan={current_pan:.1f}°, Tilt={current_tilt:.1f}° ")
                print(f"[八字绕行] 匹配到第{self.fig8_current_index+1}个关键点（距离={min_distance:.2f}°）")
            else:
                print(f"[WARNING] 当前位置距离轨迹点较远（距离={min_distance:.2f}°，容限={tolerance}°）")
                print(f"[WARNING] 使用最接近的关键点：第{self.fig8_current_index+1}个")
        
        # 检查是否已完成一圈
        if self.fig8_current_index >= len(self.fig8_waypoints):
            print("[INFO] 增量式八字绕行已完成一圈，重置到起点")
            self.fig8_current_index = 0
            return False
        
        # 获取当前关键点
        current_point = self.fig8_waypoints[self.fig8_current_index]
        point_no = self.fig8_current_index + 1  # 点号（1-12）
        
        # 移动到当前关键点
        print(f"[八字绕行] 移动到关键点 {point_no}/{len(self.fig8_waypoints)}: "
              f"Pan={current_point['pan']:.1f}°, Tilt={current_point['tilt']:.1f}° "
              f"(相位={current_point['phase_deg']:.0f}°)")
        
        self.control_gimbal(
            pan_angle=current_point['pan'],
            tilt_angle=current_point['tilt'],
            speed=self.fig8_params.get('speed', 100),
            acc=self.fig8_params.get('acc', 0)
        )
        
        # 构建返回信息
        result = {
            'index': self.fig8_current_index,
            'point_no': point_no,
            'pan': current_point['pan'],
            'tilt': current_point['tilt'],
            'total_points': len(self.fig8_waypoints),
            'phase': current_point['phase_deg'],
            'pause_duration': self.fig8_params.get('pause_duration', 3)
        }
        
        # 前进到下一个点
        self.fig8_current_index += 1
        
        # 保存状态
        self._save_fig8_state()
        
        return result
    
    def read_feedback(self):
        """
        读取 ESP32 的云台反馈数据（非阻塞）
        
        Returns:
            反馈数据字典，如果没有数据返回 None
        """
        if not self.ser or not self.ser.is_open:
            return None
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    feedback = json.loads(line)
                    with self.lock:
                        self.last_feedback = feedback
                    return feedback
        except Exception as e:
            print(f"[ERROR] Failed to read feedback: {e}")
        
        return None
    
    def start_feedback_listener(self):
        """启动后台线程持续读取反馈数据"""
        self.running = True
        thread = threading.Thread(target=self._feedback_loop, daemon=True)
        thread.start()
        print("[INFO] Feedback listener started")
    
    def _feedback_loop(self):
        """后台循环读取反馈数据"""
        while self.running:
            feedback = self.read_feedback()
            if feedback:
                self._process_feedback(feedback)
            time.sleep(0.001)  # 1ms 的读取周期
    
    def _process_feedback(self, feedback):
        """处理反馈数据"""
        feedback_type = feedback.get("T")
        
        if feedback_type == self.FEEDBACK_GIMBAL_AGX:  # 1007
            # 提取云台反馈
            timestamp = feedback.get("ts", 0)
            pan = feedback.get("pan", 0)
            tilt = feedback.get("tilt", 0)
            roll = feedback.get("r", 0)
            pitch = feedback.get("p", 0)
            yaw = feedback.get("y", 0)
            voltage = feedback.get("v", 0)
            
            print(f"[RX] Type: 1007 | TS: {timestamp} | Pan: {pan:.2f}° | Tilt: {tilt:.2f}° | "
                  f"RPY: ({roll:.2f}, {pitch:.2f}, {yaw:.2f}) | V: {voltage:.2f}V")
    
    def get_last_feedback(self):
        """获取最后一条反馈数据"""
        with self.lock:
            return self.last_feedback.copy() if self.last_feedback else None
    
    def wait_for_feedback(self, timeout=5):
        """
        等待接收反馈数据
        
        Args:
            timeout: 超时时间（秒）
        
        Returns:
            反馈数据字典
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            feedback = self.get_last_feedback()
            if feedback:
                return feedback
            time.sleep(0.01)
        return None


# ============================================================================
# 使用示例
# ============================================================================

def main():
    # 创建控制器实例
    controller = GimbalController(port='/dev/ttyTHS1')  # 根据实际修改端口
    
    # 连接到 ESP32
    if not controller.connect():
        sys.exit(1)
    
    # 启动后台反馈监听线程
    controller.start_feedback_listener()
    
    # 切换到云台模块模式以开启 1007 反馈
    controller.set_module_type(module_type=2)
    time.sleep(0.2)

    # 云台回中
    print("\n=== 云台回中 ===")
    controller.control_gimbal(pan_angle=0, tilt_angle=0, speed=0, acc=0)
    
    try:
        # 同步时间戳
        print("\n=== 同步时间戳 ===")
        controller.sync_time()
        time.sleep(0.5)
        
        # 等待反馈
        print("\n=== 监听云台反馈（10秒） ===")
        feedback = controller.wait_for_feedback(timeout=1)

        if feedback:
            print(f"收到反馈: {feedback}")
        
        # # 执行八字绕行，每隔30度停留3秒
        # print("\n=== 执行八字绕行模式 ===")
        # controller.figure8_pattern_with_pause(horizontal_amp=70, vertical_amp=30, pause_duration=3, phase_pause_deg=30)
        # time.sleep(2)
        
        # 增量式八字绕行示例（可选）
        print("\n=== 执行增量式八字绕行模式 ===")
        controller.init_figure8_incremental(horizontal_amp=50, vertical_amp=30, phase_pause_deg=30, pause_duration=3, tolerance=3.0)
        time.sleep(1)
        
        # 逐步执行八字绕行（每次调用移动到下一个关键点）
        for i in range(30):  # 最多执行30步（12个点 + 余量）
            result = controller.figure8_incremental_step()
            if result is False:
                print("[INFO] 八字绕行步序完成")
                break
            if isinstance(result, dict):
                print(f"[步骤{i+1}] 当前在关键点 {result['point_no']}/{result['total_points']}")
            time.sleep(5)  # 每5秒执行一步
        
        time.sleep(2)
        
        # 云台回中
        print("\n=== 云台回中 ===")
        controller.control_gimbal(pan_angle=0, tilt_angle=0, speed=0, acc=0)
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted")
    finally:
        controller.disconnect()


if __name__ == '__main__':
    # 支持命令行参数: init, step, status, reset
    command = sys.argv[1] if len(sys.argv) > 1 else 'step'
    
    controller = GimbalController(port='/dev/ttyTHS1')
    
    try:
        if command == 'init':
            # 初始化模式
            print("[命令] 初始化增量式八字绕行")
            controller.connect()
            controller.start_feedback_listener()
            controller.set_module_type(module_type=2)
            time.sleep(0.2)
            
            # 从命令行参数获取参数，或使用默认值
            h_amp = float(sys.argv[2]) if len(sys.argv) > 2 else 50
            v_amp = float(sys.argv[3]) if len(sys.argv) > 3 else 30
            p_pause = float(sys.argv[4]) if len(sys.argv) > 4 else 30
            p_duration = float(sys.argv[5]) if len(sys.argv) > 5 else 3
            
            controller.init_figure8_incremental(
                horizontal_amp=h_amp,
                vertical_amp=v_amp,
                phase_pause_deg=p_pause,
                pause_duration=p_duration,
                tolerance=3.0
            )
            time.sleep(0.5)
            
        elif command == 'step':
            # 执行一步模式（默认命令）
            print("[命令] 执行八字绕行一步")
            controller.connect()
            controller.start_feedback_listener()
            controller.set_module_type(module_type=2)
            # time.sleep(0.2)
            
            # 初始化（如果还没有的话）
            if not controller.fig8_waypoints:
                controller.init_figure8_incremental(
                    horizontal_amp=50,
                    vertical_amp=30,
                    phase_pause_deg=30,
                    pause_duration=3,
                    tolerance=3.0
                )
                # time.sleep(0.5)
            
            # 执行一步
            result = controller.figure8_incremental_step()
            if result:
                print(f"[执行完成] 关键点 {result['point_no']}/{result['total_points']}")
                print(f"  Pan={result['pan']:.1f}°, Tilt={result['tilt']:.1f}°")
                print(f"  建议停留 {result['pause_duration']} 秒")
            else:
                print("[执行完成] 八字绕行已完成一圈")
                
        elif command == 'status':
            # 状态查询模式
            print("[命令] 查询八字绕行状态")
            try:
                if os.path.exists(controller.fig8_state_file):
                    with open(controller.fig8_state_file, 'r') as f:
                        state = json.load(f)
                        current_idx = state.get('current_index', 0)
                        print(f"当前点位: {current_idx + 1}/12")
                        print(f"参数: {state.get('params', {})}")
                else:
                    print("状态文件不存在，请先执行 'init' 命令")
            except Exception as e:
                print(f"查询状态失败: {e}")
                
        elif command == 'reset':
            # 重置状态
            print("[命令] 重置八字绕行状态")
            try:
                if os.path.exists(controller.fig8_state_file):
                    os.remove(controller.fig8_state_file)
                    print("状态已重置")
                else:
                    print("状态文件不存在")
            except Exception as e:
                print(f"重置状态失败: {e}")
        
        elif command == 'continuation':
            print("[命令] 执行连续八字绕行")
            controller.connect()
            controller.start_feedback_listener()
            controller.set_module_type(module_type=2)
            controller.figure8_pattern_with_pause(horizontal_amp=30,
                                                    vertical_amp=20,
                                                    pause_duration=0, 
                                                    phase_pause_deg=30,
                                                    speed=100, acc=0)
                
        else:
            print(f"未知命令: {command}")
            print("\n支持的命令:")
            print("  init [h_amp] [v_amp] [phase] [duration]  - 初始化八字绕行")
            print("  step                                     - 执行一步（默认）")
            print("  status                                   - 查询当前状态")
            print("  reset                                    - 重置状态")
            print("  continuation                             - 连续绕行")
            
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted")
    finally:
        controller.control_stop_8_rotation()
        controller.disconnect()
