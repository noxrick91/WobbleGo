"""
JY61P 姿态传感器数据采集
"""

import serial
import struct
import math
import time
from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class IMUData:
    """IMU原始数据"""
    # 加速度 (g)
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    # 角速度 (deg/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    # 欧拉角 (deg)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # 时间戳
    timestamp: float = 0.0


class JY61PReader:
    """JY61P姿态传感器读取器"""
    
    FRAME_HEAD = 0x55
    ACC_FRAME = 0x51
    GYRO_FRAME = 0x52
    ANGLE_FRAME = 0x53
    
    ACC_SCALE = 16.0 / 32768.0
    GYRO_SCALE = 2000.0 / 32768.0
    ANGLE_SCALE = 180.0 / 32768.0
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.data = IMUData()
        self._buffer = bytearray()
        
        # 角度零点偏移（校准后设置）
        self.angle_offset = 0.0
        
        # 互补滤波器状态
        self._comp_angle: Optional[float] = None  # 滤波后的角度
        self._last_timestamp: Optional[float] = None
        self._comp_alpha = 0.02  # 加速度计权重（越小越信任陀螺仪，动态时更准确）
        
        # 用于统计实际采样率
        self._sample_count = 0
        self._last_rate_time = time.time()
        
    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001  # 减小超时以支持200Hz
            )
            print(f"[JY61P] 连接成功: {self.port} @ {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"[JY61P] 连接失败: {e}")
            return False
    
    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[JY61P] 已断开连接")
    
    def _parse_frame(self, frame: bytes) -> bool:
        if len(frame) != 11 or frame[0] != self.FRAME_HEAD:
            return False
        
        checksum = sum(frame[:10]) & 0xFF
        if checksum != frame[10]:
            return False
        
        frame_type = frame[1]
        d0 = struct.unpack('<h', frame[2:4])[0]
        d1 = struct.unpack('<h', frame[4:6])[0]
        d2 = struct.unpack('<h', frame[6:8])[0]
        
        if frame_type == self.ACC_FRAME:
            self.data.acc_x = d0 * self.ACC_SCALE
            self.data.acc_y = d1 * self.ACC_SCALE
            self.data.acc_z = d2 * self.ACC_SCALE
        elif frame_type == self.GYRO_FRAME:
            self.data.gyro_x = d0 * self.GYRO_SCALE
            self.data.gyro_y = d1 * self.GYRO_SCALE
            self.data.gyro_z = d2 * self.GYRO_SCALE
        elif frame_type == self.ANGLE_FRAME:
            self.data.roll = d0 * self.ANGLE_SCALE
            self.data.pitch = d1 * self.ANGLE_SCALE
            self.data.yaw = d2 * self.ANGLE_SCALE
            self.data.timestamp = time.time()
            self._sample_count += 1
            return True
        
        return False
    
    def get_sample_rate(self) -> float:
        """获取实际采样率 (Hz)"""
        now = time.time()
        elapsed = now - self._last_rate_time
        if elapsed >= 1.0:
            rate = self._sample_count / elapsed
            self._sample_count = 0
            self._last_rate_time = now
            return rate
        return -1  # 还未到统计周期
    
    def update(self) -> bool:
        if not self.serial or not self.serial.is_open:
            return False
        
        available = self.serial.in_waiting
        if available > 0:
            self._buffer.extend(self.serial.read(available))
        
        got_complete_data = False
        while len(self._buffer) >= 11:
            try:
                head_idx = self._buffer.index(self.FRAME_HEAD)
                if head_idx > 0:
                    self._buffer = self._buffer[head_idx:]
            except ValueError:
                self._buffer.clear()
                break
            
            if len(self._buffer) < 11:
                break
            
            frame = bytes(self._buffer[:11])
            if self._parse_frame(frame):
                got_complete_data = True
            self._buffer = self._buffer[11:]
        
        if len(self._buffer) > 1000:
            self._buffer = self._buffer[-100:]
        
        return got_complete_data
    
    def get_pole_angle_rad(self) -> float:
        """使用互补滤波器计算摆杆角度 (-π 到 +π)
        
        互补滤波器融合：
        - 加速度计：低频准确（静态），但动态时被离心力/切向加速度污染
        - 陀螺仪：高频准确（动态），但会漂移
        
        angle = (1-alpha) * (prev_angle + gyro*dt) + alpha * accel_angle
        """
        # 加速度计角度（仅在静态时准确）
        acc_angle = math.atan2(self.data.acc_x, self.data.acc_y)
        
        # 陀螺仪角速度 (rad/s)
        gyro_rad = self.get_pole_velocity_rad()
        
        now = self.data.timestamp if self.data.timestamp > 0 else time.time()
        
        if self._comp_angle is None or self._last_timestamp is None:
            # 首次初始化，用加速度计角度
            self._comp_angle = acc_angle
            self._last_timestamp = now
        else:
            dt = now - self._last_timestamp
            if dt <= 0 or dt > 0.1:  # 防止异常 dt
                dt = 0.005  # 默认 200Hz
            self._last_timestamp = now
            
            # 陀螺仪积分预测
            gyro_prediction = self._comp_angle + gyro_rad * dt
            
            # 互补滤波：动态时主要信任陀螺仪，静态时缓慢校正到加速度计
            # 对角度差做周期性处理
            angle_diff = acc_angle - gyro_prediction
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            self._comp_angle = gyro_prediction + self._comp_alpha * angle_diff
        
        # 应用零点偏移
        angle = self._comp_angle - self.angle_offset
        
        # 归一化到 [-π, π]
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        return angle
    
    def get_pole_velocity_rad(self) -> float:
        """获取摆杆角速度 (rad/s)
        
        根据安装方式选择对应的陀螺仪轴：
        - Y轴朝下，如果摆动在 XY 平面，旋转轴是 Z → 使用 gyro_z
        - Y轴朝下，如果摆动在 ZY 平面，旋转轴是 X → 使用 gyro_x
        """
        gyro_deg_s = self.data.gyro_z
        return math.radians(gyro_deg_s)
    
    def get_observation(self) -> np.ndarray:
        """获取与仿真环境匹配的观测向量
        
        Returns:
            [cos(θ), sin(θ), pole_vel/10, flywheel_vel/50]
        """
        pole_angle = self.get_pole_angle_rad()
        pole_vel = self.get_pole_velocity_rad()
        
        obs = np.array([
            math.cos(pole_angle),
            math.sin(pole_angle),
            pole_vel / 10.0,
        ], dtype=np.float32)
        
        return obs
    
    def calibrate_zero(self, duration: float = 3.0):
        """零点校准 - 在下垂位置执行
        
        校准时摆杆静止，加速度计角度可靠，用于确定零点偏移。
        """
        print(f"零点校准中，请保持摆杆下垂静止... ({duration}秒)")
        
        angles = []
        start = time.time()
        while time.time() - start < duration:
            if self.update():
                # 校准时静止，加速度计角度可靠
                acc_x = self.data.acc_x
                acc_y = self.data.acc_y
                raw_angle = math.atan2(acc_x, acc_y)
                angles.append(raw_angle)
            time.sleep(0.001)  # 1ms循环，支持200Hz采样
        
        if angles:
            self.angle_offset = sum(angles) / len(angles)
            # 重置互补滤波器，以校准后的零点重新初始化
            self._comp_angle = None
            self._last_timestamp = None
            print(f"校准完成！零点偏移: {math.degrees(self.angle_offset):.2f}°, 采样数: {len(angles)}")
        else:
            print("校准失败：未收到数据")