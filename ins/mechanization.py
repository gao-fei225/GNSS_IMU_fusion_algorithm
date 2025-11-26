"""
INS编排算法
负责纯惯导解算，积分得到名义状态
"""

import numpy as np


class INSMechanization:
    """
    INS编排解算器
    
    负责：
    - 使用IMU数据进行姿态、速度、位置的积分
    - 维护名义状态（nominal state）
    - 作为ESKF的基础
    
    名义状态包含：
    - attitude: 姿态（四元数表示）
    - velocity: 速度（3维，导航坐标系）
    - position: 位置（3维，导航坐标系）
    
    每个时间步执行：
    1. 用角速度更新姿态（四元数积分）
    2. 将机体加速度旋转到导航坐标系
    3. 减去重力加速度
    4. 积分得到速度
    5. 积分得到位置
    """
    
    def __init__(self, initial_state, gravity=None):
        """
        初始化INS编排解算器
        
        Args:
            initial_state: 初始状态字典，包含
                - attitude: 初始姿态（四元数 [qw, qx, qy, qz]）
                - velocity: 初始速度 [vx, vy, vz]（m/s）
                - position: 初始位置 [x, y, z]（m）
            gravity: 重力向量（导航坐标系），默认为ENU坐标系的重力向量
        """
        self.attitude = np.array(initial_state['attitude'], dtype=np.float64)
        self.velocity = np.array(initial_state['velocity'], dtype=np.float64)
        self.position = np.array(initial_state['position'], dtype=np.float64)
        
        # 重力向量（导航坐标系，ENU: [0, 0, -9.81], NED: [0, 0, 9.81]）
        if gravity is None:
            self.gravity = np.array([0, 0, -9.81], dtype=np.float64)  # ENU坐标系
        else:
            self.gravity = np.array(gravity, dtype=np.float64)
        
        # 归一化四元数
        self.attitude = self.attitude / np.linalg.norm(self.attitude)
    
    def update(self, gyro, accel, dt):
        """
        使用IMU数据更新名义状态
        
        Args:
            gyro: 三轴角速度 [wx, wy, wz]（rad/s，机体坐标系）
            accel: 三轴加速度 [ax, ay, az]（m/s²，机体坐标系）
            dt: 时间间隔（秒）
        """
        gyro = np.array(gyro, dtype=np.float64)
        accel = np.array(accel, dtype=np.float64)
        
        # 1. 更新姿态（四元数积分）
        self.update_attitude(gyro, dt)
        
        # 2. 更新速度（将加速度转到导航系，减去重力，积分）
        self.update_velocity(accel, dt)
        
        # 3. 更新位置（积分速度）
        self.update_position(dt)
    
    def update_attitude(self, gyro, dt):
        """
        使用角速度更新姿态（四元数微分方程积分）
        
        四元数微分方程：dq/dt = 0.5 * Ω(ω) * q
        其中 Ω(ω) 是角速度的反对称矩阵形式
        
        Args:
            gyro: 三轴角速度 [wx, wy, wz]（rad/s，机体坐标系）
            dt: 时间间隔（秒）
        """
        wx, wy, wz = gyro
        
        # 构建四元数微分方程的Omega矩阵
        # Ω = [[0,   -wx,  -wy,  -wz],
        #      [wx,   0,    wz,  -wy],
        #      [wy,  -wz,   0,    wx],
        #      [wz,   wy,  -wx,   0]]
        Omega = np.array([
            [0,   -wx,  -wy,  -wz],
            [wx,   0,    wz,  -wy],
            [wy,  -wz,   0,    wx],
            [wz,   wy,  -wx,   0]
        ], dtype=np.float64)
        
        # 四元数微分：dq/dt = 0.5 * Omega * q
        q_dot = 0.5 * Omega @ self.attitude
        
        # 一阶欧拉积分：q(t+dt) = q(t) + dq/dt * dt
        self.attitude = self.attitude + q_dot * dt
        
        # 归一化四元数（保持单位长度）
        self.attitude = self.attitude / np.linalg.norm(self.attitude)
    
    def update_velocity(self, accel, dt):
        """
        使用加速度更新速度
        
        注意：加速度计测量的是比力（specific force），即 f = a - g
        其中 a 是真实加速度，g 是重力加速度
        要得到真实加速度：a = f + g
        
        步骤：
        1. 将机体系比力旋转到导航系
        2. 加上重力加速度得到真实加速度
        3. 积分得到速度变化
        
        Args:
            accel: 三轴比力 [fx, fy, fz]（m/s²，机体坐标系，加速度计读数）
            dt: 时间间隔（秒）
        """
        # 获取旋转矩阵（机体系到导航系）
        C_b_n = self.get_rotation_matrix()
        
        # 将机体系比力转换到导航系
        specific_force_nav = C_b_n @ accel
        
        # 加上重力加速度得到真实加速度
        # a = f + g（注意：这里加上重力，因为比力 f = a - g）
        accel_nav = specific_force_nav + self.gravity
        
        # 积分得到速度：v(t+dt) = v(t) + a * dt
        self.velocity = self.velocity + accel_nav * dt
    
    def update_position(self, dt):
        """
        使用速度更新位置
        
        积分速度得到位置变化：p(t+dt) = p(t) + v * dt
        
        Args:
            dt: 时间间隔（秒）
        """
        # 积分速度得到位置
        self.position = self.position + self.velocity * dt
    
    def get_state(self):
        """
        获取当前名义状态
        
        Returns:
            dict: 包含attitude、velocity、position的字典
        """
        return {
            'attitude': self.attitude.copy(),
            'velocity': self.velocity.copy(),
            'position': self.position.copy()
        }
    
    def get_rotation_matrix(self):
        """
        从四元数获取旋转矩阵（机体系到导航系）
        
        四元数 q = [qw, qx, qy, qz] 对应的旋转矩阵：
        C = [[1-2(qy²+qz²),  2(qx·qy-qw·qz),  2(qx·qz+qw·qy)],
             [2(qx·qy+qw·qz),  1-2(qx²+qz²),    2(qy·qz-qw·qx)],
             [2(qx·qz-qw·qy),  2(qy·qz+qw·qx),  1-2(qx²+qy²)]]
        
        Returns:
            np.ndarray: 3x3旋转矩阵（机体系到导航系）
        """
        qw, qx, qy, qz = self.attitude
        
        # 计算旋转矩阵的各个元素
        C = np.array([
            [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz),      1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy),      2*(qy*qz + qw*qx),      1 - 2*(qx**2 + qy**2)]
        ], dtype=np.float64)
        
        return C
    
    def get_euler_angles(self):
        """
        从四元数获取欧拉角（Roll-Pitch-Yaw）
        
        约定：ZYX旋转顺序（Yaw-Pitch-Roll）
        
        Returns:
            np.ndarray: [roll, pitch, yaw]（弧度）
        """
        qw, qx, qy, qz = self.attitude
        
        # Roll (x轴旋转)
        roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
        
        # Pitch (y轴旋转)
        sin_pitch = 2*(qw*qy - qz*qx)
        # 限制sin_pitch在[-1, 1]范围内，避免arcsin出错
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
        pitch = np.arcsin(sin_pitch)
        
        # Yaw (z轴旋转)
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        
        return np.array([roll, pitch, yaw])
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        
        Args:
            roll: 滚转角（弧度）
            pitch: 俯仰角（弧度）
            yaw: 偏航角（弧度）
        
        Returns:
            np.ndarray: 四元数 [qw, qx, qy, qz]
        """
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return np.array([qw, qx, qy, qz])
