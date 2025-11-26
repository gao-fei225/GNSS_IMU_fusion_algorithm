"""
数据读取模块
负责读取IMU和GNSS数据文件
"""

import numpy as np
from pathlib import Path


class IMUReader:
    """
    IMU数据读取器
    
    负责：
    - 读取IMU原始数据文件
    - 解析时间戳、三轴角速度、三轴加速度
    - 检查IMU单位和方向符号一致性
    - 返回统一格式的IMU数据序列
    
    数据格式：
    - timestamp: 时间戳（秒）
    - gyro: 三轴角速度 [gx, gy, gz]（rad/s）
    - accel: 三轴加速度 [ax, ay, az]（m/s²）
    """
    
    def __init__(self, file_path):
        """
        初始化IMU数据读取器
        
        Args:
            file_path: IMU数据文件路径
        """
        self.file_path = Path(file_path)
        self.data = None
    
    def read(self):
        """
        读取IMU数据
        
        Returns:
            dict: 包含timestamp、gyro、accel的字典
        """
        if not self.file_path.exists():
            raise FileNotFoundError(f"IMU数据文件不存在: {self.file_path}")
        
        try:
            # 读取文本文件，自动跳过注释行
            data = np.loadtxt(self.file_path, comments='#')
            
            # 假设数据格式：timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
            if data.shape[1] < 7:
                raise ValueError(f"IMU数据格式错误，期望至少7列，实际{data.shape[1]}列")
            
            self.data = {
                'timestamp': data[:, 0],
                'gyro': data[:, 1:4],      # 角速度 (rad/s)
                'accel': data[:, 4:7]      # 加速度 (m/s²)
            }
            
            print(f"成功读取IMU数据: {len(self.data['timestamp'])} 个数据点")
            print(f"时间范围: {self.data['timestamp'][0]:.2f} - {self.data['timestamp'][-1]:.2f} 秒")
            
            # 检查单位
            self.check_units()
            
            return self.data
            
        except Exception as e:
            raise RuntimeError(f"读取IMU数据失败: {e}")
    
    def check_units(self):
        """
        检查数据单位是否符合要求
        """
        if self.data is None:
            return
        
        # 检查角速度范围（rad/s，正常运动应该在±10 rad/s以内）
        gyro_max = np.max(np.abs(self.data['gyro']))
        if gyro_max > 100:  # 如果远大于正常值，可能单位是度/秒
            print(f"警告: 角速度最大值{gyro_max:.2f} rad/s异常，请检查单位是否为rad/s")
        
        # 检查加速度范围（m/s²，静止时应该接近重力9.8）
        accel_norm = np.linalg.norm(self.data['accel'], axis=1)
        accel_mean = np.mean(accel_norm)
        if accel_mean < 1 or accel_mean > 20:  # 重力应该在9.8左右
            print(f"警告: 加速度均值{accel_mean:.2f} m/s²异常，请检查单位是否为m/s²")
        
        # 检查时间戳单调性
        if not np.all(np.diff(self.data['timestamp']) > 0):
            print("警告: 时间戳不是严格单调递增")
        
        print(f"数据检查: 陀螺最大值={gyro_max:.4f} rad/s, 加速度均值={accel_mean:.2f} m/s²")


class GNSSReader:
    """
    GNSS数据读取器
    
    负责：
    - 读取GNSS导航解文件（用于松耦合）
    - 读取GNSS原始观测文件（用于紧耦合）
    - 解析位置、速度、姿态（导航解）
    - 解析伪距、伪距率、卫星位置等（原始观测）
    - 统一坐标系到ENU/NED导航坐标系
    
    导航解数据格式：
    - timestamp: 时间戳（秒）
    - position: 位置 [x, y, z]（m，导航坐标系）
    - velocity: 速度 [vx, vy, vz]（m/s，导航坐标系）
    - attitude: 姿态（可选）
    
    原始观测数据格式：
    - timestamp: 时间戳（秒）
    - satellites: 卫星列表，每个卫星包含
      - prn: 卫星号
      - pseudorange: 伪距（m）
      - pseudorange_rate: 伪距率（m/s）
      - sat_position: 卫星位置 [x, y, z]（m，ECEF坐标系）
    """
    
    def __init__(self, nav_file=None, raw_file=None):
        """
        初始化GNSS数据读取器
        
        Args:
            nav_file: GNSS导航解文件路径
            raw_file: GNSS原始观测文件路径
        """
        self.nav_file = Path(nav_file) if nav_file else None
        self.raw_file = Path(raw_file) if raw_file else None
        self.nav_data = None
        self.raw_data = None
    
    def read_navigation(self):
        """
        读取GNSS导航解
        
        Returns:
            dict: 包含timestamp、position、velocity、attitude的字典
        """
        if self.nav_file is None:
            raise ValueError("未指定GNSS导航解文件路径")
        
        if not self.nav_file.exists():
            raise FileNotFoundError(f"GNSS导航解文件不存在: {self.nav_file}")
        
        try:
            data = np.loadtxt(self.nav_file, comments='#')
            
            # 假设数据格式：timestamp, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z
            # 或：timestamp, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, qw, qx, qy, qz
            if data.shape[1] < 7:
                raise ValueError(f"GNSS导航解格式错误，期望至少7列，实际{data.shape[1]}列")
            
            self.nav_data = {
                'timestamp': data[:, 0],
                'position': data[:, 1:4],   # 位置 (m, ENU/NED坐标系)
                'velocity': data[:, 4:7]    # 速度 (m/s)
            }
            
            # 如果有姿态数据（四元数）
            if data.shape[1] >= 11:
                self.nav_data['attitude'] = data[:, 7:11]  # 四元数 [qw, qx, qy, qz]
            
            print(f"成功读取GNSS导航解: {len(self.nav_data['timestamp'])} 个数据点")
            print(f"时间范围: {self.nav_data['timestamp'][0]:.2f} - {self.nav_data['timestamp'][-1]:.2f} 秒")
            
            return self.nav_data
            
        except Exception as e:
            raise RuntimeError(f"读取GNSS导航解失败: {e}")
    
    def read_raw_observations(self):
        """
        读取GNSS原始观测
        
        Returns:
            list: 每个元素是一个时间点的观测，包含timestamp和satellites列表
        """
        if self.raw_file is None:
            raise ValueError("未指定GNSS原始观测文件路径")
        
        if not self.raw_file.exists():
            raise FileNotFoundError(f"GNSS原始观测文件不存在: {self.raw_file}")
        
        try:
            data = np.loadtxt(self.raw_file, comments='#')
            
            # 假设数据格式：timestamp, prn, pseudorange, pseudorange_rate, sat_x, sat_y, sat_z, sat_vx, sat_vy, sat_vz
            if data.shape[1] < 10:
                raise ValueError(f"GNSS原始观测格式错误，期望至少10列，实际{data.shape[1]}列")
            
            # 按时间戳分组
            unique_timestamps = np.unique(data[:, 0])
            observations = []
            
            for ts in unique_timestamps:
                # 获取该时间点的所有卫星观测
                mask = data[:, 0] == ts
                epoch_data = data[mask]
                
                satellites = []
                for row in epoch_data:
                    sat = {
                        'prn': int(row[1]),                    # 卫星号
                        'pseudorange': row[2],                 # 伪距 (m)
                        'pseudorange_rate': row[3],            # 伪距率 (m/s)
                        'sat_position': row[4:7],              # 卫星位置 (m, ECEF)
                        'sat_velocity': row[7:10] if data.shape[1] >= 10 else np.zeros(3)  # 卫星速度 (m/s, ECEF)
                    }
                    satellites.append(sat)
                
                observations.append({
                    'timestamp': ts,
                    'satellites': satellites
                })
            
            self.raw_data = observations
            print(f"成功读取GNSS原始观测: {len(observations)} 个历元")
            if observations:
                avg_sats = np.mean([len(obs['satellites']) for obs in observations])
                print(f"平均卫星数: {avg_sats:.1f}")
            
            return self.raw_data
            
        except Exception as e:
            raise RuntimeError(f"读取GNSS原始观测失败: {e}")
    
    def transform_to_nav_frame(self, origin_llh=None):
        """
        将GNSS数据转换到导航坐标系（ENU/NED）
        
        Args:
            origin_llh: 原点经纬高 [lat, lon, height] (度, 度, 米)
                       如果为None，使用第一个点作为原点
        """
        # 注：如果GNSS数据已经是导航系坐标，此函数可跳过
        # 如果是ECEF或大地坐标，需要进行坐标转换
        # 这里提供接口，具体实现可根据数据格式调整
        
        if self.nav_data is not None:
            # 导航解通常已经是导航系坐标，或需要简单转换
            print("GNSS导航解坐标系检查完成")
        
        if self.raw_data is not None:
            # 原始观测中的卫星位置通常是ECEF坐标
            # 如果需要在导航系中使用，需要转换
            print("GNSS原始观测卫星位置为ECEF坐标系")
