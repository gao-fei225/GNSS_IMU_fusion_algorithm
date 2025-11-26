"""
时间对齐模块
负责将IMU和GNSS数据按时间对齐
"""

import numpy as np
from scipy import interpolate


class TimeAligner:
    """
    时间对齐器
    
    负责：
    - 以IMU采样时间为主时间轴
    - 对每个IMU时间点，关联或插值GNSS数据
    - 标记GNSS不可用时间点（缺失+人为遮挡）
    - 生成统一的时间序列数据
    
    输出数据格式：
    每个时间点包含：
    - timestamp: 时间戳
    - imu: IMU测量（gyro, accel）
    - gnss_nav: GNSS导航解（若有）
    - gnss_raw: GNSS原始观测（若有）
    - reference: 参考真值（若有）
    - gnss_available: GNSS可用标志
    - in_outage: 是否处于模拟遮挡期
    """
    
    def __init__(self):
        """
        初始化时间对齐器
        """
        self.aligned_data = None
    
    def align(self, imu_data, gnss_nav_data=None, gnss_raw_data=None, 
              reference_data=None, outage_periods=None):
        """
        对齐IMU和GNSS数据
        
        Args:
            imu_data: IMU数据字典
            gnss_nav_data: GNSS导航解数据（可选）
            gnss_raw_data: GNSS原始观测数据（可选）
            reference_data: 参考真值数据（可选）
            outage_periods: GNSS遮挡时间段列表，格式：[[t1_start, t1_end], [t2_start, t2_end], ...]
        
        Returns:
            list: 对齐后的数据序列
        """
        if imu_data is None or 'timestamp' not in imu_data:
            raise ValueError("IMU数据无效")
        
        # 以IMU时间戳为主时间轴
        imu_timestamps = imu_data['timestamp']
        n_samples = len(imu_timestamps)
        
        # 标记GNSS遮挡时间段
        outage_mask = self.mark_outages(imu_timestamps, outage_periods)
        
        # 初始化对齐数据列表
        aligned_data = []
        
        print(f"开始时间对齐，IMU数据点: {n_samples}")
        
        for i, ts in enumerate(imu_timestamps):
            data_point = {
                'timestamp': ts,
                'imu': {
                    'gyro': imu_data['gyro'][i],
                    'accel': imu_data['accel'][i]
                },
                'gnss_available': False,
                'in_outage': outage_mask[i]
            }
            
            # 对齐GNSS导航解（如果有）
            if gnss_nav_data is not None and not outage_mask[i]:
                gnss_nav = self.interpolate_gnss(ts, gnss_nav_data)
                if gnss_nav is not None:
                    data_point['gnss_nav'] = gnss_nav
                    data_point['gnss_available'] = True
            
            # 对齐GNSS原始观测（如果有）
            if gnss_raw_data is not None and not outage_mask[i]:
                gnss_raw = self._find_nearest_raw_obs(ts, gnss_raw_data)
                if gnss_raw is not None:
                    data_point['gnss_raw'] = gnss_raw
                    data_point['gnss_available'] = True
            
            # 对齐参考真值（如果有）
            if reference_data is not None:
                ref = self.interpolate_gnss(ts, reference_data)
                if ref is not None:
                    data_point['reference'] = ref
            
            aligned_data.append(data_point)
        
        self.aligned_data = aligned_data
        
        # 统计信息
        gnss_available_count = sum(1 for d in aligned_data if d['gnss_available'])
        outage_count = sum(1 for d in aligned_data if d['in_outage'])
        
        print(f"时间对齐完成:")
        print(f"  总数据点: {len(aligned_data)}")
        print(f"  GNSS可用: {gnss_available_count} ({gnss_available_count/len(aligned_data)*100:.1f}%)")
        print(f"  GNSS遮挡: {outage_count} ({outage_count/len(aligned_data)*100:.1f}%)")
        
        return aligned_data
    
    def interpolate_gnss(self, imu_timestamp, gnss_data):
        """
        对GNSS数据进行插值，获取指定IMU时间点的GNSS值
        
        Args:
            imu_timestamp: IMU时间戳
            gnss_data: GNSS数据序列（字典格式，包含timestamp、position、velocity等）
        
        Returns:
            插值后的GNSS数据，若无有效数据则返回None
        """
        if gnss_data is None or 'timestamp' not in gnss_data:
            return None
        
        gnss_timestamps = gnss_data['timestamp']
        
        # 检查IMU时间戳是否在GNSS时间范围内
        if imu_timestamp < gnss_timestamps[0] or imu_timestamp > gnss_timestamps[-1]:
            return None
        
        # 线性插值
        result = {}
        
        # 插值位置
        if 'position' in gnss_data:
            interp_func = interpolate.interp1d(gnss_timestamps, gnss_data['position'], 
                                              axis=0, kind='linear', fill_value='extrapolate')
            result['position'] = interp_func(imu_timestamp)
        
        # 插值速度
        if 'velocity' in gnss_data:
            interp_func = interpolate.interp1d(gnss_timestamps, gnss_data['velocity'], 
                                              axis=0, kind='linear', fill_value='extrapolate')
            result['velocity'] = interp_func(imu_timestamp)
        
        # 插值姿态（四元数，使用球面插值更好，这里简化为线性）
        if 'attitude' in gnss_data:
            interp_func = interpolate.interp1d(gnss_timestamps, gnss_data['attitude'], 
                                              axis=0, kind='linear', fill_value='extrapolate')
            result['attitude'] = interp_func(imu_timestamp)
            # 四元数归一化
            result['attitude'] = result['attitude'] / np.linalg.norm(result['attitude'])
        
        return result if result else None
    
    def mark_outages(self, timestamps, outage_periods):
        """
        标记GNSS遮挡时间段
        
        Args:
            timestamps: 时间戳数组
            outage_periods: 遮挡时间段列表，格式：[[t1_start, t1_end], [t2_start, t2_end], ...]
        
        Returns:
            布尔数组，True表示处于遮挡期
        """
        outage_mask = np.zeros(len(timestamps), dtype=bool)
        
        if outage_periods is None or len(outage_periods) == 0:
            return outage_mask
        
        # 标记每个遮挡时间段
        for period in outage_periods:
            if len(period) != 2:
                continue
            
            t_start, t_end = period
            # 找到在遮挡时间段内的所有时间点
            mask = (timestamps >= t_start) & (timestamps <= t_end)
            outage_mask |= mask
        
        return outage_mask
    
    def _find_nearest_raw_obs(self, imu_timestamp, gnss_raw_data, max_time_diff=0.1):
        """
        查找最接近IMU时间戳的GNSS原始观测
        
        Args:
            imu_timestamp: IMU时间戳
            gnss_raw_data: GNSS原始观测数据列表
            max_time_diff: 最大时间差阈值（秒）
        
        Returns:
            最接近的GNSS原始观测，若无满足条件的观测则返回None
        """
        if gnss_raw_data is None or len(gnss_raw_data) == 0:
            return None
        
        # 找到最接近的观测历元
        min_diff = float('inf')
        nearest_obs = None
        
        for obs in gnss_raw_data:
            time_diff = abs(obs['timestamp'] - imu_timestamp)
            if time_diff < min_diff:
                min_diff = time_diff
                nearest_obs = obs
        
        # 如果时间差超过阈值，返回None
        if min_diff > max_time_diff:
            return None
        
        return nearest_obs
