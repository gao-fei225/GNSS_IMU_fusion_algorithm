"""
数据IO模块测试脚本
测试IMU/GNSS数据读取和时间对齐功能
"""

import sys
from pathlib import Path
import numpy as np

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent.parent))

from io.data_reader import IMUReader, GNSSReader
from io.time_aligner import TimeAligner


def generate_test_data():
    """
    生成测试数据文件
    """
    print("=" * 60)
    print("生成测试数据")
    print("=" * 60)
    
    # 创建data目录
    data_dir = Path(__file__).parent.parent / 'data'
    data_dir.mkdir(exist_ok=True)
    
    # 生成IMU测试数据
    print("\n[1/3] 生成IMU测试数据...")
    duration = 10.0  # 10秒
    imu_rate = 100.0  # 100 Hz
    n_imu = int(duration * imu_rate)
    
    imu_time = np.linspace(0, duration, n_imu)
    # 模拟静止状态：小噪声 + 重力
    gyro_data = np.random.normal(0, 0.001, (n_imu, 3))  # 角速度噪声
    accel_data = np.random.normal([0, 0, 9.81], 0.01, (n_imu, 3))  # 加速度 + 重力
    
    imu_file = data_dir / 'test_imu.txt'
    with open(imu_file, 'w') as f:
        f.write("# IMU Test Data\n")
        f.write("# timestamp gyro_x gyro_y gyro_z accel_x accel_y accel_z\n")
        for i in range(n_imu):
            f.write(f"{imu_time[i]:.4f} {gyro_data[i,0]:.6f} {gyro_data[i,1]:.6f} {gyro_data[i,2]:.6f} ")
            f.write(f"{accel_data[i,0]:.6f} {accel_data[i,1]:.6f} {accel_data[i,2]:.6f}\n")
    
    print(f"✓ IMU数据已生成: {imu_file}")
    print(f"  数据点: {n_imu}, 频率: {imu_rate} Hz")
    
    # 生成GNSS导航解测试数据
    print("\n[2/3] 生成GNSS导航解测试数据...")
    gnss_rate = 1.0  # 1 Hz
    n_gnss = int(duration * gnss_rate) + 1
    
    gnss_time = np.linspace(0, duration, n_gnss)
    # 模拟简单运动：匀速直线
    position = np.column_stack([
        gnss_time * 10,  # x方向10 m/s
        gnss_time * 0.5,  # y方向0.5 m/s
        np.zeros(n_gnss)  # z保持不变
    ])
    velocity = np.array([[10.0, 0.5, 0.0]] * n_gnss)
    
    gnss_nav_file = data_dir / 'test_gnss_nav.txt'
    with open(gnss_nav_file, 'w') as f:
        f.write("# GNSS Navigation Solution Test Data\n")
        f.write("# timestamp pos_x pos_y pos_z vel_x vel_y vel_z\n")
        for i in range(n_gnss):
            f.write(f"{gnss_time[i]:.4f} {position[i,0]:.2f} {position[i,1]:.2f} {position[i,2]:.2f} ")
            f.write(f"{velocity[i,0]:.2f} {velocity[i,1]:.2f} {velocity[i,2]:.2f}\n")
    
    print(f"✓ GNSS导航解已生成: {gnss_nav_file}")
    print(f"  数据点: {n_gnss}, 频率: {gnss_rate} Hz")
    
    # 生成GNSS原始观测测试数据
    print("\n[3/3] 生成GNSS原始观测测试数据...")
    n_sats = 8  # 8颗卫星
    
    gnss_raw_file = data_dir / 'test_gnss_raw.txt'
    with open(gnss_raw_file, 'w') as f:
        f.write("# GNSS Raw Observations Test Data\n")
        f.write("# timestamp prn pseudorange pseudorange_rate sat_x sat_y sat_z sat_vx sat_vy sat_vz\n")
        
        for t in gnss_time:
            for sat_id in range(1, n_sats + 1):
                # 模拟卫星位置和观测
                sat_pos = np.array([20e6, 10e6 + sat_id * 1e6, 15e6])  # ECEF坐标
                pseudorange = 20e6 + np.random.normal(0, 1)  # 伪距 + 噪声
                pseudorange_rate = 100 + np.random.normal(0, 0.1)  # 伪距率 + 噪声
                sat_vel = np.array([1000, -500 + sat_id * 100, 200])  # 卫星速度
                
                f.write(f"{t:.4f} {sat_id} {pseudorange:.2f} {pseudorange_rate:.4f} ")
                f.write(f"{sat_pos[0]:.1f} {sat_pos[1]:.1f} {sat_pos[2]:.1f} ")
                f.write(f"{sat_vel[0]:.1f} {sat_vel[1]:.1f} {sat_vel[2]:.1f}\n")
    
    print(f"✓ GNSS原始观测已生成: {gnss_raw_file}")
    print(f"  历元数: {n_gnss}, 卫星数: {n_sats}")
    
    return imu_file, gnss_nav_file, gnss_raw_file


def test_imu_reader(imu_file):
    """
    测试IMU数据读取
    """
    print("\n" + "=" * 60)
    print("测试IMU数据读取")
    print("=" * 60)
    
    try:
        reader = IMUReader(imu_file)
        imu_data = reader.read()
        
        print(f"\n✓ IMU数据读取成功")
        print(f"  时间范围: {imu_data['timestamp'][0]:.2f} - {imu_data['timestamp'][-1]:.2f} 秒")
        print(f"  陀螺数据形状: {imu_data['gyro'].shape}")
        print(f"  加速度数据形状: {imu_data['accel'].shape}")
        
        return imu_data
        
    except Exception as e:
        print(f"✗ IMU数据读取失败: {e}")
        return None


def test_gnss_reader(gnss_nav_file, gnss_raw_file):
    """
    测试GNSS数据读取
    """
    print("\n" + "=" * 60)
    print("测试GNSS数据读取")
    print("=" * 60)
    
    try:
        reader = GNSSReader(nav_file=gnss_nav_file, raw_file=gnss_raw_file)
        
        # 读取导航解
        print("\n[1/2] 读取GNSS导航解...")
        gnss_nav = reader.read_navigation()
        print(f"✓ GNSS导航解读取成功")
        print(f"  位置数据形状: {gnss_nav['position'].shape}")
        print(f"  速度数据形状: {gnss_nav['velocity'].shape}")
        
        # 读取原始观测
        print("\n[2/2] 读取GNSS原始观测...")
        gnss_raw = reader.read_raw_observations()
        print(f"✓ GNSS原始观测读取成功")
        print(f"  历元数: {len(gnss_raw)}")
        if gnss_raw:
            print(f"  第一个历元卫星数: {len(gnss_raw[0]['satellites'])}")
        
        return gnss_nav, gnss_raw
        
    except Exception as e:
        print(f"✗ GNSS数据读取失败: {e}")
        return None, None


def test_time_aligner(imu_data, gnss_nav, gnss_raw):
    """
    测试时间对齐
    """
    print("\n" + "=" * 60)
    print("测试时间对齐")
    print("=" * 60)
    
    try:
        aligner = TimeAligner()
        
        # 定义遮挡时间段
        outage_periods = [[3.0, 5.0], [7.0, 8.0]]  # 3-5秒和7-8秒
        
        # 对齐数据
        aligned_data = aligner.align(
            imu_data=imu_data,
            gnss_nav_data=gnss_nav,
            gnss_raw_data=gnss_raw,
            outage_periods=outage_periods
        )
        
        print(f"\n✓ 时间对齐成功")
        print(f"  对齐后数据点: {len(aligned_data)}")
        
        # 检查数据结构
        sample = aligned_data[100]
        print(f"\n样本数据点 (t={sample['timestamp']:.2f}s):")
        print(f"  IMU可用: {'gyro' in sample['imu']}")
        print(f"  GNSS可用: {sample['gnss_available']}")
        print(f"  处于遮挡: {sample['in_outage']}")
        if 'gnss_nav' in sample:
            print(f"  GNSS位置: {sample['gnss_nav']['position']}")
        
        return aligned_data
        
    except Exception as e:
        print(f"✗ 时间对齐失败: {e}")
        return None


def cleanup_test_data():
    """
    清理测试数据
    """
    print("\n" + "=" * 60)
    print("清理测试数据")
    print("=" * 60)
    
    data_dir = Path(__file__).parent.parent / 'data'
    test_files = ['test_imu.txt', 'test_gnss_nav.txt', 'test_gnss_raw.txt']
    
    for filename in test_files:
        filepath = data_dir / filename
        if filepath.exists():
            filepath.unlink()
            print(f"✓ 已删除: {filename}")


def main():
    """
    主测试函数
    """
    print("\n" + "=" * 60)
    print("数据IO模块功能测试")
    print("=" * 60)
    
    # 1. 生成测试数据
    imu_file, gnss_nav_file, gnss_raw_file = generate_test_data()
    
    # 2. 测试IMU读取
    imu_data = test_imu_reader(imu_file)
    
    # 3. 测试GNSS读取
    gnss_nav, gnss_raw = test_gnss_reader(gnss_nav_file, gnss_raw_file)
    
    # 4. 测试时间对齐
    if imu_data is not None and gnss_nav is not None:
        aligned_data = test_time_aligner(imu_data, gnss_nav, gnss_raw)
    
    # 5. 清理测试数据
    cleanup_test_data()
    
    print("\n" + "=" * 60)
    print("所有测试完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
