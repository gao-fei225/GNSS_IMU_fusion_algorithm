"""
转换Awesome GINS数据集格式
将增量型IMU数据转换为角速度和加速度
"""

import numpy as np
from pathlib import Path


def convert_awesome_gins_data(imu_file, output_file=None):
    """
    转换Awesome GINS数据集为标准IMU格式
    
    数据格式（增量型）：
    - 列1: GPS周秒（s）
    - 列2-4: 增量角度 x,y,z（rad）
    - 列5-7: 增量速度 x,y,z（m/s）
    
    Args:
        imu_file: IMU数据文件路径
        output_file: 输出文件路径（可选）
    
    Returns:
        输出文件路径
    """
    imu_file = Path(imu_file)
    
    print("=" * 60)
    print("转换Awesome GINS数据集")
    print("=" * 60)
    
    # 读取数据
    print("\n[1/3] 读取增量型IMU数据...")
    data = np.loadtxt(imu_file)
    
    gps_time = data[:, 0]
    delta_angle = data[:, 1:4]  # rad (列1-3)
    delta_velocity = data[:, 4:7]  # m/s (列4-6)
    
    print(f"  数据点数: {len(data)}")
    print(f"  时间范围: {gps_time[0]:.2f} - {gps_time[-1]:.2f} 秒")
    print(f"  GPS周秒: {gps_time[0]:.2f}")
    
    # 计算时间间隔
    dt = np.diff(gps_time)
    dt_mean = dt.mean()
    dt_std = dt.std()
    freq = 1.0 / dt_mean
    
    print(f"  平均采样间隔: {dt_mean*1000:.2f} ms")
    print(f"  采样频率: {freq:.1f} Hz")
    print(f"  时间间隔标准差: {dt_std*1000:.4f} ms")
    
    # 转换为角速度和加速度
    print("\n[2/3] 转换为角速度和加速度...")
    n = len(data)
    
    # 角速度 = 增量角度 / 时间间隔
    gyro = np.zeros((n-1, 3))
    accel = np.zeros((n-1, 3))
    timestamps = np.zeros(n-1)
    
    for i in range(n-1):
        gyro[i] = delta_angle[i] / dt[i]  # rad/s
        accel[i] = delta_velocity[i] / dt[i]  # m/s²
        timestamps[i] = gps_time[i] - gps_time[0]  # 转换为相对时间，从0开始
    
    print(f"  转换后数据点数: {len(gyro)}")
    print(f"  陀螺范围: [{gyro.min():.4f}, {gyro.max():.4f}] rad/s")
    print(f"  加速度范围: [{accel.min():.4f}, {accel.max():.4f}] m/s²")
    print(f"  加速度均值: {accel.mean():.2f} m/s²")
    
    # 保存
    print("\n[3/3] 保存转换后的数据...")
    if output_file is None:
        output_file = imu_file.parent / 'imu_data.txt'
    else:
        output_file = Path(output_file)
    
    with open(output_file, 'w') as f:
        f.write("# IMU数据（从Awesome GINS增量数据转换）\n")
        f.write("# 坐标系: FRD（前-右-下）\n")
        f.write("# timestamp gyro_x gyro_y gyro_z accel_x accel_y accel_z\n")
        f.write("# 单位: s, rad/s, rad/s, rad/s, m/s^2, m/s^2, m/s^2\n")
        
        for i in range(len(timestamps)):
            f.write(f"{timestamps[i]:.6f} ")
            f.write(f"{gyro[i, 0]:.10f} {gyro[i, 1]:.10f} {gyro[i, 2]:.10f} ")
            f.write(f"{accel[i, 0]:.10f} {accel[i, 1]:.10f} {accel[i, 2]:.10f}\n")
    
    print(f"  ✓ 已保存到: {output_file}")
    print(f"  文件大小: {output_file.stat().st_size / 1024:.1f} KB")
    
    print("\n" + "=" * 60)
    print("转换完成！")
    print("=" * 60)
    
    return str(output_file)


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("用法: python convert_awesome_gins.py <IMU数据文件>")
        print("\n示例:")
        print('  python convert_awesome_gins.py "data/多IMU车载GNSS、INS数据集/awesome-gins-datasets-main/ICM20602/ICM20602/ICM20602.txt"')
        sys.exit(1)
    
    imu_file = sys.argv[1]
    output_file = convert_awesome_gins_data(imu_file)
    
    print(f"\n现在可以运行纯INS解算：")
    print(f"python runners/run_pure_ins.py --imu {output_file} --output results/awesome_gins")
