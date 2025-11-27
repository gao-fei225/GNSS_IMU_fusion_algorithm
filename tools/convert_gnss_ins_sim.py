"""
转换gnss-ins-sim数据集格式
将分散的CSV文件合并为统一的IMU数据格式
"""

import numpy as np
from pathlib import Path
import pandas as pd


def convert_gnss_ins_sim_data(data_dir, output_file=None):
    """
    转换gnss-ins-sim数据集为统一的IMU格式
    
    Args:
        data_dir: gnss-ins-sim数据集目录
        output_file: 输出文件路径（可选，默认在数据目录下生成imu_data.txt）
    
    Returns:
        输出文件路径
    """
    data_dir = Path(data_dir)
    
    print("=" * 60)
    print("转换gnss-ins-sim数据集")
    print("=" * 60)
    
    # 1. 读取时间戳
    print("\n[1/4] 读取时间戳...")
    time_file = data_dir / 'time.csv'
    time_data = pd.read_csv(time_file)
    timestamps = time_data.iloc[:, 0].values
    print(f"  时间范围: {timestamps[0]:.2f} - {timestamps[-1]:.2f} 秒")
    print(f"  数据点数: {len(timestamps)}")
    
    # 2. 读取陀螺仪数据（deg/s -> rad/s）
    print("\n[2/4] 读取陀螺仪数据...")
    gyro_file = data_dir / 'gyro-0.csv'
    gyro_data = pd.read_csv(gyro_file)
    gyro = gyro_data.values * np.pi / 180.0  # deg/s -> rad/s
    print(f"  陀螺仪数据形状: {gyro.shape}")
    print(f"  已转换单位: deg/s -> rad/s")
    
    # 3. 读取加速度计数据（m/s²）
    print("\n[3/4] 读取加速度计数据...")
    accel_file = data_dir / 'accel-0.csv'
    accel_data = pd.read_csv(accel_file)
    accel = accel_data.values
    print(f"  加速度计数据形状: {accel.shape}")
    print(f"  单位: m/s²")
    
    # 4. 合并并保存
    print("\n[4/4] 合并并保存...")
    
    # 确定输出文件
    if output_file is None:
        output_file = data_dir / 'imu_data.txt'
    else:
        output_file = Path(output_file)
    
    # 写入文件
    with open(output_file, 'w') as f:
        f.write("# IMU数据（从gnss-ins-sim转换）\n")
        f.write("# 坐标系: NED（北-东-地）\n")
        f.write("# timestamp gyro_x gyro_y gyro_z accel_x accel_y accel_z\n")
        f.write("# 单位: s, rad/s, rad/s, rad/s, m/s^2, m/s^2, m/s^2\n")
        
        for i in range(len(timestamps)):
            f.write(f"{timestamps[i]:.6f} ")
            f.write(f"{gyro[i, 0]:.10f} {gyro[i, 1]:.10f} {gyro[i, 2]:.10f} ")
            f.write(f"{accel[i, 0]:.10f} {accel[i, 1]:.10f} {accel[i, 2]:.10f}\n")
    
    print(f"  ✓ 已保存到: {output_file}")
    print(f"  文件大小: {output_file.stat().st_size / 1024:.1f} KB")
    
    # 5. 读取参考真值（可选）
    ref_pos_file = data_dir / 'ref_pos.csv'
    ref_vel_file = data_dir / 'ref_vel.csv'
    ref_att_file = data_dir / 'ref_att_euler.csv'
    
    if ref_pos_file.exists() and ref_vel_file.exists() and ref_att_file.exists():
        print("\n[额外] 转换参考真值...")
        ref_pos = pd.read_csv(ref_pos_file).values
        ref_vel = pd.read_csv(ref_vel_file).values
        ref_att = pd.read_csv(ref_att_file).values
        
        ref_output = data_dir / 'reference_truth.txt'
        with open(ref_output, 'w') as f:
            f.write("# 参考真值数据\n")
            f.write("# timestamp pos_x pos_y pos_z vel_x vel_y vel_z roll pitch yaw\n")
            f.write("# 单位: s, m, m, m, m/s, m/s, m/s, rad, rad, rad\n")
            
            for i in range(len(timestamps)):
                f.write(f"{timestamps[i]:.6f} ")
                f.write(f"{ref_pos[i, 0]:.6f} {ref_pos[i, 1]:.6f} {ref_pos[i, 2]:.6f} ")
                f.write(f"{ref_vel[i, 0]:.6f} {ref_vel[i, 1]:.6f} {ref_vel[i, 2]:.6f} ")
                # 欧拉角已经是弧度
                f.write(f"{ref_att[i, 0]:.10f} {ref_att[i, 1]:.10f} {ref_att[i, 2]:.10f}\n")
        
        print(f"  ✓ 参考真值已保存到: {ref_output}")
    
    print("\n" + "=" * 60)
    print("转换完成！")
    print("=" * 60)
    
    return str(output_file)


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("用法: python convert_gnss_ins_sim.py <数据集目录>")
        sys.exit(1)
    
    data_dir = sys.argv[1]
    output_file = convert_gnss_ins_sim_data(data_dir)
    
    print(f"\n现在可以运行纯INS解算：")
    print(f"python runners/run_pure_ins.py --imu {output_file} --output results/gnss_ins_sim")
