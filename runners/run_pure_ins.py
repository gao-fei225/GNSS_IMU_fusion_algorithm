"""
纯INS实验脚本
仅使用IMU进行惯导解算
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# 动态加载模块避免命名冲突
import importlib.util

def load_module(module_name, file_path):
    """动态加载模块"""
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# 加载必要的模块
config_module = load_module("config_manager", project_root / "config" / "config_manager.py")
data_reader_module = load_module("data_reader", project_root / "io" / "data_reader.py")
mechanization_module = load_module("mechanization", project_root / "ins" / "mechanization.py")

ConfigManager = config_module.ConfigManager
IMUReader = data_reader_module.IMUReader
GNSSReader = data_reader_module.GNSSReader
INSMechanization = mechanization_module.INSMechanization


def run_pure_ins(imu_file, output_dir=None, initial_state=None, reference_file=None):
    """
    运行纯INS解算
    
    Args:
        imu_file: IMU数据文件路径
        output_dir: 输出目录（保存结果和图表）
        initial_state: 初始状态字典，包含attitude、velocity、position
        reference_file: 参考轨迹文件（可选，用于误差评估）
    
    Returns:
        dict: 包含轨迹数据的字典
    """
    print("\n" + "=" * 60)
    print("纯INS惯导解算")
    print("=" * 60)
    
    # 1. 读取IMU数据
    print("\n[1/4] 读取IMU数据...")
    reader = IMUReader(imu_file)
    imu_data = reader.read()
    
    n_samples = len(imu_data['timestamp'])
    print(f"  数据点数: {n_samples}")
    print(f"  时间范围: {imu_data['timestamp'][0]:.2f} - {imu_data['timestamp'][-1]:.2f} 秒")
    
    # 2. 初始化INS
    print("\n[2/4] 初始化INS解算器...")
    if initial_state is None:
        # 默认初始状态：原点，静止，水平姿态
        initial_state = {
            'attitude': np.array([1.0, 0.0, 0.0, 0.0]),  # 四元数[qw, qx, qy, qz]
            'velocity': np.array([0.0, 0.0, 0.0]),       # 速度[vx, vy, vz] m/s
            'position': np.array([0.0, 0.0, 0.0])        # 位置[x, y, z] m
        }
        print("  使用默认初始状态（原点、静止、水平）")
    else:
        print("  使用用户指定的初始状态")
    
    ins = INSMechanization(initial_state)
    print(f"  初始位置: {ins.position}")
    print(f"  初始速度: {ins.velocity}")
    print(f"  初始姿态（欧拉角，度）: {np.rad2deg(ins.get_euler_angles())}")
    
    # 3. INS积分
    print("\n[3/4] 执行INS积分...")
    timestamps = imu_data['timestamp']
    
    # 存储轨迹
    trajectory = {
        'timestamp': [],
        'position': [],
        'velocity': [],
        'attitude_quat': [],
        'attitude_euler': []
    }
    
    # 记录初始状态
    trajectory['timestamp'].append(timestamps[0])
    trajectory['position'].append(ins.position.copy())
    trajectory['velocity'].append(ins.velocity.copy())
    trajectory['attitude_quat'].append(ins.attitude.copy())
    trajectory['attitude_euler'].append(ins.get_euler_angles().copy())
    
    # 逐步积分
    for i in range(1, n_samples):
        dt = timestamps[i] - timestamps[i-1]
        gyro = imu_data['gyro'][i]
        accel = imu_data['accel'][i]
        
        # 更新INS状态
        ins.update(gyro, accel, dt)
        
        # 记录轨迹
        trajectory['timestamp'].append(timestamps[i])
        trajectory['position'].append(ins.position.copy())
        trajectory['velocity'].append(ins.velocity.copy())
        trajectory['attitude_quat'].append(ins.attitude.copy())
        trajectory['attitude_euler'].append(ins.get_euler_angles().copy())
        
        # 进度显示
        if (i+1) % (n_samples // 10) == 0:
            progress = (i+1) / n_samples * 100
            print(f"  进度: {progress:.0f}%")
    
    # 转换为numpy数组
    for key in trajectory:
        if key != 'timestamp':
            trajectory[key] = np.array(trajectory[key])
    
    print("  ✓ INS积分完成")
    print(f"  最终位置: {trajectory['position'][-1]}")
    print(f"  位移范围: X[{trajectory['position'][:, 0].min():.1f}, {trajectory['position'][:, 0].max():.1f}] m")
    print(f"            Y[{trajectory['position'][:, 1].min():.1f}, {trajectory['position'][:, 1].max():.1f}] m")
    print(f"            Z[{trajectory['position'][:, 2].min():.1f}, {trajectory['position'][:, 2].max():.1f}] m")
    
    # 4. 可视化
    print("\n[4/4] 生成可视化图表...")
    plot_trajectory(trajectory, output_dir)
    
    # 5. 保存结果
    if output_dir:
        save_results(trajectory, output_dir)
    
    print("\n" + "=" * 60)
    print("纯INS解算完成！")
    print("=" * 60)
    
    return trajectory


def plot_trajectory(trajectory, output_dir=None):
    """
    绘制轨迹图表
    """
    timestamps = trajectory['timestamp']
    positions = trajectory['position']
    velocities = trajectory['velocity']
    euler_angles = np.rad2deg(trajectory['attitude_euler'])
    
    # 创建图表
    fig = plt.figure(figsize=(15, 10))
    
    # 1. 2D轨迹（俯视图）
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1)
    ax1.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='起点')
    ax1.plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=10, label='终点')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('2D轨迹（俯视图）')
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')
    
    # 2. 位置随时间变化
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(timestamps, positions[:, 0], 'r-', label='X')
    ax2.plot(timestamps, positions[:, 1], 'g-', label='Y')
    ax2.plot(timestamps, positions[:, 2], 'b-', label='Z')
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('位置 (m)')
    ax2.set_title('位置随时间变化')
    ax2.grid(True)
    ax2.legend()
    
    # 3. 速度随时间变化
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(timestamps, velocities[:, 0], 'r-', label='Vx')
    ax3.plot(timestamps, velocities[:, 1], 'g-', label='Vy')
    ax3.plot(timestamps, velocities[:, 2], 'b-', label='Vz')
    ax3.set_xlabel('时间 (s)')
    ax3.set_ylabel('速度 (m/s)')
    ax3.set_title('速度随时间变化')
    ax3.grid(True)
    ax3.legend()
    
    # 4. 姿态角随时间变化
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(timestamps, euler_angles[:, 0], 'r-', label='Roll')
    ax4.plot(timestamps, euler_angles[:, 1], 'g-', label='Pitch')
    ax4.plot(timestamps, euler_angles[:, 2], 'b-', label='Yaw')
    ax4.set_xlabel('时间 (s)')
    ax4.set_ylabel('姿态角 (度)')
    ax4.set_title('姿态角随时间变化')
    ax4.grid(True)
    ax4.legend()
    
    # 5. 3D轨迹
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    ax5.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=1)
    ax5.plot([positions[0, 0]], [positions[0, 1]], [positions[0, 2]], 'go', markersize=10, label='起点')
    ax5.plot([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]], 'ro', markersize=10, label='终点')
    ax5.set_xlabel('X (m)')
    ax5.set_ylabel('Y (m)')
    ax5.set_zlabel('Z (m)')
    ax5.set_title('3D轨迹')
    ax5.legend()
    
    # 6. 速度大小随时间变化
    ax6 = plt.subplot(2, 3, 6)
    speed = np.linalg.norm(velocities, axis=1)
    ax6.plot(timestamps, speed, 'b-')
    ax6.set_xlabel('时间 (s)')
    ax6.set_ylabel('速度大小 (m/s)')
    ax6.set_title('速度大小随时间变化')
    ax6.grid(True)
    
    plt.tight_layout()
    
    # 保存图表
    if output_dir:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        fig_path = output_path / 'pure_ins_trajectory.png'
        plt.savefig(fig_path, dpi=300, bbox_inches='tight')
        print(f"  ✓ 图表已保存: {fig_path}")
    
    plt.show()


def save_results(trajectory, output_dir):
    """
    保存结果到文件
    """
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # 保存轨迹数据
    result_file = output_path / 'pure_ins_trajectory.txt'
    with open(result_file, 'w') as f:
        f.write("# 纯INS解算结果\n")
        f.write("# timestamp pos_x pos_y pos_z vel_x vel_y vel_z roll pitch yaw\n")
        
        timestamps = trajectory['timestamp']
        positions = trajectory['position']
        velocities = trajectory['velocity']
        euler_angles = trajectory['attitude_euler']
        
        for i in range(len(timestamps)):
            f.write(f"{timestamps[i]:.4f} ")
            f.write(f"{positions[i, 0]:.4f} {positions[i, 1]:.4f} {positions[i, 2]:.4f} ")
            f.write(f"{velocities[i, 0]:.4f} {velocities[i, 1]:.4f} {velocities[i, 2]:.4f} ")
            f.write(f"{euler_angles[i, 0]:.6f} {euler_angles[i, 1]:.6f} {euler_angles[i, 2]:.6f}\n")
    
    print(f"  ✓ 结果已保存: {result_file}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='纯INS惯导解算')
    parser.add_argument('--imu', type=str, required=True, help='IMU数据文件路径')
    parser.add_argument('--output', type=str, default='results/pure_ins', help='输出目录')
    parser.add_argument('--ref', type=str, default=None, help='参考轨迹文件（可选）')
    
    args = parser.parse_args()
    
    # 运行纯INS解算
    trajectory = run_pure_ins(
        imu_file=args.imu,
        output_dir=args.output,
        reference_file=args.ref
    )
