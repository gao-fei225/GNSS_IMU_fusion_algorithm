"""
松耦合ESKF实验脚本
使用GNSS导航解修正INS
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
error_dynamics_module = load_module("error_dynamics", project_root / "models" / "error_dynamics.py")
measurement_models_module = load_module("measurement_models", project_root / "models" / "measurement_models.py")
eskf_module = load_module("eskf", project_root / "fusion" / "eskf.py")

ConfigManager = config_module.ConfigManager
IMUReader = data_reader_module.IMUReader
GNSSReader = data_reader_module.GNSSReader
INSMechanization = mechanization_module.INSMechanization
ErrorDynamics = error_dynamics_module.ErrorDynamics
LooseCouplingModel = measurement_models_module.LooseCouplingModel
ESKF = eskf_module.ESKF


# 直接定义LooseCouplingESKF类
class LooseCouplingESKF(ESKF):
    """松耦合ESKF"""
    
    def __init__(self, initial_cov, error_dynamics, measurement_model):
        super().__init__(initial_cov, error_dynamics)
        self.measurement_model = measurement_model
    
    def process_gnss_measurement(self, nominal_state, gnss_nav, gyro_bias=None, accel_bias=None):
        """处理GNSS导航解量测"""
        # 计算量测新息
        innovation = self.measurement_model.compute_innovation(nominal_state, gnss_nav)
        
        # 计算量测矩阵H
        H = self.measurement_model.compute_H_matrix(nominal_state)
        
        # 计算量测噪声协方差R
        R = self.measurement_model.compute_R_matrix()
        
        # 执行ESKF更新
        self.update(innovation, H, R)
        
        # 修正名义状态和零偏
        corrected_state, corrected_gyro_bias, corrected_accel_bias = \
            self.correct_nominal_state(nominal_state, gyro_bias, accel_bias)
        
        return corrected_state, corrected_gyro_bias, corrected_accel_bias


def run_loose_coupling(imu_file, gnss_file, output_dir=None, initial_state=None, 
                      reference_file=None, navigation_frame='ENU', 
                      use_alignment=None, alignment_truth=None):
    """
    运行松耦合ESKF实验
    
    Args:
        imu_file: IMU数据文件路径
        gnss_file: GNSS导航解文件路径
        output_dir: 输出目录
        initial_state: 初始状态
        reference_file: 参考轨迹文件
        navigation_frame: 导航坐标系
        use_alignment: 对齐方式
        alignment_truth: 真值文件路径
    
    Returns:
        dict: 融合结果
    """
    print("\n" + "=" * 60)
    print("松耦合ESKF实验")
    print("=" * 60)
    
    # 1. 加载配置
    print("\n[1/7] 加载配置...")
    config = ConfigManager()
    imu_params = config.get_imu_params()
    gnss_params = config.get_gnss_params()
    eskf_params = config.get_eskf_params()
    print("  ✓ 配置加载完成")
    
    # 2. 读取数据
    print("\n[2/7] 读取数据...")
    imu_reader = IMUReader(imu_file)
    imu_data = imu_reader.read()
    print(f"  IMU数据点数: {len(imu_data['timestamp'])}")
    
    gnss_reader = GNSSReader(nav_file=gnss_file)
    gnss_data = gnss_reader.read_navigation()
    print(f"  GNSS数据点数: {len(gnss_data['timestamp'])}")
    print("  ✓ 数据读取完成")
    
    # 3. 初始对齐
    print("\n[3/7] 初始对齐...")
    if initial_state is None:
        if use_alignment == 'truth' and alignment_truth:
            from ins.initial_alignment import load_initial_state_from_truth
            initial_state = load_initial_state_from_truth(alignment_truth, navigation_frame=navigation_frame)
        else:
            initial_state = {
                'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
                'velocity': np.array([0.0, 0.0, 0.0]),
                'position': np.array([0.0, 0.0, 0.0])
            }
    print("  ✓ 初始对齐完成")
    
    # 4. 初始化INS和ESKF
    print("\n[4/7] 初始化INS和ESKF...")
    ins = INSMechanization(initial_state, navigation_frame=navigation_frame)
    
    # 创建误差动力学模型
    error_dynamics = ErrorDynamics(imu_params)
    
    # 创建松耦合量测模型
    lc_model = LooseCouplingModel(gnss_params)
    
    # 创建初始协方差
    initial_cov = np.eye(15)
    initial_cov[0:3, 0:3] *= eskf_params['initial_cov']['attitude']
    initial_cov[3:6, 3:6] *= eskf_params['initial_cov']['velocity']
    initial_cov[6:9, 6:9] *= eskf_params['initial_cov']['position']
    initial_cov[9:12, 9:12] *= eskf_params['initial_cov']['gyro_bias']
    initial_cov[12:15, 12:15] *= eskf_params['initial_cov']['accel_bias']
    
    # 创建松耦合ESKF
    lc_eskf = LooseCouplingESKF(initial_cov, error_dynamics, lc_model)
    
    # 零偏估计
    gyro_bias = np.zeros(3)
    accel_bias = np.zeros(3)
    
    print("  ✓ 初始化完成")
    
    # 5. 融合处理
    print("\n[5/7] 执行松耦合融合...")
    timestamps = imu_data['timestamp']
    n_samples = len(timestamps)
    
    # GNSS数据索引
    gnss_idx = 0
    gnss_update_count = 0
    
    # 存储轨迹
    trajectory = {
        'timestamp': [],
        'position': [],
        'velocity': [],
        'attitude_quat': [],
        'attitude_euler': [],
        'gnss_updates': []  # 标记GNSS更新时刻
    }
    
    # 记录初始状态
    trajectory['timestamp'].append(timestamps[0])
    trajectory['position'].append(ins.position.copy())
    trajectory['velocity'].append(ins.velocity.copy())
    trajectory['attitude_quat'].append(ins.attitude.copy())
    trajectory['attitude_euler'].append(ins.get_euler_angles().copy())
    trajectory['gnss_updates'].append(False)
    
    # 主循环
    for i in range(1, n_samples):
        dt = timestamps[i] - timestamps[i-1]
        t = timestamps[i]
        
        # 读取IMU数据
        gyro = imu_data['gyro'][i]
        accel = imu_data['accel'][i]
        
        # INS预测
        ins.update(gyro - gyro_bias, accel - accel_bias, dt)
        nominal_state = ins.get_state()
        
        # ESKF预测
        imu_measurement = {'gyro': gyro, 'accel': accel}
        lc_eskf.predict(nominal_state, imu_measurement, dt)
        
        # 检查是否有GNSS更新
        gnss_updated = False
        if gnss_idx < len(gnss_data['timestamp']):
            # 查找最近的GNSS数据
            while gnss_idx < len(gnss_data['timestamp']) - 1 and \
                  abs(gnss_data['timestamp'][gnss_idx + 1] - t) < abs(gnss_data['timestamp'][gnss_idx] - t):
                gnss_idx += 1
            
            # 如果时间匹配（在0.1秒内）
            if abs(gnss_data['timestamp'][gnss_idx] - t) < 0.1:
                gnss_nav = {
                    'position': gnss_data['position'][gnss_idx],
                    'velocity': gnss_data['velocity'][gnss_idx]
                }
                
                # GNSS更新
                corrected_state, gyro_bias, accel_bias = \
                    lc_eskf.process_gnss_measurement(nominal_state, gnss_nav, 
                                                    gyro_bias, accel_bias)
                
                # 更新INS状态
                ins.position = corrected_state['position']
                ins.velocity = corrected_state['velocity']
                ins.attitude = corrected_state['attitude']
                
                gnss_updated = True
                gnss_update_count += 1
        
        # 记录轨迹
        trajectory['timestamp'].append(t)
        trajectory['position'].append(ins.position.copy())
        trajectory['velocity'].append(ins.velocity.copy())
        trajectory['attitude_quat'].append(ins.attitude.copy())
        trajectory['attitude_euler'].append(ins.get_euler_angles().copy())
        trajectory['gnss_updates'].append(gnss_updated)
        
        # 进度显示
        if (i+1) % (n_samples // 10) == 0:
            progress = (i+1) / n_samples * 100
            print(f"  进度: {progress:.0f}% (GNSS更新: {gnss_update_count}次)")
    
    # 转换为numpy数组
    for key in trajectory:
        if key not in ['timestamp', 'gnss_updates']:
            trajectory[key] = np.array(trajectory[key])
    
    print(f"  ✓ 融合完成 (共{gnss_update_count}次GNSS更新)")
    
    # 6. 读取参考真值（如果有）
    reference_data = None
    if reference_file:
        print("\n[6/7] 读取参考真值...")
        ref_reader = GNSSReader(nav_file=reference_file)
        reference_data = ref_reader.read_navigation()
        print("  ✓ 参考真值读取完成")
    
    # 7. 可视化
    print("\n[7/7] 生成可视化图表...")
    plot_results(trajectory, reference_data, output_dir)
    
    # 保存结果
    if output_dir:
        save_results(trajectory, output_dir)
    
    print("\n" + "=" * 60)
    print("松耦合ESKF实验完成！")
    print("=" * 60)
    
    return trajectory


def plot_results(trajectory, reference_data=None, output_dir=None):
    """绘制结果图表"""
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    
    timestamps = trajectory['timestamp']
    positions = trajectory['position']
    velocities = trajectory['velocity']
    euler_angles = np.rad2deg(trajectory['attitude_euler'])
    gnss_updates = trajectory['gnss_updates']
    
    # 创建图表
    fig = plt.figure(figsize=(15, 10))
    
    # 1. 2D轨迹
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1.5, label='Fusion')
    if reference_data is not None:
        ax1.plot(reference_data['position'][:, 0], reference_data['position'][:, 1], 
                'r--', linewidth=1, alpha=0.7, label='Reference')
    ax1.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='Start')
    ax1.plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=10, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('2D Trajectory (LC-ESKF)', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # 2. 位置随时间
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(timestamps, positions[:, 0], 'r-', label='X', linewidth=1.5)
    ax2.plot(timestamps, positions[:, 1], 'g-', label='Y', linewidth=1.5)
    ax2.plot(timestamps, positions[:, 2], 'b-', label='Z', linewidth=1.5)
    # 标记GNSS更新点
    gnss_times = [t for i, t in enumerate(timestamps) if gnss_updates[i]]
    if gnss_times:
        ax2.scatter(gnss_times, [positions[i, 0] for i, t in enumerate(timestamps) if gnss_updates[i]], 
                   c='k', marker='|', s=100, alpha=0.3, label='GNSS Update')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # 3. 速度随时间
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(timestamps, velocities[:, 0], 'r-', label='Vx', linewidth=1.5)
    ax3.plot(timestamps, velocities[:, 1], 'g-', label='Vy', linewidth=1.5)
    ax3.plot(timestamps, velocities[:, 2], 'b-', label='Vz', linewidth=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.set_title('Velocity vs Time', fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # 4. 姿态角随时间
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(timestamps, euler_angles[:, 0], 'r-', label='Roll', linewidth=1.5)
    ax4.plot(timestamps, euler_angles[:, 1], 'g-', label='Pitch', linewidth=1.5)
    ax4.plot(timestamps, euler_angles[:, 2], 'b-', label='Yaw', linewidth=1.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Attitude (deg)')
    ax4.set_title('Attitude vs Time', fontsize=12)
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    # 5. 3D轨迹
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    ax5.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=1.5, label='Fusion')
    if reference_data is not None:
        ax5.plot(reference_data['position'][:, 0], reference_data['position'][:, 1], 
                reference_data['position'][:, 2], 'r--', linewidth=1, alpha=0.5, label='Reference')
    ax5.plot([positions[0, 0]], [positions[0, 1]], [positions[0, 2]], 'go', markersize=10)
    ax5.plot([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]], 'ro', markersize=10)
    ax5.set_xlabel('X (m)')
    ax5.set_ylabel('Y (m)')
    ax5.set_zlabel('Z (m)')
    ax5.set_title('3D Trajectory', fontsize=12)
    ax5.legend()
    
    # 6. 速度大小
    ax6 = plt.subplot(2, 3, 6)
    speed = np.linalg.norm(velocities, axis=1)
    ax6.plot(timestamps, speed, 'b-', linewidth=1.5)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Speed (m/s)')
    ax6.set_title('Speed Magnitude', fontsize=12)
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_dir:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        fig_path = output_path / 'loose_coupling_trajectory.png'
        plt.savefig(fig_path, dpi=300, bbox_inches='tight')
        print(f"  ✓ 图表已保存: {fig_path}")
    
    plt.show()


def save_results(trajectory, output_dir):
    """保存结果"""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    result_file = output_path / 'loose_coupling_trajectory.txt'
    with open(result_file, 'w') as f:
        f.write("# 松耦合ESKF结果\n")
        f.write("# timestamp pos_x pos_y pos_z vel_x vel_y vel_z roll pitch yaw gnss_update\n")
        
        timestamps = trajectory['timestamp']
        positions = trajectory['position']
        velocities = trajectory['velocity']
        euler_angles = trajectory['attitude_euler']
        gnss_updates = trajectory['gnss_updates']
        
        for i in range(len(timestamps)):
            f.write(f"{timestamps[i]:.4f} ")
            f.write(f"{positions[i, 0]:.4f} {positions[i, 1]:.4f} {positions[i, 2]:.4f} ")
            f.write(f"{velocities[i, 0]:.4f} {velocities[i, 1]:.4f} {velocities[i, 2]:.4f} ")
            f.write(f"{euler_angles[i, 0]:.6f} {euler_angles[i, 1]:.6f} {euler_angles[i, 2]:.6f} ")
            f.write(f"{1 if gnss_updates[i] else 0}\n")
    
    print(f"  ✓ 结果已保存: {result_file}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='松耦合ESKF实验')
    parser.add_argument('--imu', type=str, required=True, help='IMU数据文件')
    parser.add_argument('--gnss', type=str, required=True, help='GNSS导航解文件')
    parser.add_argument('--output', type=str, default='results/loose_coupling', help='输出目录')
    parser.add_argument('--ref', type=str, default=None, help='参考轨迹文件')
    parser.add_argument('--frame', type=str, default='ENU', choices=['ENU', 'NED'])
    parser.add_argument('--align', type=str, default=None, choices=['static', 'truth'])
    parser.add_argument('--truth', type=str, default=None, help='真值文件路径')
    
    args = parser.parse_args()
    
    trajectory = run_loose_coupling(
        imu_file=args.imu,
        gnss_file=args.gnss,
        output_dir=args.output,
        reference_file=args.ref,
        navigation_frame=args.frame,
        use_alignment=args.align,
        alignment_truth=args.truth
    )
