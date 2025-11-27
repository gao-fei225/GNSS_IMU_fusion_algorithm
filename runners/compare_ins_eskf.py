"""
对比纯INS和松耦合ESKF的结果
生成对比图表，展示ESKF在纯INS基础上的改善
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def load_trajectory(file_path):
    """加载轨迹数据"""
    data = {}
    with open(file_path, 'r') as f:
        lines = f.readlines()
        
        # 跳过注释行
        data_lines = [line for line in lines if not line.startswith('#')]
        
        timestamps = []
        positions = []
        velocities = []
        euler_angles = []
        
        for line in data_lines:
            if line.strip():
                values = line.strip().split()
                timestamps.append(float(values[0]))
                positions.append([float(values[1]), float(values[2]), float(values[3])])
                velocities.append([float(values[4]), float(values[5]), float(values[6])])
                euler_angles.append([float(values[7]), float(values[8]), float(values[9])])
        
        data['timestamp'] = np.array(timestamps)
        data['position'] = np.array(positions)
        data['velocity'] = np.array(velocities)
        data['attitude_euler'] = np.array(euler_angles)
    
    return data


def normalize_angle(angle):
    """
    将角度归一化到[-π, π]范围
    
    Args:
        angle: 角度（弧度）
    
    Returns:
        归一化后的角度
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_errors(trajectory, reference):
    """计算误差"""
    # 确保时间对齐
    n = min(len(trajectory['timestamp']), len(reference['timestamp']))
    
    pos_error = trajectory['position'][:n] - reference['position'][:n]
    vel_error = trajectory['velocity'][:n] - reference['velocity'][:n]
    
    # 姿态角误差需要特殊处理，避免±180度跳变
    att_error = trajectory['attitude_euler'][:n] - reference['attitude_euler'][:n]
    
    # 对每个姿态角分量进行归一化
    att_error[:, 0] = normalize_angle(att_error[:, 0])  # Roll
    att_error[:, 1] = normalize_angle(att_error[:, 1])  # Pitch
    att_error[:, 2] = normalize_angle(att_error[:, 2])  # Yaw
    
    return {
        'position': pos_error,
        'velocity': vel_error,
        'attitude': att_error,
        'timestamp': trajectory['timestamp'][:n]
    }


def plot_comparison(pure_ins, loose_coupling, reference, output_dir):
    """生成对比图表"""
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 计算误差
    ins_errors = compute_errors(pure_ins, reference)
    lc_errors = compute_errors(loose_coupling, reference)
    
    # 创建大图
    fig = plt.figure(figsize=(18, 12))
    
    # 1. 2D轨迹对比
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(reference['position'][:, 0], reference['position'][:, 1], 
             'k-', linewidth=2, alpha=0.7, label='参考真值')
    ax1.plot(pure_ins['position'][:, 0], pure_ins['position'][:, 1], 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax1.plot(loose_coupling['position'][:, 0], loose_coupling['position'][:, 1], 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax1.plot(reference['position'][0, 0], reference['position'][0, 1], 'go', markersize=10)
    ax1.plot(reference['position'][-1, 0], reference['position'][-1, 1], 'ro', markersize=10)
    ax1.set_xlabel('X (m)', fontsize=11)
    ax1.set_ylabel('Y (m)', fontsize=11)
    ax1.set_title('2D轨迹对比 (俯视图)', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)
    ax1.axis('equal')
    
    # 2. 位置误差 (X方向)
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(ins_errors['timestamp'], ins_errors['position'][:, 0], 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax2.plot(lc_errors['timestamp'], lc_errors['position'][:, 0], 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax2.set_xlabel('时间 (s)', fontsize=11)
    ax2.set_ylabel('位置误差 X (m)', fontsize=11)
    ax2.set_title('X方向位置误差对比', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)
    
    # 3. 位置误差 (Y方向)
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(ins_errors['timestamp'], ins_errors['position'][:, 1], 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax3.plot(lc_errors['timestamp'], lc_errors['position'][:, 1], 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax3.set_xlabel('时间 (s)', fontsize=11)
    ax3.set_ylabel('位置误差 Y (m)', fontsize=11)
    ax3.set_title('Y方向位置误差对比', fontsize=13, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=9)
    
    # 4. 位置误差 (Z方向)
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(ins_errors['timestamp'], ins_errors['position'][:, 2], 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax4.plot(lc_errors['timestamp'], lc_errors['position'][:, 2], 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax4.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax4.set_xlabel('时间 (s)', fontsize=11)
    ax4.set_ylabel('位置误差 Z (m)', fontsize=11)
    ax4.set_title('Z方向位置误差对比', fontsize=13, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=9)
    
    # 5. 位置误差范数
    ax5 = plt.subplot(3, 3, 5)
    ins_pos_norm = np.linalg.norm(ins_errors['position'], axis=1)
    lc_pos_norm = np.linalg.norm(lc_errors['position'], axis=1)
    ax5.plot(ins_errors['timestamp'], ins_pos_norm, 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax5.plot(lc_errors['timestamp'], lc_pos_norm, 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax5.set_xlabel('时间 (s)', fontsize=11)
    ax5.set_ylabel('位置误差范数 (m)', fontsize=11)
    ax5.set_title('位置误差范数对比', fontsize=13, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.legend(fontsize=9)
    
    # 6. 速度误差范数
    ax6 = plt.subplot(3, 3, 6)
    ins_vel_norm = np.linalg.norm(ins_errors['velocity'], axis=1)
    lc_vel_norm = np.linalg.norm(lc_errors['velocity'], axis=1)
    ax6.plot(ins_errors['timestamp'], ins_vel_norm, 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax6.plot(lc_errors['timestamp'], lc_vel_norm, 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax6.set_xlabel('时间 (s)', fontsize=11)
    ax6.set_ylabel('速度误差范数 (m/s)', fontsize=11)
    ax6.set_title('速度误差范数对比', fontsize=13, fontweight='bold')
    ax6.grid(True, alpha=0.3)
    ax6.legend(fontsize=9)
    
    # 7. Roll角误差
    ax7 = plt.subplot(3, 3, 7)
    ax7.plot(ins_errors['timestamp'], np.rad2deg(ins_errors['attitude'][:, 0]), 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax7.plot(lc_errors['timestamp'], np.rad2deg(lc_errors['attitude'][:, 0]), 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax7.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax7.set_xlabel('时间 (s)', fontsize=11)
    ax7.set_ylabel('Roll误差 (度)', fontsize=11)
    ax7.set_title('Roll角误差对比', fontsize=13, fontweight='bold')
    ax7.grid(True, alpha=0.3)
    ax7.legend(fontsize=9)
    
    # 8. Pitch角误差
    ax8 = plt.subplot(3, 3, 8)
    ax8.plot(ins_errors['timestamp'], np.rad2deg(ins_errors['attitude'][:, 1]), 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax8.plot(lc_errors['timestamp'], np.rad2deg(lc_errors['attitude'][:, 1]), 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax8.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax8.set_xlabel('时间 (s)', fontsize=11)
    ax8.set_ylabel('Pitch误差 (度)', fontsize=11)
    ax8.set_title('Pitch角误差对比', fontsize=13, fontweight='bold')
    ax8.grid(True, alpha=0.3)
    ax8.legend(fontsize=9)
    
    # 9. Yaw角误差
    ax9 = plt.subplot(3, 3, 9)
    ax9.plot(ins_errors['timestamp'], np.rad2deg(ins_errors['attitude'][:, 2]), 
             'r-', linewidth=1.5, alpha=0.7, label='纯INS')
    ax9.plot(lc_errors['timestamp'], np.rad2deg(lc_errors['attitude'][:, 2]), 
             'b-', linewidth=1.5, alpha=0.7, label='松耦合ESKF')
    ax9.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax9.set_xlabel('时间 (s)', fontsize=11)
    ax9.set_ylabel('Yaw误差 (度)', fontsize=11)
    ax9.set_title('Yaw角误差对比', fontsize=13, fontweight='bold')
    ax9.grid(True, alpha=0.3)
    ax9.legend(fontsize=9)
    
    plt.suptitle('纯INS vs 松耦合ESKF性能对比', fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    
    # 保存图表
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    fig_path = output_path / 'comparison_ins_vs_eskf.png'
    plt.savefig(fig_path, dpi=300, bbox_inches='tight')
    print(f"\n对比图表已保存: {fig_path}")
    
    plt.show()
    
    return ins_errors, lc_errors


def print_statistics(pure_ins_errors, lc_errors):
    """打印统计信息"""
    print("\n" + "=" * 70)
    print("性能对比统计")
    print("=" * 70)
    
    # 位置误差统计
    ins_pos_norm = np.linalg.norm(pure_ins_errors['position'], axis=1)
    lc_pos_norm = np.linalg.norm(lc_errors['position'], axis=1)
    
    print("\n位置误差统计 (m):")
    print(f"  纯INS:")
    print(f"    均值: {np.mean(ins_pos_norm):.4f}")
    print(f"    RMS:  {np.sqrt(np.mean(ins_pos_norm**2)):.4f}")
    print(f"    最大: {np.max(ins_pos_norm):.4f}")
    print(f"  松耦合ESKF:")
    print(f"    均值: {np.mean(lc_pos_norm):.4f}")
    print(f"    RMS:  {np.sqrt(np.mean(lc_pos_norm**2)):.4f}")
    print(f"    最大: {np.max(lc_pos_norm):.4f}")
    print(f"  改善:")
    print(f"    均值改善: {(1 - np.mean(lc_pos_norm)/np.mean(ins_pos_norm))*100:.2f}%")
    print(f"    RMS改善:  {(1 - np.sqrt(np.mean(lc_pos_norm**2))/np.sqrt(np.mean(ins_pos_norm**2)))*100:.2f}%")
    
    # 速度误差统计
    ins_vel_norm = np.linalg.norm(pure_ins_errors['velocity'], axis=1)
    lc_vel_norm = np.linalg.norm(lc_errors['velocity'], axis=1)
    
    print("\n速度误差统计 (m/s):")
    print(f"  纯INS:")
    print(f"    均值: {np.mean(ins_vel_norm):.4f}")
    print(f"    RMS:  {np.sqrt(np.mean(ins_vel_norm**2)):.4f}")
    print(f"    最大: {np.max(ins_vel_norm):.4f}")
    print(f"  松耦合ESKF:")
    print(f"    均值: {np.mean(lc_vel_norm):.4f}")
    print(f"    RMS:  {np.sqrt(np.mean(lc_vel_norm**2)):.4f}")
    print(f"    最大: {np.max(lc_vel_norm):.4f}")
    print(f"  改善:")
    print(f"    均值改善: {(1 - np.mean(lc_vel_norm)/np.mean(ins_vel_norm))*100:.2f}%")
    print(f"    RMS改善:  {(1 - np.sqrt(np.mean(lc_vel_norm**2))/np.sqrt(np.mean(ins_vel_norm**2)))*100:.2f}%")
    
    # 姿态误差统计
    ins_att_norm = np.linalg.norm(pure_ins_errors['attitude'], axis=1)
    lc_att_norm = np.linalg.norm(lc_errors['attitude'], axis=1)
    
    print("\n姿态误差统计 (度):")
    print(f"  纯INS:")
    print(f"    均值: {np.rad2deg(np.mean(ins_att_norm)):.4f}")
    print(f"    RMS:  {np.rad2deg(np.sqrt(np.mean(ins_att_norm**2))):.4f}")
    print(f"    最大: {np.rad2deg(np.max(ins_att_norm)):.4f}")
    print(f"  松耦合ESKF:")
    print(f"    均值: {np.rad2deg(np.mean(lc_att_norm)):.4f}")
    print(f"    RMS:  {np.rad2deg(np.sqrt(np.mean(lc_att_norm**2))):.4f}")
    print(f"    最大: {np.rad2deg(np.max(lc_att_norm)):.4f}")
    print(f"  改善:")
    print(f"    均值改善: {(1 - np.mean(lc_att_norm)/np.mean(ins_att_norm))*100:.2f}%")
    print(f"    RMS改善:  {(1 - np.sqrt(np.mean(lc_att_norm**2))/np.sqrt(np.mean(ins_att_norm**2)))*100:.2f}%")
    
    print("\n" + "=" * 70)


def load_reference_truth(file_path):
    """加载参考真值"""
    data = np.loadtxt(file_path, comments='#')
    return {
        'timestamp': data[:, 0],
        'position': data[:, 1:4],
        'velocity': data[:, 4:7],
        'attitude_euler': data[:, 7:10]
    }


def main():
    """主函数"""
    print("\n" + "=" * 70)
    print("纯INS vs 松耦合ESKF 性能对比分析")
    print("=" * 70)
    
    # 加载数据
    print("\n加载数据...")
    pure_ins = load_trajectory('results/pure_ins_exp/pure_ins_trajectory.txt')
    loose_coupling = load_trajectory('results/loose_coupling_exp/loose_coupling_trajectory.txt')
    
    # 尝试加载真实参考轨迹
    try:
        reference = load_reference_truth('data/2025-11-21-11-49-40/reference_truth.txt')
        print("  使用参考真值数据")
    except:
        print("  警告：无法加载参考真值，使用松耦合结果作为参考")
        reference = loose_coupling
    
    print(f"  纯INS数据点数: {len(pure_ins['timestamp'])}")
    print(f"  松耦合ESKF数据点数: {len(loose_coupling['timestamp'])}")
    print(f"  参考数据点数: {len(reference['timestamp'])}")
    
    # 生成对比图
    print("\n生成对比图表...")
    ins_errors, lc_errors = plot_comparison(pure_ins, loose_coupling, reference, 'results/comparison')
    
    # 打印统计信息
    print_statistics(ins_errors, lc_errors)
    
    print("\n对比分析完成！")
    print("=" * 70)


if __name__ == "__main__":
    main()
