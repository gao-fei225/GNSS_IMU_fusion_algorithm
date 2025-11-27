"""
评估INS解算性能
与真值对比，计算误差指标
"""

import numpy as np
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt


def read_truth_nav(truth_file):
    """
    读取truth.nav格式的真值文件（Awesome GINS格式）
    
    格式：GPS周 GPS周秒 纬度 经度 高度 速度N 速度E 速度D Roll Pitch Yaw
    """
    data = np.loadtxt(truth_file)
    
    # 检查格式
    if data.shape[1] == 10:
        # gnss-ins-sim格式：timestamp pos_x pos_y pos_z vel_x vel_y vel_z roll pitch yaw
        timestamp = data[:, 0]
        pos_x = data[:, 1]
        pos_y = data[:, 2]
        pos_z = data[:, 3]
        vel_x = data[:, 4]
        vel_y = data[:, 5]
        vel_z = data[:, 6]
        roll = data[:, 7]  # 已经是弧度
        pitch = data[:, 8]
        yaw = data[:, 9]
        
        return {
            'timestamp': timestamp,
            'position': np.column_stack([pos_x, pos_y, pos_z]),
            'velocity': np.column_stack([vel_x, vel_y, vel_z]),
            'attitude': np.column_stack([roll, pitch, yaw])
        }
    else:
        # Awesome GINS格式
        gps_week = data[:, 0]
        gps_sow = data[:, 1]
        lat = data[:, 2]  # 度
        lon = data[:, 3]  # 度
        height = data[:, 4]  # 米
        vel_n = data[:, 5]  # m/s
        vel_e = data[:, 6]  # m/s
        vel_d = data[:, 7]  # m/s
        roll = np.deg2rad(data[:, 8])  # 转换为弧度
        pitch = np.deg2rad(data[:, 9])
        yaw = np.deg2rad(data[:, 10])
        
        # 转换为相对时间（从0开始）
        timestamp = gps_sow - gps_sow[0]
        
        # 转换为局部ENU坐标（以第一个点为原点）
        lat0, lon0, h0 = lat[0], lon[0], height[0]
        
        # 简化的局部坐标转换（小范围内近似）
        R_earth = 6378137.0  # WGS84地球半径
        
        pos_e = (lon - lon0) * np.pi/180 * R_earth * np.cos(lat0 * np.pi/180)
        pos_n = (lat - lat0) * np.pi/180 * R_earth
        pos_u = height - h0
        
        return {
            'timestamp': timestamp,
            'position': np.column_stack([pos_e, pos_n, pos_u]),  # ENU
            'velocity': np.column_stack([vel_e, vel_n, -vel_d]),  # ENU
            'attitude': np.column_stack([roll, pitch, yaw])
        }


def read_ins_result(result_file):
    """读取INS解算结果"""
    data = pd.read_csv(result_file, sep=' ', comment='#', header=None,
                       encoding='latin1',
                       names=['timestamp', 'pos_x', 'pos_y', 'pos_z',
                             'vel_x', 'vel_y', 'vel_z',
                             'roll', 'pitch', 'yaw'])
    
    return {
        'timestamp': data['timestamp'].values,
        'position': data[['pos_x', 'pos_y', 'pos_z']].values,
        'velocity': data[['vel_x', 'vel_y', 'vel_z']].values,
        'attitude': data[['roll', 'pitch', 'yaw']].values
    }


def interpolate_truth(truth, ins_timestamps):
    """将真值插值到INS时间戳"""
    from scipy import interpolate
    
    interp_pos = np.zeros((len(ins_timestamps), 3))
    interp_vel = np.zeros((len(ins_timestamps), 3))
    interp_att = np.zeros((len(ins_timestamps), 3))
    
    for i in range(3):
        # 位置插值
        f_pos = interpolate.interp1d(truth['timestamp'], truth['position'][:, i],
                                     kind='linear', fill_value='extrapolate')
        interp_pos[:, i] = f_pos(ins_timestamps)
        
        # 速度插值
        f_vel = interpolate.interp1d(truth['timestamp'], truth['velocity'][:, i],
                                     kind='linear', fill_value='extrapolate')
        interp_vel[:, i] = f_vel(ins_timestamps)
        
        # 姿态插值
        f_att = interpolate.interp1d(truth['timestamp'], truth['attitude'][:, i],
                                     kind='linear', fill_value='extrapolate')
        interp_att[:, i] = f_att(ins_timestamps)
    
    return {
        'position': interp_pos,
        'velocity': interp_vel,
        'attitude': interp_att
    }


def calculate_errors(ins_result, truth_interp):
    """计算误差"""
    # 位置误差
    pos_error = ins_result['position'] - truth_interp['position']
    pos_error_norm = np.linalg.norm(pos_error, axis=1)
    
    # 速度误差
    vel_error = ins_result['velocity'] - truth_interp['velocity']
    vel_error_norm = np.linalg.norm(vel_error, axis=1)
    
    # 姿态误差
    att_error = ins_result['attitude'] - truth_interp['attitude']
    # 处理角度周期性（-π到π）
    att_error = np.arctan2(np.sin(att_error), np.cos(att_error))
    att_error_deg = np.rad2deg(att_error)
    
    return {
        'position': pos_error,
        'position_norm': pos_error_norm,
        'velocity': vel_error,
        'velocity_norm': vel_error_norm,
        'attitude': att_error,
        'attitude_deg': att_error_deg
    }


def compute_statistics(errors, timestamps):
    """计算统计指标"""
    stats = {}
    
    # 位置误差统计
    stats['pos_rmse'] = np.sqrt(np.mean(errors['position_norm']**2))
    stats['pos_max'] = np.max(errors['position_norm'])
    stats['pos_mean'] = np.mean(errors['position_norm'])
    stats['pos_final'] = errors['position_norm'][-1]
    
    # 速度误差统计
    stats['vel_rmse'] = np.sqrt(np.mean(errors['velocity_norm']**2))
    stats['vel_max'] = np.max(errors['velocity_norm'])
    stats['vel_mean'] = np.mean(errors['velocity_norm'])
    stats['vel_final'] = errors['velocity_norm'][-1]
    
    # 姿态误差统计（度）
    stats['roll_rmse'] = np.sqrt(np.mean(errors['attitude_deg'][:, 0]**2))
    stats['pitch_rmse'] = np.sqrt(np.mean(errors['attitude_deg'][:, 1]**2))
    stats['yaw_rmse'] = np.sqrt(np.mean(errors['attitude_deg'][:, 2]**2))
    
    stats['roll_max'] = np.max(np.abs(errors['attitude_deg'][:, 0]))
    stats['pitch_max'] = np.max(np.abs(errors['attitude_deg'][:, 1]))
    stats['yaw_max'] = np.max(np.abs(errors['attitude_deg'][:, 2]))
    
    # 时间段分析
    duration = timestamps[-1] - timestamps[0]
    
    # 前10秒
    mask_10s = timestamps <= 10.0
    if np.any(mask_10s):
        stats['pos_10s'] = np.mean(errors['position_norm'][mask_10s])
    
    # 前60秒
    mask_60s = timestamps <= 60.0
    if np.any(mask_60s):
        stats['pos_60s'] = np.mean(errors['position_norm'][mask_60s])
    
    # 每分钟漂移率
    stats['drift_rate_per_min'] = stats['pos_final'] / (duration / 60.0)
    
    return stats


def plot_errors(errors, timestamps, output_dir):
    """绘制误差曲线"""
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    fig = plt.figure(figsize=(15, 12))
    
    # 1. 位置误差范数
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(timestamps, errors['position_norm'], 'b-', linewidth=1.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position Error (m)')
    ax1.set_title('Position Error Magnitude', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.set_yscale('log')
    
    # 2. 位置误差分量
    ax2 = plt.subplot(3, 2, 2)
    ax2.plot(timestamps, errors['position'][:, 0], 'r-', label='E', linewidth=1.5)
    ax2.plot(timestamps, errors['position'][:, 1], 'g-', label='N', linewidth=1.5)
    ax2.plot(timestamps, errors['position'][:, 2], 'b-', label='U', linewidth=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position Error (m)')
    ax2.set_title('Position Error (ENU)', fontsize=12)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. 速度误差范数
    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(timestamps, errors['velocity_norm'], 'b-', linewidth=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity Error (m/s)')
    ax3.set_title('Velocity Error Magnitude', fontsize=12)
    ax3.grid(True, alpha=0.3)
    
    # 4. 速度误差分量
    ax4 = plt.subplot(3, 2, 4)
    ax4.plot(timestamps, errors['velocity'][:, 0], 'r-', label='VE', linewidth=1.5)
    ax4.plot(timestamps, errors['velocity'][:, 1], 'g-', label='VN', linewidth=1.5)
    ax4.plot(timestamps, errors['velocity'][:, 2], 'b-', label='VU', linewidth=1.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Velocity Error (m/s)')
    ax4.set_title('Velocity Error (ENU)', fontsize=12)
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. 姿态误差
    ax5 = plt.subplot(3, 2, 5)
    ax5.plot(timestamps, errors['attitude_deg'][:, 0], 'r-', label='Roll', linewidth=1.5)
    ax5.plot(timestamps, errors['attitude_deg'][:, 1], 'g-', label='Pitch', linewidth=1.5)
    ax5.plot(timestamps, errors['attitude_deg'][:, 2], 'b-', label='Yaw', linewidth=1.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Attitude Error (deg)')
    ax5.set_title('Attitude Error', fontsize=12)
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # 6. 误差累积曲线（对数）
    ax6 = plt.subplot(3, 2, 6)
    ax6.loglog(timestamps, errors['position_norm'], 'b-', label='Position', linewidth=1.5)
    ax6.loglog(timestamps, errors['velocity_norm'], 'r-', label='Velocity', linewidth=1.5)
    ax6.set_xlabel('Time (s, log scale)')
    ax6.set_ylabel('Error (log scale)')
    ax6.set_title('Error Growth (Log-Log)', fontsize=12)
    ax6.legend()
    ax6.grid(True, alpha=0.3, which='both')
    
    plt.tight_layout()
    
    # 保存
    if output_dir:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        fig_path = output_path / 'error_analysis.png'
        plt.savefig(fig_path, dpi=300, bbox_inches='tight')
        print(f"  ✓ 误差分析图表已保存: {fig_path}")
    
    plt.show()


def generate_report(stats, output_dir, dataset_name):
    """生成报告"""
    report = f"""
{'='*70}
纯INS性能评估报告（与真值对比）
{'='*70}

数据集: {dataset_name}

【位置误差】
  RMSE:        {stats['pos_rmse']:.2f} m
  最大误差:    {stats['pos_max']:.2f} m
  平均误差:    {stats['pos_mean']:.2f} m
  最终误差:    {stats['pos_final']:.2f} m
  
  前10秒平均:  {stats.get('pos_10s', 0):.2f} m
  前60秒平均:  {stats.get('pos_60s', 0):.2f} m
  漂移率:      {stats['drift_rate_per_min']:.2f} m/min

【速度误差】
  RMSE:        {stats['vel_rmse']:.2f} m/s
  最大误差:    {stats['vel_max']:.2f} m/s
  平均误差:    {stats['vel_mean']:.2f} m/s
  最终误差:    {stats['vel_final']:.2f} m/s

【姿态误差】
  Roll RMSE:   {stats['roll_rmse']:.2f}°  (最大: {stats['roll_max']:.2f}°)
  Pitch RMSE:  {stats['pitch_rmse']:.2f}°  (最大: {stats['pitch_max']:.2f}°)
  Yaw RMSE:    {stats['yaw_rmse']:.2f}°  (最大: {stats['yaw_max']:.2f}°)

【性能等级评估】
"""
    
    # 评级
    if stats['pos_rmse'] < 10:
        grade = "优秀 ★★★★★"
    elif stats['pos_rmse'] < 100:
        grade = "良好 ★★★★☆"
    elif stats['pos_rmse'] < 1000:
        grade = "中等 ★★★☆☆"
    elif stats['pos_rmse'] < 10000:
        grade = "较差 ★★☆☆☆"
    else:
        grade = "极差 ★☆☆☆☆"
    
    report += f"  位置RMSE等级: {grade}\n"
    
    # 判断是否满足指标
    report += "\n【指标达成情况】\n"
    report += f"  前60秒水平漂移 < 10m:  {'✓ 达成' if stats.get('pos_60s', float('inf')) < 10 else '✗ 未达成'}\n"
    report += f"  姿态误差 < 1°:         {'✓ 达成' if max(stats['roll_rmse'], stats['pitch_rmse']) < 1 else '✗ 未达成'}\n"
    report += f"  速度误差 < 0.5m/s:     {'✓ 达成' if stats['vel_rmse'] < 0.5 else '✗ 未达成'}\n"
    
    report += "\n" + "="*70 + "\n"
    
    print(report)
    
    # 保存报告
    if output_dir:
        report_file = Path(output_dir) / 'performance_evaluation.txt'
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"✓ 报告已保存: {report_file}")
    
    return report


def evaluate_ins_performance(ins_result_file, truth_file, output_dir=None, dataset_name="Unknown"):
    """
    主评估函数
    """
    print("\n" + "="*70)
    print("INS性能评估（与真值对比）")
    print("="*70)
    
    # 读取数据
    print("\n[1/5] 读取真值数据...")
    truth = read_truth_nav(truth_file)
    print(f"  真值数据点数: {len(truth['timestamp'])}")
    print(f"  时间范围: {truth['timestamp'][0]:.2f} - {truth['timestamp'][-1]:.2f} 秒")
    
    print("\n[2/5] 读取INS解算结果...")
    ins_result = read_ins_result(ins_result_file)
    print(f"  INS数据点数: {len(ins_result['timestamp'])}")
    print(f"  时间范围: {ins_result['timestamp'][0]:.2f} - {ins_result['timestamp'][-1]:.2f} 秒")
    
    print("\n[3/5] 插值真值到INS时间戳...")
    truth_interp = interpolate_truth(truth, ins_result['timestamp'])
    
    print("\n[4/5] 计算误差...")
    errors = calculate_errors(ins_result, truth_interp)
    stats = compute_statistics(errors, ins_result['timestamp'])
    
    print("\n[5/5] 生成报告和图表...")
    generate_report(stats, output_dir, dataset_name)
    plot_errors(errors, ins_result['timestamp'], output_dir)
    
    print("\n" + "="*70)
    print("评估完成！")
    print("="*70)
    
    return stats, errors


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 3:
        print("用法: python evaluate_ins_performance.py <INS结果文件> <真值文件> [输出目录] [数据集名称]")
        print("\n示例:")
        print('  python evaluate_ins_performance.py \\')
        print('    results/awesome_gins_icm20602/pure_ins_trajectory.txt \\')
        print('    "data/多IMU车载GNSS、INS数据集/awesome-gins-datasets-main/ICM20602/ICM20602/truth.nav" \\')
        print('    results/awesome_gins_icm20602 \\')
        print('    "ICM20602"')
        sys.exit(1)
    
    ins_file = sys.argv[1]
    truth_file = sys.argv[2]
    output_dir = sys.argv[3] if len(sys.argv) > 3 else None
    dataset_name = sys.argv[4] if len(sys.argv) > 4 else "Unknown"
    
    evaluate_ins_performance(ins_file, truth_file, output_dir, dataset_name)
