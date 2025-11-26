"""
完整对比实验脚本
运行所有方案并进行对比
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from config import ConfigManager
from eval import ErrorMetrics, Visualizer


def run_full_experiment(config_path=None):
    """
    运行完整对比实验
    
    流程：
    1. 运行纯INS实验
    2. 运行松耦合ESKF实验
    3. 运行紧耦合ESKF实验（无约束）
    4. 运行紧耦合ESKF实验（有约束）
    5. 进行遮挡实验
    6. 对比所有方案的精度
    7. 生成论文用的图表和表格
    
    Args:
        config_path: 配置文件路径
    """
    print("=" * 60)
    print("INS/GNSS融合系统完整对比实验")
    print("=" * 60)
    
    # TODO: 1. 加载配置
    print("\n[阶段1] 加载配置...")
    config = ConfigManager(config_path)
    
    # TODO: 2. 准备数据
    print("\n[阶段2] 准备数据...")
    # 读取所有数据
    # 进行时间对齐
    
    # TODO: 3. 运行纯INS
    print("\n[阶段3] 运行纯INS实验...")
    print("-" * 60)
    # 执行纯INS
    # 保存结果
    
    # TODO: 4. 运行松耦合
    print("\n[阶段4] 运行松耦合ESKF实验...")
    print("-" * 60)
    # 执行松耦合
    # 保存结果
    
    # TODO: 5. 运行紧耦合（无约束）
    print("\n[阶段5] 运行紧耦合ESKF实验（无约束）...")
    print("-" * 60)
    # 执行紧耦合
    # 保存结果
    
    # TODO: 6. 运行紧耦合（有约束）
    print("\n[阶段6] 运行紧耦合ESKF实验（有约束）...")
    print("-" * 60)
    # 执行紧耦合+约束
    # 保存结果
    
    # TODO: 7. 遮挡对比实验
    print("\n[阶段7] GNSS遮挡对比实验...")
    print("-" * 60)
    # 对每种方案进行遮挡实验
    # 记录遮挡期间的误差变化
    
    # TODO: 8. 精度对比
    print("\n[阶段8] 精度对比分析...")
    print("-" * 60)
    # 统计各方案的误差指标
    # 生成对比表格
    
    # TODO: 9. 可视化
    print("\n[阶段9] 生成可视化图表...")
    print("-" * 60)
    # 绘制所有对比图
    # 姿态对比、误差对比、轨迹对比
    # 遮挡期间的误差演化
    
    # TODO: 10. 生成报告
    print("\n[阶段10] 生成实验报告...")
    print("-" * 60)
    # 汇总所有结果
    # 生成论文用的表格和图片
    
    print("\n" + "=" * 60)
    print("完整实验完成！")
    print("结果保存在: results/")
    print("=" * 60)


if __name__ == "__main__":
    run_full_experiment()
