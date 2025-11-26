"""
纯INS实验脚本
仅使用IMU进行惯导解算
"""

import sys
from pathlib import Path

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent.parent))

from config import ConfigManager
from io import IMUReader, GNSSReader, TimeAligner
from ins import INSMechanization
from eval import ErrorMetrics, Visualizer


def run_pure_ins_experiment(config_path=None):
    """
    运行纯INS实验
    
    流程：
    1. 加载配置
    2. 读取IMU数据和参考数据
    3. 初始化INS解算器
    4. 逐时间步进行INS积分
    5. 计算误差指标
    6. 可视化结果
    
    Args:
        config_path: 配置文件路径
    """
    print("=" * 50)
    print("纯INS惯导解算实验")
    print("=" * 50)
    
    # TODO: 1. 加载配置
    print("\n[1/6] 加载配置...")
    config = ConfigManager(config_path)
    
    # TODO: 2. 读取数据
    print("\n[2/6] 读取数据...")
    # 读取IMU数据
    # 读取参考数据（如果有）
    
    # TODO: 3. 初始化INS解算器
    print("\n[3/6] 初始化INS解算器...")
    # 设置初始状态（位置、速度、姿态）
    
    # TODO: 4. INS积分
    print("\n[4/6] 执行INS积分...")
    # 遍历所有IMU数据点
    # 更新名义状态
    # 保存轨迹
    
    # TODO: 5. 误差评估
    print("\n[5/6] 计算误差指标...")
    # 计算姿态误差、位置误差、速度误差
    # 统计各项指标
    
    # TODO: 6. 可视化
    print("\n[6/6] 生成可视化图表...")
    # 绘制轨迹、姿态、误差曲线
    
    print("\n实验完成！")
    print("=" * 50)


if __name__ == "__main__":
    # 运行纯INS实验
    run_pure_ins_experiment()
