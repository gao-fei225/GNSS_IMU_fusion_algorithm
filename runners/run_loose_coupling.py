"""
松耦合ESKF实验脚本
使用GNSS导航解修正INS
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from config import ConfigManager
from io import IMUReader, GNSSReader, TimeAligner
from ins import INSMechanization
from models import ErrorState, ErrorDynamics
from models.measurement_models import LooseCouplingModel
from fusion import LooseCouplingESKF
from eval import ErrorMetrics, Visualizer


def run_loose_coupling_experiment(config_path=None):
    """
    运行松耦合ESKF实验
    
    流程：
    1. 加载配置
    2. 读取IMU、GNSS导航解、参考数据
    3. 时间对齐
    4. 初始化INS和ESKF
    5. 逐时间步进行：INS预测 + GNSS更新
    6. 计算误差指标
    7. 可视化结果
    
    Args:
        config_path: 配置文件路径
    """
    print("=" * 50)
    print("松耦合ESKF实验")
    print("=" * 50)
    
    # TODO: 1. 加载配置
    print("\n[1/7] 加载配置...")
    config = ConfigManager(config_path)
    
    # TODO: 2. 读取数据
    print("\n[2/7] 读取数据...")
    # 读取IMU数据
    # 读取GNSS导航解
    # 读取参考数据
    
    # TODO: 3. 时间对齐
    print("\n[3/7] 时间对齐...")
    # 将IMU和GNSS对齐到统一时间轴
    # 标记GNSS遮挡时间段
    
    # TODO: 4. 初始化
    print("\n[4/7] 初始化INS和ESKF...")
    # 初始化INS解算器
    # 初始化误差动力学模型
    # 初始化松耦合ESKF
    
    # TODO: 5. 融合处理
    print("\n[5/7] 执行松耦合融合...")
    # 遍历所有时间点
    # INS预测步骤
    # 若有GNSS且未遮挡，执行ESKF更新
    # 修正名义状态
    
    # TODO: 6. 误差评估
    print("\n[6/7] 计算误差指标...")
    # 计算各项误差指标
    # 分时间段统计
    
    # TODO: 7. 可视化
    print("\n[7/7] 生成可视化图表...")
    # 绘制融合结果
    
    print("\n实验完成！")
    print("=" * 50)


if __name__ == "__main__":
    run_loose_coupling_experiment()
