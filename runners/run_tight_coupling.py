"""
紧耦合ESKF实验脚本
使用GNSS原始观测进行融合
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from config import ConfigManager
from io import IMUReader, GNSSReader, TimeAligner
from ins import INSMechanization
from models import ErrorState, ErrorDynamics
from models.measurement_models import TightCouplingModel
from fusion import TightCouplingESKF, ConstraintHandler
from eval import ErrorMetrics, Visualizer


def run_tight_coupling_experiment(config_path=None, enable_constraints=True):
    """
    运行紧耦合ESKF实验
    
    流程：
    1. 加载配置
    2. 读取IMU、GNSS原始观测、参考数据
    3. 时间对齐
    4. 初始化INS、ESKF和约束处理器
    5. 逐时间步进行：INS预测 + GNSS更新 + 约束更新
    6. 模拟GNSS遮挡场景
    7. 计算误差指标
    8. 可视化结果
    
    Args:
        config_path: 配置文件路径
        enable_constraints: 是否启用约束（基线长度、零速等）
    """
    print("=" * 50)
    print("紧耦合ESKF实验")
    if enable_constraints:
        print("（启用约束）")
    print("=" * 50)
    
    # TODO: 1. 加载配置
    print("\n[1/8] 加载配置...")
    config = ConfigManager(config_path)
    
    # TODO: 2. 读取数据
    print("\n[2/8] 读取数据...")
    # 读取IMU数据
    # 读取GNSS原始观测
    # 读取参考数据
    
    # TODO: 3. 时间对齐
    print("\n[3/8] 时间对齐...")
    # 将IMU和GNSS对齐到统一时间轴
    # 标记GNSS遮挡时间段
    
    # TODO: 4. 初始化
    print("\n[4/8] 初始化INS、ESKF和约束处理器...")
    # 初始化INS解算器
    # 初始化误差动力学模型
    # 初始化紧耦合ESKF
    # 初始化约束处理器（如果启用）
    
    # TODO: 5. 融合处理
    print("\n[5/8] 执行紧耦合融合...")
    # 遍历所有时间点
    # INS预测步骤
    # 若有GNSS原始观测且未遮挡，执行ESKF更新
    # 若启用约束，应用约束更新
    # 修正名义状态
    
    # TODO: 6. 遮挡实验
    print("\n[6/8] 遮挡实验...")
    # 按配置的遮挡时间段，屏蔽GNSS
    # 观察误差发散情况
    
    # TODO: 7. 误差评估
    print("\n[7/8] 计算误差指标...")
    # 计算各项误差指标
    # 分时间段统计（全程、正常、遮挡）
    
    # TODO: 8. 可视化
    print("\n[8/8] 生成可视化图表...")
    # 绘制融合结果
    # 标注遮挡区域
    
    print("\n实验完成！")
    print("=" * 50)


if __name__ == "__main__":
    run_tight_coupling_experiment()
