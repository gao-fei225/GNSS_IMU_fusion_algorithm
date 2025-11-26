"""
可视化模块
绘制各种评估图表
"""

import matplotlib.pyplot as plt
import numpy as np


class Visualizer:
    """
    可视化类
    
    负责：
    - 绘制姿态随时间变化曲线
    - 绘制姿态误差随时间变化曲线
    - 绘制位置轨迹对比
    - 绘制误差直方图
    - 绘制误差箱线图
    - 标注GNSS遮挡区域
    
    输出：
    - 高质量的图片文件（用于论文）
    - 表格数据（用于论文）
    """
    
    def __init__(self, output_dir):
        """
        初始化可视化器
        
        Args:
            output_dir: 输出目录
        """
        self.output_dir = output_dir
        plt.rcParams['font.sans-serif'] = ['SimHei']  # 支持中文
        plt.rcParams['axes.unicode_minus'] = False
    
    def plot_attitude_comparison(self, timestamps, attitudes_dict, reference=None,
                                 outage_periods=None, output_name='attitude_comparison.png'):
        """
        绘制姿态对比图
        
        Args:
            timestamps: 时间戳数组
            attitudes_dict: 字典，包含不同方案的姿态序列
                           例如：{'纯INS': attitudes1, '松耦合': attitudes2, '紧耦合': attitudes3}
            reference: 参考真值姿态
            outage_periods: GNSS遮挡时间段
            output_name: 输出文件名
        """
        # TODO: 绘制滚转、俯仰、航向三个子图
        # TODO: 标注GNSS遮挡区域（灰色背景）
        # TODO: 添加图例、标签
        pass
    
    def plot_attitude_errors(self, timestamps, errors_dict, outage_periods=None,
                            output_name='attitude_errors.png'):
        """
        绘制姿态误差随时间变化曲线
        
        Args:
            timestamps: 时间戳数组
            errors_dict: 字典，包含不同方案的姿态误差序列
            outage_periods: GNSS遮挡时间段
            output_name: 输出文件名
        """
        # TODO: 绘制滚转、俯仰、航向误差三个子图
        # TODO: 标注GNSS遮挡区域
        pass
    
    def plot_trajectory_2d(self, trajectories_dict, reference=None,
                          output_name='trajectory_2d.png'):
        """
        绘制2D轨迹对比图（俯视图）
        
        Args:
            trajectories_dict: 字典，包含不同方案的位置序列
            reference: 参考真值轨迹
            output_name: 输出文件名
        """
        # TODO: 绘制X-Y平面的轨迹
        # TODO: 标注起点和终点
        pass
    
    def plot_trajectory_3d(self, trajectories_dict, reference=None,
                          output_name='trajectory_3d.png'):
        """
        绘制3D轨迹对比图
        
        Args:
            trajectories_dict: 字典，包含不同方案的位置序列
            reference: 参考真值轨迹
            output_name: 输出文件名
        """
        # TODO: 绘制3D轨迹
        pass
    
    def plot_error_histogram(self, errors_dict, bins=50,
                            output_name='error_histogram.png'):
        """
        绘制误差直方图
        
        Args:
            errors_dict: 字典，包含不同方案的误差序列
            bins: 直方图柱数
            output_name: 输出文件名
        """
        # TODO: 绘制误差分布直方图
        pass
    
    def plot_error_boxplot(self, errors_dict, output_name='error_boxplot.png'):
        """
        绘制误差箱线图
        
        Args:
            errors_dict: 字典，包含不同方案的误差序列
            output_name: 输出文件名
        """
        # TODO: 绘制箱线图，显示误差的统计分布
        pass
    
    def plot_position_errors(self, timestamps, position_errors_dict,
                            outage_periods=None,
                            output_name='position_errors.png'):
        """
        绘制位置误差随时间变化曲线
        
        Args:
            timestamps: 时间戳数组
            position_errors_dict: 位置误差字典
            outage_periods: GNSS遮挡时间段
            output_name: 输出文件名
        """
        # TODO: 绘制X、Y、Z位置误差和3D距离误差
        pass
    
    def plot_velocity_errors(self, timestamps, velocity_errors_dict,
                            outage_periods=None,
                            output_name='velocity_errors.png'):
        """
        绘制速度误差随时间变化曲线
        
        Args:
            timestamps: 时间戳数组
            velocity_errors_dict: 速度误差字典
            outage_periods: GNSS遮挡时间段
            output_name: 输出文件名
        """
        # TODO: 绘制速度误差曲线
        pass
    
    def plot_covariance(self, timestamps, covariance_trace,
                       output_name='covariance.png'):
        """
        绘制协方差变化曲线
        
        Args:
            timestamps: 时间戳数组
            covariance_trace: 协方差矩阵的迹或对角元素
            output_name: 输出文件名
        """
        # TODO: 绘制协方差演化曲线
        pass
    
    def shade_outage_regions(self, ax, timestamps, outage_periods):
        """
        在图表上标注GNSS遮挡区域
        
        Args:
            ax: matplotlib轴对象
            timestamps: 时间戳数组
            outage_periods: 遮挡时间段列表
        """
        # TODO: 在指定的轴上绘制灰色背景表示遮挡区域
        pass
    
    def create_comparison_table(self, statistics_dict, output_name='comparison_table.csv'):
        """
        创建对比表格
        
        Args:
            statistics_dict: 统计指标字典
            output_name: 输出文件名
        """
        # TODO: 生成用于论文的对比表格
        pass
