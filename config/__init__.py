"""
配置模块
负责管理所有可调参数，包括数据路径、传感器参数、ESKF参数和实验设置
"""

from .config_manager import ConfigManager

__all__ = ['ConfigManager']
