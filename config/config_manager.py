"""
配置管理器
负责读取和管理项目配置参数
"""

import yaml
from pathlib import Path


class ConfigManager:
    """
    配置管理类
    
    负责：
    - 读取YAML配置文件
    - 提供数据路径配置（IMU数据、GNSS数据、参考轨迹）
    - 提供IMU参数（采样频率、噪声水平、零偏随机游走）
    - 提供GNSS参数（输出频率、伪距噪声、多普勒噪声）
    - 提供ESKF参数（状态维数、初始协方差、过程噪声Q、量测噪声R）
    - 提供实验设置（松/紧耦合开关、GNSS遮挡时间段、NN辅助开关）
    """
    
    def __init__(self, config_path=None):
        """
        初始化配置管理器
        
        Args:
            config_path: 配置文件路径，如果为None则使用默认配置
        """
        self.config = {}
        if config_path:
            self.load_config(config_path)
        else:
            self._set_default_config()
    
    def load_config(self, config_path):
        """
        从YAML文件加载配置
        
        Args:
            config_path: 配置文件路径
        """
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"配置文件不存在: {config_path}")
        
        with open(config_file, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        print(f"成功加载配置文件: {config_path}")
    
    def _set_default_config(self):
        """
        设置默认配置参数
        """
        # 查找默认配置文件
        default_config_path = Path(__file__).parent / 'default_config.yaml'
        
        if default_config_path.exists():
            self.load_config(default_config_path)
        else:
            raise FileNotFoundError(f"默认配置文件不存在: {default_config_path}")
    
    def get_data_paths(self):
        """
        获取数据路径配置
        
        Returns:
            dict: 包含IMU、GNSS、参考数据的文件路径
        """
        return self.config.get('data_paths', {})
    
    def get_imu_params(self):
        """
        获取IMU参数
        
        Returns:
            dict: IMU采样频率、噪声水平等参数
        """
        return self.config.get('imu_params', {})
    
    def get_gnss_params(self):
        """
        获取GNSS参数
        
        Returns:
            dict: GNSS输出频率、噪声水平等参数
        """
        return self.config.get('gnss_params', {})
    
    def get_eskf_params(self):
        """
        获取ESKF滤波参数
        
        Returns:
            dict: 状态维数、初始协方差、过程噪声、量测噪声等
        """
        return self.config.get('eskf_params', {})
    
    def get_experiment_settings(self):
        """
        获取实验设置
        
        Returns:
            dict: 实验开关、遮挡时间段等设置
        """
        return self.config.get('experiment', {})
    
    def get_all_config(self):
        """
        获取完整配置字典
        
        Returns:
            dict: 完整配置字典
        """
        return self.config.copy()
    
    def get_value(self, key_path, default=None):
        """
        获取配置中的特定值（支持嵌套键）
        
        Args:
            key_path: 键路径，用点号分隔，例如 'imu_params.sample_rate'
            default: 默认值，如果键不存在则返回此值
        
        Returns:
            配置值或默认值
        
        Example:
            config.get_value('imu_params.sample_rate')  # 返回100.0
            config.get_value('eskf_params.initial_cov.attitude')  # 返回0.01
        """
        keys = key_path.split('.')
        value = self.config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value
    
    def print_config(self):
        """
        打印配置信息（格式化输出）
        """
        print("\n" + "=" * 60)
        print("配置信息")
        print("=" * 60)
        
        print("\n[数据路径]")
        for key, value in self.get_data_paths().items():
            print(f"  {key}: {value}")
        
        print("\n[IMU参数]")
        imu_params = self.get_imu_params()
        print(f"  采样频率: {imu_params.get('sample_rate', 'N/A')} Hz")
        print(f"  陀螺噪声: {imu_params.get('gyro_noise', 'N/A')} rad/s/√Hz")
        print(f"  加速度噪声: {imu_params.get('accel_noise', 'N/A')} m/s²/√Hz")
        print(f"  陀螺零偏游走: {imu_params.get('gyro_bias_walk', 'N/A')} rad/s²/√Hz")
        print(f"  加计零偏游走: {imu_params.get('accel_bias_walk', 'N/A')} m/s³/√Hz")
        
        print("\n[GNSS参数]")
        gnss_params = self.get_gnss_params()
        print(f"  输出频率: {gnss_params.get('output_rate', 'N/A')} Hz")
        print(f"  位置噪声: {gnss_params.get('position_noise', 'N/A')} m")
        print(f"  速度噪声: {gnss_params.get('velocity_noise', 'N/A')} m/s")
        print(f"  伪距噪声: {gnss_params.get('pseudorange_noise', 'N/A')} m")
        print(f"  多普勒噪声: {gnss_params.get('doppler_noise', 'N/A')} m/s")
        
        print("\n[ESKF参数]")
        eskf_params = self.get_eskf_params()
        print(f"  状态维数: {eskf_params.get('state_dim', 'N/A')}")
        initial_cov = eskf_params.get('initial_cov', {})
        print(f"  初始姿态协方差: {initial_cov.get('attitude', 'N/A')} rad²")
        print(f"  初始速度协方差: {initial_cov.get('velocity', 'N/A')} m²/s²")
        print(f"  初始位置协方差: {initial_cov.get('position', 'N/A')} m²")
        
        print("\n[实验设置]")
        exp_settings = self.get_experiment_settings()
        print(f"  启用松耦合: {exp_settings.get('enable_loose_coupling', 'N/A')}")
        print(f"  启用紧耦合: {exp_settings.get('enable_tight_coupling', 'N/A')}")
        print(f"  启用约束: {exp_settings.get('enable_constraints', 'N/A')}")
        print(f"  启用NN辅助: {exp_settings.get('enable_nn_assist', 'N/A')}")
        print(f"  GNSS遮挡时间段: {exp_settings.get('gnss_outages', [])}")
        
        print("\n" + "=" * 60)
    
    def save_config(self, output_path):
        """
        保存当前配置到YAML文件
        
        Args:
            output_path: 输出文件路径
        """
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True)
        
        print(f"配置已保存到: {output_path}")
    
    def validate_config(self):
        """
        验证配置的完整性
        
        Returns:
            tuple: (is_valid, error_messages)
        """
        errors = []
        
        # 检查必需的配置项
        required_sections = ['data_paths', 'imu_params', 'gnss_params', 'eskf_params', 'experiment']
        for section in required_sections:
            if section not in self.config or not self.config[section]:
                errors.append(f"缺少必需的配置节: {section}")
        
        # 检查数据路径
        data_paths = self.get_data_paths()
        required_paths = ['imu_data', 'output_dir']
        for path_key in required_paths:
            if path_key not in data_paths:
                errors.append(f"缺少必需的数据路径: {path_key}")
        
        # 检查IMU参数
        imu_params = self.get_imu_params()
        required_imu = ['sample_rate', 'gyro_noise', 'accel_noise']
        for param in required_imu:
            if param not in imu_params:
                errors.append(f"缺少必需的IMU参数: {param}")
        
        # 检查GNSS参数
        gnss_params = self.get_gnss_params()
        required_gnss = ['output_rate']
        for param in required_gnss:
            if param not in gnss_params:
                errors.append(f"缺少必需的GNSS参数: {param}")
        
        # 检查ESKF参数
        eskf_params = self.get_eskf_params()
        if 'state_dim' not in eskf_params:
            errors.append("缺少必需的ESKF参数: state_dim")
        if 'initial_cov' not in eskf_params:
            errors.append("缺少必需的ESKF参数: initial_cov")
        
        is_valid = len(errors) == 0
        return is_valid, errors
