#!/usr/bin/env python3
"""
配置管理器 - 读取/写入 ground_station.yaml
"""
import os
import yaml


class ConfigManager:
    """管理地面站配置文件的读写"""

    DEFAULT_CONFIG_PATH = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "ground_station.yaml"
    )

    def __init__(self, config_path=None):
        self.config_path = config_path or self.DEFAULT_CONFIG_PATH
        self._config = {}
        self.load()

    def load(self):
        """从 YAML 文件加载配置"""
        if not os.path.isfile(self.config_path):
            raise FileNotFoundError(f"配置文件未找到: {self.config_path}")
        with open(self.config_path, 'r', encoding='utf-8') as f:
            self._config = yaml.safe_load(f) or {}
        return self._config

    def save(self):
        """保存配置到 YAML 文件"""
        with open(self.config_path, 'w', encoding='utf-8') as f:
            yaml.dump(self._config, f, allow_unicode=True, sort_keys=False)

    def get(self, key_path, default=None):
        """
        通过点号路径获取配置值
        例如: get('flight_controller.serial_port')
        """
        keys = key_path.split('.')
        val = self._config
        for k in keys:
            if isinstance(val, dict) and k in val:
                val = val[k]
            else:
                return default
        return val

    def set(self, key_path, value):
        """
        通过点号路径设置配置值
        例如: set('flight_controller.serial_port', '/dev/ttyUSB1')
        """
        keys = key_path.split('.')
        cfg = self._config
        for k in keys[:-1]:
            if k not in cfg:
                cfg[k] = {}
            cfg = cfg[k]
        cfg[keys[-1]] = value

    @property
    def config(self):
        return self._config


# 快捷单例
def get_config():
    return ConfigManager().config


if __name__ == "__main__":
    cm = ConfigManager()
    print("FCU URL:", cm.get('flight_controller.fcu_url'))
    print("VRPN IP:", cm.get('motion_capture.server_ip'))
    print("RTSP URL:", cm.get('camera.rtsp_url'))
