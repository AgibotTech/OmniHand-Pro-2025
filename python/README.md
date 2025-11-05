# OmniHand Pro 2025 SDK (Python) 使用说明

## 目录

- [概述](#概述)
- [系统要求](#系统要求)
- [安装指南](#安装指南)
- [快速开始](#快速开始)
- [API 参考](#api参考)

## 概述

OmniHand Pro 2025 SDK (Python) 是 OmniHand Pro 2025 系列产品的 Python 开发包，提供基于 Python 的 API 接口，方便用户快速接入 OmniHand Pro 2025 系列产品，实现控制和数据采集等功能。

## 系统要求

### 硬件要求

- ZLG can

### 软件要求

- Python 3.10 or later
- Ubuntu 22.04 x86_64

## 安装指南

### 1. 环境准备

```bash
# 若需要源码编译 python whl 包则需要安装以下依赖
pip3 install build setuptools wheel
```

### 3. 驱动安装

```bash
# 参考 ZLG 官网说明安装驱动
```

### 2. SDK 安装

到 xxxxx.git 下载对应版本的 python whl 包，如 agibot_hand_py-0.8.0-cp310-cp310-linux_x86_64.whl

```bash
pip install agibot_hand_py-0.8.0-cp310-cp310-linux_x86_64.whl --force-reinstall
```

## 快速开始

进入 python/example 目录

```python
python3 ./demo.py
```

## API 参考

### 枚举类型

#### EFinger (手指枚举)

```python
class Finger:
    THUMB = 1    # 拇指
    INDEX = 2    # 食指
    MIDDLE = 3   # 中指
    RING = 4     # 无名指
    LITTLE = 5   # 小指
```

#### EControlMode (控制模式枚举)

```python
class ControlMode:
    POSITION = 0                    # 位置控制
    VELOCITY = 1                    # 速度控制
    TORQUE = 2                      # 力矩控制
    POSITION_TORQUE = 3             # 位置-力矩混合控制
    VELOCITY_TORQUE = 4             # 速度-力矩混合控制
    POSITION_VELOCITY_TORQUE = 5    # 位置-速度-力矩混合控制
    UNKNOWN = 10                    # 未知模式
```

### 数据结构

#### TactileSensorData (触觉传感器数据)

```python
class TactileSensorData:
    online_state: int           # 传感器在线状态 (1:在线, 0:离线)
    channel_values: List[int]   # 各通道值 (9个通道)
    normal_force: int           # 法向力 (0-3000, 单位:0.1N)
    tangent_force: int         # 切向力
    tangent_force_angle: int   # 切向力角度 (0-359度)
    capacitive_approach: List[int]  # 自电容接近值 (4个通道)
```

#### JointMotorErrorReport (关节电机错误报告)

```python
class JointMotorErrorReport:
    stalled: bool          # 堵转标志
    overheat: bool        # 过热标志
    over_current: bool    # 过流标志
    motor_except: bool    # 电机异常
    commu_except: bool    # 通信异常
```

### 函数接口

#### 设备信息相关

```python
def get_vendor_info() -> str:
    """获取厂家信息

    Returns:
        str: 厂家信息长字符串，包含产品型号、序列号、硬件版本、软件版本等信息
    """
```

```python
def get_device_info() -> str:
    """获取设备信息

    Returns:
        str: 设备信息长字符串，包含设备的运行状态信息
    """
```

#### 位置控制

```python
def set_joint_position(joint_motor_index: int, position: int) -> None:
    """设置单个关节电机位置

    Args:
        joint_motor_index (int): 关节电机索引
        position (int): 电机位置，范围：0~2000
    """
```

```python
def get_joint_position(joint_motor_index: int) -> int:
    """获取单个关节电机位置

    Args:
        joint_motor_index (int): 关节电机索引

    Returns:
        int: 当前位置值
    """
```

```python
def set_all_joint_positions(positions: List[int]) -> None:
    """批量设置所有关节电机位置

    Note:
        需要提供完整的12个关节电机的位置数据

    Args:
        positions (List[int]): 所有关节的目标位置列表，长度必须为12
    """
```

```python
def get_all_joint_positions() -> List[int]:
    """批量获取所有关节电机位置

    Returns:
        List[int]: 所有关节的当前位置列表，长度为12
    """
```

#### 速度控制

```python
def set_joint_velocity(joint_motor_index: int, velocity: int) -> None:
    """设置单个关节电机速度

    Args:
        joint_motor_index (int): 关节电机索引
        velocity (int): 目标速度值
    """
```

```python
def get_joint_velocity(joint_motor_index: int) -> int:
    """获取单个关节电机速度

    Args:
        joint_motor_index (int): 关节电机索引

    Returns:
        int: 当前速度值
    """
```

```python
def set_all_joint_velocities(velocities: List[int]) -> None:
    """批量设置所有关节电机速度

    Args:
        velocities (List[int]): 所有关节的目标速度列表，长度必须为12
    """
```

```python
def get_all_joint_velocities() -> List[int]:
    """批量获取所有关节电机速度

    Returns:
        List[int]: 所有关节的当前速度列表，长度为12
    """
```

#### 力矩控制

```python
def set_joint_torque(joint_motor_index: int, torque: int) -> None:
    """设置单个关节电机力矩

    Args:
        joint_motor_index (int): 关节电机索引
        torque (int): 目标力矩值
    """
```

```python
def get_joint_torque(joint_motor_index: int) -> int:
    """获取单个关节电机力矩

    Args:
        joint_motor_index (int): 关节电机索引

    Returns:
        int: 当前力矩值
    """
```

```python
def set_all_joint_torques(torques: List[int]) -> None:
    """批量设置所有关节电机力矩

    Args:
        torques (List[int]): 所有关节的目标力矩列表，长度必须为12
    """
```

```python
def get_all_joint_torques() -> List[int]:
    """批量获取所有关节电机力矩

    Returns:
        List[int]: 所有关节的当前力矩列表，长度为12
    """
```

#### 传感器数据

```python
def get_tactile_sensor_data(finger: Finger) -> TactileSensorData:
    """获取指定手指的触觉传感器数据

    Args:
        finger (Finger): 手指枚举值，可选值：THUMB, INDEX, MIDDLE, RING, LITTLE

    Returns:
        TactileSensorData: 触觉传感器数据结构，包含：
            - online_state: 传感器在线状态 (1:在线, 0:离线)
            - channel_values: 各通道值 (9个通道)
            - normal_force: 法向力 (0-3000, 单位:0.1N)
            - tangent_force: 切向力
            - tangent_force_angle: 切向力角度 (0-359度)
            - capacitive_approach: 自电容接近值 (4个通道)
    """
```

#### 控制模式

```python
def set_control_mode(joint_motor_index: int, mode: ControlMode) -> None:
    """设置单个关节电机控制模式

    Args:
        joint_motor_index (int): 关节电机索引
        mode (ControlMode): 控制模式枚举值，可选值：
            - POSITION: 位置控制
            - VELOCITY: 速度控制
            - TORQUE: 力矩控制
            - POSITION_TORQUE: 位置-力矩混合控制
            - VELOCITY_TORQUE: 速度-力矩混合控制
            - POSITION_VELOCITY_TORQUE: 位置-速度-力矩混合控制
    """
```

```python
def get_control_mode(joint_motor_index: int) -> ControlMode:
    """获取单个关节电机控制模式

    Args:
        joint_motor_index (int): 关节电机索引

    Returns:
        ControlMode: 当前控制模式
    """
```

#### 错误处理

```python
def get_error_report(joint_motor_index: int) -> JointMotorErrorReport:
    """获取单个关节电机错误报告

    Args:
        joint_motor_index (int): 关节电机索引

    Returns:
        JointMotorErrorReport: 错误报告结构，包含：
            - stalled: 堵转标志
            - overheat: 过热标志
            - over_current: 过流标志
            - motor_except: 电机异常
            - commu_except: 通信异常
    """
```

```python
def get_all_error_reports() -> List[JointMotorErrorReport]:
    """获取所有关节电机错误报告

    Returns:
        List[JointMotorErrorReport]: 所有关节的错误报告列表，长度为12
    """
```

#### 温度监控

```python
def get_temperature_report(joint_motor_index: int) -> int:
    """获取单个关节电机温度报告

    Args:
        joint_motor_index (int): 关节电机索引

    Returns:
        int: 当前温度值
    """
```

```python
def get_all_temperature_reports() -> List[int]:
    """获取所有关节电机温度报告

    Returns:
        List[int]: 所有关节的温度值列表，长度为12
    """
```

```python
def set_temperature_report_period(joint_motor_index: int, period: int) -> None:
    """设置单个关节电机温度上报周期

    Args:
        joint_motor_index (int): 关节电机索引
        period (int): 上报周期（单位：ms）
    """
```

#### 固件更新

```python
def update_firmware(file_name: str) -> None:
    """更新固件

    Args:
        file_name (str): 固件文件名

    Note:
        更新过程中请勿断电或重启设备
    """
```

## 常见问题

**版权声明：**
