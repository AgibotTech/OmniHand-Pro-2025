# OmniHand Pro 2025 SDK Python API

## 枚举类型

### EFinger (手指枚举)

```python
# 可用值
EFinger.THUMB    # 拇指
EFinger.INDEX    # 食指
EFinger.MIDDLE   # 中指
EFinger.RING     # 无名指
EFinger.LITTLE   # 小指
```

### EControlMode (控制模式枚举)

```python
EControlMode.POSITION                    # 位置控制
EControlMode.VELOCITY                    # 速度控制
EControlMode.TORQUE                      # 力矩控制
EControlMode.POSITION_TORQUE             # 位置-力矩混合控制
EControlMode.VELOCITY_TORQUE             # 速度-力矩混合控制
EControlMode.POSITION_VELOCITY_TORQUE    # 位置-速度-力矩混合控制
EControlMode.UNKNOWN                     # 未知模式
```

## 数据结构

### TactileSensorData (触觉传感器数据)

```python
class TactileSensorData:
    online_state: int           # 传感器在线状态 (1:在线, 0:离线)
    channel_values: List[int]   # 各通道值 (9个通道)
    normal_force: int           # 法向力 (0-3000, 单位:0.1N)
    tangent_force: int          # 切向力
    tangent_force_angle: int    # 切向力角度 (0-359度)
    capacitive_approach: List[int]  # 自电容接近值 (4个通道)
```

### JointMotorErrorReport (关节电机错误报告)

```python
class JointMotorErrorReport:
    stalled: bool          # 堵转标志
    overheat: bool         # 过热标志
    over_current: bool     # 过流标志
    motor_except: bool     # 电机异常
    commu_except: bool     # 通信异常
```

### MixCtrl (混合控制结构)

```python
class MixCtrl:
    joint_index: int               # 关节索引 (1-12)
    ctrl_mode: int                 # 控制模式
    tgt_posi: Optional[int]        # 目标位置
    tgt_velo: Optional[int]        # 目标速度
    tgt_torque: Optional[int]      # 目标力矩
```

## 核心类

### AgibotHandO12

主要的灵巧手控制类，提供所有控制接口。

```python
class AgibotHandO12:
    def __init__(self, device_id: int = 1):
        """初始化灵巧手对象

        Args:
            device_id: 设备ID，默认为1
        """
        pass

    # 设备信息相关
    def get_vendor_info(self) -> str:
        """获取厂家信息"""
        pass

    def get_device_info(self) -> str:
        """获取设备信息"""
        pass

    def set_device_id(self, device_id: int) -> None:
        """设置设备ID"""
        pass

    # 位置控制
    def set_joint_position(self, joint_motor_index: int, position: int) -> None:
        """设置单个关节电机位置"""
        pass

    def get_joint_position(self, joint_motor_index: int) -> int:
        """获取单个关节电机位置"""
        pass

    def set_all_joint_positions(self, positions: List[int]) -> None:
        """批量设置所有关节电机位置"""
        pass

    def get_all_joint_positions(self) -> List[int]:
        """批量获取所有关节电机位置"""
        pass

    # 速度控制
    def set_joint_velocity(self, joint_motor_index: int, velocity: int) -> None:
        """设置单个关节电机速度"""
        pass

    def get_joint_velocity(self, joint_motor_index: int) -> int:
        """获取单个关节电机速度"""
        pass

    def set_all_joint_velocities(self, velocities: List[int]) -> None:
        """批量设置所有关节电机速度"""
        pass

    def get_all_joint_velocities(self) -> List[int]:
        """批量获取所有关节电机速度"""
        pass

    # 力矩控制
    def set_joint_torque(self, joint_motor_index: int, torque: int) -> None:
        """设置单个关节电机力矩"""
        pass

    def get_joint_torque(self, joint_motor_index: int) -> int:
        """获取单个关节电机力矩"""
        pass

    def set_all_joint_torques(self, torques: List[int]) -> None:
        """批量设置所有关节电机力矩"""
        pass

    def get_all_joint_torques(self) -> List[int]:
        """批量获取所有关节电机力矩"""
        pass

    # 传感器数据
    def get_tactile_sensor_data(self, finger: Finger) -> TactileSensorData:
        """获取指定手指的触觉传感器数据"""
        pass

    # 控制模式
    def set_control_mode(self, joint_motor_index: int, mode: ControlMode) -> None:
        """设置单个关节电机控制模式"""
        pass

    def get_control_mode(self, joint_motor_index: int) -> ControlMode:
        """获取单个关节电机控制模式"""
        pass

    def set_all_control_modes(self, modes: List[int]) -> None:
        """批量设置所有关节电机控制模式"""
        pass

    def get_all_control_modes(self) -> List[int]:
        """批量获取所有关节电机控制模式"""
        pass

    # 电流阈值控制
    def set_current_threshold(self, joint_motor_index: int, current_threshold: int) -> None:
        """设置单个关节电机电流阈值"""
        pass

    def get_current_threshold(self, joint_motor_index: int) -> int:
        """获取单个关节电机电流阈值"""
        pass

    def set_all_current_thresholds(self, current_thresholds: List[int]) -> None:
        """批量设置所有关节电机电流阈值"""
        pass

    def get_all_current_thresholds(self) -> List[int]:
        """批量获取所有关节电机电流阈值"""
        pass

    # 混合控制
    def mix_ctrl_joint_motor(self, mix_ctrls: List[MixCtrl]) -> None:
        """混合控制关节电机"""
        pass

    # 错误处理
    def get_error_report(self, joint_motor_index: int) -> JointMotorErrorReport:
        """获取单个关节电机错误报告"""
        pass

    def get_all_error_reports(self) -> List[JointMotorErrorReport]:
        """获取所有关节电机错误报告"""
        pass

    def set_error_report_period(self, joint_motor_index: int, period: int) -> None:
        """设置单个关节电机错误上报周期"""
        pass

    def set_all_error_report_periods(self, periods: List[int]) -> None:
        """批量设置所有关节电机错误上报周期"""
        pass

    # 温度监控
    def get_temperature_report(self, joint_motor_index: int) -> int:
        """获取单个关节电机温度报告"""
        pass

    def get_all_temperature_reports(self) -> List[int]:
        """获取所有关节电机温度报告"""
        pass

    def set_temperature_report_period(self, joint_motor_index: int, period: int) -> None:
        """设置单个关节电机温度上报周期"""
        pass

    def set_all_temperature_report_periods(self, periods: List[int]) -> None:
        """批量设置所有关节电机温度上报周期"""
        pass

    # 电流监控
    def get_current_report(self, joint_motor_index: int) -> int:
        """获取单个关节电机电流报告"""
        pass

    def get_all_current_reports(self) -> List[int]:
        """获取所有关节电机电流报告"""
        pass

    def set_current_report_period(self, joint_motor_index: int, period: int) -> None:
        """设置单个关节电机电流上报周期"""
        pass

    def set_all_current_report_periods(self, periods: List[int]) -> None:
        """批量设置所有关节电机电流上报周期"""
        pass

    # 调试功能
    def show_data_details(self, show: bool) -> None:
        """显示发送接收数据细节"""
        pass
```

## 详细 API 说明

### 设备信息相关

```python
def get_vendor_info(self) -> str:
    """获取厂家信息

    Returns:
        str: 厂家信息长字符串，包含产品型号、序列号、硬件版本、软件版本等信息
    """

def get_device_info(self) -> str:
    """获取设备信息

    Returns:
        str: 设备信息长字符串，包含设备的运行状态信息
    """

def set_device_id(self, device_id: int) -> None:
    """设置设备ID

    Args:
        device_id: 设备ID
    """
```

### 位置控制

```python
def set_joint_position(self, joint_motor_index: int, position: int) -> None:
    """设置单个关节电机位置

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        position: 电机位置，范围：0~2000
    """

def get_joint_position(self, joint_motor_index: int) -> int:
    """获取单个关节电机位置

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        int: 当前位置值
    """

def set_all_joint_positions(self, positions: List[int]) -> None:
    """批量设置所有关节电机位置

    Args:
        positions: 所有关节的目标位置列表，长度必须为12

    Note:
        需要提供完整的12个关节电机的位置数据
    """

def get_all_joint_positions(self) -> List[int]:
    """批量获取所有关节电机位置

    Returns:
        List[int]: 所有关节的当前位置列表，长度为12
    """
```

### 关节角控制

#### 关节角输出/输入顺序（右手）

| 索引 | 关节名称           | 最小角度 (rad) | 最大角度 (rad) | 最小角度 (°) | 最大角度 (°) | 速度限制 (rad/s) |
| ---- | ------------------ | -------------- | -------------- | ------------ | ------------ | ---------------- |
| 1    | R_thumb_roll_joint | -0.1745        | 0.8727         | -10          | 50           | 0.164            |
| 2    | R_thumb_abad_joint | -1.7453        | 0              | -100         | 0            | 0.164            |
| 3    | R_thumb_mcp_joint  | 0              | 0.8552         | 0            | 49           | 0.308            |
| 4    | R_index_abad_joint | -0.2094        | 0              | -12          | 0            | 0.164            |
| 5    | R_index_pip_joint  | 0              | 1.5708         | 0            | 90           | 0.308            |
| 6    | R_middle_pip_joint | 0              | 1.5708         | 0            | 90           | 0.308            |
| 7    | R_ring_abad_joint  | 0              | 0.1745         | 0            | 10           | 0.164            |
| 8    | R_ring_pip_joint   | 0              | 1.5708         | 0            | 90           | 0.308            |
| 9    | R_pinky_abad_joint | 0              | 0.1745         | 0            | 10           | 0.164            |
| 10   | R_pinky_pip_joint  | 0              | 1.5708         | 0            | 90           | 0.308            |

#### 关节角输出/输入顺序（左手）

| 索引 | 关节名称           | 最小角度 (rad) | 最大角度 (rad) | 最小角度 (°) | 最大角度 (°) | 速度限制 (rad/s) |
| ---- | ------------------ | -------------- | -------------- | ------------ | ------------ | ---------------- |
| 1    | L_thumb_roll_joint | -0.8727        | 0.1745         | -50          | 10           | 0.164            |
| 2    | L_thumb_abad_joint | 0              | 1.7453         | 0            | 100          | 0.164            |
| 3    | L_thumb_mcp_joint  | -0.8552        | 0              | -49          | 0            | 0.308            |
| 4    | L_index_abad_joint | 0              | 0.2094         | 0            | 12           | 0.164            |
| 5    | L_index_pip_joint  | 0              | 1.5708         | 0            | 90           | 0.308            |
| 6    | L_middle_pip_joint | 0              | 1.5708         | 0            | 90           | 0.308            |
| 7    | L_ring_abad_joint  | -0.1745        | 0              | -10          | 0            | 0.164            |
| 8    | L_ring_pip_joint   | 0              | 1.5708         | 0            | 90           | 0.308            |
| 9    | L_pinky_abad_joint | -0.2094        | 0              | -12          | 0            | 0.164            |
| 10   | L_pinky_pip_joint  | 0              | 1.5708         | 0            | 90           | 0.308            |

```python
def set_all_active_joint_angles(self, vec_angle: List[float]) -> None: ...
    """设置所有主动关节角（单位：弧度）

    Args:
        vec_angle，所有主动关节目标关节角列表，长度必须为10

    Note:
        具体输出顺序和限位请参考 assets 模型文件
    """

def get_all_active_joint_angles(self) -> List[float]: ...
    """获取所有主动关节角（单位：弧度）

    Returns:
        List[float]: 所有主动关节当前关节角列表，长度为10

    Note:
        具体输出顺序和限位请参考 assets 模型文件
    """

def get_all_joint_angles(self) -> List[float]: ...
    """获取所有主动和被动关节角（单位：弧度）

    Returns:
        List[float]: 所有主动和被动关节当前关节角列表，长度为10

    Note:
        具体输出顺序和限位请参考 assets 模型文件
    """
```

### 速度控制

```python
def set_joint_velocity(self, joint_motor_index: int, velocity: int) -> None:
    """设置单个关节电机速度

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        velocity: 目标速度值
    """

def get_joint_velocity(self, joint_motor_index: int) -> int:
    """获取单个关节电机速度

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        int: 当前速度值
    """

def set_all_joint_velocities(self, velocities: List[int]) -> None:
    """批量设置所有关节电机速度

    Args:
        velocities: 所有关节的目标速度列表，长度必须为12
    """

def get_all_joint_velocities(self) -> List[int]:
    """批量获取所有关节电机速度

    Returns:
        List[int]: 所有关节的当前速度列表，长度为12
    """
```

### 力矩控制

```python
def set_joint_torque(self, joint_motor_index: int, torque: int) -> None:
    """设置单个关节电机力矩

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        torque: 目标力矩值
    """

def get_joint_torque(self, joint_motor_index: int) -> int:
    """获取单个关节电机力矩

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        int: 当前力矩值
    """

def set_all_joint_torques(self, torques: List[int]) -> None:
    """批量设置所有关节电机力矩

    Args:
        torques: 所有关节的目标力矩列表，长度必须为12
    """

def get_all_joint_torques(self) -> List[int]:
    """批量获取所有关节电机力矩

    Returns:
        List[int]: 所有关节的当前力矩列表，长度为12
    """
```

### 传感器数据

```python
def get_tactile_sensor_data(self, finger: Finger) -> TactileSensorData:
    """获取指定手指的触觉传感器数据

    Args:
        finger: 手指枚举值，可选值：Finger.THUMB, Finger.INDEX,
               Finger.MIDDLE, Finger.RING, Finger.LITTLE

    Returns:
        TactileSensorData: 触觉传感器数据结构
    """
```

### 控制模式

```python
def set_control_mode(self, joint_motor_index: int, mode: ControlMode) -> None:
    """设置单个关节电机控制模式

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        mode: 控制模式枚举值
    """

def get_control_mode(self, joint_motor_index: int) -> ControlMode:
    """获取单个关节电机控制模式

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        ControlMode: 当前控制模式
    """

def set_all_control_modes(self, modes: List[int]) -> None:
    """批量设置所有关节电机控制模式

    Args:
        modes: 控制模式列表，长度必须为12
    """

def get_all_control_modes(self) -> List[int]:
    """批量获取所有关节电机控制模式

    Returns:
        List[int]: 控制模式列表，长度为12
    """
```

### 电流阈值控制

```python
def set_current_threshold(self, joint_motor_index: int, current_threshold: int) -> None:
    """设置单个关节电机电流阈值

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        current_threshold: 电流阈值
    """

def get_current_threshold(self, joint_motor_index: int) -> int:
    """获取单个关节电机电流阈值

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        int: 当前电流阈值
    """

def set_all_current_thresholds(self, current_thresholds: List[int]) -> None:
    """批量设置所有关节电机电流阈值

    Args:
        current_thresholds: 电流阈值列表，长度必须为12
    """

def get_all_current_thresholds(self) -> List[int]:
    """批量获取所有关节电机电流阈值

    Returns:
        List[int]: 电流阈值列表，长度为12
    """
```

### 混合控制

```python
def mix_ctrl_joint_motor(self, mix_ctrls: List[MixCtrl]) -> None:
    """混合控制关节电机

    Args:
        mix_ctrls: 混合控制参数列表
    """
```

### 错误处理

```python
def get_error_report(self, joint_motor_index: int) -> JointMotorErrorReport:
    """获取单个关节电机错误报告

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        JointMotorErrorReport: 错误报告结构
    """

def get_all_error_reports(self) -> List[JointMotorErrorReport]:
    """获取所有关节电机错误报告

    Returns:
        List[JointMotorErrorReport]: 错误报告列表，长度为12
    """

def set_error_report_period(self, joint_motor_index: int, period: int) -> None:
    """设置单个关节电机错误上报周期

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        period: 上报周期（单位：ms）
    """

def set_all_error_report_periods(self, periods: List[int]) -> None:
    """批量设置所有关节电机错误上报周期

    Args:
        periods: 上报周期列表，长度必须为12
    """
```

### 温度监控

```python
def get_temperature_report(self, joint_motor_index: int) -> int:
    """获取单个关节电机温度报告

    Note:
        查询前需要先设置上报周期

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        int: 当前温度值
    """

def get_all_temperature_reports(self) -> List[int]:
    """获取所有关节电机温度报告

    Note:
        查询前需要先设置上报周期

    Returns:
        List[int]: 温度值列表，长度为12
    """

def set_temperature_report_period(self, joint_motor_index: int, period: int) -> None:
    """设置单个关节电机温度上报周期

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        period: 上报周期（单位：ms）
    """

def set_all_temperature_report_periods(self, periods: List[int]) -> None:
    """批量设置所有关节电机温度上报周期

    Args:
        periods: 上报周期列表，长度必须为12
    """
```

### 电流监控

```python
def get_current_report(self, joint_motor_index: int) -> int:
    """获取单个关节电机电流报告

    Note:
        查询前需要先设置上报周期

    Args:
        joint_motor_index: 关节电机索引 (1-12)

    Returns:
        int: 当前电流值
    """

def get_all_current_reports(self) -> List[int]:
    """获取所有关节电机电流报告

    Note:
        查询前需要先设置上报周期

    Returns:
        List[int]: 电流值列表，长度为12
    """

def set_current_report_period(self, joint_motor_index: int, period: int) -> None:
    """设置单个关节电机电流上报周期

    Args:
        joint_motor_index: 关节电机索引 (1-12)
        period: 上报周期（单位：ms）
    """

def set_all_current_report_periods(self, periods: List[int]) -> None:
    """批量设置所有关节电机电流上报周期

    Args:
        periods: 上报周期列表，长度必须为12
    """
```

### 调试功能

```python
def show_data_details(self, show: bool) -> None:
    """显示发送接收数据细节

    Args:
        show: 是否显示数据细节
    """
```
