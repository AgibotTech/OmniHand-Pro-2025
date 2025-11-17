# OmniHand Pro 2025 SDK C++ API

## 枚举类型

### EFinger (手指枚举)

```cpp
enum class EFinger : unsigned char {
    eThumb = 0x01,    // 拇指
    eIndex = 0x02,    // 食指
    eMiddle = 0x03,   // 中指
    eRing = 0x04,     // 无名指
    eLittle = 0x05,   // 小指
};
```

### EControlMode (控制模式枚举)

```cpp
enum class EControlMode : unsigned char {
    ePosi = 0,                    // 位置控制
    eVelo = 1,                    // 速度控制
    eTorque = 2,                  // 力矩控制
    ePosiTorque = 3,              // 位置-力矩混合控制
    eVeloTorque = 4,              // 速度-力矩混合控制
    ePosiVeloTorque = 5,          // 位置-速度-力矩混合控制
    eUnknown = 10                 // 未知模式
};
```

### EMsgType (消息类型枚举)

```cpp
enum class EMsgType : unsigned char {
    eVendorInfo = 0x01,           // 厂家信息
    eDeviceInfo = 0x02,           // 设备信息
    eCurrentThreshold = 0x03,     // 电流阈值
    eTactileSensor = 0x05,          // 触觉传感器
    eCtrlMode = 0x10,             // 控制模式
    eTorqueCtrl = 0x11,           // 力矩控制
    eVeloCtrl = 0x12,             // 速度控制
    ePosiCtrl = 0x13,             // 位置控制
    eMixCtrl = 0x14,              // 混合控制
    eErrorReport = 0x20,          // 错误报告
    eTemperatureReport = 0x21,    // 温度报告
    eCurrentReport = 0x22,        // 电流报告
};
```

## 数据结构

### TactileSensorData (触觉传感器数据)

```cpp
struct TactileSensorData {
    unsigned char online_state_;          // 传感器在线状态 (1:在线, 0:离线)
    unsigned short channel_value_[9];     // 各通道值 (9个通道)
    unsigned short normal_force_;         // 法向力 (0-3000, 单位:0.1N)
    unsigned short tangent_force_;        // 切向力
    unsigned short tangent_force_angle_;  // 切向力角度 (0-359度)
    unsigned char capa_approach_[4];      // 自电容接近值 (4个通道)
};
```

### JointMotorErrorReport (关节电机错误报告)

```cpp
struct JointMotorErrorReport {
    unsigned char stalled_ : 1;      // 堵转标志
    unsigned char overheat_ : 1;     // 过热标志
    unsigned char over_current_ : 1; // 过流标志
    unsigned char motor_except_ : 1; // 电机异常
    unsigned char commu_except_ : 1; // 通信异常
    unsigned char res1_ : 3;         // 保留位
    unsigned char res2_;             // 保留字节
};
```

### MixCtrl (混合控制结构)

```cpp
struct MixCtrl {
    unsigned char joint_index_ : 5;      // 关节索引 (1-12)
    unsigned char ctrl_mode_ : 3;        // 控制模式
    std::optional<short> tgt_posi_;      // 目标位置
    std::optional<short> tgt_velo_;      // 目标速度
    std::optional<short> tgt_torque_;    // 目标力矩
};
```

### CanId (CAN 报文 ID 结构)

```cpp
struct CanId {
    unsigned char device_id_ : 7;    // 设备ID
    unsigned char rw_flag_ : 1;      // 读写标志
    unsigned char product_id_ : 7;   // 产品ID
    unsigned char res1 : 1;          // 保留位
    unsigned char msg_type_;         // 消息类型
    unsigned char msg_id_;           // 消息ID
};
```

### Version (版本信息结构)

```cpp
struct Version {
    unsigned char major_;    // 主版本号
    unsigned char minor_;    // 次版本号
    unsigned char patch_;    // 补丁版本号
    unsigned char res_;      // 保留字节
};
```

### CommuParams (通信参数结构)

```cpp
struct CommuParams {
    unsigned char bitrate_;      // 波特率
    unsigned char sample_point_; // 采样点
    unsigned char dbitrate_;     // 数据波特率
    unsigned char dsample_point_; // 数据采样点
};
```

## AgibotHandO12 类及其函数接口

### 构造函数

```cpp
/**
 * @brief 构造函数
 * @param device_id 设备ID, 默认设备填写1
 * @param canfd_id  can设备ID, 默认设备填写0
 * @param hand_type 手型枚举值
 */
explicit AgibotHandO12(unsigned char device_id, unsigned char canfd_id, unsigned char canfd_id);
```

### 设备信息相关

```cpp
/**
 * @brief 获取厂家信息
 * @return 厂家信息长字符串，包含产品型号、序列号、硬件版本、软件版本等信息
 */
std::string GetVendorInfo();

/**
 * @brief 获取设备信息
 * @return 设备信息长字符串，包含设备的运行状态信息
 */
std::string GetDeviceInfo();

/**
 * @brief 设置设备ID
 * @param device_id 设备ID
 */
void SetDeviceId(unsigned char device_id);
```

### 位置控制

```cpp
/**
 * @brief 设置单个关节电机位置
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param posi 电机位置，范围：0~2000
 */
void SetJointMotorPosi(unsigned char joint_motor_index, short posi);

/**
 * @brief 获取单个关节电机位置
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 当前位置值
 */
short GetJointMotorPosi(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机位置
 * @param vec_posi 所有关节的目标位置向量，长度必须为12
 * @note 注意要提供完整的12个关节电机的位置数据
 */
void SetAllJointMotorPosi(std::vector<short> vec_posi);

/**
 * @brief 批量获取所有关节电机位置
 * @return 所有关节的当前位置向量，长度为12
 */
std::vector<short> GetAllJointMotorPosi();
```

### 关节角位置控制

#### 关节角输出/输入顺序（右手）

| 索引 | 关节名称 (URDF)       | 最小角度 (rad)      | 最大角度 (rad)     | 最小角度 (°) | 最大角度 (°) | 速度限制 (rad/s) |
| ---- | --------------------- | ------------------- | ------------------ | ------------ | ------------ | ---------------- |
| 1    | `R_thumb_roll_joint`  | 0.0                 | 0.9424777960769379 | 0.0          | 54.0         | 2.38             |
| 2    | `R_thumb_abad_joint`  | -1.387536755335492  | 0.0                | -79.5        | 0.0          | 2.33             |
| 3    | `R_thumb_mcp_joint`   | -0.8272860654453121 | 0.0                | -47.4        | 0.0          | 1.35             |
| 4    | `R_thumb_pip_joint`   | -1.2915436464758039 | 0.0                | -74.0        | 0.0          | 1.87             |
| 5    | `R_index_abad_joint`  | -0.2617993877991494 | 0.2617993877991494 | -15.0        | 15.0         | 2.16             |
| 6    | `R_index_mcp_joint`   | 0.0                 | 1.3526301702956054 | 0.0          | 77.5         | 2.22             |
| 7    | `R_index_pip_joint`   | 0.0                 | 1.530653753999027  | 0.0          | 87.7         | 2.49             |
| 8    | `R_middle_abad_joint` | -0.2617993877991494 | 0.2617993877991494 | -15.0        | 15.0         | 2.16             |
| 9    | `R_middle_mcp_joint`  | 0.0                 | 1.3578661580515883 | 0.0          | 77.8         | 2.22             |
| 10   | `R_middle_pip_joint`  | 0.0                 | 1.8151424220741028 | 0.0          | 104.0        | 2.16             |
| 11   | `R_ring_mcp_joint`    | 0.0                 | 1.53588974175501   | 0.0          | 88.0         | 2.54             |
| 12   | `R_pinky_mcp_joint`   | 0.0                 | 1.53588974175501   | 0.0          | 88.0         | 2.54             |

#### 关节角输出/输入顺序（左手）

| 索引 | 关节名称 (URDF)       | 最小角度 (rad)      | 最大角度 (rad)     | 最小角度 (°) | 最大角度 (°) | 速度限制 (rad/s) |
| ---- | --------------------- | ------------------- | ------------------ | ------------ | ------------ | ---------------- |
| 1    | `L_thumb_roll_joint`  | -0.9424777960769379 | 0.0                | -54.0        | 0.0          | 2.38             |
| 2    | `L_thumb_abad_joint`  | 0.0                 | 1.387536755335492  | 0.0          | 79.5         | 2.33             |
| 3    | `L_thumb_mcp_joint`   | -0.8272860654453121 | 0.0                | -47.4        | 0.0          | 1.35             |
| 4    | `L_thumb_pip_joint`   | -1.2915436464758039 | 0.0                | -74.0        | 0.0          | 1.87             |
| 5    | `L_index_abad_joint`  | -0.2617993877991494 | 0.2617993877991494 | -15.0        | 15.0         | 2.16             |
| 6    | `L_index_mcp_joint`   | 0.0                 | 1.3526301702956054 | 0.0          | 77.5         | 2.22             |
| 7    | `L_index_pip_joint`   | 0.0                 | 1.530653753999027  | 0.0          | 87.7         | 2.49             |
| 8    | `L_middle_abad_joint` | -0.2617993877991494 | 0.2617993877991494 | -15.0        | 15.0         | 2.16             |
| 9    | `L_middle_mcp_joint`  | 0.0                 | 1.3578661580515883 | 0.0          | 77.8         | 2.22             |
| 10   | `L_middle_pip_joint`  | 0.0                 | 1.8151424220741028 | 0.0          | 104.0        | 2.16             |
| 11   | `L_ring_mcp_joint`    | 0.0                 | 1.53588974175501   | 0.0          | 88.0         | 2.54             |
| 12   | `L_pinky_mcp_joint`   | 0.0                 | 1.53588974175501   | 0.0          | 88.0         | 2.54             |

**注意：** 左手通过 xacro 镜像生成，关节限制与右手相同，仅关节名称前缀从 `R_` 变为 `L_`。角度值已从弧度转换为度数（保留一位小数）。

```cpp
/**
    * @brief 设置所有主动关节角度
    * @param angles 关节角度向量（单位：弧度），长度必须为10
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
void SetAllActiveJointAngles(const std::vector<double>& angles);

/**
    * @brief 获取所有主动关节角度
    * @return 关节角度向量（单位：弧度），长度为10
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
std::vector<double> GetAllActiveJointAngles() const;

/**
    * @brief 获取所有关节角度（包括主动和被动）
    * @return 关节角度向量（单位：弧度）
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
std::vector<double> GetAllJointAngles() const;/**
    * @brief 设置所有主动关节角度
    * @param angles 关节角度向量（单位：弧度），长度必须为10
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
void SetAllActiveJointAngles(const std::vector<double>& angles);

/**
    * @brief 获取所有主动关节角度
    * @return 关节角度向量（单位：弧度），长度为10
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
std::vector<double> GetAllActiveJointAngles() const;

/**
    * @brief 获取所有关节角度（包括主动和被动）
    * @return 关节角度向量（单位：弧度）
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
std::vector<double> GetAllJointAngles() const;
```

### 速度控制

```cpp
/**
 * @brief 设置单个关节电机速度
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param velo 目标速度值
 */
void SetJointMotorVelo(unsigned char joint_motor_index, short velo);

/**
 * @brief 获取单个关节电机速度
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 当前速度值
 */
short GetJointMotorVelo(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机速度
 * @param vec_velo 所有关节的目标速度向量，长度必须为12
 */
void SetAllJointMotorVelo(std::vector<short> vec_velo);

/**
 * @brief 批量获取所有关节电机速度
 * @return 所有关节的当前速度向量，长度为12
 */
std::vector<short> GetAllJointMotorVelo();
```

### 力矩控制

```cpp
/**
 * @brief 设置单个关节电机力矩
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param torque 目标力矩值
 */
void SetJointMotorTorque(unsigned char joint_motor_index, short torque);

/**
 * @brief 获取单个关节电机力矩
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 当前力矩值
 */
short GetJointMotorTorque(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机力矩
 * @param vec_torque 所有关节的目标力矩向量，长度必须为12
 */
void SetAllJointMotorTorque(std::vector<short> vec_torque);

/**
 * @brief 批量获取所有关节电机力矩
 * @return 所有关节的当前力矩向量，长度为12
 */
std::vector<short> GetAllJointMotorTorque();
```

### 传感器数据

```cpp
/**
 * @brief 获取指定手指的触觉传感器数据
 * @param eFinger 手指枚举值
 * @return 触觉传感器数据结构
 */
TactileSensorData GetTactileSensorData(EFinger eFinger);
```

### 控制模式

```cpp
/**
 * @brief 设置单个关节电机控制模式
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param mode 控制模式枚举值
 */
void SetControlMode(unsigned char joint_motor_index, EControlMode mode);

/**
 * @brief 获取单个关节电机控制模式
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 当前控制模式
 */
EControlMode GetControlMode(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机控制模式
 * @param vec_ctrl_mode 控制模式向量，长度必须为12
 */
void SetAllControlMode(std::vector<unsigned char> vec_ctrl_mode);

/**
 * @brief 批量获取所有关节电机控制模式
 * @return 控制模式向量，长度为12
 */
std::vector<unsigned char> GetAllControlMode();
```

### 电流阈值控制

```cpp
/**
 * @brief 设置单个关节电机电流阈值
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param current_threshold 电流阈值
 */
void SetCurrentThreshold(unsigned char joint_motor_index, short current_threshold);

/**
 * @brief 获取单个关节电机电流阈值
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 当前电流阈值
 */
short GetCurrentThreshold(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机电流阈值
 * @param vec_current_threshold 电流阈值向量，长度必须为12
 */
void SetAllCurrentThreshold(std::vector<short> vec_current_threshold);

/**
 * @brief 批量获取所有关节电机电流阈值
 * @return 电流阈值向量，长度为12
 */
std::vector<short> GetAllCurrentThreshold();
```

### 混合控制

```cpp
/**
 * @brief 混合控制关节电机
 * @param vec_mix_ctrl 混合控制参数向量
 */
void MixCtrlJointMotor(std::vector<MixCtrl> vec_mix_ctrl);
```

### 错误处理

```cpp
/**
 * @brief 获取单个关节电机错误报告
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 错误报告结构
 */
JointMotorErrorReport GetErrorReport(unsigned char joint_motor_index);

/**
 * @brief 获取所有关节电机错误报告
 * @return 错误报告向量，长度为12
 */
std::vector<JointMotorErrorReport> GetAllErrorReport();

/**
 * @brief 设置单个关节电机错误上报周期
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param period 上报周期（单位：ms）
 */
void SetErrorReportPeriod(unsigned char joint_motor_index, unsigned short period);

/**
 * @brief 批量设置所有关节电机错误上报周期
 * @param vec_period 上报周期向量，长度必须为12
 */
void SetAllErrorReportPeriod(std::vector<unsigned short> vec_period);
```

### 温度监控

```cpp
/**
 * @brief 获取单个关节电机温度报告
 * @note 查询前需要先设置上报周期
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 当前温度值
 */
unsigned short GetTemperatureReport(unsigned char joint_motor_index);

/**
 * @brief 获取所有关节电机温度报告
 * @note 查询前需要先设置上报周期
 * @return 温度值向量，长度为12
 */
std::vector<unsigned short> GetAllTemperatureReport();

/**
 * @brief 设置单个关节电机温度上报周期
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param period 上报周期（单位：ms）
 */
void SetTemperReportPeriod(unsigned char joint_motor_index, unsigned short period);

/**
 * @brief 批量设置所有关节电机温度上报周期
 * @param vec_period 上报周期向量，长度必须为12
 */
void SetAllTemperReportPeriod(std::vector<unsigned short> vec_period);
```

### 电流监控

```cpp
/**
 * @brief 获取单个关节电机电流报告
 * @note 查询前需要先设置上报周期
 * @param joint_motor_index 关节电机索引 (1-12)
 * @return 当前电流值
 */
short GetCurrentReport(unsigned char joint_motor_index);

/**
 * @brief 获取所有关节电机电流报告
 * @note 查询前需要先设置上报周期
 * @return 电流值向量，长度为12
 */
std::vector<unsigned short> GetAllCurrentReport();

/**
 * @brief 设置单个关节电机电流上报周期
 * @param joint_motor_index 关节电机索引 (1-12)
 * @param period 上报周期（单位：ms）
 */
void SetCurrentReportPeriod(unsigned char joint_motor_index, unsigned short period);

/**
 * @brief 批量设置所有关节电机电流上报周期
 * @param vec_period 上报周期向量，长度必须为12
 */
void SetAllCurrentReportPeriod(std::vector<unsigned short> vec_period);
```

### 调试功能

```cpp
/**
 * @brief 显示发送接收数据细节
 * @param show 是否显示数据细节
 */
void ShowDataDetails(bool show) const;
```
