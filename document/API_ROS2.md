# omnihand_pro 灵动款 2025 SDK ROS2 接口


## ROS2 接口

| 话题名                                           | 话题描述 | 订阅 or 发布 | 消息类型                                                                                              | 备注 |  
|:----------------------------------------------|  :----:  |:--------:|---------------------------------------------------------------------------------------------------|  ---  |
| `/agihand/omnihand_pro/left/control_mode`  | 关节电机控制模式 |    发布    | [omnihand_pro_node_msgs.msg.ControlMode](#omnihand_pro_node_msgs::msg::ControlMode) | 
| `/agihand/omnihand_pro/left/current_report`  | 关节电机电流上报 |    发布    | [omnihand_pro_node_msgs.msg.CurrentReport](#omnihand_pro_node_msgs::msg::CurrentReport) | 
| `/agihand/omnihand_pro/left/current_threshold`  |关节电机电流阈值 |    发布    | [omnihand_pro_node_msgs.msg.CurrentThreshold](#omnihand_pro_node_msgs::msg::CurrentThreshold) | 
| `/agihand/omnihand_pro/left/motor_error_report`  | 关节电机错误上报 |    发布    | [omnihand_pro_node_msgs.msg.MotorErrorReport](#omnihand_pro_node_msgs::msg::MotorErrorReport) | 
| `/agihand/omnihand_pro/left/motor_pos`  | 关节电机位置 |    发布    | [omnihand_pro_node_msgs.msg.MotorPos](#omnihand_pro_node_msgs::msg::MotorPos) | 
| `/agihand/omnihand_pro/left/motor_vel`  | 关节电机速度 |    发布    | [omnihand_pro_node_msgs.msg.MotorVel](#omnihand_pro_node_msgs::msg::MotorVel) | 
| `/agihand/omnihand_pro/left/tactile_sensor`  | 触觉传感器数据 |    发布    | [omnihand_pro_node_msgs.msg.TactileSensor](#omnihand_pro_node_msgs::msg::TactileSensor) | 
| `/agihand/omnihand_pro/left/temperature_report`  | 关节电机温度上报 |    发布    | [omnihand_pro_node_msgs.msg.TemperatureReport](#omnihand_pro_node_msgs::msg::TemperatureReport) | 
| `/agihand/omnihand_pro/left/motor_angle`  | 关节电机角度 |    发布    | [omnihand_pro_node_msgs.msg.MotorAngle](#omnihand_pro_node_msgs::msg::MotorAngle) | 
| `/agihand/omnihand_pro/left/control_mode_cmd`  | 关节电机控制模式 |    订阅    | [omnihand_pro_node_msgs.msg.ControlMode](#omnihand_pro_node_msgs::msg::ControlMode) | 
| `/agihand/omnihand_pro/left/current_threshold_cmd`  | 关节电机电流阈值 |    发布    | [omnihand_pro_node_msgs.msg.CurrentThreshold](#omnihand_pro_node_msgs::msg::CurrentThreshold) | 
| `/agihand/omnihand_pro/left/motor_pos_cmd`  | 关节电机位置 |    发布    | [omnihand_pro_node_msgs.msg.MotorPos](#omnihand_pro_node_msgs::msg::MotorPos) | 
| `/agihand/omnihand_pro/left/motor_vel_cmd`  | 关节电机速度 |    发布    | [omnihand_pro_node_msgs.msg.MotorPos](#omnihand_pro_node_msgs::msg::MotorPos) | 
| `/agihand/omnihand_pro/left/motor_angle_cmd`  | 关节电机角度 |    订阅    | [omnihand_pro_node_msgs.msg.MotorAngle](#omnihand_pro_node_msgs::msg::MotorAngle) | 
| `/agihand/omnihand_pro/right/control_mode`  | 关节电机控制模式 |    发布    | [omnihand_pro_node_msgs.msg.ControlMode](#omnihand_pro_node_msgs::msg::ControlMode) | 
| `/agihand/omnihand_pro/right/current_report`  | 关节电机电流上报 |    发布    | [omnihand_pro_node_msgs.msg.CurrentReport](#omnihand_pro_node_msgs::msg::CurrentReport) | 
| `/agihand/omnihand_pro/right/current_threshold`  |关节电机电流阈值 |    发布    | [omnihand_pro_node_msgs.msg.CurrentThreshold](#omnihand_pro_node_msgs::msg::CurrentThreshold) | 
| `/agihand/omnihand_pro/right/motor_error_report`  | 关节电机错误上报 |    发布    | [omnihand_pro_node_msgs.msg.MotorErrorReport](#omnihand_pro_node_msgs::msg::MotorErrorReport) | 
| `/agihand/omnihand_pro/right/motor_pos`  | 关节电机位置 |    发布    | [omnihand_pro_node_msgs.msg.MotorPos](#omnihand_pro_node_msgs::msg::MotorPos) | 
| `/agihand/omnihand_pro/right/motor_vel`  | 关节电机速度 |    发布    | [omnihand_pro_node_msgs.msg.MotorVel](#omnihand_pro_node_msgs::msg::MotorVel) | 
| `/agihand/omnihand_pro/right/tactile_sensor`  | 触觉传感器数据 |    发布    | [omnihand_pro_node_msgs.msg.TactileSensor](#omnihand_pro_node_msgs::msg::TactileSensor) | 
| `/agihand/omnihand_pro/right/temperature_report`  | 关节电机温度上报 |    发布    | [omnihand_pro_node_msgs.msg.TemperatureReport](#omnihand_pro_node_msgs::msg::TemperatureReport) | 
| `/agihand/omnihand_pro/right/motor_angle`  | 关节电机角度 |    发布    | [omnihand_pro_node_msgs.msg.MotorAngle](#omnihand_pro_node_msgs::msg::MotorAngle) | 
| `/agihand/omnihand_pro/right/control_mode_cmd`  | 关节电机控制模式 |    订阅    | [omnihand_pro_node_msgs.msg.ControlMode](#omnihand_pro_node_msgs::msg::ControlMode) | 
| `/agihand/omnihand_pro/right/current_threshold_cmd`  | 关节电机电流阈值 |    发布    | [omnihand_pro_node_msgs.msg.CurrentThreshold](#omnihand_pro_node_msgs::msg::CurrentThreshold) | 
| `/agihand/omnihand_pro/right/motor_pos_cmd`  | 关节电机位置 |    发布    | [omnihand_pro_node_msgs.msg.MotorPos](#omnihand_pro_node_msgs::msg::MotorPos) | 
| `/agihand/omnihand_pro/right/motor_vel_cmd`  | 关节电机速度 |    发布    | [omnihand_pro_node_msgs.msg.MotorPos](#omnihand_pro_node_msgs::msg::MotorPos) | 
| `/agihand/omnihand_pro/right/motor_angle_cmd`  | 关节电机角度 |    订阅    | [omnihand_pro_node_msgs.msg.MotorAngle](#omnihand_pro_node_msgs::msg::MotorAngle) | 


## 使用方式
### 编译
  参考根目录下的README.md

## 运行
  export LD_LIBRARY_PATH=`pwd`/build/install/lib/:$LD_LIBRARY_PATH
  cd build/install/bin/
  ./omnihand_pro_node

  接下来就可以订阅和发布相关话题了, 可以使用node/scripts下的python 脚本来订阅和发布话题


## omnihand_pro_node_msgs::msg::ControlMode

```python
std_msgs/Header header

# ePosi = 0,                    // 位置控制
# eVelo = 1,                    // 速度控制
# eTorque = 2,                  // 力矩控制
# ePosiTorque = 3,              // 位置-力矩混合控制
# eVeloTorque = 4,              // 速度-力矩混合控制
# ePosiVeloTorque = 5,          // 位置-速度-力矩混合控制
# eUnknown = 10                 // 未知模式

int8[] modes
```


## omnihand_pro_node_msgs.msg.CurrentReport

```python
std_msgs/Header header
uint16[] current_reports
```


## omnihand_pro_node_msgs.msg.CurrentThreshold

```python
std_msgs/Header header
int16[] current_thresholds

```


## omnihand_pro_node_msgs.msg.MotorErrorReport

```python
std_msgs/Header header

# unsigned char stalled_ : 1;      // 堵转标志
# unsigned char overheat_ : 1;     // 过热标志
# unsigned char over_current_ : 1; // 过流标志
# unsigned char motor_except_ : 1; // 电机异常
# unsigned char commu_except_ : 1; // 通信异常
# unsigned char res1_ : 3;         // 保留位
# unsigned char res2_;             // 保留字节

uint16[] error_reports

```

## omnihand_pro_node_msgs.msg.MotorPos

```python
std_msgs/Header header
int16[] pos
```

## omnihand_pro_node_msgs.msg.MotorVel

```python
std_msgs/Header header
int16[] vels
```

## omnihand_pro_node_msgs.msg.MotorAngle

```python
std_msgs/Header header
float64[] angles
```

## omnihand_pro_node_msgs.msg.TactileSensor

```python
std_msgs/Header header
TactileSensorData[] tactile_datas
```

## omnihand_pro_node_msgs.msg.TactileSensorData

```python
uint8 online_state         # 1~传感器在线;0~传感器不在线
uint16[] channel_value     # 各通道值
uint16 normal_force        # 法向力:(0-3000,0.1N)
uint16 tangent_force       # 切向力
uint16 tangent_force_angle # 切向力角度，指尖向上为0度，顺时针旋转:(0~359)
uint8[] capa_approach     # 自电容接近
```

## omnihand_pro_node_msgs::msg::TemperatureReport

```python
std_msgs/Header header
uint16[] temperature_reports
```
