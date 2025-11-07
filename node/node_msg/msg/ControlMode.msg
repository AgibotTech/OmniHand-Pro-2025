std_msgs/Header header

# ePosi = 0,                    // 位置控制
# eVelo = 1,                    // 速度控制
# eTorque = 2,                  // 力矩控制
# ePosiTorque = 3,              // 位置-力矩混合控制
# eVeloTorque = 4,              // 速度-力矩混合控制
# ePosiVeloTorque = 5,          // 位置-速度-力矩混合控制
# eUnknown = 10                 // 未知模式

int8[] modes