// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "c_agibot_hand.h"

namespace py = pybind11;

PYBIND11_MODULE(agibot_hand_core, m) {
  m.doc() = "AgibotHandO12 Python Interface";

  //  Bind TouchSensorData structures
  py::class_<TouchSensorData>(m, "TouchSensorData")
      .def(py::init<>())
      .def_readwrite("online_state", &TouchSensorData::online_state_)
      .def_property_readonly("channel_values",
                             [](const TouchSensorData &self) {
                               return std::vector<unsigned short>(
                                   self.channel_value_,
                                   self.channel_value_ + 9);
                             })
      .def_readwrite("normal_force", &TouchSensorData::normal_force_)
      .def_readwrite("tangent_force", &TouchSensorData::tangent_force_)
      .def_readwrite("tangent_force_angle",
                     &TouchSensorData::tangent_force_angle_)
      .def_property_readonly(
          "capacitive_approach", [](const TouchSensorData &self) {
            return std::vector<unsigned char>(self.capa_approach_,
                                              self.capa_approach_ + 4);
          });

  //  Bind JointMotorErrorReport structure with bit field accessors
  py::class_<JointMotorErrorReport>(m, "JointMotorErrorReport")
      .def(py::init<>())
      .def_property(
          "stalled",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.stalled_); },
          [](JointMotorErrorReport &self, bool value) { self.stalled_ = static_cast<unsigned char>(value); })
      .def_property(
          "overheat",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.overheat_); },
          [](JointMotorErrorReport &self, bool value) { self.overheat_ = static_cast<unsigned char>(value); })
      .def_property(
          "over_current",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.over_current_); },
          [](JointMotorErrorReport &self, bool value) { self.over_current_ = static_cast<unsigned char>(value); })
      .def_property(
          "motor_except",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.motor_except_); },
          [](JointMotorErrorReport &self, bool value) { self.motor_except_ = static_cast<unsigned char>(value); })
      .def_property(
          "commu_except",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.commu_except_); },
          [](JointMotorErrorReport &self, bool value) { self.commu_except_ = static_cast<unsigned char>(value); });

  py::class_<MixCtrl>(m, "MixCtrl")
      .def(py::init<>())
      .def_property(
          "joint_index",
          [](const MixCtrl &self) { return static_cast<int>(self.joint_index_); },
          [](MixCtrl &self, int value) { self.joint_index_ = static_cast<unsigned char>(value); })
      .def_property(
          "ctrl_mode",
          [](const MixCtrl &self) { return static_cast<int>(self.ctrl_mode_); },
          [](MixCtrl &self, int value) { self.ctrl_mode_ = static_cast<unsigned char>(value); })
      .def_readwrite("tgt_posi", &MixCtrl::tgt_posi_)
      .def_readwrite("tgt_velo", &MixCtrl::tgt_velo_)
      .def_readwrite("tgt_torque", &MixCtrl::tgt_torque_);

  //  Bind main class
  py::class_<AgibotHandO12>(m, "AgibotHandO12")
      .def(py::init<unsigned char>(), py::arg("device_id") = DEFAULT_DEVICE_ID)
      .def("set_device_id", &AgibotHandO12::SetDeviceId)
      .def("set_joint_position", &AgibotHandO12::SetJointMotorPosi)
      .def("get_joint_position", &AgibotHandO12::GetJointMotorPosi)
      .def("set_all_joint_positions", &AgibotHandO12::SetAllJointMotorPosi)
      .def("get_all_joint_positions", &AgibotHandO12::GetAllJointMotorPosi)
      .def("set_joint_velocity", &AgibotHandO12::SetJointMotorVelo)
      .def("get_joint_velocity", &AgibotHandO12::GetJointMotorVelo)
      .def("set_all_joint_velocities", &AgibotHandO12::SetAllJointMotorVelo)
      .def("get_all_joint_velocities", &AgibotHandO12::GetAllJointMotorVelo)
      .def("set_joint_torque", &AgibotHandO12::SetJointMotorTorque)
      .def("get_joint_torque", &AgibotHandO12::GetJointMotorTorque)
      .def("set_all_joint_torques", &AgibotHandO12::SetAllJointMotorTorque)
      .def("get_all_joint_torques", &AgibotHandO12::GetAllJointMotorTorque)
      .def("get_touch_sensor_data", [](AgibotHandO12 &self, int finger_index) {
        return self.GetTouchSensorData(static_cast<EFinger>(finger_index));
      })
      .def("set_control_mode", [](AgibotHandO12 &self, int joint_motor_index, int mode) {
        self.SetControlMode(joint_motor_index, static_cast<EControlMode>(mode));
      })
      .def("get_control_mode", [](AgibotHandO12 &self, int joint_motor_index) -> int {
        return static_cast<int>(self.GetControlMode(joint_motor_index));
      })
      .def("set_all_control_modes", &AgibotHandO12::SetAllControlMode)
      .def("get_all_control_modes", &AgibotHandO12::GetAllControlMode)
      .def("set_current_threshold", &AgibotHandO12::SetCurrentThreshold)
      .def("get_current_threshold", &AgibotHandO12::GetCurrentThreshold)
      .def("set_all_current_thresholds", &AgibotHandO12::SetAllCurrentThreshold)
      .def("get_all_current_thresholds", &AgibotHandO12::GetAllCurrentThreshold)
      .def("mix_ctrl_joint_motor", &AgibotHandO12::MixCtrlJointMotor)
      .def("get_error_report", &AgibotHandO12::GetErrorReport)
      .def("get_all_error_reports", &AgibotHandO12::GetAllErrorReport)
      .def("set_error_report_period", &AgibotHandO12::SetErrorReportPeriod)
      .def("set_all_error_report_periods", &AgibotHandO12::SetAllErrorReportPeriod)
      .def("get_temperature_report", &AgibotHandO12::GetTemperatureReport)
      .def("get_all_temperature_reports", &AgibotHandO12::GetAllTemperatureReport)
      .def("set_temperature_report_period", &AgibotHandO12::SetTemperReportPeriod)
      .def("set_all_temperature_report_periods", &AgibotHandO12::SetAllTemperReportPeriod)
      .def("get_current_report", &AgibotHandO12::GetCurrentReport)
      .def("get_all_current_reports", &AgibotHandO12::GetAllCurrentReport)
      .def("set_current_report_period", &AgibotHandO12::SetCurrentReportPeriod)
      .def("set_all_current_report_periods", &AgibotHandO12::SetAllCurrentReportPeriod)
      .def("get_vendor_info", &AgibotHandO12::GetVendorInfo)
      .def("get_device_info", &AgibotHandO12::GetDeviceInfo)
      .def("show_data_details", &AgibotHandO12::ShowDataDetails);
}