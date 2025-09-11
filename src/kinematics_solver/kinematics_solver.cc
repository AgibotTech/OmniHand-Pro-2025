#include "kinematics_solver/kinematics_solver.h"

namespace omnihandProSDK {
O12KinematicsSolver::O12KinematicsSolver(const bool &hand_type) : hand_type_(hand_type) {
}

O12KinematicsSolver::~O12KinematicsSolver() {}

std::vector<int> O12KinematicsSolver::SetHandGesture(const int &gesture_num) {
  std::vector<double> active_joint_pos(MaxActiveJoint, 0.0);
  std::vector<int> motor_input(MaxActiveJoint, 0);
  switch (gesture_num) {
    case HOME:
      active_joint_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      break;

    case PAPER:
      active_joint_pos = {0.34, 0.0, -0.33, -0.375, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      break;

    case FIST:
      active_joint_pos = {0.5, -0.2, 0.0, -1.2, 0.0, 1.35, 1.53, 0.0, 1.36, 1.82, 1.55, 1.54};
      break;

    case OK:
      active_joint_pos = {0.3, -0.531, 0.0, -0.95, 0.0, 0.5, 0.65, 0.0, 0.0, 0.0, 0.0, 0.0};
      break;

    default:
      break;
  }
  motor_input = ConvertJoint2Actuator(active_joint_pos);
  return motor_input;
}

void O12KinematicsSolver::ClampAbadMcpJoint(double &x, double &y) {
  // Upper boundary points (modifiable as needed)
  double x1_top = -0.26, y1_top = 1.17273;
  double x2_top = 0.26, y2_top = 1.17273;

  // Lower boundary points (modifiable as needed)
  double x1_bot = -0.26, y1_bot = 0.163636;
  double x2_bot = 0.26, y2_bot = 0.163636;

  // Maximum and minimum y-values (modifiable as needed)
  double y_top = 1.35, y_bot = 0.0;

  // Upper boundary slope/intercept
  double dx_top = x2_top;
  double k_top = (x < 0) ? ((y_top - y1_top) / dx_top)
                         : ((y2_top - y_top) / dx_top);
  double b_top = y_top;

  // Lower boundary slope/intercept
  double k_bot = (x < 0) ? (y1_bot / x1_bot)
                         : (y2_bot / x2_bot);
  double b_bot = 0.0;

  // Calculate the upper and lower y-boundaries for the current x
  double y_max = k_top * x + b_top;
  double y_min = k_bot * x;

  // Boundary check and in-place modification (print warning and notify before adjustment)
  if (y < y_min) {
    std::cerr << "[WARN] the angle of mcp (" << y << ") is below lower bound "
              << y_min << ",adjusted to boundary.\n";
    y = y_min;
  } else if (y > y_max) {
    std::cerr << "[WARN] the angle of mcp (" << y << ") is above upper bound "
              << y_max << ",adjusted to boundary.\n";
    y = y_max;
  }
}

bool O12KinematicsSolver::CheckJointPos(std::vector<double> &active_joint_pos) {
  if (active_joint_pos.size() != active_joint_max_.size()) {
    std::cerr << "Joint input size does not match expected joint count.";
    return false;
  }

  Clamp(active_joint_max_, active_joint_min_, active_joint_pos);
  // Index finger abad & MCP joint
  ClampAbadMcpJoint(active_joint_pos[ActiveJointIndexAbAd], active_joint_pos[ActiveJointIndexMCP]);
  // Middle finger abad & MCP joint
  ClampAbadMcpJoint(active_joint_pos[ActiveJointMiddleABAD], active_joint_pos[ActiveJointMiddleMCP]);

  return true;
}

bool O12KinematicsSolver::CheckActuatorInput(const std::vector<int> &actuator_input) {
  if (actuator_input.size() != active_joint_max_.size()) {
    std::cerr << "motor Input size does not match expected motor count.";
  }

  bool all_valid = true;
  for (int i = 0; i < 12; ++i) {
    if (actuator_input[i] < motor_input_min_[i] || actuator_input[i] > motor_input_max_[i]) {
      all_valid = false;
    }
  }
  return all_valid;
}

std::vector<int> O12KinematicsSolver::ConvertJoint2Actuator(
    const std::vector<double> &active_joint_pos) {
  std::vector<double> pos = active_joint_pos;
  CheckJointPos(pos);
  std::vector<double> motor_l(MaxActiveJoint, 0);
  std::vector<int> motor_input(MaxActiveJoint, 0);
  motor_l = ActiveJoint2MotorLength(pos);
  motor_input = MotorLength2MotorInput(motor_l);
  return motor_input;
}

std::vector<double> O12KinematicsSolver::ActiveJoint2MotorLength(
    const std::vector<double> &active_jont_pos) {
  std::vector<double> joint_pos = active_jont_pos;

  // Convert joint angles to positive values
  if (hand_type_) {
    joint_pos[ActiveJointThumbRoll] = -joint_pos[ActiveJointThumbRoll];
    joint_pos[ActiveJointIndexAbAd] = -joint_pos[ActiveJointIndexAbAd];
    joint_pos[ActiveJointMiddleABAD] = -joint_pos[ActiveJointMiddleABAD];
    joint_pos[ActiveJointThumbMCP] = -joint_pos[ActiveJointThumbMCP];
    joint_pos[ActiveJointThumbPIP] = -joint_pos[ActiveJointThumbPIP];
  } else {
    joint_pos[ActiveJointThumbAbAd] = -joint_pos[ActiveJointThumbAbAd];
    joint_pos[ActiveJointThumbMCP] = -joint_pos[ActiveJointThumbMCP];
    joint_pos[ActiveJointThumbPIP] = -joint_pos[ActiveJointThumbPIP];
  }

  std::vector<double> motor_length(ActuatorCount, 0.0);
  motor_length[ActuatorThumbRoll] =
      PredictPoly(joint_pos[ActiveJointThumbRoll], coeffs_thumb_roll_) * 1e-3;
  motor_length[ActuatorThumbABAD] =
      PredictPoly(joint_pos[ActiveJointThumbAbAd], coeffs_thumb_abad_) * 1e-3;
  motor_length[ActuatorThumbMCP] =
      PredictPoly(joint_pos[ActiveJointThumbMCP], coeffs_thumb_mcp_) * 1e-3;
  motor_length[ActuatorThumbPIP] =
      PredictPoly(joint_pos[ActiveJointThumbPIP], coeffs_thumb_pip_) * 1e-3;

  motor_length[ActuatorIndex1] =
      PredictPoly33(joint_pos[ActiveJointIndexAbAd],
                    joint_pos[ActiveJointIndexMCP], coeffs_index1_);
  motor_length[ActuatorIndex2] =
      PredictPoly33(joint_pos[ActiveJointIndexAbAd],
                    joint_pos[ActiveJointIndexMCP], coeffs_index2_);
  motor_length[ActuatorIndex3] = FingerPIPFitPredict(
      joint_pos[ActiveJointIndexAbAd], joint_pos[ActiveJointIndexMCP],
      joint_pos[ActiveJointIndexPIP], index_PIP_coeffs_);
  motor_length[ActuatorMiddle1] =
      PredictPoly33(joint_pos[ActiveJointMiddleABAD],
                    joint_pos[ActiveJointMiddleMCP], coeffs_index1_);
  motor_length[ActuatorMiddle2] =
      PredictPoly33(joint_pos[ActiveJointMiddleABAD],
                    joint_pos[ActiveJointMiddleMCP], coeffs_index2_);
  motor_length[ActuatorMiddle3] = FingerPIPFitPredict(
      joint_pos[ActiveJointMiddleABAD], joint_pos[ActiveJointMiddleMCP],
      joint_pos[ActiveJointMiddlePIP], middle_PIP_coeffs_);
  motor_length[ActuatorRing] =
      PredictPoly(joint_pos[ActiveJointRingMCP], coeffs_ring_pinky_) * 1e-3;
  motor_length[ActuatorPinky] =
      PredictPoly(joint_pos[ActiveJointPinkyMCP], coeffs_ring_pinky_) * 1e-3;
  Clamp(motor_max_, motor_min_, motor_length);
  return motor_length;
}

std::vector<int> O12KinematicsSolver::MotorLength2MotorInput(const std::vector<double> &motor_length) {
  std::vector<int> motor_input(ActuatorCount, 0);
  motor_input = Scale(motor_max_, motor_min_, motor_length, motor_input_max_,
                      motor_input_min_);
  return motor_input;
}

// Convert Actuator input values to joint angles
std::vector<double> O12KinematicsSolver::ConvertActuator2Joint(
    const std::vector<int> &motor_input) {
  // check actuator_input size
  CheckActuatorInput(motor_input);
  std::vector<double> motor_l(ActuatorCount, 0);
  std::vector<double> pos(MaxActiveJoint, 0);
  motor_l = MotorInput2MotorLength(motor_input);
  pos = MotorLength2ActiveJoint(motor_l);
  return pos;
}

std::vector<double> O12KinematicsSolver::MotorInput2MotorLength(const std::vector<int> &motor_input) {
  std::vector<double> motor_length(ActuatorCount, 0.0);
  motor_length = Scale(motor_input_max_, motor_input_min_, motor_input, motor_max_,
                       motor_min_);
  return motor_length;
};

std::vector<double> O12KinematicsSolver::MotorLength2ActiveJoint(const std::vector<double> &motor_length_) {
  std::vector<double> motor_length = motor_length_;
  Clamp(motor_max_, motor_min_, motor_length);
  std::vector<double> active_joint_pos(MaxActiveJoint, 0.0);
  // thumb
  active_joint_pos[ActiveJointThumbRoll] =
      PredictPoly(motor_length[ActuatorThumbRoll] * 1e3, coeffs_thumb_roll_Motor2Joint_);
  active_joint_pos[ActiveJointThumbAbAd] =
      PredictPoly(motor_length[ActuatorThumbABAD] * 1e3, coeffs_thumb_abad_Motor2Joint_);
  active_joint_pos[ActiveJointThumbMCP] =
      PredictPoly(motor_length[ActuatorThumbMCP] * 1e3, coeffs_thumb_mcp_Motor2Joint_);
  active_joint_pos[ActiveJointThumbPIP] =
      PredictPoly(motor_length[ActuatorThumbPIP] * 1e3, coeffs_thumb_pip_Motor2Joint_);

  // index
  active_joint_pos[ActiveJointIndexAbAd] =
      PredictPoly33(motor_length[ActuatorIndex2],
                    motor_length[ActuatorIndex1], coeffs_abad_Motor2Joint_);
  active_joint_pos[ActiveJointIndexMCP] =
      PredictPoly33(motor_length[ActuatorIndex2],
                    motor_length[ActuatorIndex1], coeffs_mcp_Motor2Joint_);
  active_joint_pos[ActiveJointIndexPIP] = FingerPIPFitPredict(
      active_joint_pos[ActiveJointIndexAbAd], active_joint_pos[ActiveJointIndexMCP],
      motor_length[ActuatorIndex3], index_PIP_coeffs_Motor2Joint_);

  // middle
  active_joint_pos[ActiveJointMiddleABAD] =
      PredictPoly33(motor_length[ActuatorMiddle2],
                    motor_length[ActuatorMiddle1], coeffs_abad_Motor2Joint_);
  active_joint_pos[ActiveJointMiddleMCP] =
      PredictPoly33(motor_length[ActuatorMiddle2],
                    motor_length[ActuatorMiddle1], coeffs_mcp_Motor2Joint_);
  active_joint_pos[ActiveJointMiddlePIP] = FingerPIPFitPredict(
      active_joint_pos[ActiveJointMiddleABAD], active_joint_pos[ActiveJointMiddleMCP],
      motor_length[ActuatorMiddle3], middle_PIP_coeffs_Motor2Joint_);

  // ring
  active_joint_pos[ActiveJointRingMCP] =
      PredictPoly(motor_length[ActuatorRing] * 1e3, coeffs_ring_pinky_Motor2Joint_);

  // pinky
  active_joint_pos[ActiveJointPinkyMCP] =
      PredictPoly(motor_length[ActuatorPinky] * 1e3, coeffs_ring_pinky_Motor2Joint_);

  if (hand_type_) {
    active_joint_pos[ActiveJointIndexAbAd] = -active_joint_pos[ActiveJointIndexAbAd];
    active_joint_pos[ActiveJointMiddleABAD] = -active_joint_pos[ActiveJointMiddleABAD];
    active_joint_pos[ActiveJointThumbMCP] = -active_joint_pos[ActiveJointThumbMCP];
    active_joint_pos[ActiveJointThumbPIP] = -active_joint_pos[ActiveJointThumbPIP];
  } else {
    active_joint_pos[ActiveJointThumbAbAd] = -active_joint_pos[ActiveJointThumbAbAd];
    active_joint_pos[ActiveJointThumbMCP] = -active_joint_pos[ActiveJointThumbMCP];
    active_joint_pos[ActiveJointThumbPIP] = -active_joint_pos[ActiveJointThumbPIP];
  }
  Clamp(active_joint_max_, active_joint_min_, active_joint_pos);
  return active_joint_pos;
}

double O12KinematicsSolver::PredictPoly33(const double &abad, const double &mcp,
                                          const std::vector<double> &coeffs) {
  if (coeffs.size() != 10) {
    throw std::invalid_argument(
        "predictPoly33 requires exactly 10 coefficients.");
  }

  double x2 = abad * abad;
  double y2 = mcp * mcp;
  double x3 = x2 * abad;
  double y3 = y2 * mcp;

  return coeffs[0] +               // p00
         coeffs[1] * abad +        // p10*x
         coeffs[2] * mcp +         // p01*y
         coeffs[3] * x2 +          // p20*x^2
         coeffs[4] * abad * mcp +  // p11*x*y
         coeffs[5] * y2 +          // p02*y^2
         coeffs[6] * x3 +          // p30*x^3
         coeffs[7] * x2 * mcp +    // p21*x^2*y
         coeffs[8] * abad * y2 +   // p12*x*y^2
         coeffs[9] * y3;           // p03*y^3
}

double O12KinematicsSolver::PredictPoly(const double &x,
                                        const std::vector<double> &coeffs) {
  double result = 0.0;
  double power = 1.0;

  for (size_t i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * power;
    power *= x;
  }
  return result;
}

double O12KinematicsSolver::FingerPIPFitPredict(const double &abad,
                                                const double &mcp,
                                                const double &pip,
                                                const std::vector<double> &coeffs) {
  if (coeffs.size() != 20) {
    throw std::invalid_argument("Expected 20 coefficients.");
  }

  std::vector<double> X = {
      pip,                 // (0,0,1)
      pip * pip,           // (0,0,2)
      mcp,                 // (0,1,0)
      mcp * pip,           // (0,1,1)
      mcp * pip * pip,     // (0,1,2)
      mcp * mcp,           // (0,2,0)
      mcp * mcp * pip,     // (0,2,1)
      abad,                // (1,0,0)
      abad * pip,          // (1,0,1)
      abad * pip * pip,    // (1,0,2)
      abad * mcp,          // (1,1,0)
      abad * mcp * pip,    // (1,1,1)
      abad * mcp * mcp,    // (1,2,0)
      abad * abad,         // (2,0,0)
      abad * abad * pip,   // (2,0,1)
      abad * abad * mcp,   // (2,1,0)
      1.0,                 // (0,0,0)
      abad * abad * abad,  // (3,0,0)
      mcp * mcp * mcp,     // (0,3,0)
      pip * pip * pip      // (0,0,3)
  };

  double y = 0.0;
  for (size_t i = 0; i < coeffs.size(); ++i) {
    y += coeffs[i] * X[i];
  }

  return y;
}

// get all joint positions from active positions
std::vector<double> O12KinematicsSolver::GetAllJointPos(const std::vector<double> &active_joint_pos) {
  assert(active_joint_pos.size() == MaxActiveJoint);
  std::vector<double> all_joint_pos(MaxJoint, 0);
  // active joint
  // thumb
  all_joint_pos[JThumbRoll] = active_joint_pos[ActiveJointThumbRoll];
  all_joint_pos[JThumbAbad] = active_joint_pos[ActiveJointThumbAbAd];
  all_joint_pos[JThumbMCP] = active_joint_pos[ActiveJointThumbMCP];
  all_joint_pos[JThumbPIP] = active_joint_pos[ActiveJointThumbPIP];
  // index
  all_joint_pos[JIndexAbad] = active_joint_pos[ActiveJointIndexAbAd];
  all_joint_pos[JIndexMCP] = active_joint_pos[ActiveJointIndexMCP];
  all_joint_pos[JIndexPIP] = active_joint_pos[ActiveJointIndexPIP];
  // middle
  all_joint_pos[JMiddleAbad] = active_joint_pos[ActiveJointMiddleABAD];
  all_joint_pos[JMiddleMCP] = active_joint_pos[ActiveJointMiddleMCP];
  all_joint_pos[JMiddlePIP] = active_joint_pos[ActiveJointMiddlePIP];
  // ring
  all_joint_pos[JRingMCP] = active_joint_pos[ActiveJointRingMCP];
  // pinky
  all_joint_pos[JPinkyMCP] = active_joint_pos[ActiveJointPinkyMCP];
  //  passive joint
  // thumb
  all_joint_pos[JThumbDIP] =
      PredictPoly(active_joint_pos[ActiveJointThumbPIP], thumb_dip_coeff_);
  // index
  all_joint_pos[JIndexDIP] =
      PredictPoly(active_joint_pos[ActiveJointIndexPIP], index_dip_coeff_);
  // middle
  all_joint_pos[JMiddleDIP] =
      PredictPoly(active_joint_pos[ActiveJointMiddlePIP], middle_dip_coeff_);
  // ring
  all_joint_pos[JRingPIP] =
      PredictPoly(active_joint_pos[ActiveJointPinkyMCP], ring_pip_coeff_);
  all_joint_pos[JRingDIP] =
      PredictPoly(active_joint_pos[ActiveJointPinkyMCP], ring_dip_coeff_);
  // pinky
  all_joint_pos[JPinkyPIP] =
      PredictPoly(active_joint_pos[ActiveJointPinkyMCP], ring_pip_coeff_);
  all_joint_pos[JPinkyDIP] =
      PredictPoly(active_joint_pos[ActiveJointPinkyMCP], ring_dip_coeff_);
  return all_joint_pos;
}

template <typename T>
void O12KinematicsSolver::Clamp(const std::vector<T> &max,
                                const std::vector<T> &min,
                                std::vector<T> &value) {
  if (max.size() != min.size() || max.size() != value.size()) {
    throw std::invalid_argument(
        "Clamp: 'max', 'min' and 'value' vectors must have the same length");
  }
  // std::cout<<max[0]<<" "<<min[0]<<std::endl;
  for (size_t i = 0; i < value.size(); ++i) {
    if (max[i] >= min[i]) {  // 先把 value[i] 与 min[i] 做比较，再与 max[i] 比较
      value[i] = std::min(max[i], std::max(min[i], value[i]));
    } else {
      value[i] = std::min(min[i], std::max(max[i], value[i]));
    }
  }
}

template <typename InputT, typename OutputT>
std::vector<OutputT> O12KinematicsSolver::Scale(const std::vector<InputT> &max,
                                                const std::vector<InputT> &min,
                                                const std::vector<InputT> &value,
                                                const std::vector<OutputT> &target_max,
                                                const std::vector<OutputT> &target_min) {
  size_t n = value.size();
  if (min.size() != n || max.size() != n || target_min.size() != n ||
      target_max.size() != n) {
    throw std::invalid_argument(
        "Scale: all input vectors must have the same length");
  }

  // 1. 先 clamp
  std::vector<InputT> clamped = value;
  Clamp(max, min, clamped);

  // 2. 线性映射
  std::vector<OutputT> result(n);
  for (size_t i = 0; i < n; ++i) {
    if (max[i] == min[i]) {
      throw std::invalid_argument(
          "Scale: max and min values must differ for each element");
    }
    double ratio = static_cast<double>(clamped[i] - min[i]) / static_cast<double>(max[i] - min[i]);
    // std::cout<<"ratio:"<<ratio<<"="<<clamped[i]<<"-"<< min[i]<<"/"<<max[i]<<"-"<<min[i]<< std::endl;
    //  ratio 自然落在 [0,1]，无需再手动 clamp
    double mapped = ratio * (target_max[i] - target_min[i]) + target_min[i];
    result[i] = static_cast<OutputT>(mapped);
  }
  return result;
}
}  // namespace omnihandProSDK
