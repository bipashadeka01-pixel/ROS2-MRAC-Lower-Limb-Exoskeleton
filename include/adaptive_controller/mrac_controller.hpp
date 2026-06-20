#ifndef ADAPTIVE_CONTROLLER__MRAC_CONTROLLER_HPP_
#define ADAPTIVE_CONTROLLER__MRAC_CONTROLLER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MRACController : public rclcpp::Node
{
public:
  MRACController();

private:
  // ── Control loop ──────────────────────────────────────────────────────────
  void control_loop();

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ── Reference trajectory (sinusoidal gait pattern) ────────────────────────
  void compute_reference(
    double t,
    Eigen::VectorXd & qd,
    Eigen::VectorXd & dqd,
    Eigen::VectorXd & ddqd);

  // ── MRAC core ─────────────────────────────────────────────────────────────
  Eigen::VectorXd compute_sliding_surface(
    const Eigen::VectorXd & e,
    const Eigen::VectorXd & de);

  Eigen::MatrixXd compute_regressor(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & dq,
    const Eigen::VectorXd & ddqr);

  void update_parameters(
    const Eigen::MatrixXd & Y,
    const Eigen::VectorXd & s);

  Eigen::VectorXd compute_torque(
    const Eigen::MatrixXd & Y,
    const Eigen::VectorXd & s);

  // ── Safety ────────────────────────────────────────────────────────────────
  Eigen::VectorXd apply_safety(const Eigen::VectorXd & tau);

  // ── Publishing ────────────────────────────────────────────────────────────
  void publish_joint_states();

  // ── Utility ───────────────────────────────────────────────────────────────
  double clamp(double value, double low, double high);

  // ══════════════════════════════════════════════════════════════════════════
  // Constants
  // ══════════════════════════════════════════════════════════════════════════
  static constexpr int N_JOINTS = 8;
  // N_PARAMS = 3 regressors per joint (inertia, damping, gravity)
  static constexpr int N_PARAMS = N_JOINTS * 3;  // = 24

  // ══════════════════════════════════════════════════════════════════════════
  // Joint metadata  (order must match joint_names_ exactly)
  // ══════════════════════════════════════════════════════════════════════════
  std::vector<std::string> joint_names_ = {
    "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4",
    "left_joint_1",  "left_joint_2",  "left_joint_3",  "left_joint_4"
  };

  // Per-joint URDF limits — must stay in sync with exoskeleton.urdf.xacro
  const std::vector<double> q_lower_ = {
    -0.523, -0.349, 0.000, -0.524,   // right 1-4
    -0.523, -0.349, 0.000, -0.524    // left  1-4
  };
  const std::vector<double> q_upper_ = {
    0.523, 1.920, 2.094, 0.349,      // right 1-4
    0.523, 1.920, 2.049, 0.349       // left  1-4
  };

  // Per-joint torque limits — taken directly from URDF <limit effort="...">
  // FIX: original had tau_max_=10.0 for ALL joints.
  //      Joints 3 & 4 (knee/ankle) need up to 100 Nm.
  //      A 10 Nm cap silently clips 90% of the commanded torque → no motion.
  const std::vector<double> tau_max_per_joint_ = {
    50.0,   // right_joint_1  hip ab/adduction
    80.0,   // right_joint_2  hip flex/extension
    100.0,  // right_joint_3  knee flexion
    100.0,  // right_joint_4  ankle
    50.0,   // left_joint_1   hip ab/adduction
    80.0,   // left_joint_2   hip flex/extension
    100.0,  // left_joint_3   knee flexion
    50.0    // left_joint_4   ankle
  };

  // ══════════════════════════════════════════════════════════════════════════
  // ROS2 interfaces
  // ══════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_state_pub_;

  // ══════════════════════════════════════════════════════════════════════════
  // Timing
  // ══════════════════════════════════════════════════════════════════════════
  rclcpp::Time node_start_time_;
  rclcpp::Time control_start_time_;

  double dt_ = 0.001;  // 1 kHz — must match timer_ period in .cpp

  // FIX: settling_time_ is now actually used in control_loop().
  // control_enabled_ stays false until this many seconds have elapsed
  // after the first joint state arrives, preventing a torque spike on startup.
  double settling_time_ = 2.0;  // seconds

  // ══════════════════════════════════════════════════════════════════════════
  // State vectors
  // ══════════════════════════════════════════════════════════════════════════
  Eigen::VectorXd q_;          // measured joint positions    [rad]
  Eigen::VectorXd dq_;         // measured joint velocities   [rad/s]
  Eigen::VectorXd q_initial_;  // positions at first callback (startup ref)
  Eigen::VectorXd tau_prev_;   // previous tick torque output (rate limiter)

  // ══════════════════════════════════════════════════════════════════════════
  // Adaptive parameters
  // All of these are exposed as ROS2 parameters so you can tune from the
  // launch file without recompiling.
  // ══════════════════════════════════════════════════════════════════════════

  // θ̂ — adaptive parameter vector (inertia, damping, gravity per joint)
  Eigen::VectorXd theta_hat_;

  // Γ — adaptation gain (scalar × I).
  // Larger = faster convergence but risks oscillation.
  // ROS2 param: "adaptation_gain"  default: 0.5
  // FIX: was hardcoded 0.2 with no way to tune at runtime.
  Eigen::MatrixXd Gamma_;

  // Λ — sliding surface gain (scalar × I).
  // Higher = more aggressive error correction.
  // ROS2 param: "lambda_gain"  default: 4.0
  Eigen::MatrixXd Lambda_;

  // Kd — sliding surface damping (scalar × I).
  // Higher = smoother motion, slower response.
  // ROS2 param: "kd_gain"  default: 2.5
  Eigen::MatrixXd Kd_;

  // Torque slew rate limit [Nm/s].
  // At dt=0.001 s, max change per tick = tau_rate_max_ * 0.001
  // FIX: was 50.0 Nm/s → only 0.05 Nm/tick — far too restrictive.
  //      Raised to 500 Nm/s → 0.5 Nm/tick, responsive but safe.
  // ROS2 param: "torque_rate_max"  default: 500.0
  double tau_rate_max_ = 500.0;

  // ══════════════════════════════════════════════════════════════════════════
  // State flags
  // ══════════════════════════════════════════════════════════════════════════

  // Set true on first valid joint_state message
  bool joint_state_received_ = false;

  // FIX: control_enabled_ is now actually used.
  // Stays false during settling_time_ after first joint state.
  // control_loop() publishes zero torque until this is true.
  bool control_enabled_ = false;

  // Restricts left knee (left_joint_3) to a small safe range.
  // FIX: was hardcoded true with no runtime override.
  // ROS2 param: "protect_left_knee"  default: true
  bool protect_left_knee_ = true;
};

#endif  // ADAPTIVE_CONTROLLER__MRAC_CONTROLLER_HPP_