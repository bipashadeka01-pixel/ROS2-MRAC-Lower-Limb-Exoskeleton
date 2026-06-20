#include "adaptive_controller/mrac_controller.hpp"

MRACController::MRACController()
: Node("mrac_controller")
{
  // ── Declare ROS2 parameters (tunable from launch file) ──────────────────
  // FIX: previously all gains were hardcoded — you had to recompile to retune.
  this->declare_parameter("adaptation_gain",   0.5);
  this->declare_parameter("lambda_gain",        4.0);
  this->declare_parameter("kd_gain",            2.5);
  this->declare_parameter("torque_rate_max",  500.0);
  this->declare_parameter("settling_time",      2.0);
  this->declare_parameter("protect_left_knee", true);

  double gamma_scalar  = this->get_parameter("adaptation_gain").as_double();
  double lambda_scalar = this->get_parameter("lambda_gain").as_double();
  double kd_scalar     = this->get_parameter("kd_gain").as_double();
  tau_rate_max_        = this->get_parameter("torque_rate_max").as_double();
  settling_time_       = this->get_parameter("settling_time").as_double();
  protect_left_knee_   = this->get_parameter("protect_left_knee").as_bool();

  // ── Initialise state vectors ─────────────────────────────────────────────
  q_         = Eigen::VectorXd::Zero(N_JOINTS);
  dq_        = Eigen::VectorXd::Zero(N_JOINTS);
  q_initial_ = Eigen::VectorXd::Zero(N_JOINTS);
  tau_prev_  = Eigen::VectorXd::Zero(N_JOINTS);

  // ── Initialise adaptive parameters ──────────────────────────────────────
  theta_hat_ = Eigen::VectorXd::Ones(N_PARAMS);
  Gamma_  = gamma_scalar  * Eigen::MatrixXd::Identity(N_PARAMS, N_PARAMS);
  Lambda_ = lambda_scalar * Eigen::MatrixXd::Identity(N_JOINTS, N_JOINTS);
  Kd_     = kd_scalar     * Eigen::MatrixXd::Identity(N_JOINTS, N_JOINTS);

  node_start_time_   = this->now();
  control_start_time_ = this->now();

  // ── ROS2 interfaces ──────────────────────────────────────────────────────
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/measured_joint_states", 10,
    std::bind(&MRACController::joint_state_callback, this, std::placeholders::_1));

  torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mrac_joint_commands", 10);

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);

  desired_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/mrac_desired_states", 10);
  // 1 kHz control loop
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&MRACController::control_loop, this));

  RCLCPP_INFO(this->get_logger(),
    "MRAC controller started | γ=%.2f λ=%.2f Kd=%.2f settle=%.1fs left_knee_protect=%s",
    gamma_scalar, lambda_scalar, kd_scalar, settling_time_,
    protect_left_knee_ ? "ON" : "OFF");
}

// ─────────────────────────────────────────────────────────────────────────────
// Callback: incoming joint states from hardware / simulation
// ─────────────────────────────────────────────────────────────────────────────
void MRACController::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
    for (size_t j = 0; j < joint_names_.size(); j++) {
      if (msg->name[i] == joint_names_[j]) {
        if (i < msg->position.size()) q_(j)  = msg->position[i];
        if (i < msg->velocity.size()) dq_(j) = msg->velocity[i];
      }
    }
  }

  if (!joint_state_received_) {
    q_initial_          = q_;
    control_start_time_ = this->now();
    joint_state_received_ = true;
    RCLCPP_INFO(this->get_logger(),
      "First joint state received. Settling for %.1f s ...", settling_time_);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Reference trajectory: sinusoidal gait pattern
// ─────────────────────────────────────────────────────────────────────────────
void MRACController::compute_reference(
  double t,
  Eigen::VectorXd & qd,
  Eigen::VectorXd & dqd,
  Eigen::VectorXd & ddqd)
{
  qd   = Eigen::VectorXd::Zero(N_JOINTS);
  dqd  = Eigen::VectorXd::Zero(N_JOINTS);
  ddqd = Eigen::VectorXd::Zero(N_JOINTS);

  const double freq = 0.5;
  const double w    = 2.0 * M_PI * freq;

  const double sr = std::sin(w * t);
  const double cr = std::cos(w * t);
  const double sl = std::sin(w * t + M_PI);   // left leg is 180° out of phase
  const double cl = std::cos(w * t + M_PI);

  // ── Right leg ──
  qd(0) = 0.08 * sr;            dqd(0) = 0.08 * w * cr;          ddqd(0) = -0.08 * w*w * sr;
  qd(1) = 0.35 * sr + 0.15;    dqd(1) = 0.35 * w * cr;          ddqd(1) = -0.35 * w*w * sr;
  qd(2) = 0.55 * std::max(0.0, -sr);
  dqd(2)  = (-sr > 0.0) ? -0.55 * w * cr : 0.0;
  ddqd(2) = (-sr > 0.0) ?  0.55 * w*w * sr : 0.0;
  qd(3) = -0.18 * sr;           dqd(3) = -0.18 * w * cr;         ddqd(3) =  0.18 * w*w * sr;

  // ── Left leg ──
  qd(4) = 0.08 * sl;            dqd(4) = 0.08 * w * cl;          ddqd(4) = -0.08 * w*w * sl;
  qd(5) = 0.35 * sl + 0.15;    dqd(5) = 0.35 * w * cl;          ddqd(5) = -0.35 * w*w * sl;

  // FIX: protect_left_knee_ is now a ROS2 param — branch is actually reachable
  if (protect_left_knee_) {
    // Restrict left knee to a small safe range until validated on hardware
    qd(6) = 0.08 * std::max(0.0, -sl);
    dqd(6) = 0.0;   ddqd(6) = 0.0;
  } else {
    qd(6) = 0.55 * std::max(0.0, -sl);
    dqd(6)  = (-sl > 0.0) ? -0.55 * w * cl : 0.0;
    ddqd(6) = (-sl > 0.0) ?  0.55 * w*w * sl : 0.0;
  }
  qd(7) = -0.18 * sl;           dqd(7) = -0.18 * w * cl;         ddqd(7) =  0.18 * w*w * sl;

  // Clamp to URDF joint limits
  for (int i = 0; i < N_JOINTS; i++) {
    qd(i) = clamp(qd(i), q_lower_[i], q_upper_[i]);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sliding surface:  s = ė + Λ·e
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXd MRACController::compute_sliding_surface(
  const Eigen::VectorXd & e,
  const Eigen::VectorXd & de)
{
  return de + Lambda_ * e;
}

// ─────────────────────────────────────────────────────────────────────────────
// Regressor matrix Y  (N_JOINTS × N_PARAMS)
// Column groups per joint i: [ddqr_i, dq_i, sin(q_i)]
//   → captures inertia, viscous friction, gravity terms
// ─────────────────────────────────────────────────────────────────────────────
Eigen::MatrixXd MRACController::compute_regressor(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & dq,
  const Eigen::VectorXd & ddqr)
{
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(N_JOINTS, N_PARAMS);
  for (int i = 0; i < N_JOINTS; i++) {
    int base = 3 * i;
    Y(i, base + 0) = ddqr(i);
    Y(i, base + 1) = dq(i);
    Y(i, base + 2) = std::sin(q(i));
  }
  return Y;
}

// ─────────────────────────────────────────────────────────────────────────────
// MIT/Slotine-Li parameter update:  θ̂̇ = -Γ · Yᵀ · s
// ─────────────────────────────────────────────────────────────────────────────
void MRACController::update_parameters(
  const Eigen::MatrixXd & Y,
  const Eigen::VectorXd & s)
{
  Eigen::VectorXd theta_dot = -Gamma_ * Y.transpose() * s;
  theta_hat_ += theta_dot * dt_;

  // Bound parameters to prevent unbounded growth (parameter drift)
  for (int i = 0; i < N_PARAMS; i++) {
    theta_hat_(i) = clamp(theta_hat_(i), -20.0, 20.0);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Torque:  τ = Y·θ̂ - Kd·s
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXd MRACController::compute_torque(
  const Eigen::MatrixXd & Y,
  const Eigen::VectorXd & s)
{
  return Y * theta_hat_ - Kd_ * s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Safety: per-joint torque saturation + slew rate limit
// FIX: now uses per-joint limits from tau_max_per_joint_ instead of
//      a single 10 Nm cap that was clipping knee/ankle torques by 90%.
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXd MRACController::apply_safety(const Eigen::VectorXd & tau)
{
  Eigen::VectorXd safe_tau = tau;
  const double max_change_per_tick = tau_rate_max_ * dt_;

  for (int i = 0; i < N_JOINTS; i++) {
    // 1. Per-joint torque saturation (from URDF limits)
    safe_tau(i) = clamp(safe_tau(i), -tau_max_per_joint_[i], tau_max_per_joint_[i]);

    // 2. Slew rate limit (prevents actuator current spikes)
    double delta = safe_tau(i) - tau_prev_(i);
    if (delta >  max_change_per_tick) safe_tau(i) = tau_prev_(i) + max_change_per_tick;
    if (delta < -max_change_per_tick) safe_tau(i) = tau_prev_(i) - max_change_per_tick;
  }

  tau_prev_ = safe_tau;
  return safe_tau;
}

// ─────────────────────────────────────────────────────────────────────────────
// Publish joint states for robot_state_publisher → TF → RViz
// ─────────────────────────────────────────────────────────────────────────────
void MRACController::publish_joint_states()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name         = joint_names_;
  msg.position.resize(N_JOINTS);
  msg.velocity.resize(N_JOINTS);
  msg.effort.resize(N_JOINTS);

  for (int i = 0; i < N_JOINTS; i++) {
    msg.position[i] = q_(i);
    msg.velocity[i] = dq_(i);
    msg.effort[i]   = tau_prev_(i);
  }
  joint_state_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main control loop (1 kHz)
// FIX: control_enabled_ and settling_time_ are now actually used here.
//      Torque output is zero until settling is complete, then ramps up
//      linearly over the first settling_time_ seconds to avoid spikes.
// ─────────────────────────────────────────────────────────────────────────────
void MRACController::control_loop()
{
  const double t = (this->now() - node_start_time_).seconds();

  // ── Always publish joint states so RViz stays live ──────────────────────
  publish_joint_states();

  // ── Wait for first real sensor data ─────────────────────────────────────
  if (!joint_state_received_) {
    // Integrate reference passively so the model is ready when sensor arrives
    Eigen::VectorXd qd, dqd, ddqd;
    compute_reference(t, qd, dqd, ddqd);
    // Publish desired trajectory so we can log and plot it
    sensor_msgs::msg::JointState desired_msg;
    desired_msg.header.stamp = this->now();
    desired_msg.name = joint_names_;
    desired_msg.position = std::vector<double>(qd.data(), qd.data() + N_JOINTS);
    desired_msg.velocity = std::vector<double>(dqd.data(), dqd.data() + N_JOINTS);
    desired_state_pub_->publish(desired_msg);
    q_  = qd;
    dq_ = dqd;
    return;
  }

  // ── Settling window: ramp torque from 0 after startup ───────────────────
  const double elapsed = (this->now() - control_start_time_).seconds();
  if (!control_enabled_) {
    if (elapsed < settling_time_) {
      // Zero torque during settling — just publish zeros
      std_msgs::msg::Float64MultiArray zero_msg;
      zero_msg.data.assign(N_JOINTS, 0.0);
      torque_pub_->publish(zero_msg);
      return;
    }
    control_enabled_ = true;
    RCLCPP_INFO(this->get_logger(), "Settling complete. Control enabled.");
  }

  // ── Ramp factor: linearly scale torque for first settling_time_ after enable
  // This creates a smooth 0→1 blend instead of a hard step.
  const double ramp = std::min(1.0, (elapsed - settling_time_) / settling_time_);

  // ── Reference ────────────────────────────────────────────────────────────
  Eigen::VectorXd qd, dqd, ddqd;
  compute_reference(t, qd, dqd, ddqd);

  // ── Errors ───────────────────────────────────────────────────────────────
  const Eigen::VectorXd e  = q_  - qd;
  const Eigen::VectorXd de = dq_ - dqd;

  // ── Sliding surface ──────────────────────────────────────────────────────
  const Eigen::VectorXd s = compute_sliding_surface(e, de);

  // ── Reference velocity/acceleration for regressor ────────────────────────
  const Eigen::VectorXd dqr  = dqd - Lambda_ * e;
  const Eigen::VectorXd ddqr = ddqd - Lambda_ * de;

  // ── Regressor ────────────────────────────────────────────────────────────
  const Eigen::MatrixXd Y = compute_regressor(q_, dqr, ddqr);

  // ── Adapt parameters ─────────────────────────────────────────────────────
  update_parameters(Y, s);

  // ── Compute + safety-limit torque ────────────────────────────────────────
  Eigen::VectorXd tau      = compute_torque(Y, s);
  Eigen::VectorXd safe_tau = apply_safety(tau * ramp);   // apply ramp here

  // ── Publish torque commands ───────────────────────────────────────────────
  std_msgs::msg::Float64MultiArray torque_msg;
  torque_msg.data.resize(N_JOINTS);
  for (int i = 0; i < N_JOINTS; i++) {
    torque_msg.data[i] = safe_tau(i);
  }
  torque_pub_->publish(torque_msg);
}

// ─────────────────────────────────────────────────────────────────────────────
double MRACController::clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MRACController>());
  rclcpp::shutdown();
  return 0;
}