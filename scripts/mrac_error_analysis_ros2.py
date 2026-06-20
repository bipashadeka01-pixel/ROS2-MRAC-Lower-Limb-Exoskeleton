#!/usr/bin/env python3
"""
mrac_error_analysis_ros2.py  –  ROS 2 Jazzy / rclpy
=====================================================
Matches the EXACT controller in mrac_controller.cpp:

  Topics consumed:
    /joint_states      (sensor_msgs/JointState)   – actual q, dq
    /torque_commands   (std_msgs/Float64MultiArray) – tau from controller

  Topics published (now plottable in rqt_plot / PlotJuggler):
    /mrac/tracking_error   (sensor_msgs/JointState)
        .position  = q_desired - q_measured        [rad]
        .velocity  = dq_desired - dq_measured      [rad/s]
        .effort    = torque commands               [Nm]

    /mrac/reference_state  (sensor_msgs/JointState)
        .position  = q_desired(t)
        .velocity  = dq_desired(t)

    /mrac/error_metrics    (std_msgs/Float64MultiArray)
        data[0..7]   = per-joint RMSE  [deg]   <- plot these
        data[8..15]  = per-joint MAE   [deg]
        data[16..23] = per-joint peak  [deg]
        data[24]     = sliding surface norm  |s|

Reference model mirrors compute_reference() in mrac_controller.cpp exactly:
    q_d(i,t) = q0(i) + ramp(t) * 0.03 * sin(2π*0.05*t + i*π/8)
    ramp(t)  = min(t/5, 1.0)
    (phase offset per joint = i * π / N_JOINTS)

Run (in a new terminal while your launch file is active):
    ros2 run adaptive_controller mrac_error_analysis_ros2.py

Then plot RMSE for all 8 joints:
    ros2 run rqt_plot rqt_plot \
      /mrac/error_metrics/data[0] /mrac/error_metrics/data[1] \
      /mrac/error_metrics/data[2] /mrac/error_metrics/data[3] \
      /mrac/error_metrics/data[4] /mrac/error_metrics/data[5] \
      /mrac/error_metrics/data[6] /mrac/error_metrics/data[7]

Plot reference vs actual for joint 0 (right_joint_1):
    ros2 run rqt_plot rqt_plot \
      /mrac/reference_state/position[0] \
      /joint_states/position[0]

Plot sliding surface norm (convergence indicator):
    ros2 run rqt_plot rqt_plot /mrac/error_metrics/data[24]
"""

import math
import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ── must match mrac_controller.cpp ────────────────────────────────────
JOINT_NAMES = [
    "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4",
    "left_joint_1",  "left_joint_2",  "left_joint_3",  "left_joint_4",
]
JOINT_LABELS = [
    "R hip ab/ad ", "R hip flex  ", "R knee      ", "R ankle     ",
    "L hip ab/ad ", "L hip flex  ", "L knee      ", "L ankle     ",
]
N_JOINTS   = 8
AMP        = 0.03          # rad   – matches cpp amp
FREQ       = 0.05          # Hz    – matches cpp freq
RAMP_TIME  = 5.0           # s     – matches cpp ramp_time
SETTLE     = 5.0           # s     – node_start + 3s wait + 2s settle = 5s
                           #         (3.0 wait + 2.0 settling_time in cpp)
WINDOW     = 500           # samples (~5 s at 100 Hz)
WARN_DEG   = 3.0           # flag joints above this RMSE
# ──────────────────────────────────────────────────────────────────────


def reference(t: float, q0: np.ndarray):
    """
    Mirrors MRACController::compute_reference() exactly.
    Returns (q_d, dq_d) [rad, rad/s].
    """
    omega = 2.0 * math.pi * FREQ
    ramp  = min(t / RAMP_TIME, 1.0)
    q_d   = np.zeros(N_JOINTS)
    dq_d  = np.zeros(N_JOINTS)
    for i in range(N_JOINTS):
        phase  = i * math.pi / N_JOINTS
        q_d[i]  = q0[i] + ramp * AMP * math.sin(omega * t + phase)
        dq_d[i] = ramp * AMP * omega * math.cos(omega * t + phase)
    return q_d, dq_d


class MRACErrorAnalysis(Node):

    def __init__(self):
        super().__init__("mrac_error_analysis")

        self._q   = np.zeros(N_JOINTS)
        self._dq  = np.zeros(N_JOINTS)
        self._tau = np.zeros(N_JOINTS)
        self._q0  = None          # captured once, after settle period
        self._joint_idx: dict = {}

        self._buf_e  = [deque(maxlen=WINDOW) for _ in range(N_JOINTS)]
        self._buf_de = [deque(maxlen=WINDOW) for _ in range(N_JOINTS)]
        self._buf_s  = deque(maxlen=WINDOW)   # sliding surface norm
        self._peak   = np.zeros(N_JOINTS)
        self._samples = 0

        self._node_t0 = self.get_clock().now().nanoseconds * 1e-9
        self._ctrl_t0 = None   # set once control phase begins

        # ── Lambda must match cpp (Identity * 2.0) ──
        self._Lambda = 2.0 * np.eye(N_JOINTS)

        # subscribers
        self.create_subscription(
            JointState, "/joint_states", self._cb_joints, 10)
        self.create_subscription(
            Float64MultiArray, "/torque_commands", self._cb_torque, 10)

        # publishers
        self._pub_err     = self.create_publisher(
            JointState, "/mrac/tracking_error", 10)
        self._pub_ref     = self.create_publisher(
            JointState, "/mrac/reference_state", 10)
        self._pub_metrics = self.create_publisher(
            Float64MultiArray, "/mrac/error_metrics", 10)

        # timers
        self.create_timer(0.02,  self._publish_cb)   # 50 Hz
        self.create_timer(2.0,   self._report_cb)    # terminal report

        self.get_logger().info(
            "\n"
            "  mrac_error_analysis running\n"
            "  Subscribing : /joint_states, /torque_commands\n"
            "  Publishing  : /mrac/tracking_error\n"
            "                /mrac/reference_state\n"
            "                /mrac/error_metrics\n"
            "\n"
            "  rqt_plot RMSE (all joints):\n"
            "    ros2 run rqt_plot rqt_plot \\\n"
            "      /mrac/error_metrics/data[0] /mrac/error_metrics/data[1] \\\n"
            "      /mrac/error_metrics/data[2] /mrac/error_metrics/data[3] \\\n"
            "      /mrac/error_metrics/data[4] /mrac/error_metrics/data[5] \\\n"
            "      /mrac/error_metrics/data[6] /mrac/error_metrics/data[7]\n"
            "\n"
            "  Sliding surface norm (convergence):\n"
            "    ros2 run rqt_plot rqt_plot /mrac/error_metrics/data[24]\n"
        )

    # ── subscribers ───────────────────────────────────────────────────

    def _cb_joints(self, msg: JointState):
        if not self._joint_idx:
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES:
                    self._joint_idx[name] = i

        for j, jname in enumerate(JOINT_NAMES):
            idx = self._joint_idx.get(jname)
            if idx is not None:
                if idx < len(msg.position):
                    self._q[j]  = msg.position[idx]
                if idx < len(msg.velocity):
                    self._dq[j] = msg.velocity[idx]

    def _cb_torque(self, msg: Float64MultiArray):
        for i in range(min(N_JOINTS, len(msg.data))):
            self._tau[i] = msg.data[i]

    # ── 50 Hz publish ─────────────────────────────────────────────────

    def _publish_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed_from_node = now - self._node_t0

        # mirror the cpp settle logic: wait 3 s for joint_states, then 2 s settle
        if elapsed_from_node < SETTLE:
            return

        # latch q0 once at the moment control enables (same as cpp)
        if self._ctrl_t0 is None:
            self._q0     = self._q.copy()
            self._ctrl_t0 = now
            self.get_logger().info(
                f"Control phase started. q0 = {np.round(self._q0, 4).tolist()}")

        t     = now - self._ctrl_t0
        q_d, dq_d = reference(t, self._q0)
        e     = q_d  - self._q
        de    = dq_d - self._dq

        # sliding surface s = de + Lambda * e
        s     = de + self._Lambda @ e
        s_norm = float(np.linalg.norm(s))

        stamp = self.get_clock().now().to_msg()

        # tracking error msg
        err            = JointState()
        err.header.stamp = stamp
        err.name       = JOINT_NAMES
        err.position   = e.tolist()
        err.velocity   = de.tolist()
        err.effort     = self._tau.tolist()
        self._pub_err.publish(err)

        # reference msg
        ref            = JointState()
        ref.header.stamp = stamp
        ref.name       = JOINT_NAMES
        ref.position   = q_d.tolist()
        ref.velocity   = dq_d.tolist()
        self._pub_ref.publish(ref)

        # buffer
        for i in range(N_JOINTS):
            self._buf_e[i].append(math.degrees(e[i]))
            self._buf_de[i].append(math.degrees(de[i]))
            if abs(e[i]) > self._peak[i]:
                self._peak[i] = abs(e[i])
        self._buf_s.append(s_norm)
        self._samples += 1

        # metrics msg  [RMSE×8, MAE×8, peak×8, s_norm]
        rmse = self._rmse()
        mae  = self._mae()
        peak = np.degrees(self._peak)

        m      = Float64MultiArray()
        m.data = rmse.tolist() + mae.tolist() + peak.tolist() + [s_norm]
        self._pub_metrics.publish(m)

    # ── terminal report ───────────────────────────────────────────────

    def _report_cb(self):
        if self._samples < 20:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._node_t0
            remaining = max(0.0, SETTLE - elapsed)
            if remaining > 0:
                self.get_logger().info(
                    f"Waiting for settle period... {remaining:.1f}s remaining")
            else:
                self.get_logger().warn(
                    "Insufficient samples yet — is /joint_states publishing?")
            return

        rmse = self._rmse()
        mae  = self._mae()
        peak = np.degrees(self._peak)
        s_mean = float(np.mean(self._buf_s)) if self._buf_s else 0.0

        sep = "═" * 66
        print(f"\n{sep}")
        print(f"  MRAC Error Report  |  {self._samples} samples  "
              f"|  |s| mean = {s_mean:.4f}")
        print(f"  {'Joint':<14}  {'RMSE [°]':>9}  {'MAE [°]':>9}  "
              f"{'Peak [°]':>9}")
        print(f"  {'─'*14}  {'─'*9}  {'─'*9}  {'─'*9}")
        for i in range(N_JOINTS):
            flag = "  <-- !" if rmse[i] > WARN_DEG else ""
            print(f"  {JOINT_LABELS[i]}  {rmse[i]:9.3f}  {mae[i]:9.3f}  "
                  f"{peak[i]:9.3f}{flag}")
        print(f"  {'─'*14}  {'─'*9}  {'─'*9}  {'─'*9}")
        print(f"  {'Mean':<14}  {np.mean(rmse):9.3f}  {np.mean(mae):9.3f}")
        print(sep)

    # ── stats helpers ─────────────────────────────────────────────────

    def _rmse(self):
        return np.array([
            math.sqrt(np.mean(np.array(b)**2)) if b else 0.0
            for b in self._buf_e
        ])

    def _mae(self):
        return np.array([
            np.mean(np.abs(np.array(b))) if b else 0.0
            for b in self._buf_e
        ])


def main(args=None):
    rclpy.init(args=args)
    node = MRACErrorAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()