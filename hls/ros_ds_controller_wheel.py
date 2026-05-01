#!/usr/bin/env python3
"""ROS2 control node for FTServo arm with DS command input.

Responsibilities:
- Read robot state from HLS servos.
- Publish state to MATLAB/RLearn side.
- Receive DS Cartesian velocity command from ROS.
- Run FSM: CONTROL uses wheel (speed) mode, all other states use torque mode.
- Send speed or torque commands to servos depending on FSM state.

FSM / hardware mode mapping:
    IDLE    → torque mode  (gravity compensation + light damping)
    LEARN   → torque mode  (gravity compensation, kinesthetic teaching)
    FREEZE  → torque mode  (stiffness + damping around frozen joint position)
    CONTROL → wheel  mode  (Cartesian velocity → joint velocity via J pseudo-inverse)
"""

from __future__ import annotations

import enum
import json
import traceback
from pathlib import Path
from typing import Optional
import sys
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from servo_interface import HlsServoInterface, ServoInterfaceError


class FSM(enum.Enum):
    IDLE = "IDLE"
    CONTROL = "CONTROL"
    FREEZE = "FREEZE"
    LEARN = "LEARN"


def _damped_pseudoinverse(jacobian: np.ndarray, damping: float) -> np.ndarray:
    """Damped least-squares pseudo-inverse: J^T (J J^T + λ² I)^{-1}."""
    jjt = jacobian @ jacobian.T
    return jacobian.T @ np.linalg.inv(jjt + damping * np.eye(jjt.shape[0]))


class RosDsController(Node):
    def __init__(self) -> None:
        super().__init__("ros_ds_controller")

        # --- Parameters ---
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 1000000)
        self.declare_parameter("servo_ids", [1, 2, 3])
        self.declare_parameter("control_hz", 250.0)
        self.declare_parameter("twist_timeout_s", 0.25)
        self.declare_parameter("max_twist_mps", 0.25)
        self.declare_parameter("torque_limit_units", 1800)
        self.declare_parameter("mode", "IDLE")

        # Torque-mode gains (IDLE / LEARN / FREEZE)
        self.declare_parameter("k_dq", 0.35)
        self.declare_parameter("k_idle_damping", -0.2)
        self.declare_parameter("k_freeze", 1.2)
        self.declare_parameter("k_learn_damping", 0.0)
        self.declare_parameter("k_null", 0.1)
        self.declare_parameter("k_null_damping", 0.08)
        self.declare_parameter("q_null", [0.0, 0.0, 0.0])
        self.declare_parameter("gravity_scale", 1.0)

        # Wheel-mode CONTROL parameters
        # Damping factor on the pseudo-inverse to handle near-singular configs.
        self.declare_parameter("wheel_pinv_damping", 1e-2)
        # Hard limit on individual joint speed commands (rad/s).
        self.declare_parameter("wheel_max_joint_rad_s", 3.0)

        # Null-space (torque-mode CONTROL legacy params, kept for compatibility)
        self.declare_parameter("activate_ns", False)
        self.declare_parameter("principle_damping", 7.0)
        self.declare_parameter("orthogonal_damping", 4.0)
        self.declare_parameter("null_stiffness", 1.0)
        self.declare_parameter("null_damping", 2.0)
        self.declare_parameter("pseudoinverse_damping", 1e-2)

        # Learn-mode recording controls
        self.declare_parameter("learn_sampling_hz", 100.0)
        self.declare_parameter("learn_start_trajectory", False)
        self.declare_parameter("learn_stop_trajectory", False)
        self.declare_parameter("learn_stop_recording", False)
        self.declare_parameter("learn_clear_recording", False)
        self.declare_parameter("learn_resample_points", 250)
        self.declare_parameter("learn_output_file", "learned_trajectories.mat")

        self.declare_parameter("state_topic", "/rlearn/state")
        self.declare_parameter("twist_topic", "/rlearn/command_twist")
        self.declare_parameter("ee_frame_name", "servo_to_shaft")

        # --- Robot model ---
        here = Path(__file__).resolve().parent
        zob = here / "zobotics"
        self.robot = RobotWrapper.BuildFromURDF(str(zob / "robot.urdf"), package_dirs=[str(zob)])
        self.model = self.robot.model
        self.data = self.model.createData()
        self.nq = self.model.nq

        if self.nq != len(self.get_parameter("servo_ids").value):
            raise RuntimeError("servo_ids length must match robot model nq")

        ee_frame_name = str(self.get_parameter("ee_frame_name").value)
        try:
            self.ee_frame_id = int(self.model.getFrameId(ee_frame_name))
            if self.ee_frame_id < 0 or self.ee_frame_id >= self.model.nframes:
                raise ValueError("Invalid frame id")
        except Exception:  # pylint: disable=broad-except
            self.ee_frame_id = self.model.nframes - 1
            fallback = self.model.frames[self.ee_frame_id].name
            self.get_logger().warning(
                f"ee_frame_name '{ee_frame_name}' not found, falling back to '{fallback}'"
            )
        self.joint_names = list(self.model.names[1 : self.nq + 1])

        # --- Servo interface ---
        self.servo = HlsServoInterface(
            port=self.get_parameter("serial_port").value,
            baud=int(self.get_parameter("baud_rate").value),
            ids=self.get_parameter("servo_ids").value,
        )
        self.servo.open()
        ping = self.servo.ping()
        if not all(ping.values()):
            raise RuntimeError(f"Servo ping failed: {ping}")

        # Start in torque mode (safe default for IDLE).
        self.servo.set_mode_torque()
        self.servo.set_torque_enable(True)

        # --- FSM state ---
        self.fsm = FSM.IDLE
        # Track which hardware mode is currently active so we only switch when needed.
        self._hw_mode: str = "torque"   # "torque" | "wheel"

        # --- Twist state ---
        self.last_twist = np.zeros(3, dtype=np.float64)
        self.cartesian_twist = np.zeros(3, dtype=np.float64)
        self.last_twist_stamp_ns: int = 0
        self.q_freeze: Optional[np.ndarray] = None
        self._last_mode_warning = ""

        # --- Learn mode buffers ---
        self.learn_trajectories: list[np.ndarray] = []  # each trajectory: (6, N)
        self.learn_current_samples: list[np.ndarray] = []
        self.learn_active_traj = False
        self.learn_session_active = True
        self.learn_last_sample_ns = 0

        # --- ROS I/O ---
        state_topic = self.get_parameter("state_topic").value
        twist_topic = self.get_parameter("twist_topic").value

        self.state_pub = self.create_publisher(Float64MultiArray, state_topic, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.fsm_pub = self.create_publisher(String, "/controller/fsm", 10)
        self.learn_status_pub = self.create_publisher(String, "/controller/learn_status", 10)

        self.create_subscription(Float64MultiArray, twist_topic, self._on_twist_command, 10)

        period = 1.0 / float(self.get_parameter("control_hz").value)
        self.create_timer(period, self._run_control_cycle)

        self.get_logger().info("ROS DS controller started (wheel-mode CONTROL)")

    # ------------------------------------------------------------------
    # Subscriber
    # ------------------------------------------------------------------

    def _on_twist_command(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64).reshape(-1)
        if data.size < 3:
            self.get_logger().warning("Ignoring twist command with less than 3 components")
            return

        command = data[:3]
        max_twist = float(self.get_parameter("max_twist_mps").value)
        nrm = np.linalg.norm(command)
        if nrm > max_twist > 0:
            command = (command / nrm) * max_twist

        self.last_twist = command
        self.last_twist_stamp_ns = self.get_clock().now().nanoseconds

    # ------------------------------------------------------------------
    # Main control cycle
    # ------------------------------------------------------------------

    def _run_control_cycle(self) -> None:
        try:
            sample = self.servo.read_joint_state()
            q = sample.position_rad
            dq = sample.velocity_rad_s

            # Kinematics
            pin.forwardKinematics(self.model, self.data, q, dq)
            pin.updateFramePlacements(self.model, self.data)

            ee_pos = self.data.oMf[self.ee_frame_id].translation.copy()
            jac = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_frame_id,
                pin.LOCAL_WORLD_ALIGNED,
            )
            j_lin = jac[0:3, :]
            ee_vel = j_lin @ dq

            # Cartesian velocity filter with underflow guard
            _VEL_DEADBAND = 1e-6
            ee_vel_clean = np.where(np.abs(ee_vel) < _VEL_DEADBAND, 0.0, ee_vel)
            alpha = 0.2
            self.cartesian_twist = alpha * ee_vel_clean + (1.0 - alpha) * self.cartesian_twist
            if np.linalg.norm(self.cartesian_twist) < _VEL_DEADBAND:
                self.cartesian_twist = np.zeros(3)

            self._publish_state(sample.timestamp, ee_pos, ee_vel)
            self._publish_joint_state(sample.timestamp, q, dq)

            # Learn controls (edge-triggered via bool parameters)
            self._handle_learning_parameter_commands()

            # FSM transitions — also handles hardware mode switch
            self._update_fsm()

            if self.fsm == FSM.LEARN:
                self._record_learning_sample(ee_pos, ee_vel)

            # --- Send command depending on active FSM ---
            if self.fsm == FSM.CONTROL:
                # Wheel mode: compute and send joint speed command
                dq_des = self._compute_wheel_speed(j_lin)
                print("dq_des (rad/s):", np.round(dq_des, 4))
                self.servo.send_speed_rad_s(dq_des)
            else:
                # Torque mode: gravity compensation law
                g = -pin.computeGeneralizedGravity(self.model, self.data, q)
                g *= float(self.get_parameter("gravity_scale").value)
                tau = self._compute_torque(self.fsm, q, dq, jac, j_lin, self.cartesian_twist, g)
                torque_limit_units = int(self.get_parameter("torque_limit_units").value)
                self.servo.send_torque_nm(tau, clip=torque_limit_units)

            msg = String()
            msg.data = self.fsm.value
            self.fsm_pub.publish(msg)

        except ServoInterfaceError as err:
            self.get_logger().error(f"Servo communication error: {err}")
        except Exception as err:  # pylint: disable=broad-except
            self.get_logger().error(f"Control cycle exception: {err}")
            self.get_logger().debug(traceback.format_exc())
        except KeyboardInterrupt:
            pass

    # ------------------------------------------------------------------
    # FSM update — handles hardware mode transitions
    # ------------------------------------------------------------------

    def _update_fsm(self) -> None:
        mode = str(self.get_parameter("mode").value).upper().strip()

        if mode == "CONTROL":
            new_fsm = FSM.CONTROL
        elif mode == "FREEZE":
            if self.fsm != FSM.FREEZE:
                self.q_freeze = None
            new_fsm = FSM.FREEZE
        elif mode == "LEARN":
            new_fsm = FSM.LEARN
        elif mode == "IDLE":
            new_fsm = FSM.IDLE
        else:
            warning = f"Unknown mode '{mode}', falling back to IDLE"
            if warning != self._last_mode_warning:
                self.get_logger().warning(warning)
                self._last_mode_warning = warning
            new_fsm = FSM.IDLE

        # Hardware mode switch only when FSM actually changes
        if new_fsm != self.fsm:
            self._switch_hardware_mode(new_fsm)

        self.fsm = new_fsm

        if self.fsm != FSM.FREEZE:
            self.q_freeze = None

    def _switch_hardware_mode(self, target_fsm: FSM) -> None:
        """Switch servo hardware mode when entering / leaving CONTROL."""
        if target_fsm == FSM.CONTROL and self._hw_mode != "wheel":
            self.get_logger().info("Switching servos to WHEEL mode for CONTROL")
            self.servo.set_mode_wheel()
            
            # In wheel mode, the goal torque register acts as the output torque limit.
            # Initialize it high, along with 0 speed, otherwise it will be stuck with 
            # the last gravity compensation torque value (which is too weak to spin it).
            max_torque = float(self.get_parameter("torque_limit_units").value)
            self.servo.send_torque_units(np.full(len(self.servo.ids), max_torque))
            self.servo.send_speed_rad_s(np.zeros(len(self.servo.ids)))
            self._hw_mode = "wheel"

        elif target_fsm != FSM.CONTROL and self._hw_mode != "torque":
            self.get_logger().info(f"Switching servos back to TORQUE mode for {target_fsm.value}")
            # Zero speed before leaving wheel mode
            self.servo.send_speed_rad_s(np.zeros(len(self.servo.ids)))
            self.servo.set_mode_torque()
            self._hw_mode = "torque"

    # ------------------------------------------------------------------
    # CONTROL: wheel-mode speed command
    # ------------------------------------------------------------------

    def _compute_wheel_speed(self, j_lin: np.ndarray) -> np.ndarray:
        """Convert Cartesian v_des [m/s] to joint velocities [rad/s].

        Uses a damped pseudo-inverse of the linear Jacobian.
        Clamps each joint to wheel_max_joint_rad_s.
        If the twist command has timed out, returns zero velocities.
        """
        # Timeout guard: if MATLAB stopped publishing, stop the robot
        timeout_s = float(self.get_parameter("twist_timeout_s").value)
        now_ns = self.get_clock().now().nanoseconds
        if self.last_twist_stamp_ns > 0:
            age_s = (now_ns - self.last_twist_stamp_ns) * 1e-9
            if age_s > timeout_s:
                return np.zeros(self.nq, dtype=np.float64)

        v_des = self.last_twist  # shape (3,)

        # Damped pseudo-inverse J_lin^# = J_lin^T (J_lin J_lin^T + λ² I)^{-1}
        damping = float(self.get_parameter("wheel_pinv_damping").value)
        j_pinv = _damped_pseudoinverse(j_lin, damping)   # shape (nq, 3)

        dq_des = j_pinv @ v_des  # shape (nq,)

        # Clamp individual joint speeds
        max_dq = float(self.get_parameter("wheel_max_joint_rad_s").value)
        dq_des = np.clip(dq_des, -max_dq, max_dq)

        return dq_des

    # ------------------------------------------------------------------
    # Torque law (IDLE / LEARN / FREEZE)
    # ------------------------------------------------------------------

    def _compute_torque(
        self,
        fsm: FSM,
        q: np.ndarray,
        dq: np.ndarray,
        jac: np.ndarray,
        j_lin: np.ndarray,
        cartesian_twist: np.ndarray,
        g: np.ndarray,
    ) -> np.ndarray:
        q_null = np.asarray(self.get_parameter("q_null").value, dtype=np.float64)

        if fsm == FSM.FREEZE:
            if self.q_freeze is None:
                self.q_freeze = q.copy()
            k_freeze = float(self.get_parameter("k_freeze").value)
            k_d = float(self.get_parameter("k_learn_damping").value)
            return g + k_freeze * (self.q_freeze - q) - k_d * dq

        if fsm == FSM.LEARN:
            # Kinesthetic teaching mode: gravity compensation + very light damping
            k_learn = float(self.get_parameter("k_learn_damping").value)
            return g - k_learn * dq

        # IDLE: gravity compensation + light damping
        k_idle = float(self.get_parameter("k_idle_damping").value)
        return g - k_idle * dq

    # ------------------------------------------------------------------
    # Learn-mode helpers
    # ------------------------------------------------------------------

    def _handle_learning_parameter_commands(self) -> None:
        if bool(self.get_parameter("learn_clear_recording").value):
            self.learn_trajectories.clear()
            self.learn_current_samples.clear()
            self.learn_active_traj = False
            self.learn_session_active = True
            self._publish_learn_status("cleared")
            self.set_parameters([Parameter("learn_clear_recording", value=False)])

        if bool(self.get_parameter("learn_start_trajectory").value):
            if self.fsm != FSM.LEARN:
                self.get_logger().warning("learn_start_trajectory ignored because mode != LEARN")
            else:
                if self.learn_active_traj:
                    self.get_logger().warning("Trajectory already active")
                else:
                    self.learn_current_samples = []
                    self.learn_active_traj = True
                    self.learn_session_active = True
                    self.learn_last_sample_ns = 0
                    self._publish_learn_status("trajectory_started")
            self.set_parameters([Parameter("learn_start_trajectory", value=False)])

        if bool(self.get_parameter("learn_stop_trajectory").value):
            self._finalize_current_trajectory()
            self.set_parameters([Parameter("learn_stop_trajectory", value=False)])

        if bool(self.get_parameter("learn_stop_recording").value):
            if self.learn_active_traj:
                self._finalize_current_trajectory()
            self.learn_session_active = False
            output = self._export_recorded_trajectories()
            self._publish_learn_status(f"recording_stopped:{output}")
            self.set_parameters([Parameter("learn_stop_recording", value=False)])

    def _record_learning_sample(self, ee_pos: np.ndarray, ee_vel: np.ndarray) -> None:
        if not self.learn_session_active or not self.learn_active_traj:
            return

        sampling_hz = float(self.get_parameter("learn_sampling_hz").value)
        if sampling_hz <= 0:
            return
        period_ns = int(1e9 / sampling_hz)
        now_ns = self.get_clock().now().nanoseconds
        if self.learn_last_sample_ns != 0 and (now_ns - self.learn_last_sample_ns) < period_ns:
            return

        sample = np.hstack((ee_pos.reshape(3), ee_vel.reshape(3))).astype(np.float64)
        self.learn_current_samples.append(sample)
        self.learn_last_sample_ns = now_ns

    def _finalize_current_trajectory(self) -> None:
        if not self.learn_active_traj:
            self.get_logger().warning("No active trajectory to stop")
            return

        if len(self.learn_current_samples) < 2:
            self.get_logger().warning("Discarding trajectory with less than 2 samples")
            self.learn_current_samples = []
            self.learn_active_traj = False
            self.learn_last_sample_ns = 0
            self._publish_learn_status("trajectory_discarded")
            return

        arr = np.asarray(self.learn_current_samples, dtype=np.float64).T  # (6, N)
        self.learn_trajectories.append(arr)
        self.learn_current_samples = []
        self.learn_active_traj = False
        self.learn_last_sample_ns = 0
        self._publish_learn_status(f"trajectory_stopped:count={len(self.learn_trajectories)}")

    def _export_recorded_trajectories(self) -> str:
        if not self.learn_trajectories:
            self.get_logger().warning("No trajectories recorded, nothing to export")
            return "none"

        points = int(self.get_parameter("learn_resample_points").value)
        points = max(points, 2)
        trajs = [self._resample_traj(traj, points) for traj in self.learn_trajectories]
        tensor = np.stack(trajs, axis=2)  # (6, N, nTraj)

        output_file = str(self.get_parameter("learn_output_file").value)
        output_path = Path(output_file)
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path

        try:
            from scipy.io import savemat  # type: ignore

            savemat(str(output_path), {"trajectories": tensor})
            self.get_logger().info(f"Saved trajectories to {output_path}")
            return str(output_path)
        except Exception as err:  # pylint: disable=broad-except
            fallback = output_path.with_suffix(".npz")
            np.savez_compressed(str(fallback), trajectories=tensor)
            meta = fallback.with_suffix(".json")
            meta.write_text(json.dumps({"shape": list(tensor.shape), "variable": "trajectories"}, indent=2))
            self.get_logger().warning(
                f"Could not export .mat ({err}); saved fallback {fallback} and {meta}"
            )
            return str(fallback)

    @staticmethod
    def _resample_traj(traj: np.ndarray, n_points: int) -> np.ndarray:
        # traj shape: (6, N)
        old_n = traj.shape[1]
        if old_n == n_points:
            return traj
        t_old = np.linspace(0.0, 1.0, old_n)
        t_new = np.linspace(0.0, 1.0, n_points)
        out = np.zeros((traj.shape[0], n_points), dtype=np.float64)
        for k in range(traj.shape[0]):
            out[k, :] = np.interp(t_new, t_old, traj[k, :])
        return out

    def _publish_learn_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.learn_status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_state(self, timestamp: float, ee_pos: np.ndarray, ee_vel: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [
            float(timestamp),
            float(ee_pos[0]),
            float(ee_pos[1]),
            float(ee_pos[2]),
            float(ee_vel[0]),
            float(ee_vel[1]),
            float(ee_vel[2]),
        ]
        self.state_pub.publish(msg)

    def _publish_joint_state(self, timestamp: float, q: np.ndarray, dq: np.ndarray) -> None:
        msg = JointState()
        sec = int(timestamp)
        nsec = int((timestamp - sec) * 1e9)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.name = self.joint_names
        msg.position = q.tolist()
        msg.velocity = dq.tolist()
        self.joint_pub.publish(msg)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        self.get_logger().info("Shutting down controller")
        try:
            if self._hw_mode == "wheel":
                self.servo.send_speed_rad_s(np.zeros(len(self.servo.ids)))
                self.servo.set_mode_torque()
            self.servo.send_torque_units(np.zeros(len(self.servo.ids)))
            self.servo.set_torque_enable(False)
            self.servo.close()
        except Exception as err:  # pylint: disable=broad-except
            self.get_logger().warning(f"Error while closing servo interface: {err}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[RosDsController] = None
    try:
        node = RosDsController()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()