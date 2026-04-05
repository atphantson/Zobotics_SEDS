#!/usr/bin/env python3
"""ROS2 control node for FTServo arm with DS command input.

Responsibilities:
- Read robot state from HLS servos.
- Publish state to MATLAB/RLearn side.
- Receive DS Cartesian velocity command from ROS.
- Run FSM and impedance-like torque control.
- Send torque commands to servos.
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

        self.declare_parameter("k_dq", 0.35)
        # Set to 0.0 to match compensation.py pure gravity compensation in IDLE.
        self.declare_parameter("k_idle_damping", -0.2)
        self.declare_parameter("k_freeze", 1.2)
        self.declare_parameter("k_learn_damping", 0.0)
        self.declare_parameter("k_null", 0.1)
        self.declare_parameter("k_null_damping", 0.08)

        # Parameters matching MATLAB bridge CONTROL behavior
        self.declare_parameter("activate_ns", False)
        self.declare_parameter("principle_damping", 7.0)
        self.declare_parameter("orthogonal_damping", 4.0)
        self.declare_parameter("null_stiffness", 1.0)
        self.declare_parameter("null_damping", 2.0)
        self.declare_parameter("pseudoinverse_damping", 1e-2)

        self.declare_parameter("q_null", [0.0, 0.0, 0.0])
        self.declare_parameter("gravity_scale", 1.0)

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
        self.servo.set_mode_torque()
        self.servo.set_torque_enable(True)

        # --- State ---
        self.fsm = FSM.IDLE
        self.last_twist = np.zeros(3, dtype=np.float64)
        self.cartesian_twist = np.zeros(3, dtype=np.float64)
        self.last_twist_stamp_ns = 0
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

        self.get_logger().info("ROS DS controller started")

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

            # Match MATLAB bridge velocity filtering
            alpha = 0.2
            self.cartesian_twist = alpha * ee_vel + (1.0 - alpha) * self.cartesian_twist

            self._publish_state(sample.timestamp, ee_pos, ee_vel)
            self._publish_joint_state(sample.timestamp, q, dq)

            # Learn controls (edge-triggered via bool parameters)
            self._handle_learning_parameter_commands()

            # FSM transitions
            self._update_fsm()

            if self.fsm == FSM.LEARN:
                self._record_learning_sample(ee_pos, ee_vel)

            # Dynamics terms
            g = -pin.computeGeneralizedGravity(self.model, self.data, q)
            g *= float(self.get_parameter("gravity_scale").value)

            #Control law
            tau = self._compute_torque(self.fsm, q, dq, jac, j_lin, self.cartesian_twist, g)

            # Send command
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
            self.set_mode_torque()

    def _update_fsm(self) -> None:
        mode = str(self.get_parameter("mode").value).upper().strip()
        if mode == "CONTROL":
            self.fsm = FSM.CONTROL
            self.q_freeze = None
            return

        if mode == "FREEZE":
            if self.fsm != FSM.FREEZE:
                self.q_freeze = None
            self.fsm = FSM.FREEZE
            return

        if mode == "LEARN":
            self.fsm = FSM.LEARN
            self.q_freeze = None
            return

        if mode == "IDLE":
            self.fsm = FSM.IDLE
            self.q_freeze = None
            return

        # Fallback safety
        self.fsm = FSM.IDLE
        warning = f"Unknown mode '{mode}', falling back to IDLE"
        if warning != self._last_mode_warning:
            self.get_logger().warning(warning)
            self._last_mode_warning = warning

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

        if fsm == FSM.CONTROL:
            # Compliant-twist-like task torque in Cartesian space.
            # D = d_orth * I + (d_principle - d_orth) * n n^T, with n = v_des / ||v_des||.
            v_des = self.last_twist
            v_meas = cartesian_twist

            d_principle = float(self.get_parameter("principle_damping").value)
            d_orthogonal = float(self.get_parameter("orthogonal_damping").value)

            v_norm = np.linalg.norm(v_des)
            if v_norm > 1e-12:
                n_dir = v_des / v_norm
                damping_mat = d_orthogonal * np.eye(3) + (d_principle - d_orthogonal) * np.outer(n_dir, n_dir)
            else:
                damping_mat = d_orthogonal * np.eye(3)

            cartesian_force = damping_mat @ (v_des - v_meas)
            tau = j_lin.T @ cartesian_force

            # Null-space projected joint damping/stiffness correction as in MATLAB bridge
            if bool(self.get_parameter("activate_ns").value):
                pinv_damping = float(self.get_parameter("pseudoinverse_damping").value)
                jac_damped_inv = self._damped_pseudoinverse(jac, damping=pinv_damping)
                projector = np.eye(self.nq) - jac_damped_inv @ jac

                null_stiffness = float(self.get_parameter("null_stiffness").value)
                null_damping = float(self.get_parameter("null_damping").value)
                null_error = null_stiffness * (q_null - q) - null_damping * dq
                tau += projector @ null_error
            
            tau += g
            hls_units = np.clip(np.rint(tau * 10.197162129779 / (8.3 * 0.0065)), -2047, 2047).astype(np.int32)
            g_units = np.clip(np.rint(g * 10.197162129779 / (8.3 * 0.0065)), -2047, 2047).astype(np.int32)
            print("torque units:", hls_units)
            print("gravity units:", g_units)

            return tau

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
            # Fallback without scipy
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

    @staticmethod
    def _damped_pseudoinverse(jacobian: np.ndarray, damping: float = 1e-2) -> np.ndarray:
        jjt = jacobian @ jacobian.T
        return jacobian.T @ np.linalg.inv(jjt + damping * np.eye(jjt.shape[0]))

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

    def shutdown(self) -> None:
        self.get_logger().info("Shutting down controller")
        try:
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
