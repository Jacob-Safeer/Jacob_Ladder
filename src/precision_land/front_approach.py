#!/usr/bin/env python3
"""
Python reimplementation of the C++ FrontApproach mode.
Listens to /front/target_pose and sends PX4 trajectory setpoints to stop at a
front-facing ArUco tag.
"""

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleLocalPosition


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    return q if norm == 0.0 else q / norm


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif m00 > m11 and m00 > m22:
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S
    return quat_normalize(np.array([w, x, y, z], dtype=float))


class FrontApproachNode(Node):
    class State:
        IDLE = "Idle"
        SEARCH = "Search"
        APPROACH = "Approach"
        FINISHED = "Finished"

    def __init__(self) -> None:
        super().__init__("front_approach_py")

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.front_target_sub = self.create_subscription(
            PoseStamped, "/front/target_pose", self.front_target_cb, qos
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.local_position_cb,
            qos,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_cb, qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )

        self.declare_parameter("front_hold_distance", 1.0)
        self.declare_parameter("front_delta_position", 0.25)
        self.declare_parameter("front_delta_velocity", 0.25)
        self.declare_parameter("front_target_timeout", 3.0)
        self.declare_parameter("front_pid_kp", 1.2)
        self.declare_parameter("front_pid_ki", 0.0)
        self.declare_parameter("front_pid_kd", 0.0)
        self.declare_parameter("front_pid_max_velocity", 2.5)
        self.declare_parameter("front_pid_integral_limit", 1.5)
        self.declare_parameter("front_pid_kp_z", 1.0)
        self.declare_parameter("front_pid_max_velocity_z", 1.0)

        self.param_hold_distance = (
            self.get_parameter("front_hold_distance").get_parameter_value().double_value
        )
        self.param_delta_position = (
            self.get_parameter("front_delta_position").get_parameter_value().double_value
        )
        self.param_delta_velocity = (
            self.get_parameter("front_delta_velocity").get_parameter_value().double_value
        )
        self.param_target_timeout = (
            self.get_parameter("front_target_timeout").get_parameter_value().double_value
        )

        self.param_kp_xy = (
            self.get_parameter("front_pid_kp").get_parameter_value().double_value
        )
        self.param_ki_xy = (
            self.get_parameter("front_pid_ki").get_parameter_value().double_value
        )
        self.param_kd_xy = (
            self.get_parameter("front_pid_kd").get_parameter_value().double_value
        )
        self.param_max_velocity_xy = (
            self.get_parameter("front_pid_max_velocity").get_parameter_value().double_value
        )
        self.param_integral_limit = (
            self.get_parameter("front_pid_integral_limit").get_parameter_value().double_value
        )
        self.param_kp_z = (
            self.get_parameter("front_pid_kp_z").get_parameter_value().double_value
        )
        self.param_max_velocity_z = (
            self.get_parameter("front_pid_max_velocity_z").get_parameter_value().double_value
        )

        self.state = self.State.SEARCH
        self.front_tag = None  # type: Optional[dict]
        self.target_lost_prev = True

        self.local_position = None  # type: Optional[VehicleLocalPosition]
        self.attitude_quat = None  # type: Optional[np.ndarray]

        self.integral_xy = np.zeros(2, dtype=float)
        self.prev_error_xy = np.zeros(2, dtype=float)
        self.has_prev_error = False

        front_matrix = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
        self.front_optical_to_body = rot_to_quat(front_matrix)

        self.prev_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 50.0, self.control_loop)

    def front_target_cb(self, msg: PoseStamped) -> None:
        tag = {
            "position": np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                dtype=float,
            ),
            "orientation": quat_normalize(
                np.array(
                    [
                        msg.pose.orientation.w,
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                    ],
                    dtype=float,
                )
            ),
            "timestamp": self.get_clock().now(),
        }

        if not np.isfinite(tag["position"]).all() or not np.isfinite(tag["orientation"]).all():
            return

        transformed = self.transform_tag_to_world(tag)
        if transformed is not None:
            self.front_tag = transformed

    def local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg

    def attitude_cb(self, msg: VehicleAttitude) -> None:
        self.attitude_quat = quat_normalize(np.array(msg.q, dtype=float))

    def target_expired(self, now: rclpy.time.Time) -> bool:
        if self.front_tag is None:
            return True
        return (now.nanoseconds - self.front_tag["timestamp"].nanoseconds) * 1e-9 > self.param_target_timeout

    def transform_tag_to_world(self, tag: dict) -> Optional[dict]:
        if self.local_position is None or self.attitude_quat is None:
            return None

        vehicle_pos = np.array(
            [self.local_position.x, self.local_position.y, self.local_position.z],
            dtype=float,
        )
        R_vehicle = quat_to_rot(self.attitude_quat)
        R_camera = quat_to_rot(self.front_optical_to_body)

        tag_pos_body = R_camera @ tag["position"]
        world_position = vehicle_pos + R_vehicle @ tag_pos_body

        tag_orientation_world = quat_normalize(
            quat_multiply(quat_multiply(self.attitude_quat, self.front_optical_to_body), tag["orientation"])
        )

        return {
            "position": world_position,
            "orientation": tag_orientation_world,
            "timestamp": tag["timestamp"],
        }

    def position_reached(self, target: np.ndarray) -> bool:
        if self.local_position is None:
            return False
        pos = np.array(
            [self.local_position.x, self.local_position.y, self.local_position.z], dtype=float
        )
        vel = np.array(
            [self.local_position.vx, self.local_position.vy, self.local_position.vz],
            dtype=float,
        )
        delta = target - pos
        return (
            np.linalg.norm(delta[:2]) < self.param_delta_position
            and abs(delta[2]) < self.param_delta_position
            and np.linalg.norm(vel) < self.param_delta_velocity
        )

    def reset_controller(self) -> None:
        self.integral_xy[:] = 0.0
        self.prev_error_xy[:] = 0.0
        self.has_prev_error = False

    def publish_position_setpoint(self, position: np.ndarray, yaw: float = math.nan) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = position.astype(np.float32).tolist()
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = yaw
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def publish_velocity_setpoint(self, velocity: np.ndarray, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = velocity.astype(np.float32).tolist()
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = yaw
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.prev_time.nanoseconds) * 1e-9
        self.prev_time = now
        if dt <= 0.0:
            return

        if self.local_position is None or self.attitude_quat is None:
            return

        target_lost = self.target_expired(now)
        if target_lost != self.target_lost_prev:
            if target_lost:
                self.get_logger().info(f"Front target lost while in {self.state}")
            else:
                self.get_logger().info("Front target acquired")
        self.target_lost_prev = target_lost

        if self.state == self.State.IDLE:
            return

        if self.state == self.State.SEARCH:
            hold = np.array(
                [self.local_position.x, self.local_position.y, self.local_position.z],
                dtype=float,
            )
            self.publish_position_setpoint(hold)
            if self.front_tag is not None and not target_lost:
                self.state = self.State.APPROACH
            return

        if self.state == self.State.APPROACH:
            if target_lost:
                self.state = self.State.SEARCH
                return

            vehicle_pos = np.array(
                [self.local_position.x, self.local_position.y, self.local_position.z],
                dtype=float,
            )
            tag_pos = self.front_tag["position"]
            to_tag = tag_pos - vehicle_pos
            delta_xy = to_tag[:2]
            desired_xy = delta_xy.copy()
            distance_xy = np.linalg.norm(delta_xy)
            if distance_xy > 1e-3:
                hold = float(self.param_hold_distance)
                desired_xy = delta_xy - delta_xy / distance_xy * hold if distance_xy > hold else np.zeros(2)

            target_position = np.array(
                [vehicle_pos[0] + desired_xy[0], vehicle_pos[1] + desired_xy[1], tag_pos[2]],
                dtype=float,
            )
            error_xy = np.array(
                [
                    target_position[0] - self.local_position.x,
                    target_position[1] - self.local_position.y,
                ],
                dtype=float,
            )
            self.integral_xy += error_xy * dt
            self.integral_xy = np.clip(
                self.integral_xy,
                -self.param_integral_limit,
                self.param_integral_limit,
            )

            derivative_xy = (
                (error_xy - self.prev_error_xy) / dt if self.has_prev_error and dt > 1e-3 else np.zeros(2)
            )

            vel_xy = (
                self.param_kp_xy * error_xy
                + self.param_ki_xy * self.integral_xy
                + self.param_kd_xy * derivative_xy
            )
            speed = np.linalg.norm(vel_xy)
            if speed > self.param_max_velocity_xy:
                vel_xy = vel_xy / speed * self.param_max_velocity_xy

            error_z = target_position[2] - self.local_position.z
            vel_z = self.param_kp_z * error_z
            vel_z = float(np.clip(vel_z, -self.param_max_velocity_z, self.param_max_velocity_z))

            velocity_cmd = np.array([vel_xy[0], vel_xy[1], vel_z], dtype=float)
            desired_yaw = math.atan2(to_tag[1], to_tag[0])
            self.publish_velocity_setpoint(velocity_cmd, desired_yaw)

            self.prev_error_xy = error_xy
            self.has_prev_error = True

            if self.position_reached(target_position):
                self.state = self.State.FINISHED
            return

        if self.state == self.State.FINISHED:
            hold = np.array(
                [self.local_position.x, self.local_position.y, self.local_position.z],
                dtype=float,
            )
            self.publish_position_setpoint(hold)
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
