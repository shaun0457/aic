#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

"""CheatCode + lerobot-format dataset recorder.

Records CheatCode demonstrations as (observation, action) pairs.

Observation (26D state + 3 cameras):
  - TCP pose: position xyz (3) + orientation xyzw (4)
  - TCP velocity: linear xyz (3) + angular xyz (3)
  - TCP error (6)
  - Joint positions (7)

Action (7D, matches RunACT output space):
  - [vx, vy, vz, wx, wy, wz, 0.0]  (cartesian twist in base_link)
  - Sourced from controller_state.tcp_velocity at each step.

Dataset layout (per episode):
  /OUTPUT_DIR/episode_NNNNNN/
      frames.npz          -- arrays: states (T,26), actions (T,7)
      left/   TTTTT.jpg
      center/ TTTTT.jpg
      right/  TTTTT.jpg
      meta.json           -- task, success, fps, n_frames

Run 100 episodes by looping `docker exec` calls (see run_batch.sh).
"""

import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

OUTPUT_DIR = os.environ.get("RECORD_OUTPUT_DIR", "/tmp/aic_dataset")
IMAGE_SCALE = 0.25  # must match RunACT / AICRobotAICControllerConfig


def _obs_to_state(obs_msg: Observation) -> np.ndarray:
    """Extract 26D state vector from Observation, matching RunACT order."""
    cs = obs_msg.controller_state
    tcp_pose = cs.tcp_pose
    tcp_vel = cs.tcp_velocity
    joints = list(obs_msg.joint_states.position[:7])
    return np.array(
        [
            tcp_pose.position.x,
            tcp_pose.position.y,
            tcp_pose.position.z,
            tcp_pose.orientation.x,
            tcp_pose.orientation.y,
            tcp_pose.orientation.z,
            tcp_pose.orientation.w,
            tcp_vel.linear.x,
            tcp_vel.linear.y,
            tcp_vel.linear.z,
            tcp_vel.angular.x,
            tcp_vel.angular.y,
            tcp_vel.angular.z,
            cs.tcp_error[0],
            cs.tcp_error[1],
            cs.tcp_error[2],
            cs.tcp_error[3],
            cs.tcp_error[4],
            cs.tcp_error[5],
            joints[0],
            joints[1],
            joints[2],
            joints[3],
            joints[4],
            joints[5],
            joints[6],
        ],
        dtype=np.float32,
    )


def _obs_to_action(obs_msg: Observation) -> np.ndarray:
    """Extract 7D action from tcp_velocity, matching RunACT action space."""
    v = obs_msg.controller_state.tcp_velocity
    return np.array(
        [v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z, 0.0],
        dtype=np.float32,
    )


def _decode_image(ros_img, scale: float) -> np.ndarray:
    """ROS Image msg → (H, W, 3) uint8 numpy, optionally downscaled."""
    img = np.frombuffer(ros_img.data, dtype=np.uint8).reshape(
        ros_img.height, ros_img.width, 3
    )
    if scale != 1.0:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return img


class _EpisodeBuffer:
    def __init__(self, episode_dir: Path, fps: float):
        self.episode_dir = episode_dir
        self.fps = fps
        self.states: list[np.ndarray] = []
        self.actions: list[np.ndarray] = []
        self.images: dict[str, list[np.ndarray]] = {
            "left": [],
            "center": [],
            "right": [],
        }
        for cam in self.images:
            (episode_dir / cam).mkdir(parents=True, exist_ok=True)

    def add(self, obs_msg: Observation):
        self.states.append(_obs_to_state(obs_msg))
        self.actions.append(_obs_to_action(obs_msg))
        t = len(self.states) - 1
        for cam, ros_img in [
            ("left", obs_msg.left_image),
            ("center", obs_msg.center_image),
            ("right", obs_msg.right_image),
        ]:
            img = _decode_image(ros_img, IMAGE_SCALE)
            cv2.imwrite(str(self.episode_dir / cam / f"{t:05d}.jpg"), img)

    def save(self, task_name: str, success: bool):
        states = np.stack(self.states)   # (T, 26)
        actions = np.stack(self.actions)  # (T, 7)
        np.savez_compressed(
            self.episode_dir / "frames.npz",
            states=states,
            actions=actions,
        )
        meta = {
            "task": task_name,
            "success": success,
            "n_frames": len(self.states),
            "fps": self.fps,
        }
        with open(self.episode_dir / "meta.json", "w") as f:
            json.dump(meta, f, indent=2)


class CheatCodeRecorder(Policy):
    """CheatCode policy that simultaneously records a lerobot-compatible dataset."""

    def __init__(self, parent_node):
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0
        self._max_integrator_windup = 0.05
        self._task = None
        super().__init__(parent_node)

        self._output_dir = Path(OUTPUT_DIR)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"CheatCodeRecorder: dataset → {self._output_dir}")

    # ---- TF helpers (copied from CheatCode) ----

    def _wait_for_tf(self, target_frame, source_frame, timeout_sec=10.0):
        start = self.time_now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(target_frame, source_frame, Time())
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(
                        f"Waiting for TF '{source_frame}' → '{target_frame}'..."
                    )
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(f"TF '{source_frame}' not available after {timeout_sec}s")
        return False

    def calc_gripper_pose(
        self,
        port_transform: Transform,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        reset_xy_integrator: bool = False,
    ) -> Pose:
        q_port = (
            port_transform.rotation.w,
            port_transform.rotation.x,
            port_transform.rotation.y,
            port_transform.rotation.z,
        )
        plug_tf = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            f"{self._task.cable_name}/{self._task.plug_name}_link",
            Time(),
        )
        q_plug = (
            plug_tf.transform.rotation.w,
            plug_tf.transform.rotation.x,
            plug_tf.transform.rotation.y,
            plug_tf.transform.rotation.z,
        )
        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)

        gripper_tf = self._parent_node._tf_buffer.lookup_transform(
            "base_link", "gripper/tcp", Time()
        )
        q_gripper = (
            gripper_tf.transform.rotation.w,
            gripper_tf.transform.rotation.x,
            gripper_tf.transform.rotation.y,
            gripper_tf.transform.rotation.z,
        )
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)
        q_slerp = quaternion_slerp(q_gripper, q_gripper_target, slerp_fraction)

        gripper_xyz = (
            gripper_tf.transform.translation.x,
            gripper_tf.transform.translation.y,
            gripper_tf.transform.translation.z,
        )
        port_xy = (port_transform.translation.x, port_transform.translation.y)
        plug_xyz = (
            plug_tf.transform.translation.x,
            plug_tf.transform.translation.y,
            plug_tf.transform.translation.z,
        )
        plug_tip_gripper_offset = (
            gripper_xyz[0] - plug_xyz[0],
            gripper_xyz[1] - plug_xyz[1],
            gripper_xyz[2] - plug_xyz[2],
        )

        tip_x_error = port_xy[0] - plug_xyz[0]
        tip_y_error = port_xy[1] - plug_xyz[1]

        if reset_xy_integrator:
            self._tip_x_error_integrator = 0.0
            self._tip_y_error_integrator = 0.0
        else:
            self._tip_x_error_integrator = np.clip(
                self._tip_x_error_integrator + tip_x_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )
            self._tip_y_error_integrator = np.clip(
                self._tip_y_error_integrator + tip_y_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )

        i_gain = 0.15
        target_x = port_xy[0] + i_gain * self._tip_x_error_integrator
        target_y = port_xy[1] + i_gain * self._tip_y_error_integrator
        target_z = port_transform.translation.z + z_offset - plug_tip_gripper_offset[2]

        blend_xyz = (
            position_fraction * target_x + (1.0 - position_fraction) * gripper_xyz[0],
            position_fraction * target_y + (1.0 - position_fraction) * gripper_xyz[1],
            position_fraction * target_z + (1.0 - position_fraction) * gripper_xyz[2],
        )

        return Pose(
            position=Point(x=blend_xyz[0], y=blend_xyz[1], z=blend_xyz[2]),
            orientation=Quaternion(w=q_slerp[0], x=q_slerp[1], y=q_slerp[2], z=q_slerp[3]),
        )

    # ---- Episode directory numbering ----

    def _next_episode_dir(self) -> Path:
        existing = sorted(self._output_dir.glob("episode_*"))
        n = len(existing)
        ep_dir = self._output_dir / f"episode_{n:06d}"
        ep_dir.mkdir(parents=True, exist_ok=True)
        return ep_dir

    # ---- Main entry point ----

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(f"CheatCodeRecorder.insert_cable() task: {task}")
        self._task = task

        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        cable_tip_frame = f"{task.cable_name}/{task.plug_name}_link"

        for frame in [port_frame, cable_tip_frame]:
            if not self._wait_for_tf("base_link", frame):
                return False

        try:
            port_tf = self._parent_node._tf_buffer.lookup_transform(
                "base_link", port_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not look up port transform: {ex}")
            return False

        port_transform = port_tf.transform
        ep_dir = self._next_episode_dir()
        fps = 1.0 / 0.05  # 20 Hz loop
        buf = _EpisodeBuffer(ep_dir, fps)

        self.get_logger().info(f"Recording episode → {ep_dir}")

        z_offset = 0.2

        # --- Approach phase (100 steps × 0.05s = 5s) ---
        for t in range(0, 100):
            interp_fraction = t / 100.0
            obs = get_observation()
            if obs is not None:
                buf.add(obs)
            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(
                        port_transform,
                        slerp_fraction=interp_fraction,
                        position_fraction=interp_fraction,
                        z_offset=z_offset,
                        reset_xy_integrator=True,
                    ),
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed during approach: {ex}")
            self.sleep_for(0.05)

        # --- Insertion phase (descend until z_offset < -0.015) ---
        while True:
            if z_offset < -0.015:
                break
            z_offset -= 0.0005
            obs = get_observation()
            if obs is not None:
                buf.add(obs)
            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(port_transform, z_offset=z_offset),
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed during insertion: {ex}")
            self.sleep_for(0.05)

        self.get_logger().info("Waiting for connector to stabilize...")
        self.sleep_for(5.0)

        # Save episode
        task_name = f"{task.cable_name}/{task.plug_name} → {task.target_module_name}/{task.port_name}"
        buf.save(task_name=task_name, success=True)
        self.get_logger().info(f"Episode saved: {len(buf.states)} frames → {ep_dir}")

        return True
