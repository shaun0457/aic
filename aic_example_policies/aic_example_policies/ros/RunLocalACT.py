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

import json
import os
import time

import cv2
import draccus
import numpy as np
import torch
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Vector3, Wrench
from geometry_msgs.msg import Twist
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from pathlib import Path
from rclpy.node import Node
from safetensors.torch import load_file
from typing import Dict


# Checkpoint directory: ROS param > env var > default path inside container
_DEFAULT_CHECKPOINT_DIR = "/checkpoint"


class RunLocalACT(Policy):
    """ACT policy loaded from a local checkpoint directory.

    The checkpoint directory must contain:
      - config.json
      - model.safetensors
      - policy_preprocessor_step_3_normalizer_processor.safetensors

    Set the path via:
      ROS parameter  policy_checkpoint_dir  (highest priority)
      Env variable   AIC_POLICY_CHECKPOINT_DIR
      Default        /checkpoint  (path inside the Docker container)
    """

    def __init__(self, parent_node: Node):
        super().__init__(parent_node)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # ------------------------------------------------------------------ #
        # 1. Resolve checkpoint directory
        # ------------------------------------------------------------------ #
        if not parent_node.has_parameter("policy_checkpoint_dir"):
            parent_node.declare_parameter(
                "policy_checkpoint_dir",
                os.environ.get("AIC_POLICY_CHECKPOINT_DIR", _DEFAULT_CHECKPOINT_DIR),
            )
        checkpoint_dir = (
            parent_node.get_parameter("policy_checkpoint_dir")
            .get_parameter_value()
            .string_value
        )
        policy_path = Path(checkpoint_dir)

        if not policy_path.exists():
            raise FileNotFoundError(
                f"Checkpoint directory not found: {policy_path}. "
                "Set --ros-args -p policy_checkpoint_dir:=<path> or "
                "env AIC_POLICY_CHECKPOINT_DIR=<path>."
            )

        self.get_logger().info(f"Loading ACT policy from local path: {policy_path}")

        # ------------------------------------------------------------------ #
        # 2. Load config and weights
        # ------------------------------------------------------------------ #
        config_path = policy_path / "config.json"
        with open(config_path) as f:
            config_dict = json.load(f)
        config_dict.pop("type", None)  # draccus rejects unknown keys

        config = draccus.decode(ACTConfig, config_dict)

        self.policy = ACTPolicy(config)
        self.policy.load_state_dict(load_file(policy_path / "model.safetensors"))
        self.policy.eval()
        self.policy.to(self.device)

        self.get_logger().info(f"ACT Policy loaded on {self.device}")

        # ------------------------------------------------------------------ #
        # 3. Load normalisation statistics
        # ------------------------------------------------------------------ #
        stats = load_file(
            policy_path
            / "policy_preprocessor_step_3_normalizer_processor.safetensors"
        )

        def get_stat(key: str, shape):
            return stats[key].to(self.device).view(*shape)

        self.img_stats = {
            "left": {
                "mean": get_stat("observation.images.left_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.left_camera.std", (1, 3, 1, 1)),
            },
            "center": {
                "mean": get_stat("observation.images.center_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.center_camera.std", (1, 3, 1, 1)),
            },
            "right": {
                "mean": get_stat("observation.images.right_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.right_camera.std", (1, 3, 1, 1)),
            },
        }

        self.state_mean = get_stat("observation.state.mean", (1, -1))
        self.state_std = get_stat("observation.state.std", (1, -1))
        self.action_mean = get_stat("action.mean", (1, -1))
        self.action_std = get_stat("action.std", (1, -1))

        self.image_scaling = 0.25  # must match AICRobotAICControllerConfig

        self.get_logger().info("Normalisation statistics loaded.")

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _img_to_tensor(
        raw_img,
        device: torch.device,
        scale: float,
        mean: torch.Tensor,
        std: torch.Tensor,
    ) -> torch.Tensor:
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        if scale != 1.0:
            img_np = cv2.resize(
                img_np, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        tensor = (
            torch.from_numpy(img_np)
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(device)
        )
        return (tensor - mean) / std

    def _prepare_observations(self, obs_msg: Observation) -> Dict[str, torch.Tensor]:
        obs = {
            "observation.images.left_camera": self._img_to_tensor(
                obs_msg.left_image,
                self.device,
                self.image_scaling,
                self.img_stats["left"]["mean"],
                self.img_stats["left"]["std"],
            ),
            "observation.images.center_camera": self._img_to_tensor(
                obs_msg.center_image,
                self.device,
                self.image_scaling,
                self.img_stats["center"]["mean"],
                self.img_stats["center"]["std"],
            ),
            "observation.images.right_camera": self._img_to_tensor(
                obs_msg.right_image,
                self.device,
                self.image_scaling,
                self.img_stats["right"]["mean"],
                self.img_stats["right"]["std"],
            ),
        }

        tcp_pose = obs_msg.controller_state.tcp_pose
        tcp_vel = obs_msg.controller_state.tcp_velocity
        state_np = np.array(
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
                *obs_msg.controller_state.tcp_error,
                *obs_msg.joint_states.position[:7],
            ],
            dtype=np.float32,
        )

        raw_state = torch.from_numpy(state_np).float().unsqueeze(0).to(self.device)
        obs["observation.state"] = (raw_state - self.state_mean) / self.state_std
        return obs

    def _build_motion_update(self, twist: Twist, frame_id: str = "base_link") -> MotionUpdate:
        msg = MotionUpdate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.velocity = twist
        msg.target_stiffness = np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten()
        msg.target_damping = np.diag(
            [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        ).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
        return msg

    # ------------------------------------------------------------------ #
    # Policy entry point
    # ------------------------------------------------------------------ #

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.policy.reset()
        self.get_logger().info(f"RunLocalACT.insert_cable() task: {task}")

        start_time = time.time()

        while time.time() - start_time < 30.0:
            loop_start = time.time()

            obs_msg = get_observation()
            if obs_msg is None:
                self.get_logger().info("No observation received, waiting...")
                time.sleep(0.05)
                continue

            obs_tensors = self._prepare_observations(obs_msg)

            with torch.inference_mode():
                normalized_action = self.policy.select_action(obs_tensors)

            raw_action = (normalized_action * self.action_std) + self.action_mean
            action = raw_action[0].cpu().numpy()

            self.get_logger().info(f"Action: {action}")

            twist = Twist(
                linear=Vector3(x=float(action[0]), y=float(action[1]), z=float(action[2])),
                angular=Vector3(x=float(action[3]), y=float(action[4]), z=float(action[5])),
            )
            move_robot(motion_update=self._build_motion_update(twist))
            send_feedback("in progress...")

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, 0.25 - elapsed))

        self.get_logger().info("RunLocalACT.insert_cable() complete.")
        return True
