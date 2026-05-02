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

# No module-level heavy imports — keeps importlib.import_module() fast so the
# ROS 2 lifecycle GetState service can respond before the engine times out.


class _LightPolicy:
    """Minimal stand-in for aic_model.policy.Policy with no ROS message imports."""
    def __init__(self, parent_node):
        self._parent_node = parent_node

    def get_logger(self):
        return self._parent_node.get_logger()

    def get_clock(self):
        return self._parent_node.get_clock()


class RunACT(_LightPolicy):
    def __init__(self, parent_node):
        super().__init__(parent_node)

        import os
        import json
        import torch
        import numpy as np
        import cv2
        import draccus
        from pathlib import Path
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.act.configuration_act import ACTConfig
        from safetensors.torch import load_file

        self._np = np
        self._cv2 = cv2
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        policy_path = Path(
            "/root/.cache/huggingface/hub/models--shaun0457--act_aic"
            "/snapshots/25f6c067ea29d50b1b94221aff0c1fd7e10518fa"
        )

        with open(policy_path / "config.json", "r") as f:
            config_dict = json.load(f)
            if "type" in config_dict:
                del config_dict["type"]

        config = draccus.decode(ACTConfig, config_dict)
        self.policy = ACTPolicy(config)
        self.policy.load_state_dict(load_file(policy_path / "model.safetensors"))
        self.policy.eval()
        self.policy.to(self.device)

        self.get_logger().info(f"ACT Policy loaded on {self.device} from {policy_path}")

        stats = load_file(
            policy_path / "policy_preprocessor_step_3_normalizer_processor.safetensors"
        )

        def get_stat(key, shape):
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
        self.image_scaling = 0.25

        self.get_logger().info("Normalization statistics loaded successfully.")

    def _img_to_tensor(self, raw_img, scale, mean, std):
        import torch
        np = self._np
        cv2 = self._cv2
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        if scale != 1.0:
            img_np = cv2.resize(img_np, None, fx=scale, fy=scale,
                                interpolation=cv2.INTER_AREA)
        tensor = (
            torch.from_numpy(img_np).permute(2, 0, 1).float().div(255.0)
            .unsqueeze(0).to(self.device)
        )
        return (tensor - mean) / std

    def prepare_observations(self, obs_msg):
        import torch
        np = self._np
        obs = {
            "observation.images.left_camera": self._img_to_tensor(
                obs_msg.left_image, self.image_scaling,
                self.img_stats["left"]["mean"], self.img_stats["left"]["std"],
            ),
            "observation.images.center_camera": self._img_to_tensor(
                obs_msg.center_image, self.image_scaling,
                self.img_stats["center"]["mean"], self.img_stats["center"]["std"],
            ),
            "observation.images.right_camera": self._img_to_tensor(
                obs_msg.right_image, self.image_scaling,
                self.img_stats["right"]["mean"], self.img_stats["right"]["std"],
            ),
        }

        tcp_pose = obs_msg.controller_state.tcp_pose
        tcp_vel = obs_msg.controller_state.tcp_velocity
        state_np = np.array(
            [
                tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z,
                tcp_pose.orientation.x, tcp_pose.orientation.y,
                tcp_pose.orientation.z, tcp_pose.orientation.w,
                tcp_vel.linear.x, tcp_vel.linear.y, tcp_vel.linear.z,
                tcp_vel.angular.x, tcp_vel.angular.y, tcp_vel.angular.z,
                *obs_msg.controller_state.tcp_error,
                *obs_msg.joint_states.position[:7],
            ],
            dtype=np.float32,
        )
        raw_state_tensor = (
            torch.from_numpy(state_np).float().unsqueeze(0).to(self.device)
        )
        obs["observation.state"] = (raw_state_tensor - self.state_mean) / self.state_std
        return obs

    def insert_cable(self, task, get_observation, move_robot, send_feedback, **kwargs):
        import time
        import torch

        self.policy.reset()
        self.get_logger().info(f"RunACT.insert_cable() enter. Task: {task}")
        start_time = time.time()
        time_limit = getattr(task, 'time_limit', 150)

        while time.time() - start_time < time_limit:
            loop_start = time.time()
            observation_msg = get_observation()
            if observation_msg is None:
                self.get_logger().info("No observation received.")
                continue

            obs_tensors = self.prepare_observations(observation_msg)
            with torch.inference_mode():
                normalized_action = self.policy.select_action(obs_tensors)

            raw_action_tensor = (normalized_action * self.action_std) + self.action_mean
            action = raw_action_tensor[0].cpu().numpy()
            self.get_logger().info(f"Action: {action}")

            from geometry_msgs.msg import Twist, Vector3
            twist = Twist(
                linear=Vector3(x=float(action[0]), y=float(action[1]), z=float(action[2])),
                angular=Vector3(x=float(action[3]), y=float(action[4]), z=float(action[5])),
            )
            move_robot(motion_update=self.set_cartesian_twist_target(twist))
            send_feedback("in progress...")

            elapsed = time.time() - loop_start
            time.sleep(max(0, 0.25 - elapsed))

        self.get_logger().info("RunACT.insert_cable() exiting...")
        return True

    def set_cartesian_twist_target(self, twist, frame_id="base_link"):
        import numpy as np
        from geometry_msgs.msg import Vector3, Wrench
        from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode

        motion_update_msg = MotionUpdate()
        motion_update_msg.velocity = twist
        motion_update_msg.header.frame_id = frame_id
        motion_update_msg.header.stamp = self.get_clock().now().to_msg()
        motion_update_msg.target_stiffness = np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten()
        motion_update_msg.target_damping = np.diag(
            [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        ).flatten()
        motion_update_msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        motion_update_msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        motion_update_msg.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_VELOCITY
        )
        return motion_update_msg
