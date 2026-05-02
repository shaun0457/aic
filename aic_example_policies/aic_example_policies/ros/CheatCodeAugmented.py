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

import numpy as np
from geometry_msgs.msg import Pose, Transform

from aic_example_policies.ros.CheatCode import CheatCode
from aic_model.policy import GetObservationCallback, MoveRobotCallback, SendFeedbackCallback
from aic_task_interfaces.msg import Task


class CheatCodeAugmented(CheatCode):
    """CheatCode with random XY approach perturbation for training data diversity.

    During approach (position_fraction: 0→1), a random XY offset is injected and
    decays linearly to zero so the robot always converges to the correct port position
    before insertion.
    """

    # Max perturbation in each axis (metres). ±30 mm covers realistic approach variance.
    NOISE_XY_MAX = 0.03

    def __init__(self, parent_node):
        self._approach_noise_x = 0.0
        self._approach_noise_y = 0.0
        super().__init__(parent_node)

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().warn(
            "CheatCodeAugmented is for training-data recording only — do NOT submit."
        )
        self._approach_noise_x = np.random.uniform(-self.NOISE_XY_MAX, self.NOISE_XY_MAX)
        self._approach_noise_y = np.random.uniform(-self.NOISE_XY_MAX, self.NOISE_XY_MAX)
        self.get_logger().info(
            f"CheatCodeAugmented noise:  x={self._approach_noise_x*1000:+.1f}mm  "
            f"y={self._approach_noise_y*1000:+.1f}mm"
        )
        return super().insert_cable(task, get_observation, move_robot, send_feedback)

    def calc_gripper_pose(
        self,
        port_transform: Transform,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        reset_xy_integrator: bool = False,
    ) -> Pose:
        pose = super().calc_gripper_pose(
            port_transform, slerp_fraction, position_fraction, z_offset, reset_xy_integrator
        )
        # Noise decays to zero as position_fraction → 1 (robot converges to port).
        # During insertion phase position_fraction is always 1.0, so no noise is applied.
        noise_scale = 1.0 - position_fraction
        pose.position.x += noise_scale * self._approach_noise_x
        pose.position.y += noise_scale * self._approach_noise_y
        return pose
