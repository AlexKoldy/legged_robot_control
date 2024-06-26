import numpy as np
from abc import ABC
from pydrake.all import (
    MultibodyPlant,
    Context,
)
from typing import List

from tracking_objective import TrackingObjective


class JointAngleTrackingObjective(TrackingObjective):
    def __init__(
        self,
        plant: MultibodyPlant,
        plant_context: Context,
        finite_states_to_track: List[int],
        k_p: np.ndarray,
        k_d: np.ndarray,
        joint_name: str,
    ):
        """
        TODO
        """
        super().__init__(plant, plant_context, finite_states_to_track, k_p, k_d)

        self.joint_pos_idx = self.plant.GetJointByName(joint_name).position_start()
        self.joint_vel_idx = self.plant.GetJointByName(joint_name).velocity_start()

    def calc_y(self) -> np.ndarray:
        """
        TODO
        """
        return self.plant.GetPositions(self.context)[
            self.joint_pos_idx : self.joint_pos_idx + 1
        ].ravel()

    def calc_y_dot(self) -> np.ndarray:
        """
        TODO
        """
        return self.plant.GetVelocities(self.context)[
            self.joint_vel_idx : self.joint_vel_idx + 1
        ].ravel()

    def calc_J(self) -> np.ndarray:
        """
        TODO
        """
        J = np.zeros(self.plant.num_positions())
        J[self.joint_pos_idx] = 1
        return J

    def calc_J_dot_v(self) -> np.ndarray:
        """
        TODO: figure out if there's any issues with J_dot_v since
        num_positions() and num_velocities() has different values
        (due to quaternion)
        """
        J_dot_v = np.array([0])

        return J_dot_v
