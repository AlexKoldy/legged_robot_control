import numpy as np
from abc import ABC
from pydrake.all import (
    MultibodyPlant,
    Context,
    JacobianWrtVariable,
)

from tracking_objective import TrackingObjective


class CenterOfMassPositionTrackingObjective(TrackingObjective):
    def __init__(
        self,
        plant: MultibodyPlant,
        context: Context,
        # finite_states_to_track: List[int],
        k_p: np.ndarray,
        k_d: np.ndarray,
    ):
        """
        TODO
        """
        super().__init__(
            plant,
            context,
            # finite_states_to_track,
            k_p,
            k_d,
        )

    def calc_y(self) -> np.ndarray:
        """
        TODO
        """
        return self.plant.CalcCenterOfMassPositionInWorld(self.context).ravel()

    def calc_y_dot(self) -> np.ndarray:
        """
        TODO
        """
        return (self.CalcJ() @ self.plant.GetVelocities(self.context)).ravel()

    def calc_J(self) -> np.ndarray:
        """
        TODO
        """
        return self.plant.CalcJacobianCenterOfMassTranslationalVelocity(
            self.context,
            JacobianWrtVariable.kQDot,
            self.plant.world_frame(),
            self.plant.world_frame(),
        )

    def calc_J_dot_v(self) -> np.ndarray:
        """
        TODO
        """
        return self.plant.CalcBiasCenterOfMassTranslationalAcceleration(
            self.context,
            JacobianWrtVariable.kQDot,
            self.plant.world_frame(),
            self.plant.world_frame(),
        ).ravel()
