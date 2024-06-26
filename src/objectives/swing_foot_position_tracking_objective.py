import numpy as np
from pydrake.all import (
    MultibodyPlant,
    Context,
    JacobianWrtVariable,
)
from typing import Dict

from objectives.tracking_objective import TrackingObjective
from utils import PointOnFrame


class SwingFootPositionTrackingObjective(TrackingObjective):
    def __init__(
        self,
        plant: MultibodyPlant,
        context: Context,
        k_p: np.ndarray,
        k_d: np.ndarray,
        points_to_track: Dict[str, PointOnFrame],
    ):
        """
        TODO
        """
        super().__init__(
            plant,
            context,
            k_p,
            k_d,
        )
        self.points_to_track = points_to_track

    def calc_y(self) -> np.ndarray:
        """
        TODO
        """
        point_to_track = self.points_to_track[self.support_mode]
        return self.plant.CalcPointsPositions(
            self.context,
            point_to_track.frame,
            point_to_track.point,
            self.plant.world_frame(),
        ).ravel()

    def calc_y_dot(self) -> np.ndarray:
        """
        TODO
        """
        return (self.CalcJ() @ self.plant.GetVelocities(self.context)).ravel()

    def calc_J(self) -> np.ndarray:
        """
        TODO
        """
        point_to_track = self.points_to_track[self.support_mode]
        return self.plant.CalcJacobianTranslationalVelocity(
            self.context,
            JacobianWrtVariable.kV,
            point_to_track.frame,
            point_to_track.pt,
            self.plant.world_frame(),
            self.plant.world_frame(),
        )

    def calc_J_dot_v(self) -> np.ndarray:
        """
        TODO
        """
        point_to_track = self.points_to_track[self.support_mode]
        return self.plant.CalcBiasTranslationalAcceleration(
            self.context,
            JacobianWrtVariable.kV,
            point_to_track.frame,
            point_to_track.pt,
            self.plant.world_frame(),
            self.plant.world_frame(),
        ).ravel()
