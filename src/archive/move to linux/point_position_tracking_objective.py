import numpy as np
from abc import ABC
from pydrake.all import (
    MultibodyPlant,
    Context,
)
from typing import List

from tracking_objective import TrackingObjective


class PointPositionTrackingObjective(TrackingObjective):
    def __init__(
        self,
        plant: MultibodyPlant,
        plant_context: Context,
        finite_states_to_track: List[int],
        k_p: np.ndarray,
        k_d: np.ndarray,
        points_to_track: Dict[int, PointOnFrame],
    ):
        """
        TODO
        """
        super().__init__(plant, plant_context, finite_states_to_track, k_p, k_d)
        self.points_to_track = points_to_track

    def CalcY(self) -> np.ndarray:
        pt_to_track = self.points_to_track[self.fsm]
        return self.plant.CalcPointsPositions(
            self.context, pt_to_track.frame, pt_to_track.pt, self.plant.world_frame()
        ).ravel()

    def CalcJ(self) -> np.ndarray:
        pt_to_track = self.points_to_track[self.fsm]
        return self.plant.CalcJacobianTranslationalVelocity(
            self.context,
            JacobianWrtVariable.kV,
            pt_to_track.frame,
            pt_to_track.pt,
            self.plant.world_frame(),
            self.plant.world_frame(),
        )

    def CalcYdot(self) -> np.ndarray:
        return (self.CalcJ() @ self.plant.GetVelocities(self.context)).ravel()

    def CalcJdotV(self) -> np.ndarray:
        pt_to_track = self.points_to_track[self.fsm]
        return self.plant.CalcBiasTranslationalAcceleration(
            self.context,
            JacobianWrtVariable.kV,
            pt_to_track.frame,
            pt_to_track.pt,
            self.plant.world_frame(),
            self.plant.world_frame(),
        ).ravel()
