import numpy as np
from abc import ABC
from pydrake.all import (
    MultibodyPlant,
    Context,
)
from typing import List


class TrackingObjective(ABC):
    def __init__(
        self,
        plant: MultibodyPlant,
        plant_context: Context,
        finite_states_to_track: List[int],
        k_p: np.ndarray,
        k_d: np.ndarray,
    ):
        """
        TODO
        """
        self.k_p = k_p
        self.k_d = k_d
        self.fsm_states_to_track = finite_states_to_track
        self.plant = plant
        self.context = plant_context

        self.J = None
        self.JdotV = None
        self.yddot_cmd = None
        self.fsm = None
