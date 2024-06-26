import numpy as np
from abc import ABC
from pydrake.all import (
    MultibodyPlant,
    Context,
    Trajectory,
)


class TrackingObjective(ABC):
    def __init__(
        self,
        plant: MultibodyPlant,
        context: Context,
        k_p: np.ndarray,
        k_d: np.ndarray,
    ):
        """
        TODO
        """
        self.k_p = k_p
        self.k_d = k_d
        self.plant = plant
        self.context = context

        self.J = None
        self.J_dot_v = None
        self.y_ddot_cmd = None
        self.support_mode = None

    def calculate(
        self,
        # t: float,
        # y_traj: Trajectory,
        # support_mode: str,
    ):
        # self.support_mode = support_mode

        y = self.calc_y()
        y_dot = self.calc_y_dot()

        self.J = self.calc_J()
        self.J_dot_v = self.calc_J_dot_v()

        # TODO: fix this
        # y_des = y_traj.value(t).ravel()
        # y_dot_des = y_traj.derivative(1).value(t).ravel()
        # y_ddot_des = y_traj.derivative(2).value(t).ravel()
        y_des = np.array([1.33884302e-03, 1.04911005e-03, 1.22932388])
        y_dot_des = np.zeros(3)
        y_ddot_des = np.zeros(3)

        self.y_ddot_cmd = (
            y_ddot_des - self.k_p @ (y - y_des) - self.k_d @ (y_dot - y_dot_des)
        )
