import numpy as np
from pydrake.all import Frame
from dataclasses import dataclass


@dataclass
class PointOnFrame:
    frame: Frame
    point: np.ndarray
