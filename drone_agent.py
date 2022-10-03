from dataclasses import dataclass
from typing import Dict, Tuple
from mpi4py import MPI
import numpy as np

from repast4py import core, random
from repast4py.space import DiscretePoint as dpt

class Drone(core.Agent):

    TYPE = 0
    OFFSETS = np.array([-1, 0, 1])

    def __init__(self, local_id: int, rank: int, pt: dpt):
        super().__init__(id=local_id, type=Drone.TYPE, rank=rank)
        self.pt = pt


    def save(self) -> Tuple:
        return (self.uid, self.pt.x, self.pt.y)

    def fly(self, grid):
        xy_dirs = random.default_rng.choice(Drone.OFFSETS, size=2)
        self.pt = grid.move(self, dpt(self.pt.x + xy_dirs[0], self.pt.y + xy_dirs[1], 0))