from typing import Tuple
from enum import Enum
from repast4py import core

from distance_utils import find_agent_by_id, distance
from drone_agent import Drone
from path_utils import PathsController


class SendingPhase(Enum):
    SENDING_TO_END = 0
    GOING_BACK_TO_START = 1
    SENDING_ENDED = 2
    STUCK = 3

    def __int__(self):
        return self.value


class Worker(core.Agent):

    TYPE = 2

    def __init__(self, local_id: int, rank: int, path_id, current_drone_id, data_id, package_id, tick):
        super().__init__(id=local_id, type=Worker.TYPE, rank=rank)

        self.path_id = path_id
        self.way_back_distances = []
        self.current_drone_id = current_drone_id
        self.data_id = data_id
        self.package_id = package_id
        self.next_tick_to_move = tick

        self.sending_phase = SendingPhase.SENDING_TO_END


    def save(self) -> Tuple:
        return (self.uid, self.path_id, self.current_drone_id,
                self.data_id, self.package_id, self.next_tick_to_move,
                self.way_back_distances, self.sending_phase)


    def move_forward(self, drones, current_tick, path, speed_distances):
        if not self.current_drone_id in path.next_drones.keys():
            self.sending_phase = SendingPhase.GOING_BACK_TO_START
            back_drone_id = path.back_drones[self.current_drone_id]
            path.connection_business[(back_drone_id, self.current_drone_id)] = False
            self.next_tick_to_move = current_tick + 1
            return 0

        next_drone_id = path.next_drones[self.current_drone_id]
        current_drone = find_agent_by_id(drones, self.current_drone_id)
        next_drone = find_agent_by_id(drones, next_drone_id)
        distance_to_next_drone = distance(current_drone.pt, next_drone.pt)

        if self.current_drone_id in path.back_drones.keys():
            back_drone_id = path.back_drones[self.current_drone_id]
            path.connection_business[(back_drone_id, self.current_drone_id)] = False

        if distance_to_next_drone > speed_distances[2]:
            self.sending_phase = SendingPhase.STUCK
            return -1

        if path.connection_business[(self.current_drone_id, next_drone_id)]:
            return -2

        path.connection_business[(self.current_drone_id, next_drone_id)] = True
        self.current_drone_id = next_drone_id
        if distance_to_next_drone > speed_distances[0]:
            self.next_tick_to_move = current_tick + (distance_to_next_drone - speed_distances[0] + 1)
        else:
            self.next_tick_to_move = current_tick + 1
        return 0

    def move_back(self, drones, current_tick, path, speed_distances):
        if not self.current_drone_id in path.back_drones.keys():
            self.sending_phase = SendingPhase.SENDING_ENDED
            return -3

        back_drone_id = path.back_drones[self.current_drone_id]
        current_drone = find_agent_by_id(drones, self.current_drone_id)
        next_drone = find_agent_by_id(drones, back_drone_id)
        distance_to_next_drone = distance(current_drone.pt, next_drone.pt)

        if distance_to_next_drone > speed_distances[2]:
            self.sending_phase = SendingPhase.STUCK
            return -4

        self.way_back_distances.insert(0, distance_to_next_drone)
        self.current_drone_id = back_drone_id
        self.next_tick_to_move = current_tick + 1
        return 0


    def send(self, drones, current_tick: int, paths_controller: PathsController, speed_distances) -> int:
        path = paths_controller.paths[self.path_id]
        if self.sending_phase == SendingPhase.SENDING_TO_END:
            return self.move_forward(drones, current_tick, path, speed_distances)
        elif self.sending_phase == SendingPhase.GOING_BACK_TO_START:
            return self.move_back(drones, current_tick, path, speed_distances)
        else:
            return -5

    def get_location_point(self, context):
        current_drone = find_agent_by_id(context.agents(Drone.TYPE), self.current_drone_id)
        return current_drone.pt