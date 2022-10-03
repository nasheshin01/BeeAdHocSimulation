from random import choice
from typing import Tuple
from repast4py import core
from enum import Enum
from distance_utils import find_agent_by_id, distance
from drone_agent import Drone


class ExplorePhase(Enum):
    SEARCHING_END_DRONE = 0
    GOING_BACK_TO_START = 1
    SCOUTING_ENDED = 2
    STUCK = 3

    def __int__(self):
        return self.value


class Scout(core.Agent):

    TYPE = 1

    def __init__(self, local_id: int, rank: int, start_drone_id: int, end_drone_id: int, energy_limit: int):
        super().__init__(id=local_id, type=Scout.TYPE, rank=rank)
        self.start_drone_id = start_drone_id
        self.end_drone_id = end_drone_id

        self.path = [start_drone_id]
        self.way_back_distances = []
        self.current_drone_index = 0

        self.explore_phase = ExplorePhase.SEARCHING_END_DRONE
        self.energy_limit = energy_limit

    def reset(self, energy_limit):
        self.path = [self.start_drone_id]
        self.way_back_distances = []
        self.current_drone_index = 0
        self.explore_phase = ExplorePhase.SEARCHING_END_DRONE
        self.energy_limit = energy_limit


    def save(self) -> Tuple:
        return (self.uid, self.start_drone_id, self.end_drone_id, self.path, self.way_back_distances,
                self.current_drone_index, self.explore_phase, self.energy_limit)

    def cluster_drones_by_location(self, drones, current_point, speed_distances):
        clustered_drones = {'safe': [], 'close': [], 'danger': []}

        for drone in drones:
            if drone.id in self.path:
                continue

            distance_to_drone = distance(drone.pt, current_point)
            if distance_to_drone <= speed_distances[0]:
                clustered_drones['safe'].append(drone)
                continue

            if distance_to_drone <= speed_distances[1]:
                clustered_drones['close'].append(drone)
                continue
            
            if distance_to_drone <= speed_distances[2]:
                clustered_drones['danger'].append(drone)
                continue
            
        return clustered_drones

    def move_forward(self, drones, speed_distances):
        current_drone = find_agent_by_id(drones, self.path[self.current_drone_index])
        drones_to_choose = []
        clustered_drones = self.cluster_drones_by_location(drones, current_drone.pt, speed_distances)
        if len(clustered_drones['danger']) > 0:
            drones_to_choose = clustered_drones['danger']
        if len(clustered_drones['close']) > 0:
            drones_to_choose = clustered_drones['close']
        if len(clustered_drones['safe']) > 0:
            drones_to_choose = clustered_drones['safe']
        if len(drones_to_choose) == 0:
            self.explore_phase = ExplorePhase.STUCK
            return False

        next_drone = choice(drones_to_choose)
        distance_to_drone = distance(current_drone.pt, next_drone.pt)
        if distance_to_drone < speed_distances[0]:
            self.energy_limit -= 1
        else:
            self.energy_limit -= (distance_to_drone - speed_distances[0] + 1)
        if self.energy_limit <= 0:
            self.explore_phase = ExplorePhase.STUCK
            return False
        
        self.path.append(next_drone.id)
        self.current_drone_index += 1

        if next_drone.id == self.end_drone_id:
            self.explore_phase = ExplorePhase.GOING_BACK_TO_START

        return True

    def move_back(self, drones, speed_distances):
        current_drone = find_agent_by_id(drones, self.path[self.current_drone_index])
        next_drone = find_agent_by_id(drones, self.path[self.current_drone_index - 1])
        distance_to_next_drone = distance(current_drone.pt, next_drone.pt)

        if distance_to_next_drone > speed_distances[2]:
            self.explore_phase = ExplorePhase.STUCK
            return False

        self.way_back_distances.insert(0, distance_to_next_drone)
        if next_drone.id == self.start_drone_id:
            self.explore_phase = ExplorePhase.SCOUTING_ENDED
            return False

        self.current_drone_index -= 1
        return True

    def explore(self, drones, speed_distances):
        if self.explore_phase == ExplorePhase.SEARCHING_END_DRONE:
            return self.move_forward(drones, speed_distances)
        elif self.explore_phase == ExplorePhase.GOING_BACK_TO_START:
            return self.move_back(drones, speed_distances)
        else:
            return False
        
    def get_location_point(self, context):
        current_drone = find_agent_by_id(context.agents(Drone.TYPE), self.path[self.current_drone_index])
        return current_drone.pt