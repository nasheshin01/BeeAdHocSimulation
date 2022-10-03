from select import select
import unittest
from data_utils import DataController
import distance_utils
import agent_utils

from repast4py.space import DiscretePoint as dpt
from drone_agent import Drone
from path_utils import PathsController
from scout_agent import Scout, ExplorePhase
from worker_agent import Worker, SendingPhase
from repast4py import space
from mpi4py import MPI

SPEED_DISTANCES = [5, 10, 15]

class ScoutTests(unittest.TestCase):

    def test_move_forward_CanMoveToSafeDrone_MovedToSafeDrone(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 10))]
        scout = Scout(0, 0, 0, 2, 100)

        scout.move_forward(drones, SPEED_DISTANCES)

        self.assertEqual(1, scout.path[scout.current_drone_index])

    def test_move_forward_CanMoveToCloseAndDangerDrones_MovedToCloseDrone(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 6)), Drone(2, 0, dpt(0, 11))]
        scout = Scout(0, 0, 0, 2, 100)

        scout.move_forward(drones, SPEED_DISTANCES)

        self.assertEqual(1, scout.path[scout.current_drone_index])
        
    def test_move_forward_CanMoveOnlyToDangerDrones_MovedToDangerDrone(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 11)), Drone(2, 0, dpt(0, 20))]
        scout = Scout(0, 0, 0, 2, 100)

        scout.move_forward(drones, SPEED_DISTANCES)

        self.assertEqual(1, scout.path[scout.current_drone_index])

    def test_move_forward_NoSuitableDrones_ExplorePhaseIsStuck(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 20)), Drone(2, 0, dpt(0, 20))]
        scout = Scout(0, 0, 0, 2, 100)

        scout.move_forward(drones, SPEED_DISTANCES)

        self.assertEqual(scout.explore_phase, ExplorePhase.STUCK)

    def test_move_forward_ZeroEnergy_ExplorePhaseIsStuck(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 1))]
        scout = Scout(0, 0, 0, 2, 0)

        scout.move_forward(drones, SPEED_DISTANCES)

        self.assertEqual(scout.explore_phase, ExplorePhase.STUCK)

    def test_move_forward_MovedToEndDrone_ExplorePhaseIsGoingBackToStart(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1))]
        scout = Scout(0, 0, 0, 1, 100)

        scout.move_forward(drones, SPEED_DISTANCES)

        self.assertEqual(scout.explore_phase, ExplorePhase.GOING_BACK_TO_START)

    def test_move_back_NextDroneIsCanBeReached_MovedTonextDrone(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 2))]
        scout = Scout(0, 0, 0, 2, 100)
        scout.path = [0, 1, 2]
        scout.current_drone_index = 2
        scout.explore_phase = ExplorePhase.GOING_BACK_TO_START

        scout.move_back(drones, SPEED_DISTANCES)

        self.assertEqual(1, scout.path[scout.current_drone_index])

    def test_move_back_NextDroneIsTooDistant_ExplorePhaseIsStuck(self):

        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 20))]
        scout = Scout(0, 0, 0, 2, 100)
        scout.path = [0, 1, 2]
        scout.current_drone_index = 2
        scout.explore_phase = ExplorePhase.GOING_BACK_TO_START

        scout.move_back(drones, SPEED_DISTANCES)

        self.assertEqual(scout.explore_phase, ExplorePhase.STUCK)

    def test_move_back_NextDroneIsStart_ExplorePhaseIsScoutingEnded(self):

        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 20))]
        scout = Scout(0, 0, 0, 2, 100)
        scout.path = [0, 1, 2]
        scout.current_drone_index = 1
        scout.explore_phase = ExplorePhase.GOING_BACK_TO_START

        scout.move_back(drones, SPEED_DISTANCES)

        self.assertEqual(scout.explore_phase, ExplorePhase.SCOUTING_ENDED)
    
class WorkerTests(unittest.TestCase):

    def test_move_forward_NextDroneIsReachable_CurrentDroneIdChangedToNextDroneId(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 2))]
        paths_controller = PathsController(SPEED_DISTANCES)
        paths_controller.try_add_path([0, 1, 2], [1, 1])
        worker = Worker(0, 0, 0, 1, 0, 0, 0)

        worker.move_forward(drones, 0, paths_controller.paths[0], SPEED_DISTANCES)

        self.assertEqual(worker.current_drone_id, 2)

    def test_move_forward_CurrentDroneIsEndDrone_SendingPhaseIsGoingBackToStart(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 2))]
        paths_controller = PathsController(SPEED_DISTANCES)
        paths_controller.try_add_path([0, 1, 2], [1, 1])
        worker = Worker(0, 0, 0, 2, 0, 0, 0)

        worker.move_forward(drones, 0, paths_controller.paths[0], SPEED_DISTANCES)

        self.assertEqual(worker.sending_phase, SendingPhase.GOING_BACK_TO_START)

    def test_move_forward_NextDroneIsTooDistant_SendingPhaseIsStuck(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 20))]
        paths_controller = PathsController(SPEED_DISTANCES)
        paths_controller.try_add_path([0, 1, 2], [1, 1])
        worker = Worker(0, 0, 0, 1, 0, 0, 0)

        worker.move_forward(drones, 0, paths_controller.paths[0], SPEED_DISTANCES)

        self.assertEqual(worker.sending_phase, SendingPhase.STUCK)

    def test_move_forward_EdgeToNextDroneIsBusy_ReturnNegative2(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 2))]
        paths_controller = PathsController(SPEED_DISTANCES)
        paths_controller.try_add_path([0, 1, 2], [1, 1])
        paths_controller.paths[0].connection_business[1, 2] = True
        worker = Worker(0, 0, 0, 1, 0, 0, 0)

        code_result = worker.move_forward(drones, 0, paths_controller.paths[0], SPEED_DISTANCES)

        self.assertEqual(-2, code_result)

    def test_move_back_NextDroneIsReachable_CurrentDroneIdChangedToNextDroneId(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 2))]
        paths_controller = PathsController(SPEED_DISTANCES)
        paths_controller.try_add_path([0, 1, 2], [1, 1])
        worker = Worker(0, 0, 0, 2, 0, 0, 0)
        worker.sending_phase = SendingPhase.GOING_BACK_TO_START

        worker.move_back(drones, 0, paths_controller.paths[0], SPEED_DISTANCES)

        self.assertEqual(worker.current_drone_id, 1)

    def test_move_back_NextDroneIsUnreachable_SendingPhaseIsStuck(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 20))]
        paths_controller = PathsController(SPEED_DISTANCES)
        paths_controller.try_add_path([0, 1, 2], [1, 1])
        worker = Worker(0, 0, 0, 2, 0, 0, 0)
        worker.sending_phase = SendingPhase.GOING_BACK_TO_START

        worker.move_back(drones, 0, paths_controller.paths[0], SPEED_DISTANCES)

        self.assertEqual(worker.sending_phase, SendingPhase.STUCK)

    def test_move_back_CurrentDroneIsStartDrone_SendingPhaseIsStuck(self):
        drones = [Drone(0, 0, dpt(0, 0)), Drone(1, 0, dpt(0, 1)), Drone(2, 0, dpt(0, 2))]
        paths_controller = PathsController(SPEED_DISTANCES)
        paths_controller.try_add_path([0, 1, 2], [1, 1])
        worker = Worker(0, 0, 0, 0, 0, 0, 0)
        worker.sending_phase = SendingPhase.GOING_BACK_TO_START

        worker.move_back(drones, 0, paths_controller.paths[0], SPEED_DISTANCES)

        self.assertEqual(worker.sending_phase, SendingPhase.SENDING_ENDED)

if __name__ == '__main__':
    unittest.main()