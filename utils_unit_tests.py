from importlib.resources import path
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

class DistanceUtilsTests(unittest.TestCase):

    def test_distance_TwoPositivePoints_ReturnDistance(self):
        x1, x2, y1, y2 = 0, 3, 0, 4
        p1, p2 = dpt(x1, y1), dpt(x2, y2)
        true_result = 5
        result = distance_utils.distance(p1, p2)

        self.assertAlmostEqual(result, true_result)

    def test_distance_TwoNegativePoints_ReturnDistance(self):
        x1, x2, y1, y2 = 0, -3, 0, -4
        p1, p2 = dpt(x1, y1), dpt(x2, y2)
        true_result = 5
        result = distance_utils.distance(p1, p2)

        self.assertAlmostEqual(result, true_result)

    def test_find_agent_by_id_AgentsHasId_ReturnAgent(self):
        drones = []
        for i in range(10):
            drones.append(Drone(i, 0, dpt(0, 0)))

        self.assertEqual(drones[7], distance_utils.find_agent_by_id(drones, 7))

    def test_find_agent_by_id_AgentsHasNoId_ReturnNone(self):
        drones = []
        for i in range(10):
            drones.append(Drone(i, 0, dpt(0, 0)))

        self.assertEqual(None, distance_utils.find_agent_by_id(drones, 11))

class AgentUtilsTests(unittest.TestCase):

    def test_restore_drone_DroneAgent_ReturnEqualDroneAgent(self):
        drone = Drone(0, 0, dpt(0, 0))

        newDrone = agent_utils.restore_agent(drone.save())

        self.assertEqual(drone.uid, newDrone.uid)
        self.assertEqual(drone.pt, newDrone.pt)

    def test_restore_drone_ScoutAgent_ReturnEqualScoutAgent(self):
        scout = Scout(0, 0, 0, 1, 5)
        scout.path = [0, 1, 2]
        scout.way_back_distances = [0, 1, 2]
        scout.current_drone_index = 2
        scout.explore_phase = ExplorePhase.SEARCHING_END_DRONE

        newScout = agent_utils.restore_agent(scout.save())

        self.assertEqual(scout.uid, newScout.uid)
        self.assertEqual(scout.start_drone_id, newScout.start_drone_id)
        self.assertEqual(scout.end_drone_id, newScout.end_drone_id)
        self.assertEqual(scout.current_drone_index, newScout.current_drone_index)
        self.assertEqual(scout.explore_phase, newScout.explore_phase)
        self.assertEqual(scout.energy_limit, newScout.energy_limit)
        self.assertListEqual(scout.path, newScout.path)
        self.assertListEqual(scout.way_back_distances, newScout.way_back_distances)

    def test_restore_drone_WorkerAgent_ReturnEqualWorkerAgent(self):
        worker = Worker(0, 0, 0, 0, 0, 0, 5)
        worker.way_back_distances = [0, 1, 2]
        worker.sending_phase = SendingPhase.SENDING_TO_END

        newWorker = agent_utils.restore_agent(worker.save())

        self.assertEqual(worker.uid, newWorker.uid)
        self.assertEqual(worker.path_id, newWorker.path_id)
        self.assertEqual(worker.current_drone_id, newWorker.current_drone_id)
        self.assertEqual(worker.data_id, newWorker.data_id)
        self.assertEqual(worker.package_id, newWorker.package_id)
        self.assertEqual(worker.next_tick_to_move, newWorker.next_tick_to_move)
        self.assertEqual(worker.sending_phase, newWorker.sending_phase)
        self.assertListEqual(worker.way_back_distances, newWorker.way_back_distances)

class DataUtilsTests(unittest.TestCase):

    def test_add_data_NewData_AddedNewData(self):
        data_controller = DataController()
        
        data_len_before = len(data_controller.datas)
        data_controller.add_data(8)

        self.assertEqual(data_len_before, len(data_controller.datas) - 1)
        self.assertEqual(8, len(data_controller.datas[0].packages))

    def test_get_not_sent_packages_AllPackagesNotSent_ReturnAllPackages(self):
        data_controller = DataController()
        data_controller.add_data(8)
        data_controller.add_data(8)

        not_sent_packages = data_controller.get_not_sent_packages()

        self.assertEqual(16, len(not_sent_packages))

    def test_get_not_sent_packages_AllPackagesSent_ReturnZeroPackages(self):
        data_controller = DataController()
        data_controller.add_data(8)
        data_controller.add_data(8)

        not_sent_packages = data_controller.get_not_sent_packages()
        for package in not_sent_packages:
            package[1].package_state = 1

        not_sent_packages = data_controller.get_not_sent_packages()

        self.assertEqual(0, len(not_sent_packages))

    def test_get_not_sent_packages_AllPackagesNotDeliveredOrLost_ReturnFalse(self):
        data_controller = DataController()
        data_controller.add_data(8)
        data_controller.add_data(8)

        is_all_packages_delivered_or_lost = data_controller.is_all_packages_delivered_or_lost()

        self.assertEqual(False, is_all_packages_delivered_or_lost)

    def test_get_not_sent_packages_AllPackagesDeliveredOrLost_ReturnTrue(self):
        data_controller = DataController()
        data_controller.add_data(8)
        data_controller.add_data(8)

        not_sent_packages = data_controller.get_not_sent_packages()
        for package in not_sent_packages:
            package[1].package_state = 2

        is_all_packages_delivered_or_lost = data_controller.is_all_packages_delivered_or_lost()

        self.assertEqual(True, is_all_packages_delivered_or_lost)

class PathUtilsTests(unittest.TestCase):

    def test_try_add_path_PathCloseToDisconnect_ReturnFalse(self):
        paths_controller = PathsController([5, 10, 15])

        result = paths_controller.try_add_path([0, 1, 2], [5, 15])

        self.assertEqual(False, result)

    def test_try_add_path_PathNotIntersect_ReturnTrue(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [5, 5])
        result = paths_controller.try_add_path([0, 4, 2], [5, 5])

        self.assertEqual(True, result)

    def test_try_add_path_PathIntersectButSpeedSmaller_ReturnFalse(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [6, 6])
        result = paths_controller.try_add_path([0, 1, 2], [7, 7])

        self.assertEqual(False, result)

    def test_try_add_path_PathIntersectButSpeedBigger_ReturnTrue(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [6, 6])
        result = paths_controller.try_add_path([0, 1, 2], [5, 5])

        self.assertEqual(True, result)

    def test_try_add_path_PathIntersectWithSomePaths_ReturnFalse(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [6, 6])
        paths_controller.try_add_path([0, 3, 2], [6, 6])
        result = paths_controller.try_add_path([0, 1, 3, 2], [5, 5, 5])

        self.assertEqual(False, result)

    def test_update_path_distances_PathCannotBeUsedMore_ChangedFieldCanBeUsedToFalse(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [6, 6])
        paths_controller.update_path_distances(0, [11, 11])

        self.assertEqual(False, paths_controller.paths[0].is_can_be_used)

    def test_update_path_distances_PathCanBeUsedMore_ChangedFieldCanBeUsedToTrue(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [6, 6])
        paths_controller.update_path_distances(0, [6, 6])

        self.assertEqual(True, paths_controller.paths[0].is_can_be_used)

    def test_update_path_distances_AllPathsCanBeUsed_ReturnAllPaths(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [6, 6])
        paths_controller.try_add_path([0, 3, 2], [6, 6])

        paths = paths_controller.get_paths_can_be_used()

        self.assertEqual(len(paths_controller.paths), len(paths))

    def test_update_path_distances_AllPathsCannnotBeUsed_ReturnZeroPaths(self):
        paths_controller = PathsController([5, 10, 15])

        paths_controller.try_add_path([0, 1, 2], [6, 6])
        paths_controller.update_path_distances(0, [11, 11])

        paths = paths_controller.get_paths_can_be_used()

        self.assertEqual(0, len(paths))

    
if __name__ == '__main__':
    unittest.main()