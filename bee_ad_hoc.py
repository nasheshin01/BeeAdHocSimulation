from typing import Dict, Type
from mpi4py import MPI

from repast4py import core, random, space, schedule, logging, parameters
from repast4py import context as ctx
import repast4py
from repast4py.space import DiscretePoint as dpt
from data_utils import DataController

from scout_agent import ExplorePhase
from path_utils import PathsController
from agent_utils import restore_agent, Scout, Drone, Worker
from params_utils import check_params


class Model:
    """
    The Model class encapsulates the simulation, and is
    responsible for initialization (scheduling events, creating agents,
    and the grid the agents inhabit), and the overall iterating
    behavior of the model.

    Args:
        comm: the mpi communicator over which the model is distributed.
        params: the simulation input parameters
    """

    def __init__(self, comm: MPI.Intracomm, params: Dict):
        self.runner = schedule.init_schedule_runner(comm)

        self.runner.schedule_repeating_event(1, params['data.generate_period'], self.generate_data)
        self.runner.schedule_repeating_event(1, params['drone.move_period'], self.move_drones)
        self.runner.schedule_repeating_event(1, 1, self.move_scouts)
        self.runner.schedule_repeating_event(1, 1, self.log_agents)
        self.runner.schedule_end_event(self.at_end)

        self.context = ctx.SharedContext(comm)

        box = space.BoundingBox(0, params['world.width'], 0, params['world.height'], 0, 0)
        self.grid = space.SharedGrid(name='grid', bounds=box, borders=space.BorderType.Sticky,
                                     occupancy=space.OccupancyType.Multiple, buffer_size=2, comm=comm)
        self.context.add_projection(self.grid)

        self.rank = comm.Get_rank()
        rng = repast4py.random.default_rng
        drones = []
        for i in range(params['drone.count']):
            pt = self.grid.get_random_local_pt(rng)
            drone = Drone(i, self.rank, pt)
            drones.append(drone)
            self.context.add(drone)
            self.grid.move(drone, pt)

        self.start_drone_id, self.end_drone_id = drones[0].id, drones[-1].id

        self.drone_logger = logging.TabularLogger(comm, params['drone_log_file'], ['tick', 'drone_id', 'pos_x', 'pos_y', 'is_start', 'is_end'])
        self.scout_logger = logging.TabularLogger(comm, params['scout_log_file'], ['tick', 'scout_id', 'drone_id', 'pos_x', 'pos_y', 'explore_phase'])
        self.worker_logger = logging.TabularLogger(comm, params['worker_log_file'], ['tick', 'worker_id', 'drone_id', 'pos_x', 'pos_y', 'sending_phase'])
        self.data_state_logger = logging.TabularLogger(comm, params['data_state_log_file'], ['tick', 'data_id', 'package_id', 'state', 'worker_id'])
        self.paths_count_logger = logging.TabularLogger(comm, params['paths_count_log_file'], ['tick', 'paths_count'])
        self.scouting_logger = logging.TabularLogger(comm, params['scouting_log_file'], ['tick', 'paths_count'])

        self.scout_energy_limit = params['scout.energy_limit']
        for i in range(params['scout.count']):
            scout = Scout(i, self.rank, self.start_drone_id, self.end_drone_id, self.scout_energy_limit)
            self.context.add(scout)

        self.paths = []
        self.new_path_id = 0
        self.params = params
      

        self.log_agents()

        self.speed_distances = [params['drone.stable_sending_speed_max_distance'],
                                params['drone.close_to_disconnect_radius_distance'],
                                params['drone.drone_radius_distance']]
        self.paths_controller = PathsController(self.speed_distances)
        self.data_controller = DataController()
        self.new_worker_id = 0
        self.package_count_delivered = 0
        self.package_count_lost = 0

    def drone_agents_to_list(self):
        return [drone for drone in self.context.agents(Drone.TYPE)]

    def generate_data(self):
        if len(self.data_controller.datas.keys()) >= self.params['data.count']:
            return

        generated_data = self.data_controller.add_data(self.params['data.size'])

        for package in generated_data.packages.values():
            self.data_state_logger.log_row(self.runner.schedule.tick, generated_data.data_id, package.package_id, package.package_state, -1)

        self.data_state_logger.write()

        self.try_create_workers()

    def try_create_workers(self):
        packages = self.data_controller.get_not_sent_packages()
        if len(packages) <= 0:
            return

        can_be_used_paths = self.paths_controller.get_paths_can_be_used()

        if len(can_be_used_paths) <= 0:
            return

        while len(packages) > 0:
            for path in can_be_used_paths:
                data_id, package = packages.pop(0)
                worker = Worker(self.new_worker_id, self.rank,
                                path.path_id, self.start_drone_id,
                                data_id, package.package_id,
                                self.runner.schedule.tick)
                package.package_state = 1
                self.new_worker_id += 1
                self.context.add(worker)

                self.data_state_logger.log_row(self.runner.schedule.tick, data_id, package.package_id, package.package_state, worker.id)
                self.data_state_logger.write()

                self.context.synchronize(restore_agent)
                self.move_worker(worker)


    def move_drones(self):
        for drone in self.context.agents(Drone.TYPE):
            drone.fly(self.grid)

        self.context.synchronize(restore_agent)

    def move_scouts(self):
        for scout in self.context.agents(Scout.TYPE):
            scout.explore(self.drone_agents_to_list(), self.speed_distances)

        self.context.synchronize(restore_agent)

        for scout in self.context.agents(Scout.TYPE):
            if scout.explore_phase == ExplorePhase.SEARCHING_END_DRONE or scout.explore_phase == ExplorePhase.GOING_BACK_TO_START:
                return

        path_found = 0
        for scout in self.context.agents(Scout.TYPE):
            if scout.explore_phase == ExplorePhase.SCOUTING_ENDED:
                self.paths_controller.try_add_path(scout.path, scout.way_back_distances)
                path_found += 1

            scout.reset(self.scout_energy_limit)

        self.scouting_logger.log_row(self.runner.schedule.tick, path_found)
        self.paths_count_logger.log_row(self.runner.schedule.tick, len(self.paths_controller.get_paths_can_be_used()))

        self.scouting_logger.write()
        self.paths_count_logger.write()
        
        self.context.synchronize(restore_agent)

        self.try_create_workers()


    def move_worker(self, worker):
        code_result = worker.send(self.drone_agents_to_list(), self.runner.schedule.tick, self.paths_controller, self.speed_distances)

        if code_result == 0:
            self.runner.schedule_event(worker.next_tick_to_move, lambda: self.move_worker(worker))
        elif code_result == -1:
            self.paths_controller.paths[worker.path_id].is_can_be_used = False
            self.data_controller.datas[worker.data_id].packages[worker.package_id].package_state = -1

            self.data_state_logger.log_row(self.runner.schedule.tick, worker.data_id, worker.package_id, -1, worker.id)
            self.paths_count_logger.log_row(self.runner.schedule.tick, len(self.paths_controller.get_paths_can_be_used()))
            self.data_state_logger.write()
            self.paths_count_logger.write()

            self.context.remove(worker)
        elif code_result == -2:
            self.runner.schedule_event(self.runner.schedule.tick + 1, lambda: self.move_worker(worker))
        elif code_result == -3:
            self.paths_controller.update_path_distances(worker.path_id, worker.way_back_distances)
            self.data_controller.datas[worker.data_id].packages[worker.package_id].package_state = 2

            self.data_state_logger.log_row(self.runner.schedule.tick, worker.data_id, worker.package_id, 2, worker.id)
            self.data_state_logger.write()

            self.context.remove(worker)
        elif code_result == -4:
            self.paths_controller.paths[worker.path_id].is_can_be_used = False
            self.data_controller.datas[worker.data_id].packages[worker.package_id].package_state = 2

            self.data_state_logger.log_row(self.runner.schedule.tick, worker.data_id, worker.package_id, 2, worker.id)
            self.paths_count_logger.log_row(self.runner.schedule.tick, len(self.paths_controller.get_paths_can_be_used()))
            self.data_state_logger.write()
            self.paths_count_logger.write()

            self.context.remove(worker)
        else:
            print('Unknown code_result', code_result)

        self.context.synchronize(restore_agent)

        if len(self.data_controller.datas.keys()) >= self.params['data.count'] and self.data_controller.is_all_packages_delivered_or_lost():
            self.runner.stop()


    def log_agents(self):
        tick = self.runner.schedule.tick
        for drone in self.context.agents(Drone.TYPE):
            is_start = 0
            is_end = 0
            if drone.id == self.start_drone_id:
                is_start = 1
            if drone.id == self.end_drone_id:
                is_end = 1
            self.drone_logger.log_row(tick, drone.id, drone.pt.x, drone.pt.y, is_start, is_end)

        for scout in self.context.agents(Scout.TYPE):
            pt = scout.get_location_point(self.context)
            self.scout_logger.log_row(self.runner.schedule.tick, scout.id, scout.path[scout.current_drone_index], pt.x, pt.y, int(scout.explore_phase))

        try:
            for worker in self.context.agents(Worker.TYPE):
                pt = worker.get_location_point(self.context)
                self.worker_logger.log_row(self.runner.schedule.tick, worker.id, worker.current_drone_id, pt.x, pt.y, int(worker.sending_phase))
        except KeyError:
            pass
        
        self.drone_logger.write()
        self.scout_logger.write()
        self.worker_logger.write()

    def at_end(self):
        self.drone_logger.close()
        self.scout_logger.close()

    def start(self):
        self.runner.execute()


def run(params: Dict):
    model = Model(MPI.COMM_WORLD, params)
    model.start()


if __name__ == "__main__":
    parser = parameters.create_args_parser()
    args = parser.parse_args()
    params = parameters.init_params(args.parameters_file, args.parameters)
    out = check_params(params)
    if out[0]:
        run(params)
    else:
        print(out[1])