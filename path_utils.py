class Path:

    def __init__(self, path_id, drones_path, distances, speed_distances) -> None:
        self.path_id = path_id
        self.drones_path = drones_path

        self.next_drones = {}
        for i in range(len(self.drones_path) - 1):
            self.next_drones[self.drones_path[i]] = self.drones_path[i + 1]

        self.back_drones = {}
        for i in range(len(self.drones_path) - 1):
            self.back_drones[self.drones_path[i + 1]] = self.drones_path[i]

        self.distances = distances

        self.connection_business = {}
        for i in range(len(self.distances)):
            self.connection_business[(self.drones_path[i], self.drones_path[i + 1])] = False

        self.speed = 0
        for distance in self.distances:
            if distance < speed_distances[0]:
                self.speed += 1
            else:
                self.speed += 1 / (distance - speed_distances[0] + 1)

        self.speed /= len(distances)
        self.is_can_be_used = True
        self.speed_distances = speed_distances

    def is_intersect(self, path):
        for connection in self.connection_business.keys():
            if connection in path.connection_business:
                return True


    def is_path_close_to_disconnect(self):
        for distance in self.distances:
            if distance >= self.speed_distances[1]:
                return True

        return False

class PathsController:

    def __init__(self, speed_distances) -> None:
        self.new_path_id = 0
        self.paths = {}
        self.speed_distances = speed_distances

    def try_add_path(self, node_ids, distances):
        new_path = Path(self.new_path_id, node_ids, distances, self.speed_distances)
        if new_path.is_path_close_to_disconnect():
            return False

        can_be_used_paths = self.get_paths_can_be_used()
        intersect_paths = []
        for path in can_be_used_paths:
            if path.is_intersect(new_path):
                intersect_paths.append(path)
        
        if len(intersect_paths) == 0:
            self.paths[self.new_path_id] = new_path
            self.new_path_id += 1
            return True
        elif len(intersect_paths) == 1:
            if new_path.speed < intersect_paths[0].speed:
                return False
            
            intersect_paths[0].is_can_be_used = False
            self.paths[self.new_path_id] = new_path
            self.new_path_id += 1
            return True
        else:
            return False

    def update_path_distances(self, path_id, distances):
        updated_path = Path(path_id, self.paths[path_id].drones_path, distances, self.speed_distances)
        if updated_path.is_path_close_to_disconnect():
            updated_path.is_can_be_used = False

        self.paths[path_id] = updated_path

    def get_paths_can_be_used(self):
        can_be_used_paths = []
        for path in self.paths.values():
            if path.is_can_be_used:
                can_be_used_paths.append(path)

        return can_be_used_paths



        
