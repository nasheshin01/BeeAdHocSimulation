from scout_agent import Scout
from drone_agent import Drone
from worker_agent import Worker
from repast4py.space import DiscretePoint as dpt


agent_cache = {}
def restore_agent(agent_data):
    uid = agent_data[0]
    if uid[1] == Drone.TYPE:
        if uid in agent_cache:
            return agent_cache[uid]

        pt = dpt(agent_data[1], agent_data[2], 0)
        drone = Drone(uid[0], uid[2], pt)
        agent_cache[uid] = drone
        
        return drone
    elif uid[1] == Scout.TYPE:
        if uid in agent_cache:
            return agent_cache[uid]
    
        scout = Scout(uid[0], uid[2], agent_data[1], agent_data[2], agent_data[7])
        scout.path = agent_data[3]
        scout.way_back_distances = agent_data[4]
        scout.current_drone_index = agent_data[5]
        scout.explore_phase = agent_data[6]

        agent_cache[uid] = scout

        return scout
    elif uid[1] == Worker.TYPE:
        if uid in agent_cache:
            return agent_cache[uid]
    
        worker = Worker(uid[0], uid[2], agent_data[1], agent_data[2],
                        agent_data[3], agent_data[4], agent_data[5])
        worker.way_back_distances = agent_data[6]
        worker.sending_phase = agent_data[7]

        agent_cache[uid] = worker

        return worker
    else:
        pass