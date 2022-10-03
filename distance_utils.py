def find_agent_by_id(agents, id):
    for agent in agents:
        if id == agent.id:
            return agent

def distance(pt1, pt2):
    return ((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2) ** 0.5