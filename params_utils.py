def check_params(params) -> tuple:

    if params['drone.move_period'] <= 0:
        return (False, "drone.move_period cannot be less than one")

    if params['drone.count'] < 2:
        return (False, "drone.count cannot be less than two")

    if params['drone.drone_radius_distance'] <= 0:
        return (False, "drone.drone_radius_distance cannot be less than one")

    if params['drone.close_to_disconnect_radius_distance'] >= params['drone.drone_radius_distance']:
        return (False, "drone.close_to_disconnect_radius_distance cannot be more or equal drone.drone_radius_distance")

    if params['drone.stable_sending_speed_max_distance'] >= params['drone.close_to_disconnect_radius_distance']:
        return (False, "drone.stable_sending_speed_max_distance be more or equal drone.close_to_disconnect_radius_distance")

    if params['scout.count'] <= 0:
        return (False, "scout.count cannot be less than one")

    if params['scout.energy_limit'] <= 0:
        return (False, "scout.energy_limit cannot be less than one")

    if params['data.count'] <= 0:
        return (False, "data.count cannot be less than one")

    if params['data.size'] <= 0:
        return (False, "data.size cannot be less than one")

    if params['data.generate_period'] <= 0:
        return (False, "data.generate_period cannot be less than one")

    if params['world.width'] <= 0:
        return (False, "world.width cannot be less than one")

    if params['world.height'] <= 0:
        return (False, "world.height cannot be less than one")

    return (True, "All input parameters are correct")