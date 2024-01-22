#carla_utils.py
import carla
import random

def connect_to_carla(host, port):
    client = carla.Client(host,port)
    client.set_timeout(200)
    return client

def load_world(client, world_name):
    return client.load_world(world_name)

def get_blueprint_library(world):
    return world.get_blueprint_library()

def find_vehicle_blueprint(get_blueprint_library, model_name):
    return get_blueprint_library.find(model_name)

def get_spawn_point(world):
    return random.choice(world.get_map().get_spawn_points())

def spawn_actor(world, blueprint, spawn_point):
    return world.spawn_actor(blueprint, spawn_point)

def destroy_actors(actor_list):
    for actor in actor_list:
        actor.destroy()