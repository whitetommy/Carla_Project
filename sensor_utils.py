# sensor_utils.py
import numpy as np

def create_camera_blueprint(blueprint_library, image_size_x, image_size_y, fov):
    cam_bp = blueprint_library.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", f"{image_size_x}")
    cam_bp.set_attribute("image_size_y", f"{image_size_y}")
    cam_bp.set_attribute("fov", f"{fov}")
    return cam_bp

def spawn_camera_sensor(world, blueprint, spawn_point, vehicle, callback_function):
    sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
    sensor.listen(lambda data: callback_function(data))
    return sensor
