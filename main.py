# main.py
import time
import sys
import glob
import os
import cv2
import numpy as np
import carla

from carla_utils import connect_to_carla, load_world, get_blueprint_library, find_vehicle_blueprint, get_spawn_point, spawn_actor, destroy_actors
from sensor_utils import create_camera_blueprint, spawn_camera_sensor
from data_utils import read_columns_from_csv, convert_gps_to_relative_coordinates

IM_WIDTH = 640
IM_HEIGHT = 480
ORG = (50,70)   # starting location of letter
Font = cv2.FONT_ITALIC # shape of font
str = 0

def process_img(image):
    text = "brake : " + str
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    cv2.putText(i2, text, ORG, Font, 1, (255,0,0), 2)
    i3 = i2[:, :, :3]
    cv2.imshow("Camera View", i3)
    cv2.waitKey(1)
    return i3/255.0

def control_vehicle(vehicle, rpm, speed, brake, steer):
    throttle = min(speed / 100.0, 1.0)
    brake = min(max(brake, 0.0), 1.0)
    steer = np.clip(steer, -1.0, 1.0)

    control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)
    vehicle.apply_control(control)

if __name__ == "__main__":
    actor_list = []

    try:
        client = connect_to_carla('localhost', 2000)
        client.set_timeout(200)
        world = load_world(client, 'Town02')
        blueprint_library = get_blueprint_library(world)

        vehicle_bp = find_vehicle_blueprint(blueprint_library, 'vehicle.tesla.model3')
        spawn_point = get_spawn_point(world)

        vehicle = spawn_actor(world, vehicle_bp, spawn_point)
        actor_list.append(vehicle)
        
        vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))

        # Camera
        cam_bp = create_camera_blueprint(blueprint_library, IM_WIDTH, IM_HEIGHT, 110)
        spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
        sensor = spawn_camera_sensor(world, cam_bp, spawn_point, vehicle, process_img)
        actor_list.append(sensor)

        # Data
        data_path = r'D:\Desktop\newData.csv'
        column_names = ['speed', 'rpm', 'brake', 'steer', 'lon', 'lat']
        columns_data = read_columns_from_csv(data_path, column_names)

        if columns_data:
            for i in range(len(columns_data['speed'])):
                speed = float(columns_data['speed'][i])
                rpm = float(columns_data['rpm'][i])
                brake = float(columns_data['brake'][i])
                steer = float(columns_data['steer'][i])

                lon = float(columns_data['lon'][i])
                lat = float(columns_data['lat'][i])

                if brake == 1:
                    str = "On"
                else :
                    str = "Off"
                print(f"brake_status : {str}")

                relative_x, relative_y = convert_gps_to_relative_coordinates(lon, lat)
                print(f"Relative Coordinates: X={relative_x}, Y={relative_y}")

                control_vehicle(vehicle, rpm, speed, brake, steer)  
                time.sleep(1)
    
    finally:
        destroy_actors(actor_list)
        print('All cleaned up!')
