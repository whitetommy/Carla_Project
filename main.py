# main.py

import time, sys, glob, os, cv2
import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


from carla_utils import connect_to_carla, load_world, get_blueprint_library, find_vehicle_blueprint, get_spawn_point, spawn_actor, destroy_actors
from sensor_utils import create_camera_blueprint, spawn_camera_sensor
from data_utils import read_columns_from_csv, geo_to_carla, calculate_yaw
from pid_utils import Controller2D
from osm_to_xodr import convert
from config_util import generate_xodr_map, set_spectator_location, set_vehicle_location

IM_WIDTH = 640  # Camera width
IM_HEIGHT = 480 # Camera height

ORG = (50,70)   # starting location of letter
Font = cv2.FONT_ITALIC # shape of font

str = "" # represent the brake status

waypoints = []
XODR_PATH = "output.xodr"

# print the image of car by using the camera sensor
def process_img(image):
    text = "brake : " + str
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    cv2.putText(i2, text, ORG, Font, 1, (255,0,0), 2)
    i3 = i2[:, :, :3]
    cv2.imshow("Camera View", i3)
    cv2.waitKey(1)  # wait the key (1ms) 
    return i3/255.0

# Car Control from rpm, speed, brake, steer

def control_vehicle(vehicle, rpm, speed, brake, steer):
    throttle = min(speed / 100.0, 1.0) 
    brake = min(max(brake, 0.0), 1.0)
    steer = np.clip(steer, -1.0, 1.0) # Data for car to move left and right side 

    control = carla.VehicleControl(throttle=throttle, steer = steer, brake = brake)
    vehicle.apply_control(control)

    #As long as we put the brake, brake light is gonna be red
    if brake == 0:
        vehicle.set_light_state(carla.VehicleLightState.NONE)
    else :
        vehicle.set_light_state(carla.VehicleLightState.Brake)

if __name__ == "__main__":
    actor_list = []

    try:
        client = connect_to_carla('localhost', 2000)

        world = generate_xodr_map(client, XODR_PATH) # Carla Map 
        set_spectator_location(world)
        blueprint_library = get_blueprint_library(world) 

        vehicle_bp = find_vehicle_blueprint(blueprint_library, 'vehicle.tesla.model3')
        spawn_point = get_spawn_point(world)

        vehicle = spawn_actor(world, vehicle_bp, spawn_point)
        actor_list.append(vehicle)

        set_vehicle_location(vehicle, x=7456, y=-2511, z=195)

        vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))

        # Camera
        cam_bp = create_camera_blueprint(blueprint_library, IM_WIDTH, IM_HEIGHT, 110)
        spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7)) # Camera spawn location
        sensor = spawn_camera_sensor(world, cam_bp, spawn_point, vehicle, process_img)
        actor_list.append(sensor)

        # Data
        data_path = r'D:\\Desktop\\newData.csv' # wherever users can set the path of data 
        column_names = ['speed', 'rpm', 'brake', 'lon', 'timestamp', 'lat', 'acc_x', 'acc_y']
        columns_data = read_columns_from_csv(data_path, column_names)


        if columns_data:
            controller = Controller2D(waypoints)

            for i in range(len(columns_data['speed'])):
                speed = float(columns_data['speed'][i])
                rpm = float(columns_data['rpm'][i])
                brake = float(columns_data['brake'][i])
                timestamp = float(columns_data['timestamp'][i])

                lon = float(columns_data['lon'][i])
                lat = float(columns_data['lat'][i])

                acceleration_x = float(columns_data['acc_x'][i])
                acceleration_y = float(columns_data['acc_y'][i])

                yaw = float(calculate_yaw(acceleration_x, acceleration_y))

                if brake == 1:
                    str = "On"
                else :
                    str = "Off"
                print(f"brake_status : {str}")

                Carla_Cor = geo_to_carla(lon, lat)
                waypoints.append([Carla_Cor.x, Carla_Cor.y, speed])

                controller.update_values(Carla_Cor.x, Carla_Cor.y, yaw, speed, timestamp, True)
                controller.update_controls()
                controller.get_commands()
                steer = controller.get_steer()

                #Carla coordinates from Geo coordinates
                carla_location = vehicle.get_location()
                carla_coordinates = (carla_location.x, carla_location.y, carla_location.z)
                print("Carla Coordinates(x,y,z):", carla_coordinates)

                control_vehicle(vehicle, rpm, speed, brake, steer)  
                time.sleep(1)

    finally:
        destroy_actors(actor_list)
        print('All cleaned up!')
