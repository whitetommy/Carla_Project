# Carla_Project
<br><br/>
이 소프트웨어는 "Carla Simulator"에서 실제 주행 데이터를 구현합니다. 실제 주행 자동차에서 데이터를 얻습니다.
speed, rpm, brake, gps(경도, 위도), steer 데이터들을 csv 파일에 넣어서 읽고 csv 파일에서 가져갑니다. 
```
data_path = r'D:\Desktop\newData.csv' # wherever users can set the path of data 
column_names = ['speed', 'rpm', 'brake', 'lon', 'timestamp', 'lat', 'acc_x', 'acc_y']
columns_data = read_columns_from_csv(data_path, column_names)
```

carla vehicle apply control로 전달합니다. carla 시뮬레이터에서 작동되는 것을 볼 수 있습니다.
```
#data_utils.py
def read_columns_from_csv(file_path, column_names):
    try:
        with open(file_path, 'r', newline='', encoding='utf-8') as csvfile:
            reader = csv.DictReader(csvfile)
            for column_name in column_names:
                if column_name not in reader.fieldnames:
                    raise ValueError(f"{column_name} column is not found! ")

            columns_data = {column_name: [] for column_name in column_names}
            
            for row in reader:
                for column_name in column_names:
                    columns_data[column_name].append(row[column_name])

        return columns_data
    except FileNotFoundError:
        print(f"file is not found!: {file_path}")
    except Exception as e:
        print(f"Error: {e}") 
```

또한, 카메라 센서를 이용하여 브레이크를 밟은 것을 보여주고 이를 판독등으로 표시하여 자동차의 이미지를 인쇄하는 기능을 구현합니다.
```
#sensor_utils.py
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
```
```
#main.py
IM_WIDTH = 640  # Camera width
IM_HEIGHT = 480 # Camera height

ORG = (50,70)   # starting location of letter
Font = cv2.FONT_ITALIC # shape of font

str = "" # represent the brake status

def process_img(image):
    text = "brake : " + str
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    cv2.putText(i2, text, ORG, Font, 1, (255,0,0), 2)
    i3 = i2[:, :, :3]
    cv2.imshow("Camera View", i3)
    cv2.waitKey(1)  # wait the key (1ms) 
    return i3/255.0

cam_bp = create_camera_blueprint(blueprint_library, IM_WIDTH, IM_HEIGHT, 110)
spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7)) # Camera spawn location
sensor = spawn_camera_sensor(world, cam_bp, spawn_point, vehicle, process_img)
actor_list.append(sensor)

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

if brake == 1:
    str = "On"
else :
    str = "Off"
print(f"brake_status : {str}")
```

차량 데이터 추출에서 steer값(좌,우회전 구현)은 제공되지 않기 때문에, pid_utils.py 파일과 cutils.py에서 횡방향 제어 알고리즘을 구현하였습니다.
```
#data_utils.py
def calculate_yaw(acceleration_x, acceleration_y):
    yaw_rad = math.atan2(acceleration_x, acceleration_y)
    yaw_deg = math.degrees(yaw_rad) # change to degree from radian
    return yaw_deg
```
```
#main.py
if columns_data:
            controller = Controller2D(waypoints)
...

controller.update_values(Carla_Cor.x, Carla_Cor.y, yaw, speed, timestamp, True)
controller.update_controls()
controller.get_commands()
steer = controller.get_steer()
```

Carla Map에서 GPS 데이터를 상대 좌표로 변환하는 기능을 가지고 있습니다. 칼라 시뮬레이터에서 GPS처럼 경도와 위도를 표현하기 위해서는 칼라 지도에서 정확한 상대 좌표가 무엇인지 알아야 합니다.
```
def geo_to_carla(latitude, longitude, altitude=0.0):
    geo_location = carla.Location(latitude, longitude, altitude)
    return geo_location

#Carla coordinates from Geo coordinates
carla_location = vehicle.get_location()
carla_coordinates = (carla_location.x, carla_location.y, carla_location.z)
print("Carla Coordinates(x,y,z):", carla_coordinates)
```
