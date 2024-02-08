# Carla_Project

이 소프트웨어는 "Carla Simulator"에서 실제 주행 데이터를 구현합니다. 실제 주행 자동차에서 데이터를 얻습니다.
speed, rpm, brake, gps(경도, 위도), steer 데이터들을 csv 파일에 넣어서 읽고 csv 파일에서 가져갑니다. 
carla vehicle apply control로 전달합니다. 
carla 시뮬레이터에서 작동되는 것을 볼 수 있습니다.

<br><br/>
또한, 카메라 센서를 이용하여 브레이크를 밟은 것을 보여주고 이를 판독등으로 표시하여 자동차의 이미지를 인쇄하는 기능을 구현합니다.
```
# main.py

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

cam_bp = create_camera_blueprint(blueprint_library, IM_WIDTH, IM_HEIGHT, 110)
spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7)) # Camera spawn location
sensor = spawn_camera_sensor(world, cam_bp, spawn_point, vehicle, process_img)
actor_list.append(sensor)
```

<br><br/>
차량 데이터 추출에서 steer값(좌,우회전 구현)은 제공되지 않기 때문에, pid_utils.py 파일에서 횡방향 제어 알고리즘을 구현하였습니다.
```
if columns_data:
            controller = Controller2D(waypoints)
...

controller.update_values(Carla_Cor.x, Carla_Cor.y, yaw, speed, timestamp, True)
controller.update_controls()
controller.get_commands()
steer = controller.get_steer()
```

<br><br/>
We can also see the front view of the car.  
```
def create_camera_blueprint(blueprint_library, image_size_x, image_size_y, fov):
    cam_bp = blueprint_library.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", f"{image_size_x}")
    cam_bp.set_attribute("image_size_y", f"{image_size_y}")
    cam_bp.set_attribute("fov", f"{fov}")
    return cam_bp
```
```
def spawn_camera_sensor(world, blueprint, spawn_point, vehicle, callback_function):
    sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
    sensor.listen(lambda data: callback_function(data))
    return sensor
```

<br><br/>
We have the function of converting the gps data to relative coordinates in Carla Map. In order to express the longitude and latitude like GPS in Carla Simulator, we need to know what the exact relative coordinates is in Carla Map.
```
def geo_to_carla(latitude, longitude, altitude=0.0):
    geo_location = carla.Location(latitude, longitude, altitude)
    return geo_location

#Carla coordinates from Geo coordinates
carla_location = vehicle.get_location()
carla_coordinates = (carla_location.x, carla_location.y, carla_location.z)
print("Carla Coordinates(x,y,z):", carla_coordinates)

```


