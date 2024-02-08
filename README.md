# Carla_Project

This software implements a real driving data in the "Carla Simulator". We get the data from the real driving car 
like speed, rpm, brake, gps(longitude, latitude), steer. We put this data in the csv file, read it and take it from csv file. Then, we deliver it to the carla vehicle apply control. After that, we can see what it's gonna be operated in the carla simulator.

Furthermore, this software implements the function of printing the image of car by using the camera sensor to show that we put the brake and represent it on the back of the car as a read light.


## main.py
```
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
relative_x = (lon - 127.0) * 111000
relative_y = (lat - 37.5) * 111000
return relative_x, relative_y
```
