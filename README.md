# Carla_Project

This software implements a real driving data in the "Carla Simulator". We get the data from the real driving car 
like speed, rpm, brake, gps(longitude, latitude), steer. We put this data in the csv file, read it and take it from csv file. Then, we deliver it to the carla vehicle apply control. After that, we can see what it's gonna be operated in the carla simulator.

Furthermore, this software implements the function of printing the image of car by using the camera sensor to show
that we put the brake and represent it on the back of the car as a read light.

Also, We have the function of converting the gps data to relative coordinates in Carla Map. In order to express the longitude and latitude like GPS in Carla Simulator, we need to know what the exact relative coordinates is in Carla Map.
```
relative_x = (lon - 127.0) * 111000
relative_y = (lat - 37.5) * 111000
return relative_x, relative_y
```
