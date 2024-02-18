# data_utils.py
import csv, carla
import math

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

def geo_to_carla(latitude, longitude, altitude=0.0):
    geo_location = carla.Location(latitude, longitude, altitude)
    return geo_location


def calculate_yaw_from_gps(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2]) # change for radian
    dLon = lon2 - lon1 # difference of longitude
    
    # calculate direction of two locations
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.atan2(y, x)
    
    # replace radian with degree
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360  # range is [0,360)
    
    return bearing

def calculate_steer_angle(car_position, car_orientation, target_position):
    """
    :param car_position: now location of car (x, y)
    :param car_orientation: now direction of car (radian)
    :param target_position: destination of car location (x, y)
    :return: steer (range -1.0 ~ 1.0, -1 is end of left, 1 is end of right)
    """
    target_vector = (target_position.x - car_position.x, target_position.y - car_position.y)
    target_angle = math.atan2(target_vector[1], target_vector[0])
    
    # calculate the difference of present point to direction point
    angle_difference = target_angle - car_orientation
    angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi # range is -pi ~ +pi
    
    steer_value = max(-1.0, min(1.0, angle_difference / math.pi))
    
    return steer_value
