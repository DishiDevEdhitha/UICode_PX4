from PIL import Image
import math
import pickle
import time
import requests
from io import *
import pandas
import os
from modules.logger_config import *
from geopy.distance import distance
from geopy import Point

GSD = 0.43
orange_text = '\033[93m'  # ANSI escape code for orange text
reset_color = '\033[0m'
latitudes = ["null", "null", "null", "null", "null"]
longitudes = ["null", "null", "null", "null", "null"]

def calculate_offset_lat_long(lat_raw, lon_raw, heading, offset_m = -8):

    lat = lat_raw / 1e7
    lon = lon_raw / 1e7

    """
    Given a latitude, longitude, heading (degrees from north), and offset in meters,
    return the new latitude and longitude.
    """
    origin = Point(lat, lon)
    # Distance takes kilometers, so convert meters to km
    d = distance(meters=offset_m)
    destination = d.destination(point=origin, bearing=heading)
    return destination.latitude, destination.longitude

def read_dict_from_file(pickle_path):
    while True:
        try:
            # Synchronous code to read from the file
            with open(pickle_path, 'rb') as file:
                loaded_dict = pickle.load(file)
            return loaded_dict
        except Exception as e:
            print('Error occurred while loading the cam_wps:', e)
            time.sleep(0.2133253665)


def get_midpoint(image_path):
    # Open the image
    print("Getting midpoint from image",  flush=True)
    response = requests.get(image_path)

# Check if the request was successful
    if response.status_code == 200:
        # Open the image from the response content
        img = Image.open(BytesIO(response.content))
        # img.show()
        # Get the width and height of the image
        width, height = img.size
        # Calculate the mid_x and mid_y coordinates
        mid_x = width // 2
        # print("in midpoint fxn")
        mid_y = height // 2
        print("midpoints", mid_x, mid_y, flush=True)
        return (mid_x, mid_y)
    else:
        print("L bro", flush=True)


def newlatlon(lat, lon, hdg, dist, movementHead, heading):
    lati = math.radians(lat)
    longi = math.radians(lon)
    rade = 6367489
    AD = dist/rade
    sumofangles = (hdg + movementHead) % 360
    newheading = math.radians(sumofangles)
    newlati = math.asin(math.sin(lati)*math.cos(AD) +
                        math.cos(lati)*math.sin(AD)*math.cos(newheading))
    newlongi = longi + math.atan2(math.sin(newheading)*math.sin(AD)
                                  * math.cos(lati), math.cos(AD)-math.sin(lati)*math.sin(newlati))
    
    print(newlati, newlongi)
    

    ret_lat0 = int(round(math.degrees(newlati*1e7)))
    ret_lon0 = int(round(math.degrees(newlongi*1e7)))

    print(ret_lat0, ret_lon0, heading, flush=True)

     # --------------------------

    ret_lat, ret_lon = calculate_offset_lat_long(ret_lat0, ret_lon0, heading)

    ret_lat = int(ret_lat * 1e7)  # 130344181
    ret_lon = int(ret_lon * 1e7)   # 775666914


     # --------------------------

    # print("Latitude and Longitude in newlatlon: %s and %s", ret_lat, ret_lon)
    return ret_lat, ret_lon


def lat_long_calculation(csv_path, img_path, image_name, x_cord, y_cord, target_no):
    print("Calculating Latitude and Longitude", flush=True)
    print(img_path, flush=True)
    point2 = get_midpoint(img_path)
    point1 = (x_cord, y_cord)
    # print("midpoint in picture: %s",point2 )

    # calculate the distance between the two points in pixels

    pixel_distance = math.sqrt(
        (point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    # calculate the angle between the two points in radians
    angle = math.degrees((math.atan2(point2[1] - point1[1], point2[0] - point1[0]))) - 90
    theta = math.degrees((math.atan2(point2[1] - point1[1], point2[0] - point1[0])))

    if theta < 0:
        theta = 360 + theta

    # convert the pixel distance to real-life distance using the GSD constant
    real_distance_gsd = (pixel_distance * GSD)/100
    logging.info("Real distance GSD: %s",real_distance_gsd )

    df = None
    while df == None:
        try:
            df = pandas.read_csv(csv_path)
            break
        except Exception as e:
            # logging.info('Error loading the csv file, reloading... %s', e)
            time.sleep(1)
            continue

    index = int(os.path.basename(image_name)[-9:-4])
    # logging.info('index: %s', index)
    while True:
        if index in df.index:
            break
        else:
            # logging.info("Waiting for data")
            time.sleep(0.1)
            # continue
    row = df.loc[index]
    lat_to_use, long_to_use, alt_to_use, heading_to_use, yaw, = row[
        'lat'], row['lon'], row['alt'], row['head'], row['yaw']
    # logging.info("Index of data %s", df.loc[index])
    print("Latitude: ", lat_to_use, "Longitutde: ", long_to_use, "Heading: ", heading_to_use, flush=True)
    if lat_to_use == None:
        # logging.info("NO LAT AND LON FOUND, EXITNG FUNCTION")
        return
    if yaw == None:
        yaw = 0

    logging.info('Desired Latitude and Longitude sent ....')
    logging.info("Pixel distance: %s", pixel_distance)
    logging.info("Angle: %s", angle)
    logging.info("Calculated Real-life distance: %s", real_distance_gsd)

    value = newlatlon(lat_to_use, long_to_use, yaw, real_distance_gsd, angle, heading_to_use)

    print("Found Real latitude and longitude: ", value, flush=True)
    lati = value[0]
    longi = value[1]

    latitudes[target_no-1] = lati
    longitudes[target_no-1] = longi

    logging.info("saved target lat and long: %s, %s", lati, longi)
    global msgs
    msgs = "Saved target "+str(target_no) + \
        " for lat and long:"+str(lati/1e7) + " ,"+str(longi/1e7)

    print(msgs, flush=True)
    # print(latitudes, longitudes, flush=True)

    return latitudes, longitudes
    # logging.info("Lats and Longs: %s", (latitudes, longitudes))
