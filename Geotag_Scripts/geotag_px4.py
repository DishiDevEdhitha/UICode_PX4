# Lot of indentation errors are there, please check the code and correct it

import os
import re
import sys
import time
import math
import shutil
import bisect
import folium
import pickle
import socket
import pexpect
import hashlib
import argparse
import threading
import subprocess
import multiprocessing
import pandas as pd
from pymavlink import mavutil
from folium.features import DivIcon

# Default values for variables, if any defaults changed please change it in the below description also
defaults = {
    'ubuntu': True,
    'UDP': True,
    'wire_conn': False,
    'socket': False,
    'ssh': False,
    'arm': False,
    'avg': False,
    'heading': False,
    'interpolate': False,
    'delete': False,
    'pix_port': '/dev/ttyUSB0',
    'baud': 57600,
    'lap_ip': '192.168.1.45',
    'jet_ip': '192.168.1.21',
    'udp': '14550',  # PX4 default
    'tcp': '5760',
    'soc_port': 6234,
    'rate': 5,
    'cam_z': 180,
    'cap_gap': 1000,
    'no_img': 1000,
    'l_path': '/home/dishita/Desktop/UI_PX4/images',
    'exif_path': '/usr/bin/exiftool'
}

# Descriptions for each argument
descriptions = {
    'ubuntu': 'Operating System being used to run this Python Automation',
    'UDP': 'Want to use UDP Connection?',
    'wire_conn': 'Pass this to enable USB connection, (Debugging purpose)',
    'socket': 'Pass it to enable the socket connection for ground laptop',
    'ssh': 'Pass it to enable the SSH connection to get the Data',
    'arm': 'will start after the arming the drone',
    'avg': 'If it is passed, the average algorithm will kick in',
    'interpolate': 'If it is passed then interpolate algorithm will kick in',
    'delete': 'Pass it to Delete the previous test_cam instead of moving this to the new test_cam_x folder',
    'heading': 'If this is passed then the yaw will be using the heading value it self, in some cases increases the accuracy',
    'pix_port': 'For Windows it is COM ports, for Ubuntu it is in the tty* format, (Ubuntu cmd = \'ls /dev/tty*\')',
    'baud': 'Set Pixhawk Baud, (Known using Mission Planner)',
    'lap_ip': 'Laptop Static IP of Jetson with a Active Connection',
    'jet_ip': 'Jetson Static IP of Jetson with a Active Connection, (cmd = \'ifconfig\')',
    'udp': 'Desired UDP port (PX4 default: 14540)',
    'tcp': 'Desired TCP port',
    'soc_port': 'Desired port, and must be used same as in the laptop receiving on ground',
    'rate': 'The rate at which the Global_Position_Int and Attitude is updating, should be above 2Hz, and not above 4Hz',
    'cam_z': 'Camera rotation angle (0, 90, 180, 270), all positive or clockwise rotation of camera direction from Pixhawk Direction',
    'cap_gap': 'Capture gap in milliseconds',
    'no_img': 'Number of Images to be taken in the automation run',
    'l_path': 'Local path, images will be saved in this path',
    'exif_path': 'Path to the ExifTool executable,(cmd = \'which exiftool\')'
}

# Create an ArgumentParser object
parser = argparse.ArgumentParser(description='Script for performing automation task.')
parser.add_argument(
    "--use-sample-images",
    action="store_true",
    help="Run without starting camera capture, just use existing images"
)


# Add arguments to override default values
for key, value in defaults.items():
    if key in ['wire_conn', 'socket', 'arm', 'avg', 'interpolate', 'heading', 'ssh', 'delete']:
        parser.add_argument(f'-{key}', action='store_true', help=f'{descriptions[key]}, Default value: \'{value}\'')
    else:
        parser.add_argument(f'-{key}', default=value, help=f'{descriptions[key]}, Default value: \'{value}\'')

# Parse the command-line arguments
args = parser.parse_args()

# Access the values of the arguments
Ubuntu = bool(args.ubuntu)
UDP = (args.UDP)
using_wire_connection = bool(args.wire_conn)
socket_en = bool(args.socket)
ssh_en = bool(args.ssh)
arm_en = bool(args.arm)
avg_en = bool(args.avg)
inter_en = bool(args.interpolate)
heading_en = bool(args.heading)
del_en = bool(args.delete)
pixhawk_port = args.pix_port
baud = args.baud
jetson_ip = args.jet_ip
udp_port = args.udp
tcp_port = args.tcp
socket_port = int(args.soc_port)
refresh_rate = int(args.rate)
cam_z_rotation = int(args.cam_z)
capture_gap = int(args.cap_gap)
no_images = int(args.no_img)
local_path = args.l_path

def make_directory(directory_path):
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
        print(f"Directory '{directory_path}' created successfully.")
    else:
        print(f"Directory '{directory_path}' already exists.")

def move_and_create_unique_folder(folder_name):
    folder_path = '/home/edhitha/DCIM'
    existing_folders = [f for f in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, f))]
    print(existing_folders)

    # Initialize a counter for postfixing the folder name
    counter = 1

    # Generate a new folder name with a postfixed number
    new_folder_name = f"{folder_name}_{counter}"
    new_name = os.path.basename(new_folder_name)
    print(new_name)
    # Keep incrementing the counter until a unique folder name is found
    while new_name in existing_folders:
        counter += 1
        new_folder_name = f"{folder_name}_{counter}"
        new_name = os.path.basename(new_folder_name)
    # Create the new folder
    os.makedirs(new_folder_name)

    print(f"Created a new folder: {new_folder_name}")

    # Check if the original folder exists
    if os.path.exists(folder_name):
        # Move the contents of the original folder to the new one
        for item in os.listdir(folder_name):
            item_path = os.path.join(folder_name, item)
            shutil.move(item_path, new_folder_name)

        print(f"Moved contents of {folder_name} to {new_folder_name}")

try:
    if not del_en:
        move_and_create_unique_folder(local_path)
    else:
        os.removedirs(local_path)
except Exception as e:
    print(e)

local_path_images = os.path.join(local_path, 'images')
make_directory(local_path_images)

# Global variables
SR = b'SR2'  # unused on PX4 (kept to preserve structure)
socket_timeout = 1
gps_data = multiprocessing.Manager().list()
cam_waypoints = []
initial_time = multiprocessing.Manager().Queue()
mavlink_conn = multiprocessing.Manager().list()
Failure = 0

print('camera_image_path:', local_path_images)

# PX4-friendly mavlink-router example (printed as guidance only)
command = (
    f"sudo mavlink-routerd -e 127.0.0.1:14540 -e 127.0.0.1:14550 "
    f"{pixhawk_port}:{baud} -e {defaults['lap_ip']}:{udp_port}"
)

def capture_image(l_path_images):
    global Failure
    global initial_time
    try:
        # Define the FFmpeg command to capture images
        capture_command = (
            f"ffmpeg -f v4l2 -framerate 30 -video_size 1920x1080 -input_format mjpeg "
            f"-i /dev/video0 -vf 'fps=1' -q:v 2 {l_path_images}/image_%05d.jpg"
        )

        # Use subprocess to run the image capture command
        process = subprocess.Popen(
            capture_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True  # ensures the logs are strings, not bytes
        )

        # Continuously read the output of the process
        while True:
            output = process.stderr.readline()  # FFmpeg logs progress to stderr
            if not output:
                print(red_text, 'Breaking the loop', reset_color)
                break

            # Check for FFmpeg's "frame=" message and capture the initial time
            if "frame=" in output:
                actual_time = time.time()  # Get the exact time when frame is captured
                initial_time.put(actual_time)  # Push the actual timestamp to the queue
                print(f"Image Captured at {actual_time}")

            # Print the output log (optional)
            print(purple_text, output, reset_color, end='')

        print(purple_text, 'Trying to communicate', reset_color)

        # Wait for the process to complete
        process.communicate()

        print("Image capture command has completed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing the image capture command: {e}")
    except Exception as ex:
        print(f"An error occurred: {ex}")

def get_newest_image_path(l_path):
    image_extensions = ["png", "jpg", "jpeg", "gif"]
    images = []
    for ext in image_extensions:
        images.extend([os.path.join(l_path, image) for image in os.listdir(l_path) if image.lower().endswith(ext)])
    if not images:
        return None

    newest_image = max(images, key=os.path.getctime)
    return newest_image

def get_img_prefix_from_local(l_path):
    file_prefix = None  # Initialize file_prefix with a default value
    try:
        # List files in the local directory
        files = os.listdir(l_path)

        # Check if there are any files
        if files:
            max_suffix = -1
            file = None
            for name in files:
                match = re.search(r'image_(\d+)', name)
                if match:
                    suffix = int(match.group(1))
                    if suffix > max_suffix:
                        max_suffix = suffix
                        file = name
            if file:
                parts = file.split('_')
                file_prefix = parts[0] + '_'

        if file_prefix:
            print(f"File Prefix: {file_prefix}")
            return file_prefix
        else:
            print("No files found in the folder.")
            return None

    except FileNotFoundError:
        print("Local directory not found.")
        return None

def is_drone_armed(master):
    try:
        # Wait for the HEARTBEAT message
        msg = None
        while msg is None or msg.get_type() != 'HEARTBEAT':
            msg = master.recv_msg()
            print('Waiting for the heartBeat ...')

        # Check if the drone is armed
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    except Exception as e:
        print(f"Error: {e}")
        return False

def collect_gps_data():
    global gps_data
    global arm_en

    # Connect to the MAVLink system (PX4-friendly connection strings)
    if using_wire_connection:
        mavlink_connection = mavutil.mavlink_connection(pixhawk_port, baud=baud)
        print(red_text, 'Waiting for Heartbeat ....', reset_color)
        mavlink_connection.wait_heartbeat()
        print(f"{green_text}HeartBeat in-sync ...{reset_color}")
    elif UDP:
        # PX4 default UDP endpoint
        mavlink_connection = mavutil.mavlink_connection(f'udp:127.0.0.1:{udp_port}')
        print(red_text, 'Waiting for Heartbeat ....', reset_color)
        mavlink_connection.wait_heartbeat()
        print(f"{green_text}HeartBeat in-sync ...{reset_color}")
    else:
        mavlink_connection = mavutil.mavlink_connection(f'tcp:127.0.0.1:{tcp_port}')
        print(red_text, 'Waiting for Heartbeat ....', reset_color)
        mavlink_connection.wait_heartbeat()
        print(f"{green_text}HeartBeat in-sync ...{reset_color}")

    # --- PX4 MESSAGE RATES (replace ArduPilot SR2_* params) ---
    # Set GLOBAL_POSITION_INT rate
    try:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            int(1e6 / max(1, refresh_rate)),
            0, 0, 0, 0, 0
        )
        # Set ATTITUDE rate (only used if heading_en is False)
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            int(1e6 / max(1, refresh_rate)),
            0, 0, 0, 0, 0
        )
        print(f'{orange_text}GLOBAL_POSITION_INT and ATTITUDE set to {refresh_rate}Hz{reset_color}')
    except Exception as e:
        print(f"{red_text}Failed to set message intervals (PX4). Proceeding anyway. Error: {e}{reset_color}")

    if arm_en:
        print('Waiting for Arming the Drone ...')
        while not is_drone_armed(mavlink_connection):
            print('Waiting for the drone to arm ...')
            time.sleep(0.5)
        print('Waiting for 5 seconds before starting ...')
        time.sleep(5)
        arm_en = False

    mavlink_conn.append(1)
    print('Receiving GPS Stream ...')

    # Define the MAVLink message types to listen for
    msg_list = ['GLOBAL_POSITION_INT']
    if not heading_en:
        msg_list.append('ATTITUDE')

    while True:
        try:
            # Receive MAVLink message
            msg = mavlink_connection.recv_match(type=msg_list, blocking=True)
            if msg is None:
                continue

            if msg.get_type() == 'GLOBAL_POSITION_INT':
                # Create a new GPS data entry
                timestamp = time.time()  # Store timestamp for every GPS entry
                if heading_en:
                    gps_entry = (
                        msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3,
                        ((getattr(msg, 'hdg', 65535) / 100.0) + cam_z_rotation) % 360,  # Heading
                        ((getattr(msg, 'hdg', 65535) / 100.0) + cam_z_rotation) % 360,  # Yaw (same as heading)
                        timestamp
                    )
                else:
                    gps_entry = (
                        msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3,
                        ((getattr(msg, 'hdg', 65535) / 100.0) + cam_z_rotation) % 360,
                        timestamp  # Now stores timestamp in all cases
                    )

                gps_data.append(gps_entry)

            elif msg.get_type() == 'ATTITUDE':
                if len(gps_data) > 0:  # Ensure there is GPS data before modifying
                    if len(gps_data[-1]) == 4:
                        yaw = math.degrees(msg.yaw)
                        yaw = (yaw + cam_z_rotation) % 360  # Normalize yaw to [0, 360)
                        gps_data[-1] = gps_data[-1][:4] + (yaw, gps_data[-1][-1])  # Keep original timestamp

            if gps_data:
                try:
                    print(orange_text, gps_data[-1], reset_color)
                except IndexError:
                    pass  # Avoid printing error if list is empty

            time.sleep(1 / (refresh_rate * 2))  # Maintain desired frequency
        except Exception as e:
            print(f"Error in GPS data collection: {e}")
            time.sleep(0.1)  # Prevent excessive errors from crashing the loop

def wait_for_image(image_dir, expected_count, timeout=5):
    """Waits until a new image appears and is non-empty."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        images = sorted([f for f in os.listdir(image_dir) if f.endswith('.jpg')])
        if len(images) >= expected_count:
            last_image = os.path.join(image_dir, images[-1])
            if os.path.getsize(last_image) > 0:
                return True  # Image is fully written
        time.sleep(0.5)  # Wait and retry
    return False  # Timeout reached, image not found or empty

# Function to process images and geo-tag them
def geotag_images(prefix, l_path):
    """Geotags a list of images using GPS data.
    Args:
        prefix, l_path
    Returns:
        None.
    """
    global sent
    global gps_data
    t_path = None
    sent = []
    image_counter = 0
    avg_time = []
    pickle_path = os.path.join(local_path, 'cam_wps.pickle')

    # Wait for at least one image timestamp
    while initial_time.empty():
        time.sleep(0.05)

    print("Starting Geotagging...")

    while True:
        try:
            # Fetch the actual timestamp for this image
            image_time = initial_time.get()

            local_image_name = f"{prefix}{image_counter:03d}.jpg"
            print(reset_color, 'Started Geotagging for', local_image_name)

            # Find the nearest GPS entry
            while True:
                try:
                    temp_data = list(gps_data)

                    if not temp_data:
                        print("Waiting for GPS data...")
                        time.sleep(0.05)
                        continue  # Retry until GPS data is available

                    gps_index = bisect.bisect_right([t[-1] for t in temp_data], image_time)
                    gps_index = max(0, min(gps_index, len(temp_data) - 1))  # Ensure index is within range

                    print('Image index for image time:', blue_text, gps_index, reset_color,
                          'Data length:', blue_text, len(temp_data), reset_color)

                    if inter_en:
                        print('Interpolating GPS Data for index:', gps_index, 'Data length:', len(temp_data))
                        gps_entry = interpolate_gps_data(gps_index, temp_data, image_time)
                    else:
                        gps_entry = temp_data[gps_index]

                    break  # Exit loop once GPS data is found
                except IndexError as e:
                    print("Error:", e, "- Retrying GPS lookup...")
                    time.sleep(1 / refresh_rate)

            # Store GPS data for mapping
            cam_waypoints.append(gps_entry)

            # Save geotagging data in a pickle file
            while True:
                try:
                    with open(pickle_path, 'wb') as file:
                        pickle.dump(cam_waypoints, file)
                    break
                except Exception:
                    time.sleep(0.0176)

            # Debugging: Print stored metadata
            try:
                if ssh_en:
                    print('$\'', local_image_name, gps_entry[0], gps_entry[1], gps_entry[2],
                          gps_entry[3], gps_entry[4], gps_entry[5], '\'$')
            except Exception as e:
                print(e)

            # Calculate time difference for validation
            avg_time.append(abs(gps_entry[-1] - image_time))
            print('Avg time difference:', blue_text, sum(avg_time) * 1000 / len(avg_time), reset_color, 'ms')

            # Check if all images have been processed
            if no_images == image_counter:
                print('All images processed.')
                break

            # Ensure image count matches expected value
            if not wait_for_image(local_path_images, image_counter + 1):
                print(f'{red_text}Image Capturing process unexpectedly ended...{reset_color}')
                print(f'{blue_text}Rerun both scripts...{reset_color} \nNow, exiting the program...')
                return None

            image_counter += 1

        except Exception as e:
            print("Geotagging error:", e)
            try:
                path_map(cam_waypoints)
            except Exception as map_error:
                print("Map error:", map_error)
            time.sleep(0.25)

def interpolate_gps_data(index, gps_data, target_time):
    """Interpolates GPS data between two timestamps using Newton interpolation."""
    if len(gps_data) < 2:
        print("Not enough GPS data for interpolation.")
        return gps_data[0]  # Return the only available GPS entry.

    # Ensure index is within range
    index = max(1, min(index, len(gps_data) - 1))

    prev_entry = gps_data[index - 1]
    next_entry = gps_data[index]

    # Calculate time difference
    time_diff_prev = next_entry[-1] - prev_entry[-1]

    # If timestamps are identical or time difference is too small, return the previous entry
    if abs(time_diff_prev) < 1e-9:
        return prev_entry

    # Ensure we have enough points for interpolation
    start_index = max(0, index - 3)  # At least 3 previous points for interpolation
    subset = gps_data[start_index: index + 1]

    print(f'Interpolating GPS Data for index {index}, subset size: {len(subset)}')

    # Interpolate using Newton's method
    # Note: Structure preserved; this uses the first field only by design.
    interpolated_values = tuple(
        newton_interpolation(subset, target_time) for _ in range(5)
    ) + (target_time,)

    return interpolated_values

def newton_interpolation(points, x):
    """Newton interpolation for a set of points."""
    n = len(points)

    # Ensure we have enough points to interpolate
    if n < 2:
        print("Not enough data points for interpolation.")
        return points[0][0]  # Return the closest available value

    # Calculate divided differences table (uses only first value in tuple)
    divided_diffs = [points[i][0] for i in range(n)]  # latitude/longitude/etc.

    for j in range(1, n):
        for i in range(n - 1, j - 1, -1):
            denominator = points[i][1] - points[i - j][1]  # Timestamp difference
            if abs(denominator) < 1e-9:  # Avoid division by zero
                return points[i][0]  # Return the closest GPS value
            divided_diffs[i] = (divided_diffs[i] - divided_diffs[i - 1]) / denominator

    # Initialize result as the last divided difference
    result = divided_diffs[-1]

    # Evaluate the interpolating polynomial
    for i in range(n - 2, -1, -1):
        result = result * (x - points[i][1]) + divided_diffs[i]

    return result

def path_map(waypoints):
    # Create a folium map centered at the first waypoint
    m = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=16)

    # Add waypoints and custom arrow markers
    for lat, lon, alt, heading, yaw, time1 in waypoints:
        # Create a custom arrow marker
        icon = DivIcon(
            icon_size=(20, 20),
            icon_anchor=(10, 10),
            html=f'<div style="transform: rotate({yaw}deg); color: red;">&#11015;</div>',
        )

        folium.Marker(
            [lat, lon],
            icon=icon,
            popup=f"Time: {time1}<br>Altitude: {alt}"
        ).add_to(m)

    web_view = os.path.join(local_path, 'waypoints_map.html')
    # Save the map to an HTML file
    m.save(web_view)
    print('Check out the updated HTML file for path review...')

if __name__ == "__main__":
    # Important global variables
    if not args.use_sample_images:
        capture_process = multiprocessing.Process(target=capture_image, args=(local_path_images,))
        capture_process.start()

    # ANSI escape code for colored text
    red_text = "\033[91m"
    blue_text = "\033[94m"
    green_text = "\033[92m"
    purple_text = "\033[95m"
    orange_text = "\033[93m"
    # Reset the text color to default after printing
    reset_color = "\033[0m"

    if not using_wire_connection:
        # Your text to be printed in red
        text_to_print = f'''To run the mavlink-router command (PX4 defaults):
Copy & Paste this in Jetson terminal or in the SSH terminal and authenticate with the password.
Change the laptop ip and UDP port for external mavlink connection.
If you are using telem connection to Jetson you might also want change the Pixhawk port and baud ...
'''
        # Print the text in red and the command which is the udp connections in blue
        print(red_text + text_to_print + blue_text + command + reset_color)

    if socket_en:
        print('Request received to start the Jetson Server')
    else:
        print(f'{red_text}Server is not enabled, if you want the server initiate to get GPS Data, close this script and run the following command:')
        print(f'{blue_text}python3 <this_script_name>.py --socket{reset_color}')

    collect_gps_data_thread = multiprocessing.Process(target=collect_gps_data)
    collect_gps_data_thread.start()
    time.sleep(1)  # Allow GPS thread to initialize

    while arm_en:
        time.sleep(1)

    # Wait for MAVLink connection with a timeout to prevent deadlocks
    mavlink_timeout = 30  # Max wait time in seconds
    start_time = time.time()
    while len(mavlink_conn) == 0:
        if time.time() - start_time > mavlink_timeout:
            print(red_text, "MAVLink connection timeout. Check your Pixhawk connection.", reset_color)
            sys.exit(1)

        print(red_text, 'Waiting for MAVLink Stream', reset_color)
        time.sleep(0.5)

        if using_wire_connection:
            print(blue_text, f'Check Pixhawk port: {pixhawk_port}:{baud}', reset_color)
        elif UDP:
            print(blue_text, f'Check UDP port: 127.0.0.1:{udp_port}', reset_color)
        else:
            print(blue_text, f'Check TCP port: 127.0.0.1:{tcp_port}', reset_color)

    # Start the image capture process
    capture_process = multiprocessing.Process(target=capture_image, args=(local_path_images,))
    capture_process.start()
    time.sleep(1)  # Allow capture process to initialize

    # Ensure initial_time is populated before proceeding
    max_wait_time = 10  # Maximum time to wait for an image timestamp (in seconds)
    start_time = time.time()
    while initial_time.empty():
        if time.time() - start_time > max_wait_time:
            print(red_text, "No image timestamps received. Check camera connection.", reset_color)
            sys.exit(1)
        time.sleep(0.05)

    if len(mavlink_conn) == 1:
        # Get the prefix from existing local images
        prefix = get_img_prefix_from_local(local_path_images)
        print(prefix)

    try:
        geotag_images(prefix, local_path_images)
    finally:
        print("Cleaning....")
        if capture_process.is_alive():
            capture_process.terminate()
            capture_process.join()

        if collect_gps_data_thread.is_alive():
            collect_gps_data_thread.terminate()
            collect_gps_data_thread.join()

    print('Program done is all Job')
