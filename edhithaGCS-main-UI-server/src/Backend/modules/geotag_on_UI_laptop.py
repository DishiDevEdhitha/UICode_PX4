import re
import os
import sys
import shutil
import pandas as pd
import folium
import signal
import subprocess
from folium.features import DivIcon
import logging

def signal_handler(sig, frame):
    print("Stopping execution...")
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)

def move_and_create_unique_folder(folder_name, l_path):
    existing_folders = [folder for folder in os.listdir(
        folder_name) if os.path.isdir(os.path.join(folder_name, folder))]

    counter = 1
    new_name = os.path.basename(l_path)
    new_folder_name = f"{folder_name}_{counter}"

    while new_name in existing_folders:
        counter += 1
        new_folder_name = f"{l_path}_{counter}"
        new_name = os.path.basename(new_folder_name)

    os.makedirs(os.path.join(new_folder_name))
    print(f"Created a new folder: {new_folder_name}")

    if os.path.exists(l_path):
        for item in os.listdir(l_path):
            item_path = os.path.join(l_path, item)
            shutil.move(item_path, new_folder_name)

        print(f"Moved contents of {l_path} to {new_folder_name}")

def parse_string_to_variables(input_string):
    match = re.match(
        r"\$' (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) '\$", input_string)

    if match:
        file, lat, lon, alt, head, yaw, time = match.groups()
        lat, lon, alt, head, yaw, time = map(float, (lat, lon, alt, head, yaw, time))
        return file, lat, lon, alt, head, yaw, time
    else:
        return None

def run_local_commands(commands, l_path, cam_waypoints):
    results_columns = ['file', 'lat', 'lon', 'alt', 'head', 'yaw', 'time']
    results_df = pd.DataFrame(columns=results_columns)
    results_csv_file = os.path.join(l_path, 'results.csv')

    for command in commands:
        try:
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            output = result.stdout
            print(output)

            if result.stderr:
                print(f"Error: {result.stderr}")

            captured_lines = re.findall(r'\$.*?\$', output, re.DOTALL)
            prev_file = None

            for line in captured_lines:
                parsed = parse_string_to_variables(line)
                if not parsed:
                    continue

                file, lat, lon, alt, head, yaw, time = parsed

                if file != prev_file:
                    print("File:", file, "Latitude:", lat, "Longitude:", lon,
                          "Altitude:", alt, "Head:", head, "Yaw:", yaw, "Time:", time)

                    try:
                        results_df.loc[len(results_df)] = {
                            'file': file,
                            'lat': lat,
                            'lon': lon,
                            'alt': alt,
                            'head': head,
                            'yaw': yaw,
                            'time': time
                        }
                    except Exception as e:
                        print(f"Error updating results_df: {e}")

                    results_df.to_csv(results_csv_file, index=False)
                    cam_waypoints.append((lat, lon, alt, head, yaw, time))

                    prev_file = file

        except Exception as e:
            print(f"Failed to run command {command}: {e}")

if __name__ == '__main__':
    while True:
        commands_to_run = [
            'cd dino && ./geotag.sh'
        ]

        l_path = "/home/edhitha/edhithaGCS-main-UI-server/Data/Test"
        os.makedirs(l_path, exist_ok=True)

        parent_path = '/home/edhitha/edhithaGCS-main-UI-server/Data/'
        move_and_create_unique_folder(parent_path, l_path)

        os.makedirs(os.path.join(l_path, 'images'), exist_ok=True)
        print("Made images folder")
        os.makedirs(os.path.join(l_path, 'cropped'), exist_ok=True)
        print("Made cropped folder")

        cam_waypoints = []

        run_local_commands(commands_to_run, l_path, cam_waypoints)
