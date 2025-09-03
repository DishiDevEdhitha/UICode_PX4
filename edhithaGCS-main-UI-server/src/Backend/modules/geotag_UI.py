import re
import os
import sys
import shutil
import pandas
import folium
import signal
import pandas
import paramiko
from folium.features import DivIcon
import subprocess
import logging
import csv
import pickle
import time

def signal_handler(sig, frame):
    print("Stopping execution...")
    sys.exit(0)

REMOTE_PATH = "edhitha@192.168.144.50:/home/edhitha/DCIM/test_cam/cam_wps.pickle "  # Change this
LOCAL_PATH = "/Users/aahil/Edhitha/edhithaGCS-main-UI-server/Data/Test/cam_wps.pickle"
CSV_PATH = "/Users/aahil/Edhitha/edhithaGCS-main-UI-server/Data/Test/"

def rsync_pickle():
    try:
        subprocess.run(["rsync", "-avz", REMOTE_PATH, LOCAL_PATH], check=True)
    except subprocess.CalledProcessError as e:
        print(f"[!] Rsync failed: {e}",flush=True)

def load_pickle():
    if not os.path.exists(LOCAL_PATH):
        return []
    try:
        with open(LOCAL_PATH, "rb") as f:
            return pickle.load(f)
    except Exception as e:
        print(f"[!] Failed to load pickle: {e}")
        return []

def convert_to_csv(data):
    if not data:
        return

    rows = []
    for idx, item in enumerate(data):
        file_id = f"image{idx:03d}"

        # assuming item is a tuple
        try:
            lat, lon, alt, head, yaw, timestamp = item
        except ValueError:
            print(f"[!] Skipping malformed item: {item}")
            continue

        row = {
            "file": file_id,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "head": head,
            "yaw": yaw,
            "time": timestamp
        }
        rows.append(row)

    try:
        with open(os.path.join(CSV_PATH, "results.csv"), "w", newline="") as csvfile:
            fieldnames = ["file", "lat", "lon", "alt", "head", "yaw", "time"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"[✓] CSV written with {len(rows)} rows.", flush=True)
    except Exception as e:
        print(f"[!] Error writing CSV: {e}")

def main():
    print("[*] Starting sync and convert loop...", flush=True)
    while True:
        rsync_pickle()
        data = load_pickle()
        convert_to_csv(data)
        time.sleep(1)

signal.signal(signal.SIGTERM, signal_handler)


def move_and_create_unique_folder(folder_name, l_path):
    # Get a list of all existing folders in the current directory
    # print(os.listdir(folder_name))
    existing_folders = [folder for folder in os.listdir(
        folder_name) if os.path.isdir(os.path.join(folder_name, folder))]

    print(existing_folders)
    # Initialize a counter for postfixing the folder name
    counter = 1

    # Generate a new folder name with a postfixed number
    new_folder_name = f"{folder_name}_{counter}"
    new_name = os.path.basename(l_path)
    print(new_name)
    # Keep incrementing the counter until a unique folder name is found
    while new_name in existing_folders:
        counter += 1
        new_folder_name = f"{l_path}_{counter}"
        new_name = os.path.basename(new_folder_name)
    # Create the new folder
    print(new_folder_name)
    os.makedirs(os.path.join(new_folder_name))

    print(f"Created a new folder: {new_folder_name}")

    # Check if the original folder exists
    if os.path.exists(l_path):
        # Move the contents of the original folder to the new one
        for item in os.listdir(l_path):
            item_path = os.path.join(l_path, item)
            shutil.move(item_path, new_folder_name)

        print(f"Moved contents of {l_path} to {new_folder_name}")




def parse_string_to_variables(input_string):
    # Use regular expression to extract values from the string
    match = re.match(
        r"\$' (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) '\$", input_string)

    if match:
        # Extract the matched groups
        file, lat, lon, alt, head, yaw, time = match.groups()

        # Convert numerical values to appropriate types
        lat, lon, alt, head, yaw, time = map(
            float, (lat, lon, alt, head, yaw, time))

        return file, lat, lon, alt, head, yaw, time
    else:
        return None


def path_map(waypoints, l_path):
    # Check if waypoints list is empty
    if not waypoints:
        # print("Waypoints list is empty. Cannot create map.")
        return

    # Create a folium map centered at the first waypoint
    m = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=6)

    # Add waypoints and custom arrow markers
    for lat, lon, alt, heading, yaw, time1 in waypoints:
        # Create a custom arrow marker
        icon = DivIcon(
            icon_size=(20, 20),
            icon_anchor=(10, 10),
            html=f'<div style="transform: rotate({
                yaw}deg); color: red;">&#11015;</div>',
        )

        folium.Marker(
            [lat, lon],
            icon=icon,
            popup=f"Time: {time1}<br>Altitude: {alt}"
        ).add_to(m)

    # Save the map to an HTML file
    m.save(os.path.join(l_path, 'waypoints_map.html'))
    print('Check out the updated HTML file for path review...')


def ssh_and_run_commands(hostname, username, password, commands, l_path, cam_waypoints):
    # Create an SSH client
    # logging.getLogger().status("SSH'ing")
    ssh_client = paramiko.SSHClient()
    # Automatically add the server's host key (this is insecure and should be done with caution)
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    results_columns = ['file', 'lat', 'lon', 'alt', 'head', 'yaw', 'time']
    results_df = pandas.DataFrame(columns=results_columns)
    # Get the current time and date

    # Create the filename with the current time and date
    results_csv_file = os.path.join(l_path, f'results.csv')

    try:
        # Connect to the remote device
        ssh_client.connect(hostname, username=username, password=password)
        print('Connected to host ...')
        # Initialize a shell
        ssh_shell = ssh_client.invoke_shell()

        # Wait for the shell to be ready
        while not ssh_shell.recv_ready():
            pass

        # Send commands
        for command in commands:
            ssh_shell.send(command + '\n')
            # Wait for the command execution to complete
            while not ssh_shell.recv_ready():
                pass
            # Read and capture the output
            output = ssh_shell.recv(65535).decode('utf-8')
            
            print(output)  # Print the output of the command

        print('Ran all the commands ..')

        while True:
            # Read and capture the output
            output = ssh_shell.recv(65535).decode('utf-8')
            # print(output)
            # Extract lines starting with '$' and ending with '$'
            captured_lines = re.findall(r'\$.*?\$', output, re.DOTALL)
            prev_file = None
            # Print the captured lines
            for line in captured_lines:
                file, lat, lon, alt, head, yaw, time = parse_string_to_variables(
                    line)
                if file != prev_file:
                    # Print the extracted values
                    print("File:", file, "Latitude:", lat, "Longitude:", lon,
                          "Altitude:", alt, "Head:", head, "Yaw:", yaw, "Time:", time)
                    # print(line.strip())

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

                    results_df.to_csv(results_csv_file)
                if prev_file == None:
                    sys.stdout.write(line)
            sys.stdout.write(output)
            # if cam_waypoints:
            #     path_map(cam_waypoints, l_path)

    except Exception as e:
        print(f"Error: {e}")
        path_map(cam_waypoints, l_path)


if __name__ == '__main__':
    # Replace these values with your actual remote device details
    while (True):
        hostname = '192.168.144.50'
        username = 'edhitha'
        password = 'password'
        commands_to_run = [
            'cd dino',
            'cd AUTOMATION',
            'python3 modified.py'
            # Add more commands as needed
        ]
        # Folder Paths
        l_path = "/Users/aahil/Edhitha/edhithaGCS-main-UI-server/Data/Test"
        os.makedirs(l_path, exist_ok=True)
        # Parent folder to save there
        parent_path = '/Users/aahil/Edhitha/edhithaGCS-main-UI-server/Data/'
        move_and_create_unique_folder(parent_path, l_path)
        os.makedirs(l_path+'/images', exist_ok=True)
        print("made images folder")
        os.makedirs(l_path+'/cropped', exist_ok=True)
        print("made cropped folder")

        # Global variables
        cam_waypoints = []

        # Run the SSH and command execution
        # ssh_and_run_commands(hostname, username, password,
        #                      commands_to_run, l_path, cam_waypoints)
        main()
        pass