import subprocess
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
import os
import cv2
import signal
import sys
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from pymavlink import mavutil
from modules.mavlink_commands import *
from modules.latlon import *
from modules.automation import *
from modules.drone_status import drone_state
from modules.logger_config import *
from flask_socketio import SocketIO, emit
from threading import Thread



app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# # Set the SocketIO instance for logging
# set_socketio_instance(socketio)

# Setup 
if __name__ == '__main__':
    setup_logging(socketio) 

@socketio.on('connect')
def handle_connect():
    logging.getLogger().status('Logger connected')

@socketio.on('disconnect')
def handle_disconnect():
    reason = request.args.get('reason', 'unknown')  # Retrieve the reason for disconnection
    logging.getLogger().status(f'Logger disconnected. Reason: {reason}')



CORS(app)


# Directories and File paths
parent_folder = "../../Data/Test"
IMAGE_DIRECTORY = "/home/dishita/Desktop/UI_PX4/images"
CROPPED_IMAGE_DIRECTORY = parent_folder+"/cropped"
DATA_FILE = '../../Data/details.txt'
csv_path = parent_folder+'/results.csv'
pickle_path = parent_folder+'/cam_wps.pickle'
geo_log_path = parent_folder+"/geo_log.txt"
geo_path = "/home/dishita/Desktop/UI_PX4/Geotag_Scripts/geotag_px4.py"
convert = "/home/dishita/Desktop/UI_PX4/edhithaGCS-main-UI-server/src/Backend/modules/sync_convert.py"
# geo_path = "./modules/geotag_UI.py"

# Global variables
global xco, yco, id, lats, longs, connection, process
global_count = 0
connection = None
is_connected = False
the_connection = None
lats = ["null", "null", "null", "null", "null"]
longs = ["null", "null", "null", "null", "null"]

# Ensure directories exist
if not os.path.exists(CROPPED_IMAGE_DIRECTORY):
    os.makedirs(CROPPED_IMAGE_DIRECTORY, exist_ok=True)

if not os.path.exists(IMAGE_DIRECTORY):
    os.makedirs(IMAGE_DIRECTORY, exist_ok=True)

class ImageEventHandler(FileSystemEventHandler):
    """Handles file system events for the image directory."""
    
    def on_created(self, event):
        if event.is_directory:
            return
        self.emit_image_update()

    def on_deleted(self, event):
        if event.is_directory:
            return
        self.emit_image_update()

    def on_modified(self, event):
        if event.is_directory:
            return
        self.emit_image_update()

    def emit_image_update(self):
        """Emit updated image URLs to connected clients."""
        images = os.listdir(IMAGE_DIRECTORY)
        socketio.emit('image_update', {'imageUrls': images})

def start_watching():
    """Start watching the image directory for changes."""
    event_handler = ImageEventHandler()
    observer = Observer()
    observer.schedule(event_handler, IMAGE_DIRECTORY, recursive=False)
    observer.start()
    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        observer.stop()
    observer.join()


def run_sync_script():
    script_path = "/home/edhitha/edhithaGCS-main-UI-server/sync_jetson.sh"
    global syncprocess
    try:
        syncprocess = subprocess.Popen(
            ["/bin/bash", script_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # Runs in a new process group
        )
        logging.info(f"Sync script started in the background with PID {process.pid}.")
        return syncprocess  # Store this in your app to terminate later if needed

    except Exception as e:
        logging.error("Unexpected error in sync script:\n" + str(e))
        return None

def stop_sync_script():
    if syncprocess:
        os.killpg(os.getpgid(syncprocess.pid), signal.SIGTERM)
        logging.getLogger().status("Sync stopped successfully.")
        logging.info(f"Sync script with PID {syncprocess.pid} terminated.")

def crop_image(image_path, x, y):
    global global_count, xco, yco
    image = cv2.imread(image_path)

    xco = int(x)
    yco = int(y)

    height, width, _ = image.shape
    crop_size = 100

    x_start = max(xco - crop_size, 0)
    x_end = min(xco + crop_size, width)
    y_start = max(yco - crop_size, 0)
    y_end = min(yco + crop_size, height)

    crop = image[y_start:y_end, x_start:x_end]

    
    logging.getLogger().status(f'Received coordinates: x={xco}, y={yco}')
    logging.getLogger().status('Cropping Image...')
    cropped_image_filename = f'cropped_image_{global_count}.png'
    cropped_image_path = os.path.join(
        CROPPED_IMAGE_DIRECTORY, cropped_image_filename)
    cv2.imwrite(cropped_image_path, crop)

    global_count += 1

    return cropped_image_filename


def run_python_file(file_path):
    # Open the output file in append mode to preserve existing content
    with open(geo_log_path, "a") as f:
        # Start the subprocess with stderr redirected to a pipe
        process = subprocess.Popen(["python", file_path], stdout=f)
    return process

def stop_python_file():
    global pyprocess
    try:
        os.kill(pyprocess.pid, signal.SIGTERM)  # Send SIGTERM signal to the process
        logging.getLogger().status("Python file stopped successfully.")
        time.sleep(5)
        # stop_sync_script()

    except ProcessLookupError:
        logging.getLogger().status("Process not found. It may have already stopped.")


@app.route('/all-images', methods=['GET'])
def all_images():
    with os.scandir(IMAGE_DIRECTORY) as entries:
        # Filter to include only files (skip directories) and sort by name if desired
        images = [entry.name for entry in entries if entry.is_file()]
        images.sort()
    return jsonify({'imageUrls': images})


@app.route('/images/<filename>', methods=['GET'])
def get_image(filename):
    return send_from_directory(IMAGE_DIRECTORY, filename)


@app.route('/crop-image', methods=['POST'])
def crop_image_endpoint():
    data = request.json
    image_url = data['imageUrl']
    x = int(data['x'])
    y = int(data['y'])

    image_path = os.path.join(IMAGE_DIRECTORY, image_url)
    cropped_image_filename = crop_image(image_path, x, y)
    cropped_image_url = f'http://192.168.144.50:9080/cropped-images/{cropped_image_filename}'

    return jsonify({'croppedImageUrl': cropped_image_url})


@app.route('/cropped-images/<filename>', methods=['GET'])
def get_cropped_image(filename):
    return send_from_directory(CROPPED_IMAGE_DIRECTORY, filename)


@app.route('/save-details', methods=['POST'])
def save_details():
    global xco, yco, id, lats, longs
    try:
        data = request.json
        image_url = data.get('selectedImageUrl')
        image_name = data.get('selectedImageUrl', '').split('/')[-1]
        cropped_name = data.get('croppedImageUrl', '').split('/')[-1]
        shape = data.get('shape', '')
        colour = data.get('colour', '')
        id = data.get('id', '')
        logging.getLogger().status(id)
        label = data.get('label', '')

        logging.getLogger().status(f"Received data: {image_name}, {shape}, {colour}, {id}, {label}")
        logging.getLogger().status("Saving Details...")
        # print(xco, yco, flush=True)
        lats, longs = lat_long_calculation(csv_path, image_url, image_name, xco, yco, id)
        latz, longz = lats[id-1], longs[id-1]
        # Write to file
        with open(DATA_FILE, 'a') as f:
            f.write(f'{image_name}: Lat={latz}, Long={longz}, id={id}, Label={label}\n')
            f.flush()
        logging.getLogger().status("Saved Succesfully.")
        return jsonify({'message': 'Details saved successfully', 'latitude': latz, 'longitude': longz}), 200
    except Exception as e:
        # print(f"Error saving details: {str(e)}", file=sys.stderr)
        logging.getLogger().status(f"Error saving detailsP: {str(e)}")
        return jsonify({'message': f'Failed to save details: {str(e)}'}), 500


# Endpoint for connecting to the drone
@app.route('/toggle-connection', methods=['POST'])
def connectDrone():
    global connection, is_connected
    connection, is_connected = toggle_connection()  # Capture the connection
    print("toggle connection clear on manual.py")
    if is_connected: 
        logging.getLogger().status("Connection Successfull...") # Check if connection is valid
        return jsonify({'message': 'Connected to the drone',
                        'system': connection.target_system,
                        'component': connection.target_component}), 200
    else:
        logging.getLogger().status(f"Failed to connect to the drone")  # Print the error message
        return jsonify({'message': 'Failed to connect to the drone'}), 500


@app.route('/arm-disarm', methods=['POST'])
def armdisarm():
    data = request.get_json()
    action = data.get('action')  # Get the action ('arm' or 'disarm')

    try:
        if action == 'arm':
            msgs = arm()  # Call the arm function
        elif action == 'disarm':
            msgs = disarm()  # Call the disarm function
        else:
            raise ValueError('Invalid action')

        return jsonify({'status': 'success', 'message': msgs}), 200

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/change-mode', methods=['POST'])
def change_mode():
    try:
        mode = request.json.get('mode')

        if mode == 'auto':
            msgs = auto()  # Call auto mode function
        elif mode == 'guided':
            msgs = guided()  # Call guided mode function
        elif mode == 'loiter':
            msgs = loiter()  # Call loiter mode function
        elif mode == 'stabilize':
            msgs = stabilize()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid mode'}), 400

        return jsonify({'status': 'success', 'message': msgs}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/takeoff', methods=['POST'])
def takeoffdrone():
    response = takeoffcommand()
    return response

@app.route('/RTL', methods=['POST'])
def rtl():
    response = rtlcommand()
    return response

@app.route('/drop', methods=['POST'])
def dropPkg(): 
    # Call the drop functions
    for i in range(4):
        drop(i+1)
        time.sleep(3)
    return "Drop command executed", 200  # Respond with a success message

@app.route('/lock-servo', methods=['POST'])
def lockservo():
    # Call the drop function
    lock(7)
    time.sleep(3)
    lock(8)
    return "Drop command executed", 200  # Respond with a success message


@app.route('/rtl', methods=['GET'])
def rtl_command():
    try:
        # Call the rtl function
        result = rtl()
        return jsonify({"status": "success", "message": result}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    

@app.route('/start_geotagg', methods=['POST'])
def start_geotagg():
    target_waypoint2 = 33
    geotag_waypoint = 21
    global pyprocess
    global geotag_process
    
    logging.getLogger().status(f"Waiting for drone to reach waypoint {geotag_waypoint}")

    while True:
            # Wait for a message
            msg = connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            logging.getLogger().status(f"Waypoint {msg.seq} reached.")
                # logging.getLogger().status(f"Waypoint reached.")s
                # logging.getLogger().status(f"Waypoint : {msg.seq}")
            if msg.seq == geotag_waypoint:
                try:
                    print("Changing directory")
                    geotag_process = subprocess.Popen(
                        ['./geotag.sh 2>&1 | tee /home/edhitha/edhithaGCS-main-UI-server/Data/Test/gtag_log.log'],
                        cwd='/home/edhitha/dino',
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        shell=True,
                        preexec_fn=os.setsid
                    )
                    # Start the geotag.sh script using Popen so it can be killed later

                    global msgs
                    msgs = "started geotagg"
                    logging.getLogger().status("Started geotagg")

                    # Optionally: let it run for a while before continuing
                    time.sleep(5)

                    logging.getLogger().status("Converting pickle file...")
                    pyprocess = run_python_file(convert)  # your sync Python script

                except Exception as e:
                    print(f"An error occurred: {e}")

            elif msg.seq == target_waypoint2:
                guided()               # Stop your sync Python f
               # Let the system run for a bit (or collect data)
                # Kill the geotag.sh script if it’s still running
                if geotag_process.poll() is None:  # still running
                    os.killpg(os.getpgid(geotag_process.pid), signal.SIGTERM)
                    logging.getLogger().status("Sent SIGTERM to geotag process")
                      # Run in background
                # stop_sync_script()
                stop_python_file()

                break

@app.route('/start_geotaggG', methods=['POST'])
def start_geotaggG():
    global pyprocess
    global geotag_process
    global msgs

    try:
        # Start the geotag.sh script using Popen so it can be killed later
        geotag_process = subprocess.Popen(
                        ['./geotag.sh'],
                        cwd='/home/edhitha/dino',
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        preexec_fn=os.setsid
                    )

        # gthread = threading.Thread(target=run_geotag)
        # gthread.start()

        global msgs
        msgs = "started geotagg"
        logging.getLogger().status("Started geotagg")

        # Optionally: let it run for a while before continuing
        # time.sleep(10)

        logging.getLogger().status("Converting Pickle...")
        pyprocess = run_python_file(convert)  # your sync Python script

    except Exception as e:
        print(f"An error occurred: {e}")
    # process = run_python_file(geo_path)  # This will run even if run_sync_script fails

    msgs = "started geotagg"
    # logging.getLogger().status("Started geotagg")  # Log this information
    # logging.getLogger().status("Running sync script...")
    return "", 204



@app.route('/stop_geotagg', methods=['POST'])
def stop_geotagg():
    global geotag_process
    logging.getLogger().status("NEW")
    if geotag_process.poll() is None:  # still running
        os.killpg(os.getpgid(geotag_process.pid), signal.SIGTERM)
        logging.getLogger().status("Sent SIGTERM to geotag process")
        stop_python_file()
    return jsonify({'status': 'Geotagging process terminated'})


@app.route('/reposition', methods=['POST'])
def repos():
    data = request.get_json()  # Extract the JSON payload
    if not data or 'id' not in data:
        return jsonify({"error": "Missing 'id' in request body"}), 400

    tno = data['id']  # Rename 'id' to 'tno'
    print(tno, flush=True)

    global connection
    # logging.getLogger().status("Moving towards target...", flush=True)
    global msg
    target_no = 0
    msg = "Target number bottle is dropping on: %s" % str(target_no)

    try:
        lati = lats[tno - 1]
        longi = longs[tno - 1]
        automation(lati, longi, tno, connection)
    except IndexError:
        return jsonify({"error": f"Invalid 'tno': {tno}"}), 400

    return "", 204

@app.route('/auto-repo', methods=['POST'])
def autorepos():

    initial_waypoint = True
    target_waypoint = 15
    global connection
    # logging.getLogger().status("Moving towards target...", flush=True)
    global msg
    target_no = 0
    msg = "Target number bottle is dropping on: %s" % str(target_no) # Replace with your desired waypoint sequence number
    for target in range(1,5):
        lati = lats[target - 1]
        longi = longs[target - 1]
        automation(lati, longi, target, connection)

        while True:
            # Wait for a message
            msg = connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            if msg:
                logging.getLogger().status(f" LOUDA Waypoint {msg.seq} reached.")
                # logging.getLogger().status(f"Waypoint : {msg.seq}")
                
                # Check if it's the target waypoint
                if msg.seq == target_waypoint:
                    break

@app.route('/dengu-repo', methods=['POST'])
def dengrepos():

    initial_waypoint = True
    target_waypoint = 15
    global connection
    # logging.getLogger().status("Moving towards target...", flush=True)
    global msg
    target_no = 0
    logging.getLogger().status(f"AH SHIT ITS FINE")
    dlats = [383157358, 383156768, 3831562, 383155611]
    dlongs = [-765522301, -765518841, -765515649, -765512484]
    msg = "Target number bottle is dropping on: %s" % str(target_no) # Replace with your desired waypoint sequence number
    for target in range(1,5):
        lati = dlats[target - 1]
        longi = dlongs[target - 1]
        automation(lati, longi, target, connection)

        while True:
            # Wait for a message
            msg = connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            if msg:
                logging.getLogger().status(f"L Waypoint {msg.seq} reached.")
                # logging.getLogger().status(f"Waypoint : {msg.seq}")

                # Check if it's the target waypoint
                if msg.seq == target_waypoint:
                    break

@app.route('/dengu-repo-2', methods=['POST'])
def dengrepos2():

    initial_waypoint = True
    target_waypoint = 15
    global connection
    # logging.getLogger().status("Moving towards target...", flush=True)
    global msg
    target_no = 0
    logging.getLogger().status(f"AH GOD ITS FINE")
    dlats = [383143994, 383144562, 383145131, 38314572]
    dlongs = [-765445107,-765448272,-765451598, -765454763]
    msg = "Target number bottle is dropping on: %s" % str(target_no) # Replace with your desired waypoint sequence number
    for target in range(1,5):
        lati = dlats[target - 1]
        longi = dlongs[target - 1]
        automation(lati, longi, target, connection)

        while True:
            # Wait for a message
            msg = connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            if msg:
                logging.getLogger().status(f"L Waypoint reached {msg.seq}.")
                # logging.getLogger().status(f"Waypoint : {msg.seq}")

                # Check if it's the target waypoint
                if msg.seq == target_waypoint:
                    break
               

if __name__ == '__main__':
    socketio.start_background_task(target=start_watching)
    socketio.run(app, host='0.0.0.0', debug=False, port=9080, allow_unsafe_werkzeug=True)
