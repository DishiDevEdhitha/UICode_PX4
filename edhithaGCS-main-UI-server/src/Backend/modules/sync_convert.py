import subprocess
import time
import pickle
import csv
import os


 # Change this
LOCAL_PATH = "/home/edhitha/DCIM/test_cam/cam_wps.pickle"
CSV_PATH = "/home/edhitha/edhithaGCS-main-UI-server/Data/Test/"

# def rsync_pickle():
#     try:
#         subprocess.run(["rsync", "-avz", REMOTE_PATH, LOCAL_PATH], check=True)
#     except subprocess.CalledProcessError as e:
#         print(f"[!] Rsync failed: {e}",flush=True)

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
        # rsync_pickle()
        data = load_pickle()
        convert_to_csv(data)
        time.sleep(1)

if __name__ == "__main__":
    # logging.getLogger().status(f"LESSSGOOO")
    print("WOOOW")
    main()
