import pickle, csv

with open("cam_wps.pickle", "rb") as f:
    data = pickle.load(f)

with open("cam_wps.csv", "w", newline="") as csvfile:
    if isinstance(data, list) and isinstance(data[0], dict):
        writer = csv.DictWriter(csvfile, fieldnames=data[0].keys())
        writer.writeheader()
        writer.writerows(data)
    else:
        # fallback if it's not a list of dicts
        writer = csv.writer(csvfile)
        for row in data:
            writer.writerow(row)
