import os
import pandas as pd
from PIL import Image
import piexif

def convert_to_rational(number):
    """Convert a float to EXIF rational format."""
    from fractions import Fraction
    fraction = Fraction(number).limit_denominator(1000000)
    return (fraction.numerator, fraction.denominator)

def write_gps_to_image(image_path, lat, lon, output_folder):
    """Write GPS data to image EXIF and save it in a new folder."""
    image = Image.open(image_path)

    # Check if the image has existing EXIF data
    exif_data = image.info.get('exif')
    if exif_data:
        exif_dict = piexif.load(exif_data)
    else:
        exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "Interop": {}}

    gps_ifd = {
        2: convert_to_rational(abs(lat)),  # GPSLatitude
        1: 'N' if lat >= 0 else 'S',        # GPSLatitudeRef
        4: convert_to_rational(abs(lon)),  # GPSLongitude
        3: 'E' if lon >= 0 else 'W',        # GPSLongitudeRef
    }

    exif_dict['GPS'] = gps_ifd

    # Convert back to bytes and save the image in the new folder
    exif_bytes = piexif.dump(exif_dict)

    # Create the output image path
    output_image_path = os.path.join(output_folder, os.path.basename(image_path))
    image.save(output_image_path, exif=exif_bytes)

# Read the CSV file
csv_file_path = '/Users/aahil/Edhitha/Mapping/output.csv'  # Update with your CSV file path
image_folder_path = '/Users/aahil/Downloads/mapImages'  # Update with your image folder path
output_folder_path = '/Users/aahil/Edhitha/Mapping/images'  # Update with your desired output folder path

# Create the output folder if it doesn't exist
os.makedirs(output_folder_path, exist_ok=True)

data = pd.read_csv(csv_file_path)

# Loop through each row in the CSV
for index, row in data.iterrows():
    file_name = row['file']
    lat = row['lat']
    lon = row['lon']
    
    # Construct the full image path
    image_path = os.path.join(image_folder_path, file_name)
    
    if os.path.exists(image_path):
        write_gps_to_image(image_path, lat, lon, output_folder_path)
        print(f'Updated GPS data for {file_name} and saved to {output_folder_path}')
    else:
        print(f'Image not found: {file_name}')
