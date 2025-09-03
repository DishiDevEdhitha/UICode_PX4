import pandas as pd
import folium

# Load the CSV file into a DataFrame
df = pd.read_csv('/Users/aahil/Edhitha/edhithaGCS-main-UI-server/Data/Test/results.csv')

# Create a map centered around the average latitude and longitude
map_center = [df['lat'].mean(), df['lon'].mean()]
my_map = folium.Map(location=map_center, zoom_start=12)

# Add markers for each (lat, lon) point in the CSV
for index, row in df.iterrows():
    folium.Marker([row['lat'], row['lon']], popup=f"Time: {row['time']}").add_to(my_map)

# Save the map as an HTML file
my_map.save('/Users/aahil/Edhitha/edhithaGCS-main-UI-server/Data/Test')
