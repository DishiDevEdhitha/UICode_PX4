import math

def haversine_distance(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Haversine formula
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Radius of Earth at given latitude
    radius_of_earth = earth_radius_at_latitude(lat1)
    
    # Distance in meters
    distance = radius_of_earth * c
    return distance

def earth_radius_at_latitude(latitude):
    # WGS-84 ellipsoid parameters
    a = 6378137.0  # Equatorial radius in meters
    b = 6356752.3  # Polar radius in meters
    
    # Convert latitude to radians
    lat_rad = math.radians(latitude)
    
    # Radius of Earth at given latitude using the formula for an ellipsoid
    numerator = (a**2 * math.cos(lat_rad))**2 + (b**2 * math.sin(lat_rad))**2
    denominator = (a * math.cos(lat_rad))**2 + (b * math.sin(lat_rad))**2
    radius = math.sqrt(numerator / denominator)
    return radius

# Example usage:
# lat1, lon1 = 34.0522, -118.2437  # Los Angeles
# lat2, lon2 = 40.7128, -74.0060   # New York
# distance = haversine_distance(lat1, lon1, lat2, lon2)

# print(f"The distance between the points is approximately {distance:.2f} meters.")
