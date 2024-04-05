from geopy.distance import geodesic
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Load the GPS data
gps_data_path = '~/lab5/src/data/data_driving_gps.csv'
gps_data = pd.read_csv(gps_data_path)

# Convert latitude and longitude to a tuple of (lat, lon) for each GPS point
gps_points = list(zip(gps_data['latitude'], gps_data['longitude']))

# Calculate the time differences between each GPS data point
time_diffs_gps = np.diff(gps_data['Time'].to_numpy())

# Initialize a list for the calculated speeds, starting with an initial speed of 0
gps_speeds = [0]

# Calculate the speed between consecutive GPS points
for i in range(1, len(gps_points)):
    distance = geodesic(gps_points[i-1], gps_points[i]).meters
    time_diff = time_diffs_gps[i-1] if i > 0 else 1
    speed = distance / time_diff
    gps_speeds.append(speed)

# Convert the speeds list to a numpy array for plotting
gps_speeds = np.array(gps_speeds)

# Plot the GPS velocity
plt.figure(figsize=(14, 7))
plt.plot(gps_data['Time'], gps_speeds, label='GPS Velocity', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('GPS Velocity Estimates')
plt.legend()
plt.grid(True)
plt.show()
