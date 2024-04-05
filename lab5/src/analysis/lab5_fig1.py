import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import medfilt

# Load your driving data CSV
data_driving_imu = pd.read_csv('~/lab5/src/data/data_driving_imu.csv')  # Update with the correct path

# Conversion factor from Tesla to milliGauss
tesla_to_mgauss = 10000000

# Extract magnetometer data
mag_x = data_driving_imu['mag_field.magnetic_field.x'].values * tesla_to_mgauss
mag_y = data_driving_imu['mag_field.magnetic_field.y'].values * tesla_to_mgauss

# Hard Iron Correction
offset_x = np.mean(mag_x)
offset_y = np.mean(mag_y)
corrected_x = mag_x - offset_x
corrected_y = mag_y - offset_y

# Soft Iron Correction
scale_x = (np.max(corrected_x) - np.min(corrected_x)) / 2
scale_y = (np.max(corrected_y) - np.min(corrected_y)) / 2
avg_scale = (scale_x + scale_y) / 2
corrected_x /= (scale_x / avg_scale)
corrected_y /= (scale_y / avg_scale)

# Compute yaw from magnetometer data
raw_yaw = np.arctan2(mag_y, mag_x)
corrected_yaw = np.arctan2(corrected_y, corrected_x)

# Normalize the time series data
time = data_driving_imu['Time'] - data_driving_imu['Time'].iloc[0]

# Wrap raw_yaw and corrected_yaw to the interval [-pi, pi]
raw_yaw_wrapped = np.arctan2(np.sin(raw_yaw), np.cos(raw_yaw))
corrected_yaw_wrapped = np.arctan2(np.sin(corrected_yaw), np.cos(corrected_yaw))

# Convert radians to degrees
raw_yaw_degrees = np.degrees(raw_yaw_wrapped)
corrected_yaw_degrees = np.degrees(corrected_yaw_wrapped)

# Apply median filter to the corrected yaw to remove spikes
corrected_yaw_filtered = medfilt(corrected_yaw_degrees, kernel_size=5)

# Set up the plot
plt.figure(figsize=(14, 7))

# Plot the raw and corrected yaw over time with the desired style
plt.plot(time, raw_yaw_degrees, label='Raw', color='blue', linestyle='-')
plt.plot(time, corrected_yaw_filtered, label='Corrected', color='orange', linestyle='-')

# Setting the title and labels
plt.title('Yaw Magnetometer Rotation - Driving')
plt.xlabel('Time (s)')
plt.ylabel('Rotation (deg)')  # The user's graph has degrees, not radians.
plt.xlim([0, 700]) # Setting the x-axis limits to 0-700 seconds as requested
plt.ylim([-180, 180]) # Setting the y-axis limits to match the provided graph's apparent limit
plt.legend() # Adding a legend to match the user's graph style
plt.grid(True)
plt.show()
