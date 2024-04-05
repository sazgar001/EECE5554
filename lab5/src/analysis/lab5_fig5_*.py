import pandas as pd
import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

# Function to calculate velocity from GPS coordinates (using UTM easting and northing)
def calculate_gps_velocity(gps_data):
    # Calculate the difference in position
    delta_easting = np.diff(gps_data['utm_easting'])
    delta_northing = np.diff(gps_data['utm_northing'])
    
    # Calculate the distance traveled between each consecutive GPS reading
    distances = np.sqrt(delta_easting**2 + delta_northing**2)
    
    # Calculate the difference in time
    delta_time = np.diff(gps_data['Time'])
    
    # Avoid division by zero in case of duplicate timestamps
    delta_time[delta_time == 0] = np.nan
    
    # Calculate velocity (distance/time)
    velocities = distances / delta_time
    
    # Handle any NaNs by forward-filling the next valid value backward
    velocities = np.nan_to_num(velocities, nan=np.nanmean(velocities))
    
    # Prepend a 0 to make the velocity array the same length as the gps_data dataframe
    velocities = np.insert(velocities, 0, 0)
    
    return velocities

# Correct the file paths
imu_data_path = '~/lab5/src/data/data_driving_imu.csv'
gps_data_path = '~/lab5/src/data/data_driving_gps.csv'

# Load the IMU data
imu_data = pd.read_csv(imu_data_path)

# Load the GPS data
gps_data = pd.read_csv(gps_data_path)

# Adjust the Time data for both IMU and GPS to start from 0
imu_data['Time'] = imu_data['Time'] - imu_data['Time'].min()
gps_data['Time'] = gps_data['Time'] - gps_data['Time'].min()

# Calculate GPS velocity
gps_velocity = calculate_gps_velocity(gps_data)

# The "before correction" velocity estimate integrates the raw acceleration data directly
imu_velocity_before_correction = cumtrapz(imu_data['imu.linear_acceleration.x'], imu_data['Time'], initial=0)

# Plotting both IMU Velocity before correction and GPS velocity
plt.figure(figsize=(14, 7))

# GPS Velocity
plt.plot(gps_data['Time'], gps_velocity, label='GPS Velocity', color='green')

# IMU Velocity Before Correction
plt.plot(imu_data['Time'], imu_velocity_before_correction, label='IMU Velocity Before Correction', color='orange', linestyle='--')

plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('IMU and GPS Velocity Estimates')
plt.legend()
plt.grid(True)
plt.show()
