import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R

# Load the IMU and GPS data
imu_data = pd.read_csv('~/lab5/src/data/data_driving_imu.csv')
gps_data = pd.read_csv('~/lab5/src/data/data_driving_gps.csv')

# Common calculations for both plots
# Process IMU data
imu_data['theta'] = cumtrapz(imu_data['imu.angular_velocity.z'], imu_data['Time'], initial=0)
imu_data['corrected_acceleration'] = imu_data['imu.linear_acceleration.x'] - imu_data['imu.linear_acceleration.x'].mean()
imu_data['forward_velocity'] = cumtrapz(imu_data['corrected_acceleration'], imu_data['Time'], initial=0)
imu_data['ve'] = imu_data['forward_velocity'] * np.cos(imu_data['theta'])
imu_data['vn'] = imu_data['forward_velocity'] * np.sin(imu_data['theta'])
imu_data['xe'] = cumtrapz(imu_data['ve'], imu_data['Time'], initial=0)
imu_data['xn'] = cumtrapz(imu_data['vn'], imu_data['Time'], initial=0)

# Process GPS data
gps_data['easting'] = (gps_data['longitude'] - gps_data['longitude'].iloc[0]) * 111319.9
gps_data['northing'] = (gps_data['latitude'] - gps_data['latitude'].iloc[0]) * 110574

# Start creating subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 14))  # You may need to adjust the figsize

# First Subplot
ax1.plot(imu_data['xe'], imu_data['xn'], label='IMU Observed Non-Calibrated Trajectory', color='blue')
ax1.plot(gps_data['easting'], gps_data['northing'], label='GPS Trajectory', color='green')
ax1.set_xlabel('Easting (m)')
ax1.set_ylabel('Northing (m)')
ax1.set_title('IMU vs GPS Observed Trajectory')
ax1.axis('equal')
ax1.legend()
ax1.grid(True)

# Second Subplot
# Shift and rotate IMU data for the second subplot
imu_shifted = np.vstack((imu_data['xe'] - imu_data['xe'].iloc[0], -imu_data['xn'] - imu_data['xn'].iloc[0]))
rotation_matrix_45 = R.from_euler('z', np.pi/(19/35)).as_matrix()[:2, :2]
imu_rotated_45 = np.dot(rotation_matrix_45, imu_shifted)

ax2.scatter(0, 0, color='red', zorder=5, label='Start Point')  # Start point at (0,0)
ax2.plot(imu_rotated_45[0], imu_rotated_45[1], label='IMU Estimated Trajectory Calibrated', color='blue')
ax2.plot(gps_data['easting'] - gps_data['easting'].iloc[0], gps_data['northing'] - gps_data['northing'].iloc[0], 
         label='GPS Trajectory', color='green')
ax2.set_xlabel('Easting (m)')
ax2.set_ylabel('Northing (m)')
ax2.set_title('IMU and GPS Trajectories Northing vs Easting Plots')
ax2.axis('equal')
ax2.legend()
ax2.grid(True)

# Adjust layout
plt.tight_layout()
plt.show()
