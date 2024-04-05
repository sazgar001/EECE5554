import pandas as pd
import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt

# Load IMU data from the uploaded CSV file
imu_data = pd.read_csv('~/lab5/src/data/data_driving_imu.csv')

# Extract the yaw rate (z-axis rotation rate) and timestamps from the IMU data
imu_time_secs = imu_data['header.stamp.secs'] + imu_data['header.stamp.nsecs'] * 1e-9
yaw_rate = imu_data['imu.angular_velocity.z']

# Calculate the relative time in seconds from the start
imu_time_relative = imu_time_secs - imu_time_secs.iloc[0]

# Integrate the yaw rate using the cumulative trapezoidal method to get the yaw angle in radians
yaw_angle = integrate.cumulative_trapezoid(yaw_rate, imu_time_relative, initial=0)

# Convert the yaw angle from radians to degrees
yaw_angle_degrees = np.degrees(yaw_angle)

# Normalize the yaw angle within the range of -180 to 180 degrees
yaw_angle_degrees_wrapped = (yaw_angle_degrees + 180) % 360 - 180

# Plot the yaw angle in degrees
plt.figure(figsize=(14, 7))
plt.plot(imu_time_relative, yaw_angle_degrees_wrapped)
plt.title('Yaw (Integrated from Gyro) Rotation - Driving')
plt.xlabel('Time (s)')
plt.ylabel('Rotation (deg)')
plt.grid(True)
plt.show()
