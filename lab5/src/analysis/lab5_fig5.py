import pandas as pd
import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

# Load the IMU data from the provided CSV file
imu_data_path = '~/lab5/src/data/data_driving_imu.csv'
imu_data = pd.read_csv(imu_data_path)

# Adjust the Time data to start from 0
imu_data['Time'] = imu_data['Time'] - imu_data['Time'].min()

# Calculate IMU velocity before correction by integrating the raw acceleration data
imu_velocity_before_correction = cumtrapz(imu_data['imu.linear_acceleration.x'], imu_data['Time'], initial=0)

# Correct the IMU data by subtracting the mean acceleration to reduce bias
corrected_acceleration = imu_data['imu.linear_acceleration.x'] - imu_data['imu.linear_acceleration.x'].mean()
imu_velocity_after_correction = cumtrapz(corrected_acceleration, imu_data['Time'], initial=0)

# Plotting IMU Velocity before and after correction on the same plot
plt.figure(figsize=(14, 7))

# IMU Velocity Before Correction
plt.plot(imu_data['Time'], imu_velocity_before_correction, label='Before Adjustment', color='orange', linestyle='--')

# IMU Velocity After Correction
plt.plot(imu_data['Time'], imu_velocity_after_correction, label='After Adjustment', color='blue')

plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Forward Velocity from IMU Accelerometer')
plt.legend()
plt.grid(True)
plt.show()