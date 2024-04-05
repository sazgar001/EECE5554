import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the IMU data
imu_data = pd.read_csv('~/lab5/src/data/data_driving_imu.csv')

# Create subplots for yaw rate in both rad/s and degrees/s
fig, axs = plt.subplots(2, 1, figsize=(14, 14))

# Plot yaw rate in rad/s
axs[0].plot(imu_data['Time'], imu_data['imu.angular_velocity.z'], label='Yaw Rate (rad/s)', color='red')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Yaw Rate (rad/s)')
axs[0].set_title('Gyro Yaw Estimation in Rad/s vs. Time')
axs[0].legend()
axs[0].grid(True)

# Convert yaw rate from rad/s to degrees/s for the second plot
imu_data['imu.angular_velocity.degrees'] = np.degrees(imu_data['imu.angular_velocity.z'])

# Plot yaw rate in degrees/s
axs[1].plot(imu_data['Time'], imu_data['imu.angular_velocity.degrees'], label='Yaw Rate (degrees/s)', color='blue')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Yaw Rate (degrees/s)')
axs[1].set_title('Gyro Yaw Estimation in Deg/s vs. Time')
axs[1].legend()
axs[1].grid(True)

# Adjust layout
plt.tight_layout()
plt.show()
