import pandas as pd
import numpy as np
from scipy.signal import filtfilt, butter
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# Load IMU data from the uploaded CSV file
df = pd.read_csv('~/lab5/src/data/data_driving_imu.csv')

# Extract necessary data
time = df['Time'].values
gyro_data = df[['imu.angular_velocity.x', 'imu.angular_velocity.y', 'imu.angular_velocity.z']].values
mag_data = df[['mag_field.magnetic_field.x', 'mag_field.magnetic_field.y', 'mag_field.magnetic_field.z']].values
quaternions = df[['imu.orientation.x', 'imu.orientation.y', 'imu.orientation.z', 'imu.orientation.w']].values

# Convert quaternions to Euler angles
euler_angles = R.from_quat(quaternions).as_euler('xyz', degrees=False)
imu_heading_estimate = euler_angles[:, 2]  # Assuming z is the yaw

# Filter settings
sampling_freq = 1 / np.mean(np.diff(time))
nyquist_freq = 0.5 * sampling_freq
lowpass_cutoff = 0.05
highpass_cutoff = 0.01

# Low pass filter for magnetometer
b, a = butter(1, lowpass_cutoff / nyquist_freq, btype='low')
mag_yaw_angle_filtered = filtfilt(b, a, np.arctan2(mag_data[:, 1], mag_data[:, 0]))

# High pass filter for gyroscope
b, a = butter(1, highpass_cutoff / nyquist_freq, btype='high')
gyro_data_filtered = filtfilt(b, a, gyro_data, axis=0)
gyro_yaw_angle_filtered = gyro_data_filtered[:, 2]  # Assuming z is the yaw

# Integrate gyro data to get the yaw angle
dt = np.mean(np.diff(time))
gyro_yaw_angle_offset = np.cumsum(gyro_yaw_angle_filtered) * dt

# Complementary filter
alpha = 0.01
complementary_filtered_yaw = alpha * gyro_yaw_angle_offset + (1 - alpha) * mag_yaw_angle_filtered

# Adjust the time array to start from zero
time_adjusted = time - time[0]

# Plotting
plt.figure(figsize=(14, 7))
plt.plot(time_adjusted, mag_yaw_angle_filtered, 'g--', label='Mag only - low pass filter')
plt.plot(time_adjusted, gyro_yaw_angle_offset, 'r', label='Gyro only - raw data')
plt.plot(time_adjusted, complementary_filtered_yaw, 'b--', label='Complementary filter alpha = 0.01')
plt.plot(time_adjusted, imu_heading_estimate, 'gray', label='Yaw from IMU (IMU Heading Estimate)')
plt.title('Heading Estimation (Yaw Rotation with Filters)')
plt.xlabel('Time (s)')
plt.ylabel('Rotation (rad)')
plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=3)
plt.grid(True)
plt.show()
