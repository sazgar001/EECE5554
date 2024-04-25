import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv('/home/sakib/punching_bag/data/sakibPunch_parsed.csv')

# Calculate the magnitude of angular velocity and linear acceleration
data['Angular Velocity Magnitude'] = np.sqrt(data['Angular Velocity X']**2 + data['Angular Velocity Y']**2 + data['Angular Velocity Z']**2)
data['Linear Acceleration Magnitude'] = np.sqrt(data['Linear Acceleration X']**2 + data['Linear Acceleration Y']**2 + data['Linear Acceleration Z']**2) - 9.81  # Assuming gravity correction

# Smooth the angular velocity and linear acceleration using a rolling window
window_size = 5  # Example window size
data['Angular Velocity Magnitude Smoothed'] = data['Angular Velocity Magnitude'].rolling(window=window_size, center=True).mean().fillna(method='bfill').fillna(method='ffill')
data['Linear Acceleration Magnitude Smoothed'] = data['Linear Acceleration Magnitude'].rolling(window=window_size, center=True).mean().fillna(method='bfill').fillna(method='ffill')

# Calculate Forward Velocity
# Assuming linear acceleration in the Z direction is 'forward' and initial velocity is zero
time_diffs = np.diff(data['Timestamp'])  # Assuming 'Timestamp' is in seconds
forward_acceleration = data['Linear Acceleration Z'].values[:-1]  # Ignoring the last value for matching diff lengths
forward_velocity = np.cumsum(forward_acceleration * time_diffs)  # Simple integration
forward_velocity = np.insert(forward_velocity, 0, 0)  # Adding initial velocity of 0 at the start
data['Forward Velocity'] = forward_velocity
# Plotting the specified graphs
fig, axs = plt.subplots(4, 1, figsize=(15, 20))

# Magnetometer Readings vs. Time
axs[0].plot(data['Timestamp'], data['Magnetic Field X'], label='Magnetic Field X', color='magenta')
axs[0].plot(data['Timestamp'], data['Magnetic Field Y'], label='Magnetic Field Y', color='orange')
axs[0].plot(data['Timestamp'], data['Magnetic Field Z'], label='Magnetic Field Z', color='cyan')
axs[0].set_title('Magnetometer Readings vs. Time')
axs[0].set_xlabel('Time (seconds)')
axs[0].set_ylabel('Magnetic Field (gauss)')
axs[0].legend()

# Linear Acceleration Magnitude vs. Time (Original and Smoothed)
axs[1].plot(data['Timestamp'], data['Linear Acceleration Magnitude'], label='Original', color='blue', alpha=0.5)
axs[1].plot(data['Timestamp'], data['Linear Acceleration Magnitude Smoothed'], label='Smoothed', color='blue', linestyle='--')
axs[1].set_title('Linear Acceleration Magnitude vs. Time')
axs[1].set_xlabel('Time (seconds)')
axs[1].set_ylabel('Acceleration (m/s²)')
axs[1].legend()

# Angular Velocity Magnitude vs. Time (Original and Smoothed)
axs[2].plot(data['Timestamp'], data['Angular Velocity Magnitude'], label='Original', color='red', alpha=0.5)
axs[2].plot(data['Timestamp'], data['Angular Velocity Magnitude Smoothed'], label='Smoothed', color='red', linestyle='--')
axs[2].set_title('Angular Velocity Magnitude vs. Time')
axs[2].set_xlabel('Time (seconds)')
axs[2].set_ylabel('Angular Velocity (rad/s)')
axs[2].legend()

# Forward Velocity vs. Time
axs[3].plot(data['Timestamp'], data['Forward Velocity'], label='Forward Velocity', color='green')
axs[3].set_title('Forward Velocity vs. Time')
axs[3].set_xlabel('Time (seconds)')
axs[3].set_ylabel('Velocity (m/s)')
axs[3].legend()

plt.tight_layout()
plt.show()
