import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load the datasets
square_data = pd.read_csv('~/imu/data/square_carter.csv')

# Normalize the timestamps to seconds relative to the start
square_data['Time'] = square_data['Timestamp'] - square_data['Timestamp'].iloc[0]

# Calculate cumulative rotation by integrating angular velocity
square_rotation_x = np.cumsum(square_data['Angular Velocity X'] * np.diff(square_data['Time'], prepend=0))
square_rotation_y = np.cumsum(square_data['Angular Velocity Y'] * np.diff(square_data['Time'], prepend=0))
square_rotation_z = np.cumsum(square_data['Angular Velocity Z'] * np.diff(square_data['Time'], prepend=0))

# Calculate magnetometer heading in degrees
square_heading = np.rad2deg(np.arctan2(square_data['Magnetic Field Y'], square_data['Magnetic Field X']))

# Figure 2: Rotational rate (Angular Velocity) for Square Data
fig2, axs2 = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
# X-axis
axs2[0].plot(square_data['Time'], square_data['Angular Velocity X'], label='X-axis', color='blue')
axs2[0].set_ylabel('Angular Velocity (rad/s)')
axs2[0].set_title('Square Rotational Rate - X-axis')
axs2[0].grid(True)
# Y-axis
axs2[1].plot(square_data['Time'], square_data['Angular Velocity Y'], label='Y-axis', color='orange')
axs2[1].set_ylabel('Angular Velocity (rad/s)')
axs2[1].set_title('Square Rotational Rate - Y-axis')
axs2[1].grid(True)
# Z-axis
axs2[2].plot(square_data['Time'], square_data['Angular Velocity Z'], label='Z-axis', color='green')
axs2[2].set_xlabel('Time (s)')
axs2[2].set_ylabel('Angular Velocity (rad/s)')
axs2[2].set_title('Sqaure Rotational Rate - Z-axis')
axs2[2].grid(True)
# Legend
for ax in axs2:
    ax.legend()
plt.tight_layout()
plt.show()

# Figure 3: Cumulative Rotation (integrated Angular Velocity) for Circle Data
fig3, axs3 = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
# X-axis
axs3[0].plot(square_data['Time'], square_rotation_x, label='X-axis', color='blue')
axs3[0].set_ylabel('Cumulative Rotation (rad)')
axs3[0].set_title('Square Cumulative Rotation - X-axis')
axs3[0].grid(True)
# Y-axis
axs3[1].plot(square_data['Time'], square_rotation_y, label='Y-axis', color='orange')
axs3[1].set_ylabel('Cumulative Rotation (rad)')
axs3[1].set_title('Square Cumulative Rotation - Y-axis')
axs3[1].grid(True)
# Z-axis
axs3[2].plot(square_data['Time'], square_rotation_z, label='Z-axis', color='green')
axs3[2].set_xlabel('Time (s)')
axs3[2].set_ylabel('Cumulative Rotation (rad)')
axs3[2].set_title('Square Cumulative Rotation - Z-axis')
axs3[2].grid(True)
# Legend
for ax in axs3:
    ax.legend()
plt.tight_layout()
plt.show()

# Figure 4: Magnetometer Heading for Circle Data
fig4, ax4 = plt.subplots(figsize=(10, 5))
ax4.plot(square_data['Time'], square_heading, label='Heading', color='blue')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Heading (degrees)')
ax4.set_title('Square Magnetometer Heading')
ax4.legend()
ax4.grid(True)
plt.tight_layout()
plt.show()