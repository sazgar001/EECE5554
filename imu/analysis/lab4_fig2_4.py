import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load the datasets
circle_data = pd.read_csv('~/imu/data/circle_carter.csv')

# Normalize the timestamps to seconds relative to the start
circle_data['Time'] = circle_data['Timestamp'] - circle_data['Timestamp'].iloc[0]

# Calculate cumulative rotation by integrating angular velocity
circle_rotation_x = np.cumsum(circle_data['Angular Velocity X'] * np.diff(circle_data['Time'], prepend=0))
circle_rotation_y = np.cumsum(circle_data['Angular Velocity Y'] * np.diff(circle_data['Time'], prepend=0))
circle_rotation_z = np.cumsum(circle_data['Angular Velocity Z'] * np.diff(circle_data['Time'], prepend=0))

# Calculate magnetometer heading in degrees
circle_heading = np.rad2deg(np.arctan2(circle_data['Magnetic Field Y'], circle_data['Magnetic Field X']))

# Figure 2: Rotational rate (Angular Velocity) for Circle Data
fig2, axs2 = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
# X-axis
axs2[0].plot(circle_data['Time'], circle_data['Angular Velocity X'], label='X-axis', color='blue')
axs2[0].set_ylabel('Angular Velocity (rad/s)')
axs2[0].set_title('Circle Rotational Rate - X-axis')
axs2[0].grid(True)
# Y-axis
axs2[1].plot(circle_data['Time'], circle_data['Angular Velocity Y'], label='Y-axis', color='orange')
axs2[1].set_ylabel('Angular Velocity (rad/s)')
axs2[1].set_title('Circle Rotational Rate - Y-axis')
axs2[1].grid(True)
# Z-axis
axs2[2].plot(circle_data['Time'], circle_data['Angular Velocity Z'], label='Z-axis', color='green')
axs2[2].set_xlabel('Time (s)')
axs2[2].set_ylabel('Angular Velocity (rad/s)')
axs2[2].set_title('Circle Rotational Rate - Z-axis')
axs2[2].grid(True)
# Legend
for ax in axs2:
    ax.legend()
plt.tight_layout()
plt.show()

# Figure 3: Cumulative Rotation (integrated Angular Velocity) for Circle Data
fig3, axs3 = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
# X-axis
axs3[0].plot(circle_data['Time'], circle_rotation_x, label='X-axis', color='blue')
axs3[0].set_ylabel('Cumulative Rotation (rad)')
axs3[0].set_title('Circle Cumulative Rotation - X-axis')
axs3[0].grid(True)
# Y-axis
axs3[1].plot(circle_data['Time'], circle_rotation_y, label='Y-axis', color='orange')
axs3[1].set_ylabel('Cumulative Rotation (rad)')
axs3[1].set_title('Circle Cumulative Rotation - Y-axis')
axs3[1].grid(True)
# Z-axis
axs3[2].plot(circle_data['Time'], circle_rotation_z, label='Z-axis', color='green')
axs3[2].set_xlabel('Time (s)')
axs3[2].set_ylabel('Cumulative Rotation (rad)')
axs3[2].set_title('Circle Cumulative Rotation - Z-axis')
axs3[2].grid(True)
# Legend
for ax in axs3:
    ax.legend()
plt.tight_layout()
plt.show()

# Figure 4: Magnetometer Heading for Circle Data
fig4, ax4 = plt.subplots(figsize=(10, 5))
ax4.plot(circle_data['Time'], circle_heading, label='Heading', color='blue')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Heading (degrees)')
ax4.set_title('Circle Magnetometer Heading')
ax4.legend()
ax4.grid(True)
plt.tight_layout()
plt.show()