import pandas as pd
import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

# Load the CSV file
file_path = '~/imu/data/square_carter.csv'
data = pd.read_csv(file_path)

# Assuming the first column is 'Timestamp' and needs normalization
# Normalize time to start from 0 and spread over 180 seconds
time = data['Timestamp'] - data['Timestamp'].iloc[0]
time = np.linspace(0, 180, len(data))

# Extract acceleration data
ax = data['Linear Acceleration X']
ay = data['Linear Acceleration Y']
az = data['Linear Acceleration Z'] + 9.81  # Adjust for gravity

# Integrate acceleration to get velocity
vx = cumtrapz(ax, time, initial=0)
vy = cumtrapz(ay, time, initial=0)
vz = cumtrapz(az, time, initial=0)

# Integrate velocity to get displacement
dx = cumtrapz(vx, time, initial=0)
dy = cumtrapz(vy, time, initial=0)
dz = cumtrapz(vz, time, initial=0)

# Plotting each measurement in separate figures for clarity
# Acceleration
fig_acc, axs_acc = plt.subplots(3, 1, figsize=(10, 15))
axs_acc[0].plot(time, ax, 'r')
axs_acc[0].set_title('Acceleration in X vs. Time')
axs_acc[0].set_ylabel('Acceleration (m/s²)')
axs_acc[1].plot(time, ay, 'g')
axs_acc[1].set_title('Acceleration in Y vs. Time')
axs_acc[1].set_ylabel('Acceleration (m/s²)')
axs_acc[2].plot(time, az, 'b')
axs_acc[2].set_title('Acceleration in Z vs. Time')
axs_acc[2].set_xlabel('Time (s)')
axs_acc[2].set_ylabel('Acceleration (m/s²)')
fig_acc.tight_layout()

# Velocity
fig_vel, axs_vel = plt.subplots(3, 1, figsize=(10, 15))
axs_vel[0].plot(time, vx, 'r')
axs_vel[0].set_title('Velocity in X vs. Time')
axs_vel[0].set_ylabel('Velocity (m/s)')
axs_vel[1].plot(time, vy, 'g')
axs_vel[1].set_title('Velocity in Y vs. Time')
axs_vel[1].set_ylabel('Velocity (m/s)')
axs_vel[2].plot(time, vz, 'b')
axs_vel[2].set_title('Velocity in Z vs. Time')
axs_vel[2].set_xlabel('Time (s)')
axs_vel[2].set_ylabel('Velocity (m/s)')
fig_vel.tight_layout()

# Displacement
fig_disp, axs_disp = plt.subplots(3, 1, figsize=(10, 15))
axs_disp[0].plot(time, dx, 'r')
axs_disp[0].set_title('Displacement in X vs. Time')
axs_disp[0].set_ylabel('Displacement (m)')
axs_disp[1].plot(time, dy, 'g')
axs_disp[1].set_title('Displacement in Y vs. Time')
axs_disp[1].set_ylabel('Displacement (m)')
axs_disp[2].plot(time, dz, 'b')
axs_disp[2].set_title('Displacement in Z vs. Time')
axs_disp[2].set_xlabel('Time (s)')
axs_disp[2].set_ylabel('Displacement (m)')
fig_disp.tight_layout()

plt.show()
