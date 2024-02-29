import pandas as pd  # Import the pandas library and alias it as 'pd'
import numpy as np   # Import the numpy library and alias it as 'np'
import matplotlib.pyplot as plt  # Import the pyplot module from matplotlib and alias it as 'plt'
from matplotlib.ticker import MultipleLocator, MaxNLocator  # Import specific classes from the matplotlib.ticker module
from scipy.spatial.transform import Rotation as R  # Import the Rotation class from scipy.spatial.transform module and alias it as 'R'

# Load the data
data_csv = pd.read_csv('/home/sakib/imu/data/sakib_home.csv')

# Normalize the timestamps to start at t=0
data_csv['Normalized Timestamp'] = data_csv['Timestamp'] - data_csv['Timestamp'].iloc[0]


#############################################################################################################################################################################
# Figure 0
#############################################################################################################################################################################

# Function to convert quaternion to Euler angles in degrees
def quaternion_to_euler(w, x, y, z):     # Convert the quaternion [w, x, y, z] to a rotation object
    r = R.from_quat([x, y, z, w])     # Convert the rotation object to Euler angles in degrees using the 'xyz' convention
    return r.as_euler('xyz', degrees=True) 

# Apply the conversion to the entire DataFrame
euler_angles = data_csv.apply(lambda row: quaternion_to_euler(row['Orientation W'], row['Orientation X'], 
                                                              row['Orientation Y'], row['Orientation Z']), axis=1)
data_csv[['Euler X', 'Euler Y', 'Euler Z']] = pd.DataFrame(euler_angles.tolist(), index=data_csv.index)

# Calculate the rotational rate ensuring no division by zero
time_diff = data_csv['Normalized Timestamp'].diff().fillna(1)
for axis in ['X', 'Y', 'Z']: # Iterate over each axis: 'X', 'Y', and 'Z'
    angle_diff = data_csv[f'Euler {axis}'].diff().fillna(0) # Calculate the difference between consecutive Euler angle values for the current axis and fill NaN values with 0
    data_csv[f'Rotational Rate {axis}'] = angle_diff / time_diff  # Calculate the rotational rate for the current axis by dividing angle difference by time difference

# Gyroscope data plotting with updated y-axis labels, ranges, and corrected rotational rate calculation
fig_gyro, axs_gyro = plt.subplots(3, 1, figsize=(12, 9))
fig_gyro.suptitle('Rotational Rate of X,Y,Z Plots', fontsize=16, y=0.95)  # Master title with increased font and centered

for i, axis in enumerate(['X', 'Y', 'Z']):
    axs_gyro[i].plot(data_csv['Normalized Timestamp'], data_csv[f'Rotational Rate {axis}'], label=f'{axis}-axis', marker='o', linestyle='-', markersize=4)
    axs_gyro[i].set_title(f'Gyro Euler Angle {axis}-axis')
    axs_gyro[i].set_ylabel('Rotational Rate (angle/s)')
    axs_gyro[i].grid(True)
    
    # Set specific y-axis limits for each plot
    if axis == 'X':
        axs_gyro[i].set_ylim([-0.2, 0.2])  # Set y-axis limits for X-axis plot
    elif axis == 'Z':
        axs_gyro[i].set_ylim([-1, 1])  # Set y-axis limits for Z-axis plot
    axs_gyro[i].xaxis.set_major_locator(MaxNLocator(integer=True, nbins=5))   # Apply MaxNLocator to the x-axis to prevent too many ticks
    
    if i == 2:  # Only add xlabel to the last subplot
        axs_gyro[i].set_xlabel('Time (s)')

fig_gyro.tight_layout(rect=[0, 0.03, 1, 0.95])

###########################################################################################################################################################################
# Figure 1
###########################################################################################################################################################################


# Accelerometer data plotting with specified y-limits for the top and bottom plots
fig_acc, axs_acc = plt.subplots(3, 1, figsize=(12, 9))
fig_acc.suptitle('Accelerometer Linear Acceleration Plots', fontsize=16, y=0.98)
y_limits_x = (0.14, 0.30)  # Bottom plot
y_limits_z = (9.55, 9.95)    # Top plot

for i, axis in enumerate(['X', 'Y', 'Z']):
    axs_acc[i].plot(data_csv['Normalized Timestamp'], data_csv[f'Linear Acceleration {axis}'], label=f'{axis}-axis', marker='x', linestyle='-', markersize=4)
    axs_acc[i].set_title(f'Accelerometer Linear Acceleration {axis}-axis')
    axs_acc[i].set_ylabel('Acceleration (m/s^2)')
    axs_acc[i].grid(True)

    # Set y-axis limits for the top and bottom subplots
    if axis == 'X':
        axs_acc[i].set_ylim(y_limits_x)
    elif axis == 'Z':
        axs_acc[i].set_ylim(y_limits_z)
        
    if i == 2:  # Only add xlabel to the last subplot
        axs_acc[i].set_xlabel('Time (s)')
fig_acc.tight_layout()

###########################################################################################################################################################################
# Figure 2
###########################################################################################################################################################################

# VN estimation data plotting with adjusted y-axis range for the X-axis
fig_vn, axs_vn = plt.subplots(3, 1, figsize=(12, 9))
fig_vn.suptitle('VN Estimation Plots', fontsize=16, y=0.95)  # Master title with increased font and centered

for i, axis in enumerate(['X', 'Y', 'Z']):  # Loop through each axis defined in the list ['X', 'Y', 'Z'].
    axs_vn[i].plot(data_csv['Normalized Timestamp'], data_csv[f'Euler {axis}'], label=f'{axis}-axis', marker='o', linestyle='-', markersize=4) # Googled this x-axis and respective Euler angle data for y-axis. Customized lines
    axs_vn[i].set_title(f'VN Estimation Rotation {axis}-axis')  # Title of subplot
    axs_vn[i].set_ylabel('Rotation (degrees)')  # Label y-axis
    axs_vn[i].grid(True)  # Enable the grid on the subplot.

    if axis == 'X':  # Adjust y-axis range for the X-axis subplot to reduce whitespace
        axs_vn[i].set_ylim([179.85, 180.05])  # y-axis limits for x plot for truncation purpose
    
    if i == 2:  # Only add xlabel to the last subplot
        axs_vn[i].set_xlabel('Time (s)')
fig_vn.tight_layout(rect=[0, 0.03, 1, 0.95])

###########################################################################################################################################################################
# Figure 3
###########################################################################################################################################################################

# Creating histograms without predefined ranges
filtered_euler_x = data_csv['Euler X'][(data_csv['Euler X'] >= 179.85) & (data_csv['Euler X'] <= 180.05)] # Plotting the histograms with the specified x-axis range and y-axis range
fig_hist, axs_hist = plt.subplots(1, 3, figsize=(18, 6))
fig_hist.suptitle('Histograms of Vectornav Rotation')

# Histogram for Euler X with specified x-axis and y-axis range
axs_hist[0].hist(filtered_euler_x, bins=50, alpha=0.75, color='blue')
axs_hist[0].set_title('Rotation in X-Axis (Euler Angles)')
axs_hist[0].set_xlabel('Angle (degrees)')
axs_hist[0].set_ylabel('Frequency')
axs_hist[0].set_xlim(179.875, 180.025)
axs_hist[0].set_ylim(0, 1600)  # Set y-axis range to 0 to 2250
axs_hist[0].grid(True)

# Histogram for Euler Y
axs_hist[1].hist(data_csv['Euler Y'], bins=50, alpha=0.75, color='blue')
axs_hist[1].set_title('Rotation in Y-Axis (Euler Angles)')
axs_hist[1].set_xlabel('Angle (degrees)')
axs_hist[1].set_ylabel('Frequency')
axs_hist[1].set_ylim(0, 2250)  # Set y-axis range to 0 to 2250
axs_hist[1].grid(True)

# Histogram for Euler Z
axs_hist[2].hist(data_csv['Euler Z'], bins=50, alpha=0.75, color='blue')
axs_hist[2].set_title('Rotation in Z-Axis (Euler Angles)')
axs_hist[2].set_xlabel('Angle (degrees)')
axs_hist[2].set_ylabel('Frequency')
axs_hist[2].set_ylim(0, 2250)  # Set y-axis range to 0 to 2250
axs_hist[2].grid(True)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show() # Show only the histogram plots
