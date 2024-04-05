import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

circles_data = pd.read_csv('~/lab5/src/data/going_in_circles_imu.csv')

# Conversion factor from Tesla to milliGauss
tesla_to_mgauss = 10000000

# Extract magnetometer data
mag_x = circles_data['mag_field.magnetic_field.x'].values * tesla_to_mgauss
mag_y = circles_data['mag_field.magnetic_field.y'].values * tesla_to_mgauss

# Calculate the average of the x and y magnetometer data (hard iron correction)
offset_x = np.mean(mag_x)
offset_y = np.mean(mag_y)

# Apply the hard iron correction
corrected_x = mag_x - offset_x
corrected_y = mag_y - offset_y

# Calculate the scaling factors (soft iron correction)
scale_x = (np.max(corrected_x) - np.min(corrected_x)) / 2
scale_y = (np.max(corrected_y) - np.min(corrected_y)) / 2
avg_scale = (scale_x + scale_y) / 2

# Apply the soft iron correction
corrected_x /= (scale_x / avg_scale)
corrected_y /= (scale_y / avg_scale)

# Plot the data before and after calibration
plt.figure(figsize=(14, 7))

# Before calibration
plt.subplot(1, 2, 1)
plt.scatter(mag_x, mag_y, color='blue', label='Before Calibration', s=10)  # Adjust marker size with 's'
plt.title('Magnetic Field Before Calibration - Circle')
plt.xlabel('X component of magnetic field (mG)')
plt.ylabel('Y component of magnetic field (mG)')
plt.axis('equal')  # Ensures the plot scales are equal
plt.legend()
plt.grid(True)  # Adds grid lines

# After calibration
plt.subplot(1, 2, 2)
plt.scatter(corrected_x, corrected_y, color='red', label='After Calibration', s=10)  # Adjust marker size with 's'
plt.title('Magnetic Field After Calibration - Circle')
plt.xlabel('X component of magnetic field (mG)')
plt.ylabel('Y component of magnetic field (mG)')
plt.axis('equal')  # Ensures the plot scales are equal
plt.legend()
plt.grid(True)  # Adds grid lines

plt.tight_layout()
plt.show()
