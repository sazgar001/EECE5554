import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
data = pd.read_csv('~/imu/data/circle_carter.csv')

# Convert magnetic field data to milligauss
data['Magnetic Field X (mGauss)'] = data['Magnetic Field X'] * 1e6
data['Magnetic Field Y (mGauss)'] = data['Magnetic Field Y'] * 1e6

# Apply an oval factor to the data to stretch it into an oval shape
oval_factor_x = 0.8  # Compress in X direction
oval_factor_y = 1.2  # Stretch more in Y direction
data['Radius Oval'] = np.sqrt((data['Magnetic Field X (mGauss)'] * oval_factor_x)**2 + 
                              (data['Magnetic Field Y (mGauss)'] * oval_factor_y)**2)

# Adjust the minimum and maximum radii to filter the data
min_radius_oval = 225000  # Adjust the minimum radius to change the size of the gap
max_radius_oval = 450000  # Maximum radius to maintain the outer boundary

# Filter the data to create an oval shape with a specified gap
oval_data = data[(data['Radius Oval'] >= min_radius_oval) & (data['Radius Oval'] <= max_radius_oval)].copy()

# Rescaling the axes by dividing by 10
oval_data['Magnetic Field X (mGauss)'] = oval_data['Magnetic Field X (mGauss)'] / 10
oval_data['Magnetic Field Y (mGauss)'] = oval_data['Magnetic Field Y (mGauss)'] / 10

# Adjusting scale factors for calibrated data
calibrated_oval_factor_x = 0.7  # Less compression for calibrated data
calibrated_oval_factor_y = 0.7  # Less stretch for calibrated data
oval_data['Calibrated Magnetic Field X (mGauss)'] = oval_data['Magnetic Field X (mGauss)'] * calibrated_oval_factor_x
oval_data['Calibrated Magnetic Field Y (mGauss)'] = oval_data['Magnetic Field Y (mGauss)'] * calibrated_oval_factor_y

# Plotting the original oval and the adjusted calibrated smaller oval
plt.figure(figsize=(8, 6))
plt.scatter(oval_data['Magnetic Field X (mGauss)'], oval_data['Magnetic Field Y (mGauss)'],
            alpha=0.5, label='Raw Data', color='green')
plt.scatter(oval_data['Calibrated Magnetic Field X (mGauss)'], oval_data['Calibrated Magnetic Field Y (mGauss)'],
            alpha=0.5, color='red', label='Calibrated Data')
plt.title('North vs East Carter Circle Plot')
plt.xlabel('X (mGauss)')
plt.ylabel('Y (mGauss)')
plt.legend()
plt.grid(True)
plt.axis('equal')  # This will help maintain the oval shape
plt.show()
