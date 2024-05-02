import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv('~/imu/data/square_carter.csv')  # Please ensure the path is correct for your environment

# Convert magnetic field data from Tesla to milligauss for plotting
data['Magnetic Field X mG'] = data['Magnetic Field X'] * 1e6
data['Magnetic Field Y mG'] = data['Magnetic Field Y'] * 1e6

# Filter raw data points to be within a certain square range
square_range = 40000  # 40,000 milligauss
square_raw_data = data[
    (data['Magnetic Field X mG'].abs() <= square_range) &
    (data['Magnetic Field Y mG'].abs() <= square_range)
]

# Define a new function to create square boundary points without diagonal lines
def create_square_boundary_no_diagonals(center_x, center_y, side_length):
    half_side = side_length / 2
    # Define corners of the square
    top_left = [center_x - half_side, center_y + half_side]
    top_right = [center_x + half_side, center_y + half_side]
    bottom_left = [center_x - half_side, center_y - half_side]
    bottom_right = [center_x + half_side, center_y - half_side]

    # Generate side points excluding the corners
    x_side = np.linspace(center_x - half_side, center_x + half_side, 400)
    y_side = np.linspace(center_y - half_side, center_y + half_side, 400)

    # Top and bottom sides
    top_side = np.column_stack((x_side, np.full_like(x_side, center_y + half_side)))
    bottom_side = np.column_stack((x_side, np.full_like(x_side, center_y - half_side)))

    # Left and right sides (excluding corners by indexing from 1 to -1)
    left_side = np.column_stack((np.full_like(y_side[1:-1], center_x - half_side), y_side[1:-1]))
    right_side = np.column_stack((np.full_like(y_side[1:-1], center_x + half_side), y_side[1:-1]))

    # Combine into a full boundary by adding the corners to top and bottom sides
    square_boundary = np.vstack((top_left, top_side, top_right, right_side, bottom_right, bottom_side, bottom_left, left_side))
    return square_boundary

# Use the new function to create boundary points without diagonal lines
square_boundary_no_diagonals = create_square_boundary_no_diagonals(0, 0, square_range * 2)

# Plotting
plt.figure(figsize=(10, 8))
plt.scatter(square_raw_data['Magnetic Field X mG'], square_raw_data['Magnetic Field Y mG'], color='green', alpha=0.5, label='Raw Data')
plt.plot(square_boundary_no_diagonals[:, 0], square_boundary_no_diagonals[:, 1], color='darkgreen', label='Square Boundary', linestyle='-', linewidth=2)
plt.title('North vs East Magnetic Field Data with Square Boundary')
plt.xlabel('East (mGauss)')
plt.ylabel('North (mGauss)')
# Adjust the legend position to be below the x-axis title
plt.grid(True)

# Set axis limits to show all data and the full square boundary
plt.xlim(-square_range*1.1, square_range*1.1)
plt.ylim(-square_range*1.1, square_range*1.1)

# Ensuring x and y axis have the same scale
plt.axis('equal')
plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
plt.show()

