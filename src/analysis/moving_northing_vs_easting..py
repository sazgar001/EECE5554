import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the data from the ODS file or CSV files.
ods_file_path = '/home/sakib/catkin_ws/src/gnss/src/data/Sakib_Walk.num.ods'
ods_data = pd.read_excel(ods_file_path, engine='odf')

# Apply filtering based on the specified easting and northing ranges
filtered_data = ods_data[(ods_data['field.utm_easting'] >= 327150) & (ods_data['field.utm_easting'] <= 327350) &
                         (ods_data['field.utm_northing'] >= 4652300) & (ods_data['field.utm_northing'] <= 4652450)]

# Convert the pandas Series to numpy arrays before plotting
easting = filtered_data['field.utm_easting'].to_numpy() # Converts the 'field.utm_easting' column in 'filtered_data' to a NumPy array, saved as 'easting'.
northing = filtered_data['field.utm_northing'].to_numpy() # Converts the 'field.utm_northing' column in 'filtered_data' to a NumPy array, saved as 'northing'.

# Calculate the line of best fit with the filtered data
m, b = np.polyfit(easting, northing, 1)

# Plotting the data with the correct line of best fit
plt.figure(figsize=(10, 6)) # Creates a new figure for the plot with a specified size (10 inches wide and 6 inches tall
plt.scatter(easting, northing, color='blue', marker='o', label='Data Points') #Scatter plot using 'easting' and 'northing' data, with blue markers dots. 

# Adding the line of best fit to the plot with increased thickness
plt.plot(easting, m * easting + b, color='red', label=f'Line of Best Fit: y={m:.2f}x+{b:.2f}', linewidth=2)  # Plotting line of best fit and increasing thickness of line

# Set plot details with increased font sizes and move the legend to the bottom right
plt.xlabel('UTM Easting', fontsize=12)
plt.ylabel('UTM Northing', fontsize=12)
plt.title('Walking Data Northing vs Easting with Line of Best Fit', fontsize=14)
plt.legend(loc='lower right') # setting location of legend 
plt.grid(True)
plt.ticklabel_format(useOffset=False, style='plain', axis='y') # Configures the y-axis to display tick labels in plain format without offset.

# Set axis limits according to the specified range to truncate the data
plt.xlim(327175, 327350)
plt.ylim(4652300, 4652450)

# Displaying the plot
plt.show()
