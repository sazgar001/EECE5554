import pandas as pd
import matplotlib.pyplot as plt

# Paths to the ODS files containing the data (couldnt use csv files for some reason)
<<<<<<< HEAD
occluded_ods_path = '/home/sakib/gnss/src/data/Sakib_occluded_num.ods'
open_ods_path = '/home/sakib/gnss/src/data/Sakib_open_num.ods'
=======
occluded_ods_path = '/home/sakib/catkin_ws/src/gnss/src/data/Sakib_occluded_num.ods'
open_ods_path = '/home/sakib/catkin_ws/src/gnss/src/data/Sakib_open_num.ods'
>>>>>>> e08659b534f722c3634df382d745807097e703b7

occluded_df = pd.read_excel(occluded_ods_path, engine='odf', usecols="H,I").dropna() # Load the occluded dataset from the specified path, selecting specific columns and dropping missing values
open_df = pd.read_excel(open_ods_path, engine='odf', usecols="H,I").dropna() # Load the open dataset from the specified path, selecting specific columns and dropping missing values
occluded_normalized = occluded_df - occluded_df.iloc[0] # Normalizing the occluded dataset by subtracting the first point's coordinates from all points
open_normalized = open_df - open_df.iloc[0]  # Normalizing the open dataset by subtracting the first point's coordinates from all points


centroid_occluded = occluded_normalized.mean() # Calculating the centroid (average position) of the normalized occluded dataset
centroid_open = open_normalized.mean() # Calculating the centroid (average position) of the normalized open dataset

occluded_centered = occluded_normalized - centroid_occluded # Centering the occluded dataset by subtracting the centroid's coordinates from all points
open_centered = open_normalized - centroid_open # Centering the open dataset by subtracting the centroid's coordinates from all points

plt.figure(figsize=(20, 12))  # Setting up the plot with a specified figure size

# Plotting the centered occluded data points with red dots
plt.scatter(occluded_centered['field.utm_easting'], occluded_centered['field.utm_northing'], 
            color='red', label='Occluded', alpha=0.7, marker='x', s=50)

# Plotting the centered open data points with blue dots
plt.scatter(open_centered['field.utm_easting'], open_centered['field.utm_northing'], 
            color='blue', label='Open', alpha=0.7, marker='o', s=50)

plt.title('Northing vs. Easting for Occluded and Open Data with Centroid Subtraction') # title of plot
plt.xlabel('Easting (Centered)') # x-axis label
plt.ylabel('Northing (Centered)') # y-axis label
plt.legend() # Add legend for occluded and open data points
plt.grid(True) # put on grid lines

# Displaying the centroid values below the plot for reference
plt.figtext(0.5, 0.02, 
            f"Occluded Centroid: Easting = {centroid_occluded['field.utm_easting']:.2f}, "
            f"Northing = {centroid_occluded['field.utm_northing']:.2f}   "
            f"Open Centroid: Easting = {centroid_open['field.utm_easting']:.2f}, "
            f"Northing = {centroid_open['field.utm_northing']:.2f}", 
            ha="center", fontsize=10)
plt.show() # display plot

