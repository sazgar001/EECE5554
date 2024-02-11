# Import necessary libraries
import matplotlib.pyplot as plt  # Matplotlib for plotting
import numpy as np              # NumPy for numerical operations
import pandas as pd             # Pandas for data manipulation

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------
# csv and spreadsheet files had issues and require cleaning. had to look up how to clean csv files based on provided documentation.
# Correct paths to datasets
occluded_ods_path = '/home/sakib/gnss/src/data/Sakib_occluded_num.ods'
open_ods_path = '/home/sakib/gnss/src/data/Sakib_open_num.ods'

# Load the datasets using Panda library
occluded_df = pd.read_excel(occluded_ods_path, engine='odf')  # Load the occluded dataset
open_df = pd.read_excel(open_ods_path, engine='odf')          # Load the open dataset

# Clean the datasets by dropping rows where either easting or northing is NaN 
occluded_df_clean = occluded_df.dropna(subset=['field.utm_easting', 'field.utm_northing'])  # Remove rows with NaN values
open_df_clean = open_df.dropna(subset=['field.utm_easting', 'field.utm_northing'])        # Remove rows with NaN values
#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------


# Calculate the centroids for both datasets
centroid_occluded = (occluded_df_clean['field.utm_easting'].mean(), occluded_df_clean['field.utm_northing'].mean())  # Calculate centroid for occluded data
centroid_open = (open_df_clean['field.utm_easting'].mean(), open_df_clean['field.utm_northing'].mean())              # Calculate centroid for open data

# Compute Euclidean distances from each point to the centroid for both datasets
occluded_df_clean['euclidean_distance'] = np.sqrt((occluded_df_clean['field.utm_easting'] - centroid_occluded[0]) ** 2 + 
                                                  (occluded_df_clean['field.utm_northing'] - centroid_occluded[1]) ** 2)  # Calculate Euclidean distances for occluded data

open_df_clean['euclidean_distance'] = np.sqrt((open_df_clean['field.utm_easting'] - centroid_open[0]) ** 2 + 
                                              (open_df_clean['field.utm_northing'] - centroid_open[1]) ** 2)              # Calculate Euclidean distances for open data
#  Create a figure and a set of subplots with two columns
fig, axs = plt.subplots(1, 2, figsize=(24, 6))  # Create a figure with 1 row and 2 columns for subplots. Make plot big size. 

# Set y-axis ranges for both histograms
y_ticks_occluded = np.arange(0, 56, 10)  # Define y-axis tick marks for occluded data (up to 55)
y_ticks_open = np.arange(0, 121, 10)    # Define y-axis tick marks for open data (up to 120)

# Occluded Data Histogram - Adjusting x-axis ticks to intervals of 2
axs[0].hist(occluded_df_clean['euclidean_distance'], bins=np.arange(9036, 9065, 2), alpha=1)  # Create histogram for open data. Bin size is width of bar graph. 
                                                                                              # 6400-6416 is range, and going by ticks of 2. Alpha is how dark bar graphs are.
axs[0].set_title('Euclidean Occluded Data Histogram')  # Set title for the subplot
axs[0].set_xlabel('Distance (m)')                    # Set x-axis label
axs[0].set_ylabel('Frequency')                       # Set y-axis label
axs[0].grid(True)                                    # Enable grid
axs[0].set_xlim(9036, 9064)                         # Set x-axis range
axs[0].set_xticks(np.arange(9036, 9065, 2))         # Set x-axis tick marks
axs[0].set_yticks(y_ticks_occluded)                 # Set y-axis tick marks for occluded data

# Open Data Histogram - Adjusting x-axis ticks to intervals of 2
axs[1].hist(open_df_clean['euclidean_distance'], bins=np.arange(6400, 6416, 2), alpha=1, color='orange')  # Create histogram for open data. Bin size is width of bar graph. 
                                                                                                          # 6400-6416 is range, and going by ticks of 2. Alpha is how dark bar graphs are.
axs[1].set_title('Euclidean Open Data Histogram')    # Set title for the subplot
axs[1].set_xlabel('Distance (m)')                    # Set x-axis label
axs[1].set_ylabel('Frequency')                       # Set y-axis label
axs[1].grid(True)                                    # Enable grid
axs[1].set_xlim(6400, 6415)                         # Set x-axis range
axs[1].set_xticks(np.arange(6400, 6416, 2))         # Set x-axis tick marks
axs[1].set_yticks(y_ticks_open)                     # Set y-axis tick marks for open data

# Display the plot for two separated figures in one window
plt.show()
