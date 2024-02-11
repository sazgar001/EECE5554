import pandas as pd  
import numpy as np  
import matplotlib.pyplot as plt  

# Load datasets
occluded_df = pd.read_excel('/home/sakib/gnss/src/data/Occluded_RTK_Sakib.num.ods', engine="odf")  
open_df = pd.read_excel('/home/sakib/gnss/src/data/Open_RTK_Sakib.num.ods', engine="odf")  

# UTM Easting and UTM Northing are treated as floats -> when looking at data a lot of repeating numbers as integers but not as decmimals
occluded_df['UTM Easting'] = occluded_df['UTM Easting'].astype(float)  # Occluded Data UTM Easting to float
occluded_df['UTM Northing'] = occluded_df['UTM Northing'].astype(float)  # Occluded Data UTM Northing to float
open_df['UTM Easting'] = open_df['UTM Easting'].astype(float)  # Open Data UTM Easting to float
open_df['UTM Northing'] = open_df['UTM Northing'].astype(float)  # Open Data UTM Northing to float

# Calculate centroids for both datasets
centroid_occluded = {
    "easting": occluded_df["UTM Easting"].mean(),  # Calculate the mean UTM Easting for occluded dataset
    "northing": occluded_df["UTM Northing"].mean()  # Calculate the mean UTM Northing for occluded dataset
}
centroid_open = {
    "easting": open_df["UTM Easting"].mean(),  # Calculate the mean UTM Easting for open dataset
    "northing": open_df["UTM Northing"].mean()  # Calculate the mean UTM Northing for open dataset
}

# Calculate Euclidean distances from the centroid for both datasets
occluded_df['Distance from Centroid'] = np.sqrt(
    (occluded_df["UTM Easting"] - centroid_occluded["easting"])**2 + 
    (occluded_df["UTM Northing"] - centroid_occluded["northing"])**2)  # Calculate Euclidean distance from centroid for occluded dataset
open_df['Distance from Centroid'] = np.sqrt(
    (open_df["UTM Easting"] - centroid_open["easting"])**2 + 
    (open_df["UTM Northing"] - centroid_open["northing"])**2)  # Calculate Euclidean distance from centroid for open dataset

# Filter the occluded dataset according to the specified range -> this is in effort to truncate data
occluded_filtered = occluded_df[
    (occluded_df['Distance from Centroid'] >= 63188) &  # Filter occluded dataset based on distance range aka x-axis
    (occluded_df['Distance from Centroid'] <= 63200)]  
occluded_bins = np.arange(63188, 63201)  # Define bins for the histograms

# Filter the open dataset according to the specified range -> this is in effort to truncate data
open_filtered = open_df[
    (open_df['Distance from Centroid'] >= 40187) & # Filter open dataset based on distance range aka x-axis
    (open_df['Distance from Centroid'] <= 40192)]  
open_bins= np.arange(40187.5, 40189.75 + 0.25, 0.25)  # Define bins for the open dataset histogram -> for some reason 40190 didnt work but 40189.75 + 0.25 worked?

# Plot the histograms for both datasets
fig, axs = plt.subplots(1, 2, figsize=(14, 6))  # Create subplots with 1 row and 2 columns -> 2 plots in one window.
fig.suptitle('RTK Occluded and Open - Histogram of Distances from Centroid')  # Set the title for the entire figure

# Occluded Dataset Histogram
axs[0].hist(occluded_filtered['Distance from Centroid'], bins=occluded_bins, color='skyblue', edgecolor='black')  # Plot histogram for occluded dataset
axs[0].set_title('RTK Euclidean Occluded Data Histogram')  # Set title for occluded dataset histogram
axs[0].set_xlabel('Distance from Centroid (m)')  # x-axis label for plot 1
axs[0].set_ylabel('Frequency')  # y-axis label for plot 1
axs[0].set_xlim(63188, 63200)  # limit for x-axis -> truncate data
axs[0].set_ylim(0, 60)  # Limit for y-axis -> no need to show frequency more than 60 since that is highest value
axs[0].set_yticks(np.arange(0, 61, 5))  # Set y-axis ticks -> intervals of 5 from 0 to 60.
axs[0].grid(True)  # Grid for plot

# Open Dataset Histogram
axs[1].hist(open_filtered['Distance from Centroid'], bins=open_bins, color='lightgreen', edgecolor='black', rwidth=1)  # Plot histogram for open dataset
axs[1].set_title('RTK Euclidean Open Data Histogram')  # Set title for open dataset histogram
axs[1].set_xlabel('Distance from Centroid')  # x-axis label for plot 1
axs[1].set_xlim(40187.75, 40189.75)  # y-axis label for plot 2
axs[1].set_xticks(np.arange(40187.75, 40189.75 + 0.25, 0.25))  # Specify decimal tickmarks for x-axis since truncation of data deals with decimals
axs[1].grid(True)  # Grid for plot

# Disable scientific notation and set a fixed format for the x-axis labels - googled this because for some reason my x-axis kept showing scientific notation (not appealing)
axs[1].ticklabel_format(style='plain', axis='x')  # Disable scientific notation for x-axis
axs[1].get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:.2f}".format(x)))  # Set format for x-axis labels

plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout of the plots -> Trial and error around to make the graphs look nice.
plt.show()  # Display the plots
