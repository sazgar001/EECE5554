import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

# Load dataset
ods_file_path = '/home/sakib/gnss/src/data/Walk_RTK_Sakib.num.ods'
ods_data = pd.read_excel(ods_file_path, engine='odf') # Load the data from the ODS file

# Define a function to format the axis ticks - https://stackoverflow.com/questions/40511476/how-to-properly-use-funcformatterfunc 
formatter = FuncFormatter(lambda val, pos: f'{int(val)}')

# Adjusting the range for Easting and Northing to the specified values - data truncation 
easting_range_min = 328106
easting_range_max = 328126
northing_range_min = 4689426
northing_range_max = 4689446

# Filtering the dataset within the specified range -> same as histogram plot but I realized I can make them variables
filtered_data_subset = ods_data[(ods_data['UTM Easting'] >= easting_range_min) & 
                                (ods_data['UTM Easting'] <= easting_range_max) &
                                (ods_data['UTM Northing'] >= northing_range_min) & 
                                (ods_data['UTM Northing'] <= northing_range_max)]

# Recalculating the line of best fit for the filtered data subset
easting_subset = filtered_data_subset['UTM Easting'].to_numpy()
northing_subset = filtered_data_subset['UTM Northing'].to_numpy()
m_subset, b_subset = np.polyfit(easting_subset, northing_subset, 1) #polyfit to make y=mx+b equation

# Plotting Parameters with Line of Best Fit
plt.figure(figsize=(10, 6)) # display ratio of plot
plt.scatter(easting_subset, northing_subset, color='blue', marker='o', label='RTK Moving Data Points') # specifying color of data points
# Setting up line of best fit equation and color for line 
plt.plot(easting_subset, m_subset * easting_subset + b_subset, color='red', label=f'Line of Best Fit: y={m_subset:.2f}x+{b_subset:.2f}', linewidth=2)
plt.xlabel('Easting (m)', fontsize=12) # x-axis label
plt.ylabel('Northing (m)', fontsize=12) # y-axis label
plt.title('RTK Moving Walk: Northing vs Easting with Line of Best Fit', fontsize=14) # title of graph
plt.legend(loc='lower right') # make legend lower part of graph
plt.grid(True) # plot gridlines
plt.gca().xaxis.set_major_formatter(formatter) # this also gets rid of scientific notation for x and y-axis
plt.gca().yaxis.set_major_formatter(formatter)
plt.xlim(easting_range_min, easting_range_max) # Setting the x-axis ranges to the specified values
plt.ylim(northing_range_min, northing_range_max) # Setting the y-axis ranges to the specified values
plt.xticks(np.arange(easting_range_min, easting_range_max+1, 2)) # Setting x-axis tick marks to increase by intervals of 2
plt.yticks(np.arange(northing_range_min, northing_range_max+1, 2)) # Setting y-axis tick marks to increase by intervals of 2
plt.show() # Display plot
