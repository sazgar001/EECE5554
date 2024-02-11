import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#Load datasets
occluded_ods_path = '/home/sakib/gnss/src/data/Occluded_RTK_Sakib.num.ods'  
open_ods_path = '/home/sakib/gnss/src/data/Open_RTK_Sakib.num.ods' 

# Because of two datasets, couldn't use the exact same technique as of RTK_moving_altitude_vs_time.py
# Had to google this - https://stackoverflow.com/questions/17834995/how-to-convert-opendocument-spreadsheets-to-a-pandas-dataframe
def read_and_prepare_data(file_path): 
    df = pd.read_excel(file_path, engine='odf')  # Read the .ods file into a Pandas DataFrame
    
    def extract_altitude(gngga_read):  # Define a function to extract altitude from 'GNGGA Read' column
        return float(gngga_read.split(',')[9])  # Split the string by ',' and extract the 9th element (altitude)
    df['Altitude'] = df['GNGGA Read'].apply(extract_altitude)  # Apply the function to create a new 'Altitude' column
    df['Normalized Time'] = df['Timestamp'] - df['Timestamp'].iloc[0]  # Calculate normalized time
    return np.array(df['Normalized Time']), np.array(df['Altitude'])    # Convert to NumPy arrays to ensure compatibility

# Prepare the data
time_occluded, altitude_occluded = read_and_prepare_data(occluded_ods_path)  # Extract time and altitude data for occluded dataset
time_open, altitude_open = read_and_prepare_data(open_ods_path)  # Extract time and altitude data for open dataset

# Plotting Parameters 
fig, ax = plt.subplots(figsize=(10, 6))  # Create a plot with a specific figure size
ax.plot(time_occluded, altitude_occluded, 'r-', marker='o', label='Occluded Data')  # Plot occluded data with red line and circle markers
ax.plot(time_open, altitude_open, 'g-', marker='o', label='Open Data')  # Plot open data with green line and circle markers
ax.set_title('RTK Stationary- Altitude vs. Time')  # Set title of the plot
ax.set_xlabel('Time (Seconds)')  # x-axis label
ax.set_ylabel('Altitude (Meters)')  # y-axis label
ax.set_xlim(0, 300)  # Set the x-axis limit
ax.set_ylim(0, 26)  # y-axis limit
ax.set_yticks(np.arange(0, 27, 2))  # y-axis ticks and limit
ax.legend(loc='lower right')  # Positioning the legend in the bottom right corner of the graph
ax.grid(True)  # grid lines for plot
plt.show()  # Show the plot

#### Main issue with altitude plots - the ROSBAGs kept reading them as 0. Had to figure out algorithm to take a look at each GNGGA string and take the digit that contained 
#### the altitude value rather than manually getting them myself. 

#### Resource links I used: 
#### 1) https://realpython.com/python-exceptions/
#### 2) https://pandas.pydata.org/docs/user_guide/text.html 
#### 3) https://github.com/csb-comren/pynmea2 -> Saw how this person did it and incorporated something similar
#### 4) https://stackoverflow.com/questions/17834995/how-to-convert-opendocument-spreadsheets-to-a-pandas-dataframe 