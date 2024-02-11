import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load datasets
ods_file_path = '/home/sakib/gnss/src/data/Walk_RTK_Sakib.num.ods'
ods_data_altitude = pd.read_excel(ods_file_path, engine='odf')

# Define a function to extract altitude from the 'GNGGA Read' string
def extract_altitude(gngga_str):
    try: 
        return float(gngga_str.split(',')[9]) # Split the string by commas and extract the altitude part which is before ",M,"
    except (IndexError, ValueError, AttributeError): # Return NaN if there is an error in parsing
        return np.nan

ods_data_altitude['Altitude'] = ods_data_altitude['GNGGA Read'].apply(extract_altitude) # Apply the function to extract altitude
ods_data_altitude.dropna(subset=['Altitude'], inplace=True) # Remove rows where altitude could not be parsed (NaN values)
ods_data_altitude['Timestamp'] = pd.to_datetime(ods_data_altitude['Timestamp'], unit='s') # Assuming 'Timestamp' is a Unix timestamp, convert it directly without specifying a format
ods_data_altitude['Time_seconds'] = (ods_data_altitude['Timestamp'] - ods_data_altitude['Timestamp'].iloc[0]).dt.total_seconds() # Calculate the seconds from the start of the recording
time_seconds_np = ods_data_altitude['Time_seconds'].values # Ensure using .values to extract numpy arrays for plotting
altitude_np = ods_data_altitude['Altitude'].values


# Plot Altitude vs. Time in seconds
plt.figure(figsize=(14, 7)) # Display size on plot 
plt.plot(time_seconds_np, altitude_np, color='blue', marker='o', linestyle='-', label='Altitude vs. Time') # making an altitude vs time plot 
plt.xlabel('Time (seconds)', fontsize=12) # x-axis label 
plt.ylabel('Altitude (M)', fontsize=12) # y-axis label
plt.title('RTK Moving Walk: Altitude vs. Time Plot', fontsize=14) # title label
plt.legend(loc='upper right')  # Adding legend to the upper right corner on graph
plt.grid(True) # have gridlines on
plt.show() # display plot


#### Main issue with altitude plots - the ROSBAGs kept reading them as 0. Had to figure out algorithm to take a look at each GNGGA string and take the digit that contained 
#### the altitude value rather than manually getting them myself.

#### Resource links I used: 
#### 1) https://realpython.com/python-exceptions/
#### 2) https://pandas.pydata.org/docs/user_guide/text.html 
#### 3) https://github.com/csb-comren/pynmea2 -> Saw how this person did it and incorporated something similar