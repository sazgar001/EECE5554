import matplotlib.pyplot as plt
import pandas as pd
import numpy as np  

# Load and read the CSV file
data = pd.read_csv('/home/sakib/gnss/src/data/Sakib_Walk.csv')
cleaned_data = data.dropna(subset=['%time', 'actual.altitude']) # Clean the data by dropping rows with NaN values in the '%time' or 'actual.altitude' columns

# Conversion from nanoseconds to seconds and cleaning data
time_in_seconds = pd.to_numeric(cleaned_data['%time'], errors='coerce') / 1e9 # conversion from nanoseconds to seconds
time_normalized_seconds = time_in_seconds - time_in_seconds.iloc[0] 
altitude = cleaned_data['actual.altitude'] # Extract the altitude data

# Convert Pandas Series to NumPy arrays
time_normalized_np = np.array(time_normalized_seconds.fillna(0))  # Fill NaN with 0 for safety
altitude_np = np.array(altitude.fillna(0))  # Similarly, fill NaN with 0

# Plotting the data with NumPy arrays to avoid indexing issues
plt.figure(figsize=(10, 6)) # Creates a new figure for the plot with a specified size (10 inches wide and 6 inches tall)
plt.plot(time_normalized_np, altitude_np, marker='o', linestyle='-', color='blue', label='Altitude vs. Time') # plot clean data with set parameters
plt.title('Moving Data - Altitude vs. Time') # title of graph
plt.xlabel('Time (Seconds)') # x-axis label
plt.ylabel('Altitude (Meters)') # y-axis label
plt.legend() # declare a legend
plt.grid(True) # make a grid
plt.show() # show plot
