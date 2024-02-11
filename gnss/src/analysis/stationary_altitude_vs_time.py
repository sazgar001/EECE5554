# Importing necessary libraries
import matplotlib.pyplot as plt  # Matplotlib -> plotting
import pandas as pd  # Pandas -> data manipulation
import numpy as np  # NumPy -> numerical operations

# Load and reading the data from two CSV files and storing them in DataFrames
occluded_data = pd.read_csv('/home/sakib/gnss/src/data/Sakib_occluded.csv')
open_data = pd.read_csv('/home/sakib/gnss/src/data/Sakib_open.csv')

# Function to prepare and clean the data -> referred to this link to clean csv: https://realpython.com/python-data-cleaning-numpy-pandas/
def prepare_data(data):
    # Clean the data by dropping rows with NaN values in the '%time' or 'actual.altitude' columns
    cleaned_data = data.dropna(subset=['%time', 'actual.altitude'])

    # Conversion from nanoseconds to seconds and cleaning data
    time_in_seconds = pd.to_numeric(cleaned_data['%time'], errors='coerce') / 1e9  # Convert nanoseconds to seconds
    time_normalized_seconds = time_in_seconds - time_in_seconds.iloc[0]  # Normalize time data
    
    # Extract the altitude data
    altitude = cleaned_data['actual.altitude']
    
    # Filtering for up to 300 seconds
    filtered_indices = time_normalized_seconds <= 300  # Filter data where time is less than or equal to 300 seconds
    time_normalized_np = np.array(time_normalized_seconds[filtered_indices])  # Convert time data to NumPy array
    altitude_np = np.array(altitude[filtered_indices])  # Convert altitude data to NumPy array
    
    return time_normalized_np, altitude_np  # Return cleaned and prepared time and altitude data

# Prepare the data for plotting using the prepare_data function
time_occluded, altitude_occluded = prepare_data(occluded_data)
time_open, altitude_open = prepare_data(open_data)

# Plotting the data on the same figure
plt.figure(figsize=(10, 6))  # Create a figure with specified size
plt.plot(time_occluded, altitude_occluded, 'r-', marker='o', label='Occluded Data')  # Plot occluded data in red dots connected 
plt.plot(time_open, altitude_open, 'g-', marker='o', label='Open Data')  # Plot open data in green dots connected
plt.title('Stationary Data - Altitude vs. Time')  # Set the title for the plot
plt.xlabel('Time (Seconds)')  # Set the label for the x-axis
plt.ylabel('Altitude (Meters)')  # Set the label for the y-axis
plt.legend()  # legend for the plot
plt.grid(True)  # grid in the plot
plt.xlim(0, 300)  # Limiting the x-axis to 300 seconds for clarity
plt.show()  # Show plot
