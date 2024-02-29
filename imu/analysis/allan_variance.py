import os  # Import the os module to interact with the operating system
import pandas as pd  # Import pandas 
import numpy as np  # Import numpy.
import matplotlib  # Import the matplotlib library for graphs
matplotlib.use('TkAgg')  # Set the backend of matplotlib to 'TkAgg' to use Tkinter for rendering plots. Did this for debugging from google
import matplotlib.pyplot as plt  # Import pyplot from matplotlib  
import allantools  # Import allantools 

file_path = '/home/sakib/imu/data/parsed_location_B.csv' # Define the path to the CSV file containing the data to be analyzed.
sample_rate = 100 # Define the sample rate of the data, which is necessary for calculating the Allan variance.

# Define a function to calculate Allan variance given data and a sample rate.
def calculate_allan_variance(data, sample_rate):
    taus, adevs, _, _ = allantools.oadev(data, rate=sample_rate, data_type='freq', taus='all') # Use the oadev function from allantools to calculate overlapping Allan deviation.
    return taus, adevs # return averaging times (taus) and allan deviations (adevs)

# Define a function to extract noise parameters (N, K, B) from the Allan variance data.
def extract_parameters(taus, adevs):   
    idx_arw = np.where(taus <= 1)[0]   # Identify indices where averaging time τ is less than or equal to 1 second for angle random walk (ARW).
    idx_rrw = np.where((taus > 1) & (taus <= 3))[0] # Identify indices for rate random walk (RRW) where τ is between 1 and 3 seconds.
    idx_bs = np.where(taus > 3)[0]     # Identify indices for bias stability (BS) where τ is greater than 3 seconds.

    # Calculate N, K, B using the identified indices and Allan deviation values.
    N = np.min(adevs[idx_arw]) / np.sqrt(taus[idx_arw[np.argmin(adevs[idx_arw])]])
    K = np.min(adevs[idx_rrw]) * np.sqrt(taus[idx_rrw[np.argmin(adevs[idx_rrw])]])
    B = np.min(adevs[idx_bs])

    return N, K, B

# Define a function to plot the Allan variance and the extracted noise parameters for each axis of a sensor.
def plot_allan_variance(taus_list, adevs_list, axes_list, sensor_type):
    plt.figure(figsize=(10, 6))  # Create a new figure with specified size for the plot.
    for taus, adevs, axis_name in zip(taus_list, adevs_list, axes_list):
        N, K, B = extract_parameters(taus, adevs) # Extract noise parameters for each axis.
        plt.loglog(taus, adevs, label=f'{axis_name} σ(τ)') # Plot the Allan deviation for each axis on a log-log scale.
    
    plt.xlabel('Averaging Time τ (s)') # Label the x and y axes and set the title of the plot.
    plt.ylabel('Allan Deviation σ(τ)')
    plt.title(f'Allan Variance for {sensor_type.upper()}')
    plt.grid(True, which="both", ls="--") # Enable grid lines on the plot for better readability.
    plt.legend(title=f'N: {N:.2e} (deg/√hr)\nK: {K:.2e} (deg/hr²)\nB: {B:.2e} (deg/hr)')     # Display a legend on the plot, showing noise parameters.
    
    target_dir = '/home/sakib/imu/analysis/' # Prepare to save the plot by checking if the target directory exists; if not, create it.
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
    plt.savefig(os.path.join(target_dir, f'allan_variance_{sensor_type}.png'))  # Save the plot as a PNG file in the specified directory.
    plt.show()  # Display the plot to the screen.

# Define a function to extract and plot Allan variance for all axes of a specified sensor type.
def plot_sensor_data(data, sample_rate, sensor_type): # Define the column names for gyroscope and accelerometer data.
    gyro_axes = ['Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z']
    accel_axes = ['Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z']
    
    # Plot Allan variance for gyroscope data if specified.
    if sensor_type == 'gyro':
        taus_list, adevs_list, axes_list = [], [], []
        for axis in gyro_axes:
            if axis in data.columns:
                taus, adevs = calculate_allan_variance(data[axis].dropna(), sample_rate)  # Calculate Allan variance for each axis and append results to lists.
                taus_list.append(taus)
                adevs_list.append(adevs)
                axes_list.append(axis)
        plot_allan_variance(taus_list, adevs_list, axes_list, sensor_type) # Call the function to plot Allan variance using the collected data.
    
    # Similarly, plot Allan variance for accelerometer data if specified.
    elif sensor_type == 'accel':
        taus_list, adevs_list, axes_list = [], [], []
        for axis in accel_axes:
            if axis in data.columns:
                taus, adevs = calculate_allan_variance(data[axis].dropna(), sample_rate)
                taus_list.append(taus)
                adevs_list.append(adevs)
                axes_list.append(axis)
        
        plot_allan_variance(taus_list, adevs_list, axes_list, sensor_type)

# Define the main function to run the Allan variance analysis.
def main():
    if not os.path.exists(file_path): # Check if the specified file path exists, and raise an error if it doesn't.
        raise FileNotFoundError(f"File not found at the specified path: {file_path}")
    
    # On Command Terminal Output to check if it works 
    print("Reading data...") # Read the CSV file into a pandas DataFrame.
    data = pd.read_csv(file_path)
    print("Data read successfully.")
    print("Columns in the dataset:", data.columns)     # Print the columns of the DataFrame to the console.
    print("Plotting Gyroscope data...")     # Plot Allan variance for gyroscope data.
    plot_sensor_data(data, sample_rate, 'gyro')
    print("Plotting Accelerometer data...") # Plot Allan variance for accelerometer data.
    plot_sensor_data(data, sample_rate, 'accel')

if __name__ == "__main__": # Check if this script is being run directly and, if so, execute the main function.
    main()
