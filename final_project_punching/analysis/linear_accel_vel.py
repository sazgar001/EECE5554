import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Provided file paths
file_paths = {
    'Andrew': '/home/sakib/punching_bag/data/andrewPunch_parsed.csv',
    'Jess': '/home/sakib/punching_bag/data/jessPunch_parsed.csv',
    'Sakib': '/home/sakib/punching_bag/data/sakibPunch_parsed.csv',
    'Satish': '/home/sakib/punching_bag/data/satishPunch_parsed.csv',
    'Shaked': '/home/sakib/punching_bag/data/shakedPunch_parsed.csv'
}

def load_data(filepath):
    """Load data from a CSV file into a DataFrame."""
    return pd.read_csv(filepath)

def preprocess_data(data):
    """Preprocess the data by normalizing timestamps and converting angular velocities to linear velocities."""
    initial_time = data['Timestamp'].iloc[0]
    data['Timestamp'] = (data['Timestamp'] - initial_time) / 1000 if data['Timestamp'].max() > 1e12 else data['Timestamp'] - initial_time
    radius = 0.3  # radius in meters
    for axis in ['X', 'Y', 'Z']:
        data[f'Linear Velocity {axis}'] = data[f'Angular Velocity {axis}'] * radius
    return data

def plot_data_normalized(data, individual_name):
    """Plot the linear velocities and accelerations with initial values normalized to zero, without 'normalized' in titles."""
    fig, axes = plt.subplots(2, 3, figsize=(18, 8))
    fig.suptitle(f"{individual_name}'s Linear Velocity and Acceleration", fontsize=16)
    
    for i, axis in enumerate(['X', 'Y', 'Z']):
        vel_column = f'Linear Velocity {axis}'
        acc_column = f'Linear Acceleration {axis}'
        
        # Normalize the velocity and acceleration to start at zero
        initial_vel = data[vel_column].iloc[0]
        initial_acc = data[acc_column].iloc[0]
        
        # Plot Velocity normalized
        axes[0, i].plot(data['Timestamp'], (data[vel_column] - initial_vel) * 100, 'g-', label='Velocity')
        axes[0, i].set_title(f'Linear Velocity {axis}')
        axes[0, i].set_xlabel('Time (s)')
        axes[0, i].set_ylabel('Velocity (m/s)')
        
        # Plot Acceleration normalized
        axes[1, i].plot(data['Timestamp'], data[acc_column] - initial_acc, 'r-', label='Acceleration')
        axes[1, i].set_title(f'Linear Acceleration {axis}')
        axes[1, i].set_xlabel('Time (s)')
        axes[1, i].set_ylabel('Acceleration (m/s²)')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

# Generate normalized plots for each individual
for name, file_path in file_paths.items():
    data = load_data(file_path)
    data = preprocess_data(data)
    plot_data_normalized(data, name)

