import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

# File paths updated to the uploaded files
file_paths = {
    'Andrew': '/home/sakib/punching_bag/data/andrewPunch_parsed.csv',
    'Jess': '/home/sakib/punching_bag/data/jessPunch_parsed.csv',
    'Sakib': '/home/sakib/punching_bag/data/sakibPunch_parsed.csv',
    'Satish': '/home/sakib/punching_bag/data/satishPunch_parsed.csv',
    'Shaked': '/home/sakib/punching_bag/data/shakedPunch_parsed.csv'
}

# Specify the folder path for saving the videos
video_folder = '/home/sakib/punching_bag/videos/'

def load_data(filepath):
    """Load data from a CSV file into a DataFrame."""
    return pd.read_csv(filepath)

def preprocess_data(data):
    """Preprocess the data by normalizing timestamps and converting angular velocities to linear velocities."""
    initial_time = data['Timestamp'].iloc[0]
    data['Timestamp'] = data['Timestamp'] - initial_time  # Normalize time
    radius = 0.3  # Radius in meters for linear velocity calculation
    for axis in ['X', 'Y', 'Z']:
        # Normalizing velocities and accelerations to start from zero
        data[f'Linear Velocity {axis}'] = (data[f'Angular Velocity {axis}'] - data[f'Angular Velocity {axis}'].iloc[0]) * radius
        data[f'Linear Acceleration {axis}'] = data[f'Linear Acceleration {axis}'] - data[f'Linear Acceleration {axis}'].iloc[0]
    return data

def animate(i, data, lines, step):
    """Animation function which updates the plots."""
    for axis in ['X', 'Y', 'Z']:
        lines[axis][0].set_data(data['Timestamp'][:i*step], data[f'Linear Velocity {axis}'][:i*step] * 100)  # Velocity converted to cm/s
        lines[axis][1].set_data(data['Timestamp'][:i*step], data[f'Linear Acceleration {axis}'][:i*step])
    return [line for line_group in lines.values() for line in line_group]

# Define the step variable outside the loop for skipping frames
step = 10  # Adjust the step if needed to speed up the animation

# Generate animated plots for each individual
for name, file_path in file_paths.items():
    data = load_data(file_path)
    data = preprocess_data(data)
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))  # Adjusted for better spacing
    fig.suptitle(f"{name}'s Linear Velocity and Acceleration", fontsize=16)
    
    lines = {}
    max_velocity = np.abs(data[[f'Linear Velocity {axis}' for axis in ['X', 'Y', 'Z']]] * 100).max().max()  # Velocity converted to cm/s
    acceleration_limit = 50  # Y-axis limit for acceleration

    # Set up the x-axis with a fixed scale
    max_time = int(data['Timestamp'].max() + 1)  # Find the max time and round up

    for i, axis in enumerate(['X', 'Y', 'Z']):
        axes[0, i].set_title(f'Linear Velocity {axis}')
        axes[0, i].set_xlabel('Time (s)')
        axes[0, i].set_ylabel('Velocity (m/s)')
        axes[0, i].grid(True)
        axes[0, i].set_xlim(0, max_time)
        axes[0, i].set_ylim(-max_velocity, max_velocity)  # Y-axis limit for velocity
        
        axes[1, i].set_title(f'Linear Acceleration {axis}')
        axes[1, i].set_xlabel('Time (s)')
        axes[1, i].set_ylabel('Acceleration (m/s²)')
        axes[1, i].grid(True)
        axes[1, i].set_xlim(0, max_time)  # Set x-axis limits from 0 to max time
        axes[1, i].set_ylim(-acceleration_limit, acceleration_limit)  # Y-axis limit for acceleration
        
        vel_line, = axes[0, i].plot([], [], 'g-', label='Velocity')
        acc_line, = axes[1, i].plot([], [], 'r-', label='Acceleration')
        lines[axis] = (vel_line, acc_line)

    # Set the x-axis tick intervals for all plots
    for ax in axes.flat:
        ax.set_xticks(np.arange(0, max_time+1, 20 if name == 'Shaked' else 10))

    plt.tight_layout(pad=3.0)  # Adjust spacing to prevent label/title overlap
    ani = FuncAnimation(fig, animate, fargs=(data, lines, step), frames=range(0, len(data), step), interval=10, blit=False)  # Reduced interval for faster animation
    
    # Ensure the video folder exists, if not, create it
    os.makedirs(video_folder, exist_ok=True)
    
    # Save the animation as a video file
    video_file = video_folder + f"{name}_punch_animation.mp4"
    ani.save(video_file, fps=10, extra_args=['-vcodec', 'libx264'])

    plt.show()
