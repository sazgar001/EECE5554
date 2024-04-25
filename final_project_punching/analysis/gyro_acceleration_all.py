import matplotlib.pyplot as plt
import pandas as pd

# File paths for each individual's data
files = {
    "Andrew": '/home/sakib/punching_bag/data/andrewPunch_parsed.csv',
    "Jess": '/home/sakib/punching_bag/data/jessPunch_parsed.csv',
    "Sakib": '/home/sakib/punching_bag/data/sakibPunch_parsed.csv',
    "Satish": '/home/sakib/punching_bag/data/satishPunch_parsed.csv',
    "Shaked": '/home/sakib/punching_bag/data/shakedPunch_parsed.csv'
}

# Custom y-axis limits for each individual
y_axis_limits = {
    "Andrew": (-20, 50),
    "Jess": (None, None),  # Use default limits for Jess
    "Sakib": (-30, 40),
    "Satish": (-50, 50),
    "Shaked": (-25, 25)
}

plt.figure(figsize=(15, 20))

for i, (name, file_path) in enumerate(files.items(), start=1):
    data = pd.read_csv(file_path)
    data['Normalized Timestamp'] = data['Timestamp'] - data['Timestamp'].iloc[0]
    
    plt.subplot(5, 1, i)
    plt.plot(data['Normalized Timestamp'], data['Linear Acceleration X'], label='X', color='red')
    plt.plot(data['Normalized Timestamp'], data['Linear Acceleration Y'], label='Y', color='green')
    plt.plot(data['Normalized Timestamp'], data['Linear Acceleration Z'], label='Z', color='blue')
    plt.title(f'{name} - Linear Acceleration')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Acceleration (m/s²)')
    plt.ylim(y_axis_limits[name])
    plt.legend()

plt.tight_layout()
plt.suptitle('Linear Acceleration Across Individuals with Custom Y-Axis Limits', y=1.02)
plt.show()

plt.figure(figsize=(15, 20))

for i, (name, file_path) in enumerate(files.items(), start=1):
    data = pd.read_csv(file_path)
    data['Normalized Timestamp'] = data['Timestamp'] - data['Timestamp'].iloc[0]
    
    plt.subplot(5, 1, i)
    plt.plot(data['Normalized Timestamp'], data['Angular Velocity X'], label='X', color='red')
    plt.plot(data['Normalized Timestamp'], data['Angular Velocity Y'], label='Y', color='green')
    plt.plot(data['Normalized Timestamp'], data['Angular Velocity Z'], label='Z', color='blue')
    plt.title(f'{name} - Gyroscopic Data (Angular Velocity)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()

plt.tight_layout()
plt.suptitle('Gyroscopic Data Across Individuals', y=1.02)
plt.show()
