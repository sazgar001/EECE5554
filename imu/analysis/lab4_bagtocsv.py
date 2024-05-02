import rosbag
import pandas as pd
import os

# Specify the ROS bag path and output CSV file path
# bag_path = os.path.expanduser('~/imu/data/circle_carter.bag')
# csv_file_path = os.path.expanduser('~/imu/data/circle_carter.csv')
bag_path = os.path.expanduser('~/imu/data/square_carter.bag')
csv_file_path = os.path.expanduser('~/imu/data/square_carter.csv')


topic_name = '/imu'
data_list = []

with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # Access the raw_IMU_string field for the raw data
        raw_IMU_string = msg.raw_IMU_string.strip()  # Store the entire string
        raw_data = raw_IMU_string.split(',')  # Split the string into components
        
        # Parse each part of the raw data string (skipping the first and last elements)
        # Adjust the indices and parsing as necessary based on the actual data structure
        orientation = [float(raw_data[1]), float(raw_data[2]), float(raw_data[3])]
        angular_velocity = [float(raw_data[4]), float(raw_data[5]), float(raw_data[6])]
        linear_acceleration = [float(raw_data[7]), float(raw_data[8]), float(raw_data[9])]
        
        # For the magnetic field, specifically handle the last value to strip the checksum
        magnetic_field_x = float(raw_data[10])
        magnetic_field_y = float(raw_data[11])
        magnetic_field_z = raw_data[12].split('*')[0]  # Split and take the first part to remove checksum
        magnetic_field_z = float(magnetic_field_z)
        
        # Combine all parsed data into a row, including the entire raw IMU data string
        row = [t.to_sec()] + orientation + angular_velocity + linear_acceleration + [magnetic_field_x, magnetic_field_y, magnetic_field_z] + [raw_IMU_string]
        
        data_list.append(row)

# Column names, adjusted to include the 'Raw IMU Data String' column
columns = ['Timestamp', 'Orientation X', 'Orientation Y', 'Orientation Z',
           'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
           'Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z',
           'Magnetic Field X', 'Magnetic Field Y', 'Magnetic Field Z', 'Raw IMU Data String']

# Convert to DataFrame and save
df = pd.DataFrame(data_list, columns=columns)
df.to_csv(csv_file_path, index=False)

print(f'Data from {topic_name} written to {csv_file_path}')
