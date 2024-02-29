import rosbag  # Imports the rosbag 
import pandas as pd  # Imports pandas f
import os  # Imports the os module 

# Specify the ROS bag path and output CSV file path
bag_path = os.path.expanduser('~/imu/data/sakib_home.bag')  # Take vectornav bag file 10 mins
csv_file_path = os.path.expanduser('~/imu/data/sakib_home.csv')  # Converts bag to csv

topic_name = '/imu'  # Defines the topic name from which to extract data within the ROS bag.
data_list = []  # Initializes an empty list to store data rows extracted from the ROS bag.

# Opens the ROS bag file using a context manager to ensure proper resource management.
with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]): # Iterates over each message in the specified topic within the bag file.
        row = [
            msg.header.stamp.to_sec(),  # Converts ROS timestamp to seconds and adds it to the row.
            msg.orientation.x,  # Extracts the x component of orientation.
            msg.orientation.y,  # Extracts the y component of orientation.
            msg.orientation.z,  # Extracts the z component of orientation.
            msg.orientation.w,  # Extracts the w component of orientation (quaternion format).
            msg.angular_velocity.x,  # Extracts the x component of angular velocity.
            msg.angular_velocity.y,  # Extracts the y component of angular velocity.
            msg.angular_velocity.z,  # Extracts the z component of angular velocity.
            msg.linear_acceleration.x,  # Extracts the x component of linear acceleration.
            msg.linear_acceleration.y,  # Extracts the y component of linear acceleration.
            msg.linear_acceleration.z,  # Extracts the z component of linear acceleration.
            msg.mag_field.magnetic_field.x,  # Extracts the x component of the magnetic field.
            msg.mag_field.magnetic_field.y,  # Extracts the y component of the magnetic field.
            msg.mag_field.magnetic_field.z,  # Extracts the z component of the magnetic field.
            msg.raw_imu_data  # Adds the raw IMU data as a string; further processing may be needed.
        ]
        data_list.append(row)  # Appends the constructed row to the data list.

# Defines the column names for the CSV file to match the data structure.
columns = ['Timestamp', 'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W',
           'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
           'Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z',
           'Magnetic Field X', 'Magnetic Field Y', 'Magnetic Field Z', 'Raw IMU Data']

# Converts the list of data rows into a pandas DataFrame, specifying column names.
df = pd.DataFrame(data_list, columns=columns)

# Saves the DataFrame to a CSV file, without including the index.
df.to_csv(csv_file_path, index=False)

# Prints a confirmation message indicating successful data extraction and saving.
print(f'Data from {topic_name} written to {csv_file_path}')
