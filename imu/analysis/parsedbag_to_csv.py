# Conversion code of Parsed ROS Location B Bag

import rosbag  # Import the rosbag library for working with ROS bag files
import pandas as pd  # Import the pandas library 
import os  # Import the os module 

# Specify the ROS bag path and output CSV file path
bag_path = os.path.expanduser('~/imu/data/parsed_location_B.bag')  # Path to the ROS bag file
csv_file_path = os.path.expanduser('~/imu/data/parsed_location_B.csv')  # Path to the output CSV file

topic_name = '/vectornav'  # topic name where IMU data is stored in the bag file
data_list = []  # Initialize empty list to store IMU data

# Open the ROS bag file
with rosbag.Bag(bag_path) as bag:  # Opens the ROS bag file for reading
    # Iterate over each message in the specified topic within the bag file
    for topic, msg, t in bag.read_messages(topics=[topic_name]):   # Extract relevant data from the message and store it in a list
        row = [
            msg.header.stamp.to_sec(),  # Timestamp of the message
            msg.orientation.x,  # X component of the orientation
            msg.orientation.y,  # Y component of the orientation
            msg.orientation.z,  # Z component of the orientation
            msg.orientation.w,  # W component of the orientation
            msg.angular_velocity.x,  # X component of the angular velocity
            msg.angular_velocity.y,  # Y component of the angular velocity
            msg.angular_velocity.z,  # Z component of the angular velocity
            msg.linear_acceleration.x,  # X component of the linear acceleration
            msg.linear_acceleration.y,  # Y component of the linear acceleration
            msg.linear_acceleration.z  # Z component of the linear acceleration
        ]
        data_list.append(row)  # Append the extracted data row to the list

# Define the column names for the CSV file
columns = ['Timestamp', 'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W',
           'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
           'Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z']

# Convert the list of data into a pandas DataFrame
df = pd.DataFrame(data_list, columns=columns)  # Create a DataFrame from the list of data

# Save the DataFrame to a CSV file
df.to_csv(csv_file_path, index=False)  # Write the DataFrame to a CSV file without including the index

# Print a message indicating successful writing of data to the CSV file
print(f'Data from {topic_name} written to {csv_file_path}')
