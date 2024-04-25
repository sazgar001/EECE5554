import rosbag
import pandas as pd
import re

# Function to parse the VNYMR string
def parse_vnymr_string(vnymr_string):
    pattern = r"\$VNYMR,([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+),([+\-]?\d+\.\d+)\*\d+"
    match = re.match(pattern, vnymr_string)
    if match:
        return match.groups()
    else:
        return [None] * 12

# Path to your ROS bag file
bag_path = '/home/sakib/punching_bag/data/testPunch_2024-04-04-09-18-01.bag'
csv_file_path = '/home/sakib/punching_bag/data/testpunch2_parsed.csv'

# Adjust the topic name as per your ROS bag's data
topic_name = '/imu'  # Topic name as per ROS bag file
data_list = []

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # Check for the attribute that contains the VN_String
        if hasattr(msg, 'VN_String'):
            vnymr_string = str(msg.VN_String)
            if "$VNYMR" in vnymr_string:
                parsed_data = parse_vnymr_string(vnymr_string)
                row = [t.to_sec()] + list(parsed_data) + [vnymr_string]
                data_list.append(row)

# Define column names based on expected data
columns = [
    'Timestamp',
    'Yaw',
    'Pitch',
    'Roll',
    'Angular Velocity X',
    'Angular Velocity Y',
    'Angular Velocity Z',
    'Linear Acceleration X',
    'Linear Acceleration Y',
    'Linear Acceleration Z',
    'Magnetic Field X',
    'Magnetic Field Y',
    'Magnetic Field Z',
    'Original String'
]

# Convert data into a pandas DataFrame
df = pd.DataFrame(data_list, columns=columns)
# Save DataFrame to CSV
df.to_csv(csv_file_path, index=False)

print(f'Data from {topic_name} written to {csv_file_path}')
