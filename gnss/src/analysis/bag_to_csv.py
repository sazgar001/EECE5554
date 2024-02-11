import rosbag  
import pandas as pd  
import os  

# This code is to convert ROS Bags into CSV Files 
# Specify the ROS bag path
bag_path = os.path.expanduser('~/gnss/src/data/Walk_RTK_Sakib.bag') # Bag is in data folder. Change file of bag when needed

# Specify the output CSV file path
csv_file_path = os.path.expanduser('~/gnss/src/data/Walk_RTK_Sakib.csv')  # Want to save into data. Change file of bag when needed


topic_name = '/GPS_Reader' # The topic containing Customrtk messages
data_list = [] # Initialize a list to store data

# Open the ROS bag file
with rosbag.Bag(bag_path) as bag:  # Open the bag file using a context manager
    # Iterate over each message in the specified topic within the bag file
    for topic, msg, t in bag.read_messages(topics=[topic_name]): # Extract relevant fields from the Customrtk message and append them to the data list
        row = [
            msg.header.stamp.to_sec(),  # Timestamp of the message
            msg.latitude,  # Latitude value
            msg.longitude,  # Longitude value
            msg.altitude,  # Altitude value
            msg.utm_easting,  # UTM Easting value
            msg.utm_northing,  # UTM Northing value
            msg.zone,  # UTM Zone
            msg.letter,  # UTM Letter
            msg.hdop,  # HDOP (Horizontal Dilution of Precision)
            msg.gngga_read,  # GNGGA Read
            msg.fix_quality,  # Fix Quality
            msg.current_time.to_sec()  # Current Time
        ]
        data_list.append(row)  # Append the extracted data row to the list

# Convert the list to a pandas DataFrame
columns = ['Timestamp', 'Latitude', 'Longitude', 'Altitude', 'UTM Easting', 'UTM Northing',
           'Zone', 'Letter', 'HDOP', 'GNGGA Read', 'Fix Quality', 'Current Time']
df = pd.DataFrame(data_list, columns=columns)  # Create a DataFrame from the data list

# Save the DataFrame to a CSV file
df.to_csv(csv_file_path, index=False)  # Write the DataFrame to a CSV file without including the index

print(f'Data from {topic_name} written to {csv_file_path}')  # Print a message indicating successful completion
