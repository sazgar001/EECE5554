# Converting bag file to parsed bag file

import pandas as pd  # Imports the pandas library
import numpy as np  # Imports the numpy library 
import re  # Imports the module 

def euler_to_quaternion(yaw, pitch, roll):
    """
    Converts Euler angles (yaw, pitch, roll) from degrees to quaternions.
    """
    # Converts yaw, pitch, and roll from degrees to radians.
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)
    roll_rad = np.radians(roll)
    
    # Calculates the components of the quaternion based on the converted radian values.
    cy = np.cos(yaw_rad * 0.5)
    sy = np.sin(yaw_rad * 0.5)
    cp = np.cos(pitch_rad * 0.5)
    sp = np.sin(pitch_rad * 0.5)
    cr = np.cos(roll_rad * 0.5)
    sr = np.sin(roll_rad * 0.5)
    
    # Computes the quaternion components using the formula for conversion from Euler angles to quaternion.
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz     # Returns the quaternion components.

def process_imu_string(imu_string):
    """
    Parses an IMU string and converts the contained Euler angles to quaternions.
    """
    if not isinstance(imu_string, str):     # Checks if the input is a string; if not, returns None.
        return None
    match = re.match(r"\$VNYMR,([-\d.]+),([-\d.]+),([-\d.]+),.*", imu_string) # Uses regular expression to match the IMU string pattern and extract yaw, pitch, and roll values.
    if match:
        yaw, pitch, roll = map(float, match.groups()) # Converts the matched string groups (yaw, pitch, roll) to floats and then to quaternion.
        return euler_to_quaternion(yaw, pitch, roll)
    else:
        return None # If the string doesn't match the pattern, returns None.

def process_imu_string_safe(imu_string):
    """Safely processes an IMU string by checking if it's a string before parsing. """
    return process_imu_string(imu_string) # Calls process_imu_string directly, effectively acting as an alias. 
location_b_df = pd.read_csv('~/imu/data/location_B.csv') # Load the CSV file containing IMU data into a DataFrame.
quaternion_data = location_b_df['IMU String'].apply(process_imu_string_safe) # Applies the safe processing function to each IMU string in the DataFrame, converting Euler angles to quaternion.

# Filters out None results and creates a new DataFrame with the quaternion data.
quaternion_df = pd.DataFrame([d for d in quaternion_data if d is not None], columns=['Orientation W', 'Orientation X', 'Orientation Y', 'Orientation Z'])
print(quaternion_df.head()) # Prints the first few rows of the newly created quaternion DataFrame to verify the results.
