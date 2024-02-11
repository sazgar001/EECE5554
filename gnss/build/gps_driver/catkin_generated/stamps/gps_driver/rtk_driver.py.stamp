#!/usr/bin/env python3

import utm  # UTM coordinate conversions
import time  # Time-related functions
import serial  # Serial library for communication
import rospy  # ROS Python library
from gps_driver.msg import Customrtk  # Custom ROS message for GPS data

# Check if GNGGA prefix is present in the input string
def is_gngga_present(input_string):
    gngga_prefix = "$GNGGA"  # Define the GNGGA prefix to be searched
    index = input_string.find(gngga_prefix)  # Use the find method to search for the GNGGA prefix in the input string
    if index != -1:  # Check if the GNGGA prefix was found (index is not -1)
        return True  # Return True if the GNGGA prefix is present in the string
    else:
        print('GNGGA not found in string')  # Print a message and return False if the GNGGA prefix is not found
        return False

# Convert degrees and decimal minutes to decimal degrees
def convert_deg_min_to_deg_dec(coord_str, is_latitude):
    coord_str = str(coord_str)  # Convert the input coordinate to a string to ensure consistent handling
    deg_len = 2 if is_latitude else 3  # Determine the length of the degrees based on latitude or longitude
    
    degrees = int(coord_str[:deg_len])  # Extract the degrees portion and convert it to an integer
    dec_minutes = float(coord_str[deg_len:])  # Extract the decimal minutes portion and convert it to a float
    dec_degrees = degrees + dec_minutes / 60  # Calculate the decimal degrees
    return dec_degrees

# Apply sign convention based on coordinate direction
def apply_coordinate_sign(coord_value, coord_dir):
    if coord_dir in ["W", "S"]:  # If west or south, make the value negative
        return -coord_value
    return coord_value

# Function to read from serial port and publish GPS data
def read_from_serial(serial_port_addr):
    with serial.Serial(serial_port_addr, 9600, timeout=1) as serial_port:  # Open the serial port with specified settings.
        while not rospy.is_shutdown():  # Keep running until ROS node is active.
            if serial_port.in_waiting > 0:  # Check if data is waiting to be read.
                line = serial_port.readline().decode('utf-8').rstrip()  # Read a line from the serial port, decode it, and remove trailing whitespace.
                if is_gngga_present(line):  # Check if the line contains GNGGA data (RTK GPS information).
                    publish_ros_message(line)  # Publish the line as a ROS message if it contains GPS data.

# Publish ROS message based on GNGGA string from RTK GNSS sensor
def publish_ros_message(gngga_data):
    parts = gngga_data.split(',')
    if len(parts) < 15:  # Ensure there are enough parts to parse
        rospy.logwarn("Incomplete GNGGA sentence: " + gngga_data)
        return
    
    # Extract and convert GPS data
    utc_time = parts[1]  # Extracts the UTC time from the GPS data parts array.
    latitude = convert_deg_min_to_deg_dec(parts[2], True)  # Converts the latitude to decimal degrees.
    lat_dir = parts[3]  # Extracts the direction (N or S) of the latitude.
    longitude = convert_deg_min_to_deg_dec(parts[4], False)  # Converts the longitude to decimal degrees.
    long_dir = parts[5]  # Extracts the direction (E or W) of the longitude.
    fix_quality = int(parts[6])  # Extracts the fix quality.
    hdop = float(parts[8])  # Extracts the HDOP value.
    altitude = float(parts[9])  # Extracts the altitude.
    
    latitude = apply_coordinate_sign(latitude, lat_dir)  # Applies the correct sign to the latitude.
    longitude = apply_coordinate_sign(longitude, long_dir)  # Applies the correct sign to the longitude.
    
    # Convert latitude and longitude to UTM
    utm_coords = utm.from_latlon(latitude, longitude)
    utm_easting, utm_northing, zone_number, zone_letter = utm_coords[:4]

    # Prepare Customgps message
    gps_msg = Customrtk()
    gps_msg.header.stamp = rospy.Time.now()
    gps_msg.latitude = latitude
    gps_msg.longitude = longitude
    gps_msg.altitude = altitude
    gps_msg.utm_easting = utm_easting
    gps_msg.utm_northing = utm_northing
    gps_msg.zone = zone_number
    gps_msg.letter = zone_letter
    gps_msg.hdop = hdop
    gps_msg.fix_quality = fix_quality
    gps_msg.gpgga_read = gngga_data  # Assigns the GNGGA data string for reference.
    gps_publisher.publish(gps_msg)  # Send the GPS data message.
    rospy.loginfo("Published GNGGA data: " + gngga_data)  # Log the published data.

if __name__ == '__main__':
    try:
        rospy.init_node('RTK_GNSS_Transmitter', anonymous=True)
        serial_port_addr = rospy.get_param('~serial_port', '/dev/pts/0')  # Get the serial port address from ROS params.
        gps_publisher = rospy.Publisher('RTK_GNSS_Reader', Customrtk, queue_size=10)  # Set up a publisher for the RTK GNSS data.
        read_from_serial(serial_port_addr)  # Read data from the serial port and publish it.
    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exception gracefully.
