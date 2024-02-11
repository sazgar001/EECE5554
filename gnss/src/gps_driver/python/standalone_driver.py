#!/usr/bin/env python3

import utm  # UTM coordinate conversions
import time  # Time-related functions
import serial  # Serial library for communcation
import rospy  # ROS Python library
from gps_driver.msg import Customgps  # Custom ROS message for GPS data

# Check if GPGGA prefix is present in the input string
def is_gpgga_present(input_string):
    gpgga_prefix = "$GPGGA"  # Define the GPGGA prefix to be searched
    index = input_string.find(gpgga_prefix)  # Use the find method to search for the GPGGA prefix in the input string
    if index != -1:  # Check if the GPGGA prefix was found (index is not -1)
        return True  # Return True if the GPGGA prefix is present in the string
    else:
        print('GPGGA not found in string')  # Print a message and return False if the GPGGA prefix is not found
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

def convert_to_utm(latitude, longitude):
    if -84 <= latitude <= 80 and -180 <= longitude <= 180:  # Validate coordinate range
        utm_result = utm.from_latlon(latitude, longitude)  # Perform UTM conversion
        return utm_result[:4]  # Return easting, northing, zone number, and zone letter
    else:
        print("Latitude or longitude out of valid range.")  # Print error if out of range

def utcToEpoch(utcTime):  # Converts UTC to epoch time
    utcString = format(utcTime, '.2f')  # UTC time to a string with two decimal places
    hours = float(utcString[:2])  # Extract hours from UTC string. float number required
    minutes_s = float(utcString[2:4])  # Extract minutes from UTC string. float number required
    seconds = float(utcString[4:6])  # Extract seconds from UTC string. float number required. 
    decimalSeconds = float(utcString[-2:])  # Extract decimal seconds from UTC string
    totalSeconds = hours * 3600 + minutes_s * 60 + seconds + decimalSeconds / 100  # Calculate total seconds since midnight 
    epochStartOfDay = time.time() - (time.time() % 86400) + (5 * 3600)  # Get current epoch time and adjust for timezone difference
    epochTime = epochStartOfDay + totalSeconds  # Calculate epoch time by adding total seconds since midnight to the start of the day
    return [int(epochTime), int((epochTime % int(epochTime)) * 10**9)]  # Return epoch time as a list containing seconds and nanoseconds

# Function to read from serial port and publish GPS data
def read_from_serial(serial_port_addr):
    with serial.Serial(serial_port_addr, 9600, timeout=1) as serial_port:  # Open the serial port with specified settings.
        while not rospy.is_shutdown():  # Keep running until ROS node is active.
            if serial_port.in_waiting > 0:  # Check if data is waiting to be read.
                line = serial_port.readline().decode('utf-8').rstrip()  # Read a line from the serial port, decode it, and remove trailing whitespace.
                if is_gpgga_present(line):  # Check if the line contains GPGGA data (GPS information).
                    publish_ros_message(line)  # Publish the line as a ROS message if it contains GPS data.

# Publish ROS message based on GPGGA string from GPS sensor
def publish_ros_message(gpgga_data):   # Extract data from GPGGA sentence
    parts = gpgga_data.split(',')
    if len(parts) < 15:  # Ensure there are enough parts to parse
        rospy.logwarn("Incomplete GPGGA sentence: " + gpgga_data)
        return
    
    # Extract and convert GPS data
    utc_time = parts[1] # Extracts the UTC time from the GPS data parts array.
    latitude = convert_deg_min_to_deg_dec(parts[2], True) # Converts the latitude from degrees and minutes to decimal degrees format.
    lat_dir = parts[3] # Extracts the direction (N or S) of the latitude.
    longitude = convert_deg_min_to_deg_dec(parts[4], False) # Converts the longitude from degrees and minutes to decimal degrees format.
    long_dir = parts[5] # Extracts the direction (E or W) of the longitude.
    latitude = apply_coordinate_sign(latitude, lat_dir) # Applies the correct sign to the latitude based on the N/S direction.
    longitude = apply_coordinate_sign(longitude, long_dir)  # Applies the correct sign to the longitude based on the E/W direction.

    # Convert latitude and longitude to UTM
    utm_coords = utm.from_latlon(latitude, longitude)
    utm_easting, utm_northing, zone_number, zone_letter = utm_coords[:4]

    # Prepare Customgps message
    gps_msg = Customgps()  # Summon the Customgps file
    gps_msg.header.stamp = rospy.Time.now()  # Sets the timestamp in the header of the message to the current time.
    gps_msg.latitude = latitude  # Assigns the latitude value to the gps_msg object.
    gps_msg.longitude = longitude  # Assigns the longitude value to the gps_msg object.
    gps_msg.utm_easting = utm_easting  # Assigns the UTM easting coordinate to the gps_msg object.
    gps_msg.utm_northing = utm_northing  # Assigns the UTM northing coordinate to the gps_msg object.
    gps_msg.zone = zone_number  # Assigns the UTM zone number to the gps_msg object.
    gps_msg.letter = zone_letter  # Assigns the UTM zone letter to the gps_msg object.
    gps_msg.hdop = float(parts[8]) if parts[8] else 0.0  # Parses the HDOP (Horizontal Dilution of Precision) value from the parts array
                                                         # and assigns it to the gps_msg object, defaults to 0.0 if parts[8] is empty.
    gps_msg.gpgga_read = gpgga_data  # Assigns the GPGGA data string to the gps_msg object for further processing or reference.
    gps_publisher.publish(gps_msg)  # Send the GPS data message.
    rospy.loginfo("Published GPGGA data: " + gpgga_data)  # Log that the data was published.

if __name__ == '__main__':  # starting if statemnt
    try:
        rospy.init_node('GPS Transmitter', anonymous=True)  # Start a new ROS node named 'GPS Transmitter'.
        serial_port_addr = rospy.get_param('~serial_port', '/dev/pts/0')  # Get the serial port address from ROS params or use default.
        gps_publisher = rospy.Publisher('GPS_Reader', Customgps, queue_size=10)  # Set up a publisher for the GPS data.
        read_from_serial(serial_port_addr)  # Read data from the serial port and publish it.
    except rospy.ROSInterruptException:  # Catch the exception if ROS is interrupted (e.g., shutdown).
        pass  # Do nothing if an interrupt exception occurs.

