#!/usr/bin/env python3

import utm   # Importing utm library -> UTM coordinate conversion
import rospy  # Rospy for ROS (Robot Operating System) functionalities
from gps_driver.msg import Customrtk  # Importing Customrtk message from gps_driver package
import serial  # Serial library for serial communication
import time  # Time library for time-related operations

# Check if GNGGA prefix is present in the input string
def is_gngga_present(input_string):
    gngga_prefix = "$GNGGA"    # Define the GNGGA prefix to be searched
    index = input_string.find(gngga_prefix) # Use the find method to search for the GNGGA prefix in the input string
    if index != -1: # Check if the GNGGA prefix was found (index is not -1)
        return True # Return True if the GNGGA prefix is present in the string
    else:
        print('GNGGA not found in string') # Print a message and return False if the GNGGA prefix is not found
        return False

# Convert degrees and decimal minutes to decimal degrees
def convert_deg_min_to_deg_dec(coordinates_latitudeorlongitude):  # Converts Decimal Minute Second coordinates to decimal degrees
    coordinateString = str(coordinates_latitudeorlongitude)  # Convert input coordinates to string
    if len(coordinateString) <= 12:  # Check if the length of the coordinate string is less than or equal to 12. Reads different than GPGGA
        degrees = coordinateString[:2]  # Extract degrees from coordinate string
        degrees_integer = int(degrees)  # Convert degrees part to integer
    elif len(coordinateString) > 12:  # If length of coordinate string is greater than 12
        degrees = coordinateString[:3]  # Extract the degrees part from the coordinate string
        degrees_integer = int(degrees)  # Convert the degrees part to an integer
    decimalMinutes = coordinateString[-10:]  # Extract the decimal minutes part from the coordinate string
    minutes_conv = float(decimalMinutes) / 60  # Convert decimal minutes to fractional minutes
    decimalDegrees = degrees_integer + minutes_conv  # decimal degrees by adding degrees and fractional minutes
    print(decimalDegrees)  # Print the calculated decimal degrees
    return decimalDegrees  # Return the calculated decimal degrees

def apply_coordinate_sign(value, direction):  # Adjusts sign for latitude/longitude based on direction
    if direction in ['W', 'S']:  # Check if direction is West or South
        return -value  # If so, return the negative of the input value
    print(value)  # If the direction is not West or South, print the input value.
    return value  # Return the input value unchanged if the direction is not West or South

def convert_to_utm(latitude, longitude):  # Converts geographic coordinates to UTM
    if -84 <= latitude <= 80 and -180 <= longitude <= 180:  # Check if latitude and longitude values are in valid ranges
        utmResult = utm.from_latlon(latitude, longitude)  # Convert latitude and longitude to UTM coordinates
        print(utmResult)  # Print the UTM coordinates 
        return utmResult[:4]  # Return the first four elements of utmResult, aka easting, northing, zone number, and zone letter

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

def publish_ros_message(rawData, publisher):
    rospy.logdebug("Received raw data: %s" % rawData)  # Log statement to print the received raw data
    data_fields = rawData.split(',')  # Split the raw data string into fields using comma as separator
    if len(data_fields) < 9:  # Check if the number of data fields is less than 9
        rospy.logwarn("Incomplete GPS data received.")  # If so, give warning message -> used this because code kept failing the past so used to debug
        return  # For debugging purposes

    gps_msg = Customrtk()  # Customrtk message object is created
    gps_msg.header.stamp = rospy.Time.now()  # Set the timestamp of the message to the current ROS time

    gps_msg.latitude = convert_deg_min_to_deg_dec(float(data_fields[2]))  # Convert latitude from DMS to decimal degrees
    gps_msg.longitude = convert_deg_min_to_deg_dec(float(data_fields[4]))  # Convert longitude from DMS to decimal degrees

    gps_msg.latitude = apply_coordinate_sign(gps_msg.latitude, data_fields[3])  # Adjust latitude sign
    gps_msg.longitude = apply_coordinate_sign(gps_msg.longitude, data_fields[5])  # Adjust longitude sign

    utm_coords = convert_to_utm(gps_msg.latitude, gps_msg.longitude)  # Convert geographic coordinates to UTM
    if utm_coords:  # If UTM coordinates are obtained
        gps_msg.utm_easting, gps_msg.utm_northing, gps_msg.zone, gps_msg.letter = utm_coords  # Assign UTM coordinates and zone information to message fields

    gps_msg.fix_quality = int(data_fields[6])  # Assign GPS fix quality
    gps_msg.hdop = float(data_fields[8])  # Assign HDOP (Horizontal Dilution of Precision)

    epoch_time = utcToEpoch(float(data_fields[1]))  # Convert UTC time to epoch time
    gps_msg.current_time = rospy.Time(epoch_time[0], epoch_time[1])  # Set the current time in the message


    # TA suggested doing this for debugging code when things do not work.
    rospy.loginfo(f"Publishing GPS Data: {rawData}")  # Log an info message indicating GPS data being published
    publisher.publish(gps_msg)  # Publish the GPS message
    rospy.loginfo("GPS Data published successfully.")  # Log an info message indicating successful publishing

def read_from_serial(port_address, publisher):
    try:
        port = serial.Serial(port_address, timeout=2)  # Open serial port with timeout of 2.
        rospy.loginfo(f"Opened serial port: {port_address}")  # Log an info message indicating the successful opening of the serial port
        while not rospy.is_shutdown():  # Continue reading serial data until ROS is shut down
            line = port.readline().decode('utf-8').strip()  # Read a line of data from the serial port, decode it from bytes to string, and strip whitespace
            if is_gngga_present(line):  # Check if the line of data contains the GNGGA prefix
                publish_ros_message(line, publisher)  # If it does, call publishGPSData function to process and publish GPS data
    except serial.SerialException as e:  # Handle any serial communication errors
        rospy.logerr(f"Failed to open serial port {port_address} with error: {e}")  # Log error message indicating the failure to open the serial port. -> Needed since this happened a lot.


if __name__ == "__main__":
    try:
        publisher_0 = rospy.Publisher('GPS_Reader', Customrtk, queue_size=10)  # Initialize ROS publisher GPS_Reader 
        rospy.init_node('GPS_Transmit', anonymous=False)  # Initialize ROS node with the name 'GPS_Transmit'
        serial_port_param = rospy.get_param('~serial_port_param', '/dev/pts/6')  # Serial port address parameter from the ROS parameter server
        rospy.loginfo(f"Starting GPS_Transmit node with serial port: {serial_port_param}")  # Indicates start of the GPS_Transmit node
        read_from_serial(serial_port_param, publisher_0)  # Call readSerialData function to read data from specified serial port -> once done publish it
    except rospy.ROSInterruptException:  # Handle any ROS-related interruptions
        rospy.loginfo("GPS_Transmit node terminated.")  # Log an info message indicating the termination of the GPS_Transmit node
