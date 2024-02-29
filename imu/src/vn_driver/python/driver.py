#!/usr/bin/env python3

import serial  # Import the serial library 
import rospy  # Import the rospy library 
from vn_driver.msg import Vectornav  # Import Vectornav message from vn_driver
import math  # Import the math library 
import atexit  # Import the atexit library -> cleanup actions when the script exits.

ser = None  # Initialize the serial port variable to None.

def convert_to_quaternion(yaw, pitch, roll):  # Function to convert Euler angles (yaw, pitch, roll) to quaternion (qw, qx, qy, qz).
    yaw_rad = math.radians(yaw)  # Convert yaw angle to radians.
    pitch_rad = math.radians(pitch)  # Convert pitch angle to radians.
    roll_rad = math.radians(roll)  # Convert roll angle to radians.
    
    # Calculate the components of the quaternion using the converted angles.
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy  # Quaternion component w
    qx = sr * cp * cy - cr * sp * sy  # Quaternion component x
    qy = cr * sp * cy + sr * cp * sy  # Quaternion component y
    qz = cr * cp * sy - sr * sp * cy  # Quaternion component z
    return qw, qx, qy, qz  # Return the quaternion components.

def imu_callback(msg): # Callback function to handle received IMU data.
    rospy.loginfo("IMU Data Received: Orientation - w: {}, x: {}, y: {}, z: {}".format(      
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z))  # Logs the orientation data received from the IMU.

def send_initial_command(ser):     # Function to send initial command to IMU.
    initial_command = '$VNWRG,07,40*59\r\n'  # Command to set the data output frequency to 40 Hz.
    rospy.loginfo("Sending initial command to IMU: " + initial_command)  # Log the command being sent.
    ser.write(initial_command.encode())  # Send the command over the serial connection.

def send_final_command():     # Function to send a final command to the IMU upon exit.
    final_command = '$VNWRG,06,14*XX\r\n'  # Command template; 'XX' should be replaced with the actual checksum.
    if ser and ser.isOpen():
        rospy.loginfo("Sending final command to IMU: " + final_command)  # Log the command being sent.
        ser.write(final_command.encode())  # Send the command over the serial connection.

def read_and_publish_imu_data(ser):   # Function to read IMU data from serial port and publish it as ROS messages.
    pub = rospy.Publisher('/imu', Vectornav, queue_size=10)  # Define a ROS publisher for the IMU data.
    rospy.Subscriber("/imu", Vectornav, imu_callback)  # Define a ROS subscriber to listen to IMU data.
    rate = rospy.Rate(40)  # Set the publishing rate to 40Hz.

    while not rospy.is_shutdown():    # Loop to continuously read and publish IMU data until ROS is shutdown or cancelled (Control C)
        if ser.in_waiting > 0:
            line = ser.readline()  # Read a line from the serial port.
            rospy.loginfo("Received line: " + repr(line))  # Log the received line.
            line_str = line.decode('utf-8').strip()  # Decode and strip the line to a string.
            
            parts = line_str.split(',')[1:]  # Split the string by commas and skip the first element.
            parts = [p.split('*')[0] for p in parts]  # Remove the checksum from each part.
            
            if len(parts) >= 11: # Ensure that there are at least 11 parts (data fields) before proceeding.
                yaw, pitch, roll, magnet_x, magnet_y, magnet_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = map(float, parts[:12]) # Convert the first 12 parts to floats and assign them to variables.
                qw, qx, qy, qz = convert_to_quaternion(yaw, pitch, roll)  # Convert Euler angles to quaternion.
                imu_msg = Vectornav()  # Create a new Vectornav message.
                
                # Populate the message fields with the converted and received data.
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "imu_link"
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z
                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z
                imu_msg.mag_field.magnetic_field.x = magnet_x
                imu_msg.mag_field.magnetic_field.y = magnet_y
                imu_msg.mag_field.magnetic_field.z = magnet_z
                imu_msg.mag_field.header.stamp = rospy.Time.now()
                imu_msg.mag_field.header.frame_id = "imu_link"
                imu_msg.raw_imu_data = line_str  # Include the raw IMU data string in the message.
                pub.publish(imu_msg)  # Publish the Vectornav message.
        else:
            rospy.logwarn("Incomplete data received: not enough parts")  # Warn if incomplete data is received.
        rate.sleep()  # Sleep for the remainder of the loop rate.

if __name__ == '__main__': # Main block to initialize the node, open the serial connection, and start reading/publishing IMU data.
    try:
        rospy.init_node('vn_driver_node', anonymous=True)  # Initialize a ROS node.
        port = rospy.get_param('~port', '/dev/ttyUSB0')  # Get the serial port parameter, default to '/dev/ttyUSB0'.
        baudrate = 115200  # Define the baud rate for serial communication.
        ser = serial.Serial(port, baudrate, timeout=1)  # Open the serial port with the specified parameters.
        rospy.loginfo("Serial port opened successfully.")  # Log success message.
        send_initial_command(ser)  # Send the initial command to configure the IMU.
        atexit.register(send_final_command)  # Register function to send final command upon exit.
        read_and_publish_imu_data(ser)  # Start the loop to read and publish IMU data.
    except serial.SerialException as e:
        rospy.logerr("Failed to open serial port {}: {}".format(port, e))  # Log error if serial port fails to open.
    except rospy.ROSInterruptException:
        pass  # Quietly handle the ROS interrupt exception.
    finally:
        if ser and ser.isOpen():
            ser.close()  # Close the serial port if it's open.
            rospy.loginfo("Serial port closed.")  # Log the serial port closure.
