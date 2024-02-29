#!/usr/bin/env python3

import serial
import rospy
from vn_driver.msg import Vectornav  # Make sure this import path matches your setup
import math
import atexit

ser = None

def convert_to_quaternion(roll, pitch, yaw):
    r, p, y = map(lambda a: math.radians(a) / 2, [roll, pitch, yaw])
    qw = math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y)
    qx = math.sin(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y)
    qy = math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y)
    qz = math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y)
    return qw, qx, qy, qz

def imu_callback(msg):
    rospy.loginfo("IMU Data Received: Orientation - w: {}, x: {}, y: {}, z: {}".format(
                  msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z))

def send_initial_command():
    initial_command = '$VNWRG,06,14*5C\r\n'
    rospy.loginfo("Sending initial command to IMU: {}".format(initial_command))
    ser.write(initial_command.encode())

def send_final_command():
    final_command = '$VNWRG,06,14*XX\r\n'  # Replace "XX" with the actual checksum
    if ser and ser.isOpen():
        rospy.loginfo("Sending final command to IMU: {}".format(final_command))
        ser.write(final_command.encode())

def read_and_publish_imu_data():
    pub = rospy.Publisher('/imu', Vectornav, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            rospy.loginfo(f"Received line: {line}")
            
            parts = line.split(',')[1:]  # Skip the first element if it's consistent with your data format
            rospy.loginfo(f"Parts: {parts}")  # Log the parts for debugging
            parts = [p.split('*')[0] for p in parts]  # Remove the checksum from each part
            
            if len(parts) >= 12:  # Adjust based on the expected number of data points
                try:
                    roll, pitch, yaw, magnet_x, magnet_y, magnet_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = map(float, parts[:12])

                    qw, qx, qy, qz = convert_to_quaternion(roll, pitch, yaw)

                    imu_msg = Vectornav()
                    imu_msg.header.stamp = rospy.Time.now()
                    imu_msg.header.frame_id = "imu_link"

                    imu_msg.orientation.w = qw
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz

                    imu_msg.angular_velocity.x = gyro_x
                    imu_msg.angular_velocity.y = gyro_y
                    imu_msg.angular_velocity.z = gyro_z

                    imu_msg.linear_acceleration.x = accel_x
                    imu_msg.linear_acceleration.y = accel_y
                    imu_msg.linear_acceleration.z = accel_z

                    # Assuming mag_field is a part of your custom Vectornav message
                    imu_msg.mag_field.magnetic_field.x = magnet_x
                    imu_msg.mag_field.magnetic_field.y = magnet_y
                    imu_msg.mag_field.magnetic_field.z = magnet_z
                    imu_msg.mag_field.header.stamp = rospy.Time.now()
                    imu_msg.mag_field.header.frame_id = "imu_link"

                    pub.publish(imu_msg)
                except ValueError as e:
                    rospy.logwarn("Data conversion error: {}".format(e))
            else:
                rospy.logwarn("Incomplete data received: not enough parts")
        else:
            rospy.logdebug("No data available on the serial port.")

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('vn_driver_node', anonymous=True)
        port = rospy.get_param('~port', '/dev/pts/0')
        baudrate = 115200
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo("Serial port opened successfully.")
        
        send_initial_command()
        atexit.register(send_final_command)
        
        read_and_publish_imu_data()

    except serial.SerialException as e:
        rospy.logerr("Failed to open serial port {}: {}".format(port, e))
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down vn_driver_node.")
    finally:
        if ser and ser.isOpen():
            ser.close()
            rospy.loginfo("Serial port closed.")