# VectorNav IMU and Magnetometer Data

Header header                               # Standard message header
geometry_msgs/Quaternion orientation        # Orientation in quaternion (w, x, y, z)
geometry_msgs/Vector3 angular_velocity      # Angular velocity
geometry_msgs/Vector3 linear_acceleration   # Linear acceleration
sensor_msgs/MagneticField mag_field         # Magnetometer data
string raw_imu_data                        # Raw IMU data string for debugging/analysis

