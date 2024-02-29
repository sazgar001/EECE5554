;; Auto-generated. Do not edit!


(when (boundp 'vn_driver::Vectornav)
  (if (not (find-package "VN_DRIVER"))
    (make-package "VN_DRIVER"))
  (shadow 'Vectornav (find-package "VN_DRIVER")))
(unless (find-package "VN_DRIVER::VECTORNAV")
  (make-package "VN_DRIVER::VECTORNAV"))

(in-package "ROS")
;;//! \htmlinclude Vectornav.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vn_driver::Vectornav
  :super ros::object
  :slots (_header _orientation _angular_velocity _linear_acceleration _mag_field _raw_imu_data ))

(defmethod vn_driver::Vectornav
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:orientation __orientation) (instance geometry_msgs::Quaternion :init))
    ((:angular_velocity __angular_velocity) (instance geometry_msgs::Vector3 :init))
    ((:linear_acceleration __linear_acceleration) (instance geometry_msgs::Vector3 :init))
    ((:mag_field __mag_field) (instance sensor_msgs::MagneticField :init))
    ((:raw_imu_data __raw_imu_data) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _orientation __orientation)
   (setq _angular_velocity __angular_velocity)
   (setq _linear_acceleration __linear_acceleration)
   (setq _mag_field __mag_field)
   (setq _raw_imu_data (string __raw_imu_data))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:angular_velocity
   (&rest __angular_velocity)
   (if (keywordp (car __angular_velocity))
       (send* _angular_velocity __angular_velocity)
     (progn
       (if __angular_velocity (setq _angular_velocity (car __angular_velocity)))
       _angular_velocity)))
  (:linear_acceleration
   (&rest __linear_acceleration)
   (if (keywordp (car __linear_acceleration))
       (send* _linear_acceleration __linear_acceleration)
     (progn
       (if __linear_acceleration (setq _linear_acceleration (car __linear_acceleration)))
       _linear_acceleration)))
  (:mag_field
   (&rest __mag_field)
   (if (keywordp (car __mag_field))
       (send* _mag_field __mag_field)
     (progn
       (if __mag_field (setq _mag_field (car __mag_field)))
       _mag_field)))
  (:raw_imu_data
   (&optional __raw_imu_data)
   (if __raw_imu_data (setq _raw_imu_data __raw_imu_data)) _raw_imu_data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Quaternion _orientation
    (send _orientation :serialization-length)
    ;; geometry_msgs/Vector3 _angular_velocity
    (send _angular_velocity :serialization-length)
    ;; geometry_msgs/Vector3 _linear_acceleration
    (send _linear_acceleration :serialization-length)
    ;; sensor_msgs/MagneticField _mag_field
    (send _mag_field :serialization-length)
    ;; string _raw_imu_data
    4 (length _raw_imu_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Quaternion _orientation
       (send _orientation :serialize s)
     ;; geometry_msgs/Vector3 _angular_velocity
       (send _angular_velocity :serialize s)
     ;; geometry_msgs/Vector3 _linear_acceleration
       (send _linear_acceleration :serialize s)
     ;; sensor_msgs/MagneticField _mag_field
       (send _mag_field :serialize s)
     ;; string _raw_imu_data
       (write-long (length _raw_imu_data) s) (princ _raw_imu_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Quaternion _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;; geometry_msgs/Vector3 _angular_velocity
     (send _angular_velocity :deserialize buf ptr-) (incf ptr- (send _angular_velocity :serialization-length))
   ;; geometry_msgs/Vector3 _linear_acceleration
     (send _linear_acceleration :deserialize buf ptr-) (incf ptr- (send _linear_acceleration :serialization-length))
   ;; sensor_msgs/MagneticField _mag_field
     (send _mag_field :deserialize buf ptr-) (incf ptr- (send _mag_field :serialization-length))
   ;; string _raw_imu_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _raw_imu_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get vn_driver::Vectornav :md5sum-) "ec6b3f22b033bcf54d221a7024af8b27")
(setf (get vn_driver::Vectornav :datatype-) "vn_driver/Vectornav")
(setf (get vn_driver::Vectornav :definition-)
      "# VectorNav IMU and Magnetometer Data

Header header                               # Standard message header
geometry_msgs/Quaternion orientation        # Orientation in quaternion (w, x, y, z)
geometry_msgs/Vector3 angular_velocity      # Angular velocity
geometry_msgs/Vector3 linear_acceleration   # Linear acceleration
sensor_msgs/MagneticField mag_field         # Magnetometer data
string raw_imu_data                        # Raw IMU data string for debugging/analysis


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/MagneticField
 # Measurement of the Magnetic Field vector at a specific location.

 # If the covariance of the measurement is known, it should be filled in
 # (if all you know is the variance of each measurement, e.g. from the datasheet,
 #just put those along the diagonal)
 # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",
 # and to use the data a covariance will have to be assumed or gotten from some
 # other source


 Header header                        # timestamp is the time the
                                      # field was measured
                                      # frame_id is the location and orientation
                                      # of the field measurement

 geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                      # field vector in Tesla
                                      # If your sensor does not output 3 axes,
                                      # put NaNs in the components not reported.

 float64[9] magnetic_field_covariance # Row major about x, y, z axes
                                      # 0 is interpreted as variance unknown
")



(provide :vn_driver/Vectornav "ec6b3f22b033bcf54d221a7024af8b27")


