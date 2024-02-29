; Auto-generated. Do not edit!


(cl:in-package vn_driver-msg)


;//! \htmlinclude Vectornav.msg.html

(cl:defclass <Vectornav> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (angular_velocity
    :reader angular_velocity
    :initarg :angular_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (linear_acceleration
    :reader linear_acceleration
    :initarg :linear_acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (mag_field
    :reader mag_field
    :initarg :mag_field
    :type sensor_msgs-msg:MagneticField
    :initform (cl:make-instance 'sensor_msgs-msg:MagneticField))
   (raw_imu_data
    :reader raw_imu_data
    :initarg :raw_imu_data
    :type cl:string
    :initform ""))
)

(cl:defclass Vectornav (<Vectornav>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vectornav>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vectornav)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vn_driver-msg:<Vectornav> is deprecated: use vn_driver-msg:Vectornav instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Vectornav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vn_driver-msg:header-val is deprecated.  Use vn_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <Vectornav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vn_driver-msg:orientation-val is deprecated.  Use vn_driver-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'angular_velocity-val :lambda-list '(m))
(cl:defmethod angular_velocity-val ((m <Vectornav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vn_driver-msg:angular_velocity-val is deprecated.  Use vn_driver-msg:angular_velocity instead.")
  (angular_velocity m))

(cl:ensure-generic-function 'linear_acceleration-val :lambda-list '(m))
(cl:defmethod linear_acceleration-val ((m <Vectornav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vn_driver-msg:linear_acceleration-val is deprecated.  Use vn_driver-msg:linear_acceleration instead.")
  (linear_acceleration m))

(cl:ensure-generic-function 'mag_field-val :lambda-list '(m))
(cl:defmethod mag_field-val ((m <Vectornav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vn_driver-msg:mag_field-val is deprecated.  Use vn_driver-msg:mag_field instead.")
  (mag_field m))

(cl:ensure-generic-function 'raw_imu_data-val :lambda-list '(m))
(cl:defmethod raw_imu_data-val ((m <Vectornav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vn_driver-msg:raw_imu_data-val is deprecated.  Use vn_driver-msg:raw_imu_data instead.")
  (raw_imu_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vectornav>) ostream)
  "Serializes a message object of type '<Vectornav>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear_acceleration) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mag_field) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'raw_imu_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'raw_imu_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vectornav>) istream)
  "Deserializes a message object of type '<Vectornav>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear_acceleration) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mag_field) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'raw_imu_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'raw_imu_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vectornav>)))
  "Returns string type for a message object of type '<Vectornav>"
  "vn_driver/Vectornav")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vectornav)))
  "Returns string type for a message object of type 'Vectornav"
  "vn_driver/Vectornav")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vectornav>)))
  "Returns md5sum for a message object of type '<Vectornav>"
  "ec6b3f22b033bcf54d221a7024af8b27")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vectornav)))
  "Returns md5sum for a message object of type 'Vectornav"
  "ec6b3f22b033bcf54d221a7024af8b27")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vectornav>)))
  "Returns full string definition for message of type '<Vectornav>"
  (cl:format cl:nil "# VectorNav IMU and Magnetometer Data~%~%Header header                               # Standard message header~%geometry_msgs/Quaternion orientation        # Orientation in quaternion (w, x, y, z)~%geometry_msgs/Vector3 angular_velocity      # Angular velocity~%geometry_msgs/Vector3 linear_acceleration   # Linear acceleration~%sensor_msgs/MagneticField mag_field         # Magnetometer data~%string raw_imu_data                        # Raw IMU data string for debugging/analysis~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/MagneticField~% # Measurement of the Magnetic Field vector at a specific location.~%~% # If the covariance of the measurement is known, it should be filled in~% # (if all you know is the variance of each measurement, e.g. from the datasheet,~% #just put those along the diagonal)~% # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",~% # and to use the data a covariance will have to be assumed or gotten from some~% # other source~%~%~% Header header                        # timestamp is the time the~%                                      # field was measured~%                                      # frame_id is the location and orientation~%                                      # of the field measurement~%~% geometry_msgs/Vector3 magnetic_field # x, y, and z components of the~%                                      # field vector in Tesla~%                                      # If your sensor does not output 3 axes,~%                                      # put NaNs in the components not reported.~%~% float64[9] magnetic_field_covariance # Row major about x, y, z axes~%                                      # 0 is interpreted as variance unknown~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vectornav)))
  "Returns full string definition for message of type 'Vectornav"
  (cl:format cl:nil "# VectorNav IMU and Magnetometer Data~%~%Header header                               # Standard message header~%geometry_msgs/Quaternion orientation        # Orientation in quaternion (w, x, y, z)~%geometry_msgs/Vector3 angular_velocity      # Angular velocity~%geometry_msgs/Vector3 linear_acceleration   # Linear acceleration~%sensor_msgs/MagneticField mag_field         # Magnetometer data~%string raw_imu_data                        # Raw IMU data string for debugging/analysis~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/MagneticField~% # Measurement of the Magnetic Field vector at a specific location.~%~% # If the covariance of the measurement is known, it should be filled in~% # (if all you know is the variance of each measurement, e.g. from the datasheet,~% #just put those along the diagonal)~% # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",~% # and to use the data a covariance will have to be assumed or gotten from some~% # other source~%~%~% Header header                        # timestamp is the time the~%                                      # field was measured~%                                      # frame_id is the location and orientation~%                                      # of the field measurement~%~% geometry_msgs/Vector3 magnetic_field # x, y, and z components of the~%                                      # field vector in Tesla~%                                      # If your sensor does not output 3 axes,~%                                      # put NaNs in the components not reported.~%~% float64[9] magnetic_field_covariance # Row major about x, y, z axes~%                                      # 0 is interpreted as variance unknown~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vectornav>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear_acceleration))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mag_field))
     4 (cl:length (cl:slot-value msg 'raw_imu_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vectornav>))
  "Converts a ROS message object to a list"
  (cl:list 'Vectornav
    (cl:cons ':header (header msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':angular_velocity (angular_velocity msg))
    (cl:cons ':linear_acceleration (linear_acceleration msg))
    (cl:cons ':mag_field (mag_field msg))
    (cl:cons ':raw_imu_data (raw_imu_data msg))
))
