// Auto-generated. Do not edit!

// (in-package vn_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Vectornav {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.orientation = null;
      this.angular_velocity = null;
      this.linear_acceleration = null;
      this.mag_field = null;
      this.raw_imu_data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('linear_acceleration')) {
        this.linear_acceleration = initObj.linear_acceleration
      }
      else {
        this.linear_acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('mag_field')) {
        this.mag_field = initObj.mag_field
      }
      else {
        this.mag_field = new sensor_msgs.msg.MagneticField();
      }
      if (initObj.hasOwnProperty('raw_imu_data')) {
        this.raw_imu_data = initObj.raw_imu_data
      }
      else {
        this.raw_imu_data = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vectornav
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.orientation, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_velocity, buffer, bufferOffset);
    // Serialize message field [linear_acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.linear_acceleration, buffer, bufferOffset);
    // Serialize message field [mag_field]
    bufferOffset = sensor_msgs.msg.MagneticField.serialize(obj.mag_field, buffer, bufferOffset);
    // Serialize message field [raw_imu_data]
    bufferOffset = _serializer.string(obj.raw_imu_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vectornav
    let len;
    let data = new Vectornav(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration]
    data.linear_acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [mag_field]
    data.mag_field = sensor_msgs.msg.MagneticField.deserialize(buffer, bufferOffset);
    // Deserialize message field [raw_imu_data]
    data.raw_imu_data = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.MagneticField.getMessageSize(object.mag_field);
    length += _getByteLength(object.raw_imu_data);
    return length + 84;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vn_driver/Vectornav';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec6b3f22b033bcf54d221a7024af8b27';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # VectorNav IMU and Magnetometer Data
    
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
     # A covariance matrix of all zeros will be interpreted as "covariance unknown",
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Vectornav(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = geometry_msgs.msg.Quaternion.Resolve(msg.orientation)
    }
    else {
      resolved.orientation = new geometry_msgs.msg.Quaternion()
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = geometry_msgs.msg.Vector3.Resolve(msg.angular_velocity)
    }
    else {
      resolved.angular_velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.linear_acceleration !== undefined) {
      resolved.linear_acceleration = geometry_msgs.msg.Vector3.Resolve(msg.linear_acceleration)
    }
    else {
      resolved.linear_acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.mag_field !== undefined) {
      resolved.mag_field = sensor_msgs.msg.MagneticField.Resolve(msg.mag_field)
    }
    else {
      resolved.mag_field = new sensor_msgs.msg.MagneticField()
    }

    if (msg.raw_imu_data !== undefined) {
      resolved.raw_imu_data = msg.raw_imu_data;
    }
    else {
      resolved.raw_imu_data = ''
    }

    return resolved;
    }
};

module.exports = Vectornav;
