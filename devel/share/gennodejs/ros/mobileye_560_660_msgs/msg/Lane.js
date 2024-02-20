// Auto-generated. Do not edit!

// (in-package mobileye_560_660_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Lane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lane_curvature = null;
      this.lane_heading = null;
      this.construction_area = null;
      this.pitch_angle = null;
      this.yaw_angle = null;
      this.right_ldw_availability = null;
      this.left_ldw_availability = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lane_curvature')) {
        this.lane_curvature = initObj.lane_curvature
      }
      else {
        this.lane_curvature = 0.0;
      }
      if (initObj.hasOwnProperty('lane_heading')) {
        this.lane_heading = initObj.lane_heading
      }
      else {
        this.lane_heading = 0.0;
      }
      if (initObj.hasOwnProperty('construction_area')) {
        this.construction_area = initObj.construction_area
      }
      else {
        this.construction_area = false;
      }
      if (initObj.hasOwnProperty('pitch_angle')) {
        this.pitch_angle = initObj.pitch_angle
      }
      else {
        this.pitch_angle = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_angle')) {
        this.yaw_angle = initObj.yaw_angle
      }
      else {
        this.yaw_angle = 0.0;
      }
      if (initObj.hasOwnProperty('right_ldw_availability')) {
        this.right_ldw_availability = initObj.right_ldw_availability
      }
      else {
        this.right_ldw_availability = false;
      }
      if (initObj.hasOwnProperty('left_ldw_availability')) {
        this.left_ldw_availability = initObj.left_ldw_availability
      }
      else {
        this.left_ldw_availability = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lane
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lane_curvature]
    bufferOffset = _serializer.float64(obj.lane_curvature, buffer, bufferOffset);
    // Serialize message field [lane_heading]
    bufferOffset = _serializer.float64(obj.lane_heading, buffer, bufferOffset);
    // Serialize message field [construction_area]
    bufferOffset = _serializer.bool(obj.construction_area, buffer, bufferOffset);
    // Serialize message field [pitch_angle]
    bufferOffset = _serializer.float64(obj.pitch_angle, buffer, bufferOffset);
    // Serialize message field [yaw_angle]
    bufferOffset = _serializer.float64(obj.yaw_angle, buffer, bufferOffset);
    // Serialize message field [right_ldw_availability]
    bufferOffset = _serializer.bool(obj.right_ldw_availability, buffer, bufferOffset);
    // Serialize message field [left_ldw_availability]
    bufferOffset = _serializer.bool(obj.left_ldw_availability, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lane
    let len;
    let data = new Lane(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_curvature]
    data.lane_curvature = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lane_heading]
    data.lane_heading = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [construction_area]
    data.construction_area = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pitch_angle]
    data.pitch_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_angle]
    data.yaw_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_ldw_availability]
    data.right_ldw_availability = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_ldw_availability]
    data.left_ldw_availability = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 35;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/Lane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86e77b353e33571bb5143fbccbe07372';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float64 lane_curvature
    float64 lane_heading
    bool construction_area
    float64 pitch_angle
    float64 yaw_angle
    bool right_ldw_availability
    bool left_ldw_availability
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lane(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lane_curvature !== undefined) {
      resolved.lane_curvature = msg.lane_curvature;
    }
    else {
      resolved.lane_curvature = 0.0
    }

    if (msg.lane_heading !== undefined) {
      resolved.lane_heading = msg.lane_heading;
    }
    else {
      resolved.lane_heading = 0.0
    }

    if (msg.construction_area !== undefined) {
      resolved.construction_area = msg.construction_area;
    }
    else {
      resolved.construction_area = false
    }

    if (msg.pitch_angle !== undefined) {
      resolved.pitch_angle = msg.pitch_angle;
    }
    else {
      resolved.pitch_angle = 0.0
    }

    if (msg.yaw_angle !== undefined) {
      resolved.yaw_angle = msg.yaw_angle;
    }
    else {
      resolved.yaw_angle = 0.0
    }

    if (msg.right_ldw_availability !== undefined) {
      resolved.right_ldw_availability = msg.right_ldw_availability;
    }
    else {
      resolved.right_ldw_availability = false
    }

    if (msg.left_ldw_availability !== undefined) {
      resolved.left_ldw_availability = msg.left_ldw_availability;
    }
    else {
      resolved.left_ldw_availability = false
    }

    return resolved;
    }
};

module.exports = Lane;
