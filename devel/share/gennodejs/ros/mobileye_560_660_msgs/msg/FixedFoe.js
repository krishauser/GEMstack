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

class FixedFoe {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.fixed_yaw = null;
      this.fixed_horizon = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('fixed_yaw')) {
        this.fixed_yaw = initObj.fixed_yaw
      }
      else {
        this.fixed_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('fixed_horizon')) {
        this.fixed_horizon = initObj.fixed_horizon
      }
      else {
        this.fixed_horizon = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FixedFoe
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [fixed_yaw]
    bufferOffset = _serializer.float64(obj.fixed_yaw, buffer, bufferOffset);
    // Serialize message field [fixed_horizon]
    bufferOffset = _serializer.float64(obj.fixed_horizon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FixedFoe
    let len;
    let data = new FixedFoe(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [fixed_yaw]
    data.fixed_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [fixed_horizon]
    data.fixed_horizon = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/FixedFoe';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b4f93d021949d9d8c671473a5bedf703';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float64 fixed_yaw
    float64 fixed_horizon
    
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
    const resolved = new FixedFoe(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.fixed_yaw !== undefined) {
      resolved.fixed_yaw = msg.fixed_yaw;
    }
    else {
      resolved.fixed_yaw = 0.0
    }

    if (msg.fixed_horizon !== undefined) {
      resolved.fixed_horizon = msg.fixed_horizon;
    }
    else {
      resolved.fixed_horizon = 0.0
    }

    return resolved;
    }
};

module.exports = FixedFoe;
