// Auto-generated. Do not edit!

// (in-package delphi_mrr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class VehicleStateMsg3 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.yaw_rate_reference_valid = null;
      this.yaw_rate_reference = null;
      this.can_veh_long_accel_qf = null;
      this.can_veh_long_accel = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('yaw_rate_reference_valid')) {
        this.yaw_rate_reference_valid = initObj.yaw_rate_reference_valid
      }
      else {
        this.yaw_rate_reference_valid = false;
      }
      if (initObj.hasOwnProperty('yaw_rate_reference')) {
        this.yaw_rate_reference = initObj.yaw_rate_reference
      }
      else {
        this.yaw_rate_reference = 0.0;
      }
      if (initObj.hasOwnProperty('can_veh_long_accel_qf')) {
        this.can_veh_long_accel_qf = initObj.can_veh_long_accel_qf
      }
      else {
        this.can_veh_long_accel_qf = 0;
      }
      if (initObj.hasOwnProperty('can_veh_long_accel')) {
        this.can_veh_long_accel = initObj.can_veh_long_accel
      }
      else {
        this.can_veh_long_accel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleStateMsg3
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [yaw_rate_reference_valid]
    bufferOffset = _serializer.bool(obj.yaw_rate_reference_valid, buffer, bufferOffset);
    // Serialize message field [yaw_rate_reference]
    bufferOffset = _serializer.float32(obj.yaw_rate_reference, buffer, bufferOffset);
    // Serialize message field [can_veh_long_accel_qf]
    bufferOffset = _serializer.uint8(obj.can_veh_long_accel_qf, buffer, bufferOffset);
    // Serialize message field [can_veh_long_accel]
    bufferOffset = _serializer.float32(obj.can_veh_long_accel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleStateMsg3
    let len;
    let data = new VehicleStateMsg3(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaw_rate_reference_valid]
    data.yaw_rate_reference_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [yaw_rate_reference]
    data.yaw_rate_reference = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_veh_long_accel_qf]
    data.can_veh_long_accel_qf = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_veh_long_accel]
    data.can_veh_long_accel = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/VehicleStateMsg3';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce47a927102040e8c016cbb9166e5057';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool    yaw_rate_reference_valid
    float32 yaw_rate_reference
    uint8   can_veh_long_accel_qf
    float32 can_veh_long_accel
    
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
    const resolved = new VehicleStateMsg3(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.yaw_rate_reference_valid !== undefined) {
      resolved.yaw_rate_reference_valid = msg.yaw_rate_reference_valid;
    }
    else {
      resolved.yaw_rate_reference_valid = false
    }

    if (msg.yaw_rate_reference !== undefined) {
      resolved.yaw_rate_reference = msg.yaw_rate_reference;
    }
    else {
      resolved.yaw_rate_reference = 0.0
    }

    if (msg.can_veh_long_accel_qf !== undefined) {
      resolved.can_veh_long_accel_qf = msg.can_veh_long_accel_qf;
    }
    else {
      resolved.can_veh_long_accel_qf = 0
    }

    if (msg.can_veh_long_accel !== undefined) {
      resolved.can_veh_long_accel = msg.can_veh_long_accel;
    }
    else {
      resolved.can_veh_long_accel = 0.0
    }

    return resolved;
    }
};

module.exports = VehicleStateMsg3;
