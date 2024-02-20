// Auto-generated. Do not edit!

// (in-package delphi_srr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SrrStatus1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_tx_look_type = null;
      this.can_tx_dsp_timestamp = null;
      this.can_tx_yaw_rate_calc = null;
      this.can_tx_vehicle_speed_calc = null;
      this.can_tx_scan_index = null;
      this.can_tx_curvature = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_tx_look_type')) {
        this.can_tx_look_type = initObj.can_tx_look_type
      }
      else {
        this.can_tx_look_type = false;
      }
      if (initObj.hasOwnProperty('can_tx_dsp_timestamp')) {
        this.can_tx_dsp_timestamp = initObj.can_tx_dsp_timestamp
      }
      else {
        this.can_tx_dsp_timestamp = 0;
      }
      if (initObj.hasOwnProperty('can_tx_yaw_rate_calc')) {
        this.can_tx_yaw_rate_calc = initObj.can_tx_yaw_rate_calc
      }
      else {
        this.can_tx_yaw_rate_calc = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_vehicle_speed_calc')) {
        this.can_tx_vehicle_speed_calc = initObj.can_tx_vehicle_speed_calc
      }
      else {
        this.can_tx_vehicle_speed_calc = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_scan_index')) {
        this.can_tx_scan_index = initObj.can_tx_scan_index
      }
      else {
        this.can_tx_scan_index = 0;
      }
      if (initObj.hasOwnProperty('can_tx_curvature')) {
        this.can_tx_curvature = initObj.can_tx_curvature
      }
      else {
        this.can_tx_curvature = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrStatus1
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_tx_look_type]
    bufferOffset = _serializer.bool(obj.can_tx_look_type, buffer, bufferOffset);
    // Serialize message field [can_tx_dsp_timestamp]
    bufferOffset = _serializer.uint32(obj.can_tx_dsp_timestamp, buffer, bufferOffset);
    // Serialize message field [can_tx_yaw_rate_calc]
    bufferOffset = _serializer.float32(obj.can_tx_yaw_rate_calc, buffer, bufferOffset);
    // Serialize message field [can_tx_vehicle_speed_calc]
    bufferOffset = _serializer.float32(obj.can_tx_vehicle_speed_calc, buffer, bufferOffset);
    // Serialize message field [can_tx_scan_index]
    bufferOffset = _serializer.uint16(obj.can_tx_scan_index, buffer, bufferOffset);
    // Serialize message field [can_tx_curvature]
    bufferOffset = _serializer.float32(obj.can_tx_curvature, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrStatus1
    let len;
    let data = new SrrStatus1(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_tx_look_type]
    data.can_tx_look_type = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_dsp_timestamp]
    data.can_tx_dsp_timestamp = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [can_tx_yaw_rate_calc]
    data.can_tx_yaw_rate_calc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_vehicle_speed_calc]
    data.can_tx_vehicle_speed_calc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_scan_index]
    data.can_tx_scan_index = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_tx_curvature]
    data.can_tx_curvature = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrStatus1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '585df8ad7a5b009cc9f6c14365cc686e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_status1
    
    std_msgs/Header header
    
    bool      can_tx_look_type
    uint32    can_tx_dsp_timestamp                     # ms
    float32   can_tx_yaw_rate_calc                     # deg/s
    float32   can_tx_vehicle_speed_calc                # m/s
    uint16    can_tx_scan_index
    float32   can_tx_curvature                         # 1/m
    
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
    const resolved = new SrrStatus1(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_tx_look_type !== undefined) {
      resolved.can_tx_look_type = msg.can_tx_look_type;
    }
    else {
      resolved.can_tx_look_type = false
    }

    if (msg.can_tx_dsp_timestamp !== undefined) {
      resolved.can_tx_dsp_timestamp = msg.can_tx_dsp_timestamp;
    }
    else {
      resolved.can_tx_dsp_timestamp = 0
    }

    if (msg.can_tx_yaw_rate_calc !== undefined) {
      resolved.can_tx_yaw_rate_calc = msg.can_tx_yaw_rate_calc;
    }
    else {
      resolved.can_tx_yaw_rate_calc = 0.0
    }

    if (msg.can_tx_vehicle_speed_calc !== undefined) {
      resolved.can_tx_vehicle_speed_calc = msg.can_tx_vehicle_speed_calc;
    }
    else {
      resolved.can_tx_vehicle_speed_calc = 0.0
    }

    if (msg.can_tx_scan_index !== undefined) {
      resolved.can_tx_scan_index = msg.can_tx_scan_index;
    }
    else {
      resolved.can_tx_scan_index = 0
    }

    if (msg.can_tx_curvature !== undefined) {
      resolved.can_tx_curvature = msg.can_tx_curvature;
    }
    else {
      resolved.can_tx_curvature = 0.0
    }

    return resolved;
    }
};

module.exports = SrrStatus1;
