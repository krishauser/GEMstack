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

class MrrControlMsgFR {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_sensitivity_profile_select = null;
      this.can_stop_frequency_frml = null;
      this.can_stop_frequency_frll = null;
      this.can_prp_factor_frml = null;
      this.can_prp_factor_frll = null;
      this.can_desired_sweep_bw_frml = null;
      this.can_desired_sweep_bw_frll = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_sensitivity_profile_select')) {
        this.can_sensitivity_profile_select = initObj.can_sensitivity_profile_select
      }
      else {
        this.can_sensitivity_profile_select = 0;
      }
      if (initObj.hasOwnProperty('can_stop_frequency_frml')) {
        this.can_stop_frequency_frml = initObj.can_stop_frequency_frml
      }
      else {
        this.can_stop_frequency_frml = 0;
      }
      if (initObj.hasOwnProperty('can_stop_frequency_frll')) {
        this.can_stop_frequency_frll = initObj.can_stop_frequency_frll
      }
      else {
        this.can_stop_frequency_frll = 0;
      }
      if (initObj.hasOwnProperty('can_prp_factor_frml')) {
        this.can_prp_factor_frml = initObj.can_prp_factor_frml
      }
      else {
        this.can_prp_factor_frml = 0.0;
      }
      if (initObj.hasOwnProperty('can_prp_factor_frll')) {
        this.can_prp_factor_frll = initObj.can_prp_factor_frll
      }
      else {
        this.can_prp_factor_frll = 0.0;
      }
      if (initObj.hasOwnProperty('can_desired_sweep_bw_frml')) {
        this.can_desired_sweep_bw_frml = initObj.can_desired_sweep_bw_frml
      }
      else {
        this.can_desired_sweep_bw_frml = 0;
      }
      if (initObj.hasOwnProperty('can_desired_sweep_bw_frll')) {
        this.can_desired_sweep_bw_frll = initObj.can_desired_sweep_bw_frll
      }
      else {
        this.can_desired_sweep_bw_frll = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrControlMsgFR
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_sensitivity_profile_select]
    bufferOffset = _serializer.uint8(obj.can_sensitivity_profile_select, buffer, bufferOffset);
    // Serialize message field [can_stop_frequency_frml]
    bufferOffset = _serializer.uint16(obj.can_stop_frequency_frml, buffer, bufferOffset);
    // Serialize message field [can_stop_frequency_frll]
    bufferOffset = _serializer.uint16(obj.can_stop_frequency_frll, buffer, bufferOffset);
    // Serialize message field [can_prp_factor_frml]
    bufferOffset = _serializer.float32(obj.can_prp_factor_frml, buffer, bufferOffset);
    // Serialize message field [can_prp_factor_frll]
    bufferOffset = _serializer.float32(obj.can_prp_factor_frll, buffer, bufferOffset);
    // Serialize message field [can_desired_sweep_bw_frml]
    bufferOffset = _serializer.uint8(obj.can_desired_sweep_bw_frml, buffer, bufferOffset);
    // Serialize message field [can_desired_sweep_bw_frll]
    bufferOffset = _serializer.uint8(obj.can_desired_sweep_bw_frll, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrControlMsgFR
    let len;
    let data = new MrrControlMsgFR(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_sensitivity_profile_select]
    data.can_sensitivity_profile_select = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_stop_frequency_frml]
    data.can_stop_frequency_frml = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_stop_frequency_frll]
    data.can_stop_frequency_frll = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_prp_factor_frml]
    data.can_prp_factor_frml = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_prp_factor_frll]
    data.can_prp_factor_frll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_desired_sweep_bw_frml]
    data.can_desired_sweep_bw_frml = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_desired_sweep_bw_frll]
    data.can_desired_sweep_bw_frll = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrControlMsgFR';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dba2c9fc1e4b47706ab1d6d7c85d6d53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8   can_sensitivity_profile_select
    uint16  can_stop_frequency_frml
    uint16  can_stop_frequency_frll
    float32 can_prp_factor_frml
    float32 can_prp_factor_frll
    uint8   can_desired_sweep_bw_frml
    uint8   can_desired_sweep_bw_frll
    
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
    const resolved = new MrrControlMsgFR(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_sensitivity_profile_select !== undefined) {
      resolved.can_sensitivity_profile_select = msg.can_sensitivity_profile_select;
    }
    else {
      resolved.can_sensitivity_profile_select = 0
    }

    if (msg.can_stop_frequency_frml !== undefined) {
      resolved.can_stop_frequency_frml = msg.can_stop_frequency_frml;
    }
    else {
      resolved.can_stop_frequency_frml = 0
    }

    if (msg.can_stop_frequency_frll !== undefined) {
      resolved.can_stop_frequency_frll = msg.can_stop_frequency_frll;
    }
    else {
      resolved.can_stop_frequency_frll = 0
    }

    if (msg.can_prp_factor_frml !== undefined) {
      resolved.can_prp_factor_frml = msg.can_prp_factor_frml;
    }
    else {
      resolved.can_prp_factor_frml = 0.0
    }

    if (msg.can_prp_factor_frll !== undefined) {
      resolved.can_prp_factor_frll = msg.can_prp_factor_frll;
    }
    else {
      resolved.can_prp_factor_frll = 0.0
    }

    if (msg.can_desired_sweep_bw_frml !== undefined) {
      resolved.can_desired_sweep_bw_frml = msg.can_desired_sweep_bw_frml;
    }
    else {
      resolved.can_desired_sweep_bw_frml = 0
    }

    if (msg.can_desired_sweep_bw_frll !== undefined) {
      resolved.can_desired_sweep_bw_frll = msg.can_desired_sweep_bw_frll;
    }
    else {
      resolved.can_desired_sweep_bw_frll = 0
    }

    return resolved;
    }
};

module.exports = MrrControlMsgFR;
