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

class MrrControlMsgNR {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_stop_frequency_nrml = null;
      this.can_prp_factor_nrml = null;
      this.can_desired_sweep_bw_nrml = null;
      this.can_radiation_ctrl = null;
      this.can_stop_frequency_nrll = null;
      this.can_prp_factor_nrll = null;
      this.can_desired_sweep_bw_nrll = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_stop_frequency_nrml')) {
        this.can_stop_frequency_nrml = initObj.can_stop_frequency_nrml
      }
      else {
        this.can_stop_frequency_nrml = 0;
      }
      if (initObj.hasOwnProperty('can_prp_factor_nrml')) {
        this.can_prp_factor_nrml = initObj.can_prp_factor_nrml
      }
      else {
        this.can_prp_factor_nrml = 0;
      }
      if (initObj.hasOwnProperty('can_desired_sweep_bw_nrml')) {
        this.can_desired_sweep_bw_nrml = initObj.can_desired_sweep_bw_nrml
      }
      else {
        this.can_desired_sweep_bw_nrml = 0;
      }
      if (initObj.hasOwnProperty('can_radiation_ctrl')) {
        this.can_radiation_ctrl = initObj.can_radiation_ctrl
      }
      else {
        this.can_radiation_ctrl = false;
      }
      if (initObj.hasOwnProperty('can_stop_frequency_nrll')) {
        this.can_stop_frequency_nrll = initObj.can_stop_frequency_nrll
      }
      else {
        this.can_stop_frequency_nrll = 0;
      }
      if (initObj.hasOwnProperty('can_prp_factor_nrll')) {
        this.can_prp_factor_nrll = initObj.can_prp_factor_nrll
      }
      else {
        this.can_prp_factor_nrll = 0;
      }
      if (initObj.hasOwnProperty('can_desired_sweep_bw_nrll')) {
        this.can_desired_sweep_bw_nrll = initObj.can_desired_sweep_bw_nrll
      }
      else {
        this.can_desired_sweep_bw_nrll = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrControlMsgNR
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_stop_frequency_nrml]
    bufferOffset = _serializer.uint16(obj.can_stop_frequency_nrml, buffer, bufferOffset);
    // Serialize message field [can_prp_factor_nrml]
    bufferOffset = _serializer.uint16(obj.can_prp_factor_nrml, buffer, bufferOffset);
    // Serialize message field [can_desired_sweep_bw_nrml]
    bufferOffset = _serializer.uint8(obj.can_desired_sweep_bw_nrml, buffer, bufferOffset);
    // Serialize message field [can_radiation_ctrl]
    bufferOffset = _serializer.bool(obj.can_radiation_ctrl, buffer, bufferOffset);
    // Serialize message field [can_stop_frequency_nrll]
    bufferOffset = _serializer.uint16(obj.can_stop_frequency_nrll, buffer, bufferOffset);
    // Serialize message field [can_prp_factor_nrll]
    bufferOffset = _serializer.uint16(obj.can_prp_factor_nrll, buffer, bufferOffset);
    // Serialize message field [can_desired_sweep_bw_nrll]
    bufferOffset = _serializer.uint8(obj.can_desired_sweep_bw_nrll, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrControlMsgNR
    let len;
    let data = new MrrControlMsgNR(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_stop_frequency_nrml]
    data.can_stop_frequency_nrml = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_prp_factor_nrml]
    data.can_prp_factor_nrml = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_desired_sweep_bw_nrml]
    data.can_desired_sweep_bw_nrml = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_radiation_ctrl]
    data.can_radiation_ctrl = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_stop_frequency_nrll]
    data.can_stop_frequency_nrll = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_prp_factor_nrll]
    data.can_prp_factor_nrll = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_desired_sweep_bw_nrll]
    data.can_desired_sweep_bw_nrll = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 11;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrControlMsgNR';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3100fbbd30b156c46cb7b9ae9e5d17a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint16 can_stop_frequency_nrml
    uint16 can_prp_factor_nrml
    uint8  can_desired_sweep_bw_nrml
    bool   can_radiation_ctrl
    uint16 can_stop_frequency_nrll
    uint16 can_prp_factor_nrll
    uint8  can_desired_sweep_bw_nrll
    
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
    const resolved = new MrrControlMsgNR(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_stop_frequency_nrml !== undefined) {
      resolved.can_stop_frequency_nrml = msg.can_stop_frequency_nrml;
    }
    else {
      resolved.can_stop_frequency_nrml = 0
    }

    if (msg.can_prp_factor_nrml !== undefined) {
      resolved.can_prp_factor_nrml = msg.can_prp_factor_nrml;
    }
    else {
      resolved.can_prp_factor_nrml = 0
    }

    if (msg.can_desired_sweep_bw_nrml !== undefined) {
      resolved.can_desired_sweep_bw_nrml = msg.can_desired_sweep_bw_nrml;
    }
    else {
      resolved.can_desired_sweep_bw_nrml = 0
    }

    if (msg.can_radiation_ctrl !== undefined) {
      resolved.can_radiation_ctrl = msg.can_radiation_ctrl;
    }
    else {
      resolved.can_radiation_ctrl = false
    }

    if (msg.can_stop_frequency_nrll !== undefined) {
      resolved.can_stop_frequency_nrll = msg.can_stop_frequency_nrll;
    }
    else {
      resolved.can_stop_frequency_nrll = 0
    }

    if (msg.can_prp_factor_nrll !== undefined) {
      resolved.can_prp_factor_nrll = msg.can_prp_factor_nrll;
    }
    else {
      resolved.can_prp_factor_nrll = 0
    }

    if (msg.can_desired_sweep_bw_nrll !== undefined) {
      resolved.can_desired_sweep_bw_nrll = msg.can_desired_sweep_bw_nrll;
    }
    else {
      resolved.can_desired_sweep_bw_nrll = 0
    }

    return resolved;
    }
};

module.exports = MrrControlMsgNR;
