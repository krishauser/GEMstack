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

class ActiveFaultLatched2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ipma_pcan_data_range_check = null;
      this.ipma_pcan_missing_msg = null;
      this.vin_signal_compare_failure = null;
      this.module_not_configured_error = null;
      this.car_cfg_not_configured_error = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ipma_pcan_data_range_check')) {
        this.ipma_pcan_data_range_check = initObj.ipma_pcan_data_range_check
      }
      else {
        this.ipma_pcan_data_range_check = false;
      }
      if (initObj.hasOwnProperty('ipma_pcan_missing_msg')) {
        this.ipma_pcan_missing_msg = initObj.ipma_pcan_missing_msg
      }
      else {
        this.ipma_pcan_missing_msg = false;
      }
      if (initObj.hasOwnProperty('vin_signal_compare_failure')) {
        this.vin_signal_compare_failure = initObj.vin_signal_compare_failure
      }
      else {
        this.vin_signal_compare_failure = false;
      }
      if (initObj.hasOwnProperty('module_not_configured_error')) {
        this.module_not_configured_error = initObj.module_not_configured_error
      }
      else {
        this.module_not_configured_error = false;
      }
      if (initObj.hasOwnProperty('car_cfg_not_configured_error')) {
        this.car_cfg_not_configured_error = initObj.car_cfg_not_configured_error
      }
      else {
        this.car_cfg_not_configured_error = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActiveFaultLatched2
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ipma_pcan_data_range_check]
    bufferOffset = _serializer.bool(obj.ipma_pcan_data_range_check, buffer, bufferOffset);
    // Serialize message field [ipma_pcan_missing_msg]
    bufferOffset = _serializer.bool(obj.ipma_pcan_missing_msg, buffer, bufferOffset);
    // Serialize message field [vin_signal_compare_failure]
    bufferOffset = _serializer.bool(obj.vin_signal_compare_failure, buffer, bufferOffset);
    // Serialize message field [module_not_configured_error]
    bufferOffset = _serializer.bool(obj.module_not_configured_error, buffer, bufferOffset);
    // Serialize message field [car_cfg_not_configured_error]
    bufferOffset = _serializer.bool(obj.car_cfg_not_configured_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActiveFaultLatched2
    let len;
    let data = new ActiveFaultLatched2(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ipma_pcan_data_range_check]
    data.ipma_pcan_data_range_check = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ipma_pcan_missing_msg]
    data.ipma_pcan_missing_msg = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [vin_signal_compare_failure]
    data.vin_signal_compare_failure = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [module_not_configured_error]
    data.module_not_configured_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [car_cfg_not_configured_error]
    data.car_cfg_not_configured_error = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/ActiveFaultLatched2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fcd978d054e184e337e27e3c35da2f6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool ipma_pcan_data_range_check
    bool ipma_pcan_missing_msg
    bool vin_signal_compare_failure
    bool module_not_configured_error
    bool car_cfg_not_configured_error
    
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
    const resolved = new ActiveFaultLatched2(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ipma_pcan_data_range_check !== undefined) {
      resolved.ipma_pcan_data_range_check = msg.ipma_pcan_data_range_check;
    }
    else {
      resolved.ipma_pcan_data_range_check = false
    }

    if (msg.ipma_pcan_missing_msg !== undefined) {
      resolved.ipma_pcan_missing_msg = msg.ipma_pcan_missing_msg;
    }
    else {
      resolved.ipma_pcan_missing_msg = false
    }

    if (msg.vin_signal_compare_failure !== undefined) {
      resolved.vin_signal_compare_failure = msg.vin_signal_compare_failure;
    }
    else {
      resolved.vin_signal_compare_failure = false
    }

    if (msg.module_not_configured_error !== undefined) {
      resolved.module_not_configured_error = msg.module_not_configured_error;
    }
    else {
      resolved.module_not_configured_error = false
    }

    if (msg.car_cfg_not_configured_error !== undefined) {
      resolved.car_cfg_not_configured_error = msg.car_cfg_not_configured_error;
    }
    else {
      resolved.car_cfg_not_configured_error = false
    }

    return resolved;
    }
};

module.exports = ActiveFaultLatched2;
