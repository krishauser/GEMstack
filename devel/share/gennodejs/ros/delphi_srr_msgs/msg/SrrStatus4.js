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

class SrrStatus4 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_tx_sw_version_host = null;
      this.can_tx_path_id_blis_ignore = null;
      this.can_tx_path_id_blis = null;
      this.can_tx_angle_misalignment = null;
      this.can_tx_auto_align_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_tx_sw_version_host')) {
        this.can_tx_sw_version_host = initObj.can_tx_sw_version_host
      }
      else {
        this.can_tx_sw_version_host = 0;
      }
      if (initObj.hasOwnProperty('can_tx_path_id_blis_ignore')) {
        this.can_tx_path_id_blis_ignore = initObj.can_tx_path_id_blis_ignore
      }
      else {
        this.can_tx_path_id_blis_ignore = 0;
      }
      if (initObj.hasOwnProperty('can_tx_path_id_blis')) {
        this.can_tx_path_id_blis = initObj.can_tx_path_id_blis
      }
      else {
        this.can_tx_path_id_blis = 0;
      }
      if (initObj.hasOwnProperty('can_tx_angle_misalignment')) {
        this.can_tx_angle_misalignment = initObj.can_tx_angle_misalignment
      }
      else {
        this.can_tx_angle_misalignment = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_auto_align_angle')) {
        this.can_tx_auto_align_angle = initObj.can_tx_auto_align_angle
      }
      else {
        this.can_tx_auto_align_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrStatus4
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_tx_sw_version_host]
    bufferOffset = _serializer.uint16(obj.can_tx_sw_version_host, buffer, bufferOffset);
    // Serialize message field [can_tx_path_id_blis_ignore]
    bufferOffset = _serializer.uint8(obj.can_tx_path_id_blis_ignore, buffer, bufferOffset);
    // Serialize message field [can_tx_path_id_blis]
    bufferOffset = _serializer.uint8(obj.can_tx_path_id_blis, buffer, bufferOffset);
    // Serialize message field [can_tx_angle_misalignment]
    bufferOffset = _serializer.float32(obj.can_tx_angle_misalignment, buffer, bufferOffset);
    // Serialize message field [can_tx_auto_align_angle]
    bufferOffset = _serializer.float32(obj.can_tx_auto_align_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrStatus4
    let len;
    let data = new SrrStatus4(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_tx_sw_version_host]
    data.can_tx_sw_version_host = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_tx_path_id_blis_ignore]
    data.can_tx_path_id_blis_ignore = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_path_id_blis]
    data.can_tx_path_id_blis = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_angle_misalignment]
    data.can_tx_angle_misalignment = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_auto_align_angle]
    data.can_tx_auto_align_angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrStatus4';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8f5e5c4790453e1f3bcd5507dd8162bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_status4
    
    std_msgs/Header header
    
    uint16    can_tx_sw_version_host
    uint8     can_tx_path_id_blis_ignore
    uint8     can_tx_path_id_blis
    float32   can_tx_angle_misalignment
    float32   can_tx_auto_align_angle
    
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
    const resolved = new SrrStatus4(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_tx_sw_version_host !== undefined) {
      resolved.can_tx_sw_version_host = msg.can_tx_sw_version_host;
    }
    else {
      resolved.can_tx_sw_version_host = 0
    }

    if (msg.can_tx_path_id_blis_ignore !== undefined) {
      resolved.can_tx_path_id_blis_ignore = msg.can_tx_path_id_blis_ignore;
    }
    else {
      resolved.can_tx_path_id_blis_ignore = 0
    }

    if (msg.can_tx_path_id_blis !== undefined) {
      resolved.can_tx_path_id_blis = msg.can_tx_path_id_blis;
    }
    else {
      resolved.can_tx_path_id_blis = 0
    }

    if (msg.can_tx_angle_misalignment !== undefined) {
      resolved.can_tx_angle_misalignment = msg.can_tx_angle_misalignment;
    }
    else {
      resolved.can_tx_angle_misalignment = 0.0
    }

    if (msg.can_tx_auto_align_angle !== undefined) {
      resolved.can_tx_auto_align_angle = msg.can_tx_auto_align_angle;
    }
    else {
      resolved.can_tx_auto_align_angle = 0.0
    }

    return resolved;
    }
};

module.exports = SrrStatus4;
