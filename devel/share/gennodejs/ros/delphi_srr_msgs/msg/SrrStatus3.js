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

class SrrStatus3 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_tx_alignment_state = null;
      this.can_tx_interface_ver_minor = null;
      this.can_tx_sw_version_arm = null;
      this.can_tx_hw_version = null;
      this.can_tx_interface_version = null;
      this.can_tx_serial_num = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_tx_alignment_state')) {
        this.can_tx_alignment_state = initObj.can_tx_alignment_state
      }
      else {
        this.can_tx_alignment_state = 0;
      }
      if (initObj.hasOwnProperty('can_tx_interface_ver_minor')) {
        this.can_tx_interface_ver_minor = initObj.can_tx_interface_ver_minor
      }
      else {
        this.can_tx_interface_ver_minor = 0;
      }
      if (initObj.hasOwnProperty('can_tx_sw_version_arm')) {
        this.can_tx_sw_version_arm = initObj.can_tx_sw_version_arm
      }
      else {
        this.can_tx_sw_version_arm = 0;
      }
      if (initObj.hasOwnProperty('can_tx_hw_version')) {
        this.can_tx_hw_version = initObj.can_tx_hw_version
      }
      else {
        this.can_tx_hw_version = 0;
      }
      if (initObj.hasOwnProperty('can_tx_interface_version')) {
        this.can_tx_interface_version = initObj.can_tx_interface_version
      }
      else {
        this.can_tx_interface_version = 0;
      }
      if (initObj.hasOwnProperty('can_tx_serial_num')) {
        this.can_tx_serial_num = initObj.can_tx_serial_num
      }
      else {
        this.can_tx_serial_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrStatus3
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_tx_alignment_state]
    bufferOffset = _serializer.uint8(obj.can_tx_alignment_state, buffer, bufferOffset);
    // Serialize message field [can_tx_interface_ver_minor]
    bufferOffset = _serializer.uint8(obj.can_tx_interface_ver_minor, buffer, bufferOffset);
    // Serialize message field [can_tx_sw_version_arm]
    bufferOffset = _serializer.uint32(obj.can_tx_sw_version_arm, buffer, bufferOffset);
    // Serialize message field [can_tx_hw_version]
    bufferOffset = _serializer.uint8(obj.can_tx_hw_version, buffer, bufferOffset);
    // Serialize message field [can_tx_interface_version]
    bufferOffset = _serializer.uint8(obj.can_tx_interface_version, buffer, bufferOffset);
    // Serialize message field [can_tx_serial_num]
    bufferOffset = _serializer.uint32(obj.can_tx_serial_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrStatus3
    let len;
    let data = new SrrStatus3(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_tx_alignment_state]
    data.can_tx_alignment_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_interface_ver_minor]
    data.can_tx_interface_ver_minor = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_sw_version_arm]
    data.can_tx_sw_version_arm = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [can_tx_hw_version]
    data.can_tx_hw_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_interface_version]
    data.can_tx_interface_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_serial_num]
    data.can_tx_serial_num = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrStatus3';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7a40100fb28cf1c5e2bf4d3c15d6aeb8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_status3
    
    std_msgs/Header header
    
    uint8     can_tx_alignment_state
    uint8     CAN_TX_ALIGNMENT_STATE_OFF=0
    uint8     CAN_TX_ALIGNMENT_STATE_INIT=1
    uint8     CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT=2
    uint8     CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT=3
    uint8     CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT=4
    
    uint8     can_tx_interface_ver_minor
    uint32    can_tx_sw_version_arm
    uint8     can_tx_hw_version
    uint8     can_tx_interface_version
    uint32    can_tx_serial_num
    
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
    const resolved = new SrrStatus3(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_tx_alignment_state !== undefined) {
      resolved.can_tx_alignment_state = msg.can_tx_alignment_state;
    }
    else {
      resolved.can_tx_alignment_state = 0
    }

    if (msg.can_tx_interface_ver_minor !== undefined) {
      resolved.can_tx_interface_ver_minor = msg.can_tx_interface_ver_minor;
    }
    else {
      resolved.can_tx_interface_ver_minor = 0
    }

    if (msg.can_tx_sw_version_arm !== undefined) {
      resolved.can_tx_sw_version_arm = msg.can_tx_sw_version_arm;
    }
    else {
      resolved.can_tx_sw_version_arm = 0
    }

    if (msg.can_tx_hw_version !== undefined) {
      resolved.can_tx_hw_version = msg.can_tx_hw_version;
    }
    else {
      resolved.can_tx_hw_version = 0
    }

    if (msg.can_tx_interface_version !== undefined) {
      resolved.can_tx_interface_version = msg.can_tx_interface_version;
    }
    else {
      resolved.can_tx_interface_version = 0
    }

    if (msg.can_tx_serial_num !== undefined) {
      resolved.can_tx_serial_num = msg.can_tx_serial_num;
    }
    else {
      resolved.can_tx_serial_num = 0
    }

    return resolved;
    }
};

// Constants for message
SrrStatus3.Constants = {
  CAN_TX_ALIGNMENT_STATE_OFF: 0,
  CAN_TX_ALIGNMENT_STATE_INIT: 1,
  CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT: 2,
  CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT: 3,
  CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT: 4,
}

module.exports = SrrStatus3;
