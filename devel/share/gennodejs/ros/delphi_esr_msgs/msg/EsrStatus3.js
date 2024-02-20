// Auto-generated. Do not edit!

// (in-package delphi_esr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EsrStatus3 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.interface_version = null;
      this.hw_version = null;
      this.sw_version_host = null;
      this.serial_num = null;
      this.sw_version_pld = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('canmsg')) {
        this.canmsg = initObj.canmsg
      }
      else {
        this.canmsg = '';
      }
      if (initObj.hasOwnProperty('interface_version')) {
        this.interface_version = initObj.interface_version
      }
      else {
        this.interface_version = 0;
      }
      if (initObj.hasOwnProperty('hw_version')) {
        this.hw_version = initObj.hw_version
      }
      else {
        this.hw_version = 0;
      }
      if (initObj.hasOwnProperty('sw_version_host')) {
        this.sw_version_host = initObj.sw_version_host
      }
      else {
        this.sw_version_host = '';
      }
      if (initObj.hasOwnProperty('serial_num')) {
        this.serial_num = initObj.serial_num
      }
      else {
        this.serial_num = '';
      }
      if (initObj.hasOwnProperty('sw_version_pld')) {
        this.sw_version_pld = initObj.sw_version_pld
      }
      else {
        this.sw_version_pld = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrStatus3
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [interface_version]
    bufferOffset = _serializer.uint8(obj.interface_version, buffer, bufferOffset);
    // Serialize message field [hw_version]
    bufferOffset = _serializer.uint8(obj.hw_version, buffer, bufferOffset);
    // Serialize message field [sw_version_host]
    bufferOffset = _serializer.string(obj.sw_version_host, buffer, bufferOffset);
    // Serialize message field [serial_num]
    bufferOffset = _serializer.string(obj.serial_num, buffer, bufferOffset);
    // Serialize message field [sw_version_pld]
    bufferOffset = _serializer.uint8(obj.sw_version_pld, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrStatus3
    let len;
    let data = new EsrStatus3(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [interface_version]
    data.interface_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [hw_version]
    data.hw_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sw_version_host]
    data.sw_version_host = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [serial_num]
    data.serial_num = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sw_version_pld]
    data.sw_version_pld = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    length += _getByteLength(object.sw_version_host);
    length += _getByteLength(object.serial_num);
    return length + 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrStatus3';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '583f51b7628983715fa50ecaf2d76bc8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Status3
    string      canmsg
    
    uint8       interface_version
    uint8       hw_version
    string      sw_version_host
    string      serial_num
    uint8       sw_version_pld
    
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
    const resolved = new EsrStatus3(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.canmsg !== undefined) {
      resolved.canmsg = msg.canmsg;
    }
    else {
      resolved.canmsg = ''
    }

    if (msg.interface_version !== undefined) {
      resolved.interface_version = msg.interface_version;
    }
    else {
      resolved.interface_version = 0
    }

    if (msg.hw_version !== undefined) {
      resolved.hw_version = msg.hw_version;
    }
    else {
      resolved.hw_version = 0
    }

    if (msg.sw_version_host !== undefined) {
      resolved.sw_version_host = msg.sw_version_host;
    }
    else {
      resolved.sw_version_host = ''
    }

    if (msg.serial_num !== undefined) {
      resolved.serial_num = msg.serial_num;
    }
    else {
      resolved.serial_num = ''
    }

    if (msg.sw_version_pld !== undefined) {
      resolved.sw_version_pld = msg.sw_version_pld;
    }
    else {
      resolved.sw_version_pld = 0
    }

    return resolved;
    }
};

module.exports = EsrStatus3;
