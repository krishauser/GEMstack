// Auto-generated. Do not edit!

// (in-package neobotix_usboard_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AnsParasetToEEPROM {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.command = null;
      this.paraset_cksum_low_byte = null;
      this.paraset_cksum_high_byte = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('paraset_cksum_low_byte')) {
        this.paraset_cksum_low_byte = initObj.paraset_cksum_low_byte
      }
      else {
        this.paraset_cksum_low_byte = 0;
      }
      if (initObj.hasOwnProperty('paraset_cksum_high_byte')) {
        this.paraset_cksum_high_byte = initObj.paraset_cksum_high_byte
      }
      else {
        this.paraset_cksum_high_byte = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnsParasetToEEPROM
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [paraset_cksum_low_byte]
    bufferOffset = _serializer.uint8(obj.paraset_cksum_low_byte, buffer, bufferOffset);
    // Serialize message field [paraset_cksum_high_byte]
    bufferOffset = _serializer.uint8(obj.paraset_cksum_high_byte, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnsParasetToEEPROM
    let len;
    let data = new AnsParasetToEEPROM(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_cksum_low_byte]
    data.paraset_cksum_low_byte = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_cksum_high_byte]
    data.paraset_cksum_high_byte = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neobotix_usboard_msgs/AnsParasetToEEPROM';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae6db4e8232618eb386bde89d0c827b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for AnsParasetToEEPROM
    
    std_msgs/Header header
    
    uint8     command                                 
    uint8     paraset_cksum_low_byte                  
    uint8     paraset_cksum_high_byte                 
    
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
    const resolved = new AnsParasetToEEPROM(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.paraset_cksum_low_byte !== undefined) {
      resolved.paraset_cksum_low_byte = msg.paraset_cksum_low_byte;
    }
    else {
      resolved.paraset_cksum_low_byte = 0
    }

    if (msg.paraset_cksum_high_byte !== undefined) {
      resolved.paraset_cksum_high_byte = msg.paraset_cksum_high_byte;
    }
    else {
      resolved.paraset_cksum_high_byte = 0
    }

    return resolved;
    }
};

module.exports = AnsParasetToEEPROM;
