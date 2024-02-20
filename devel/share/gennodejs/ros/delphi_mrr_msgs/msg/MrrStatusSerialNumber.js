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

class MrrStatusSerialNumber {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_sequence_number = null;
      this.can_serial_number = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_sequence_number')) {
        this.can_sequence_number = initObj.can_sequence_number
      }
      else {
        this.can_sequence_number = 0;
      }
      if (initObj.hasOwnProperty('can_serial_number')) {
        this.can_serial_number = initObj.can_serial_number
      }
      else {
        this.can_serial_number = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrStatusSerialNumber
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_sequence_number]
    bufferOffset = _serializer.uint16(obj.can_sequence_number, buffer, bufferOffset);
    // Serialize message field [can_serial_number]
    bufferOffset = _serializer.uint64(obj.can_serial_number, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrStatusSerialNumber
    let len;
    let data = new MrrStatusSerialNumber(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_sequence_number]
    data.can_sequence_number = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_serial_number]
    data.can_serial_number = _deserializer.uint64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrStatusSerialNumber';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5d3bfd88a3b11cee7de78d554e06de73';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint16 can_sequence_number
    uint64 can_serial_number
    
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
    const resolved = new MrrStatusSerialNumber(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_sequence_number !== undefined) {
      resolved.can_sequence_number = msg.can_sequence_number;
    }
    else {
      resolved.can_sequence_number = 0
    }

    if (msg.can_serial_number !== undefined) {
      resolved.can_serial_number = msg.can_serial_number;
    }
    else {
      resolved.can_serial_number = 0
    }

    return resolved;
    }
};

module.exports = MrrStatusSerialNumber;
