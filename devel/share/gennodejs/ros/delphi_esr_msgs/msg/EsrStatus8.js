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

class EsrStatus8 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.history_fault_0 = null;
      this.history_fault_1 = null;
      this.history_fault_2 = null;
      this.history_fault_3 = null;
      this.history_fault_4 = null;
      this.history_fault_5 = null;
      this.history_fault_6 = null;
      this.history_fault_7 = null;
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
      if (initObj.hasOwnProperty('history_fault_0')) {
        this.history_fault_0 = initObj.history_fault_0
      }
      else {
        this.history_fault_0 = 0;
      }
      if (initObj.hasOwnProperty('history_fault_1')) {
        this.history_fault_1 = initObj.history_fault_1
      }
      else {
        this.history_fault_1 = 0;
      }
      if (initObj.hasOwnProperty('history_fault_2')) {
        this.history_fault_2 = initObj.history_fault_2
      }
      else {
        this.history_fault_2 = 0;
      }
      if (initObj.hasOwnProperty('history_fault_3')) {
        this.history_fault_3 = initObj.history_fault_3
      }
      else {
        this.history_fault_3 = 0;
      }
      if (initObj.hasOwnProperty('history_fault_4')) {
        this.history_fault_4 = initObj.history_fault_4
      }
      else {
        this.history_fault_4 = 0;
      }
      if (initObj.hasOwnProperty('history_fault_5')) {
        this.history_fault_5 = initObj.history_fault_5
      }
      else {
        this.history_fault_5 = 0;
      }
      if (initObj.hasOwnProperty('history_fault_6')) {
        this.history_fault_6 = initObj.history_fault_6
      }
      else {
        this.history_fault_6 = 0;
      }
      if (initObj.hasOwnProperty('history_fault_7')) {
        this.history_fault_7 = initObj.history_fault_7
      }
      else {
        this.history_fault_7 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrStatus8
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [history_fault_0]
    bufferOffset = _serializer.uint8(obj.history_fault_0, buffer, bufferOffset);
    // Serialize message field [history_fault_1]
    bufferOffset = _serializer.uint8(obj.history_fault_1, buffer, bufferOffset);
    // Serialize message field [history_fault_2]
    bufferOffset = _serializer.uint8(obj.history_fault_2, buffer, bufferOffset);
    // Serialize message field [history_fault_3]
    bufferOffset = _serializer.uint8(obj.history_fault_3, buffer, bufferOffset);
    // Serialize message field [history_fault_4]
    bufferOffset = _serializer.uint8(obj.history_fault_4, buffer, bufferOffset);
    // Serialize message field [history_fault_5]
    bufferOffset = _serializer.uint8(obj.history_fault_5, buffer, bufferOffset);
    // Serialize message field [history_fault_6]
    bufferOffset = _serializer.uint8(obj.history_fault_6, buffer, bufferOffset);
    // Serialize message field [history_fault_7]
    bufferOffset = _serializer.uint8(obj.history_fault_7, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrStatus8
    let len;
    let data = new EsrStatus8(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [history_fault_0]
    data.history_fault_0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [history_fault_1]
    data.history_fault_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [history_fault_2]
    data.history_fault_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [history_fault_3]
    data.history_fault_3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [history_fault_4]
    data.history_fault_4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [history_fault_5]
    data.history_fault_5 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [history_fault_6]
    data.history_fault_6 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [history_fault_7]
    data.history_fault_7 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrStatus8';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c7431176d5d09e818f01fd18ce628e7e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Status8
    string      canmsg
    
    uint8       history_fault_0
    uint8       history_fault_1
    uint8       history_fault_2
    uint8       history_fault_3
    uint8       history_fault_4
    uint8       history_fault_5
    uint8       history_fault_6
    uint8       history_fault_7
    
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
    const resolved = new EsrStatus8(null);
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

    if (msg.history_fault_0 !== undefined) {
      resolved.history_fault_0 = msg.history_fault_0;
    }
    else {
      resolved.history_fault_0 = 0
    }

    if (msg.history_fault_1 !== undefined) {
      resolved.history_fault_1 = msg.history_fault_1;
    }
    else {
      resolved.history_fault_1 = 0
    }

    if (msg.history_fault_2 !== undefined) {
      resolved.history_fault_2 = msg.history_fault_2;
    }
    else {
      resolved.history_fault_2 = 0
    }

    if (msg.history_fault_3 !== undefined) {
      resolved.history_fault_3 = msg.history_fault_3;
    }
    else {
      resolved.history_fault_3 = 0
    }

    if (msg.history_fault_4 !== undefined) {
      resolved.history_fault_4 = msg.history_fault_4;
    }
    else {
      resolved.history_fault_4 = 0
    }

    if (msg.history_fault_5 !== undefined) {
      resolved.history_fault_5 = msg.history_fault_5;
    }
    else {
      resolved.history_fault_5 = 0
    }

    if (msg.history_fault_6 !== undefined) {
      resolved.history_fault_6 = msg.history_fault_6;
    }
    else {
      resolved.history_fault_6 = 0
    }

    if (msg.history_fault_7 !== undefined) {
      resolved.history_fault_7 = msg.history_fault_7;
    }
    else {
      resolved.history_fault_7 = 0
    }

    return resolved;
    }
};

module.exports = EsrStatus8;
