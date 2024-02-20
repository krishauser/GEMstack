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

class AnsToCmdConnect {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.command = null;
      this.cmd_connect_ans_d7 = null;
      this.cmd_connect_ans_d6 = null;
      this.cmd_connect_ans_d5 = null;
      this.cmd_connect_ans_d4 = null;
      this.cmd_connect_ans_d3 = null;
      this.cmd_connect_ans_d2 = null;
      this.cmd_connect_ans_d1 = null;
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
      if (initObj.hasOwnProperty('cmd_connect_ans_d7')) {
        this.cmd_connect_ans_d7 = initObj.cmd_connect_ans_d7
      }
      else {
        this.cmd_connect_ans_d7 = 0;
      }
      if (initObj.hasOwnProperty('cmd_connect_ans_d6')) {
        this.cmd_connect_ans_d6 = initObj.cmd_connect_ans_d6
      }
      else {
        this.cmd_connect_ans_d6 = 0;
      }
      if (initObj.hasOwnProperty('cmd_connect_ans_d5')) {
        this.cmd_connect_ans_d5 = initObj.cmd_connect_ans_d5
      }
      else {
        this.cmd_connect_ans_d5 = 0;
      }
      if (initObj.hasOwnProperty('cmd_connect_ans_d4')) {
        this.cmd_connect_ans_d4 = initObj.cmd_connect_ans_d4
      }
      else {
        this.cmd_connect_ans_d4 = 0;
      }
      if (initObj.hasOwnProperty('cmd_connect_ans_d3')) {
        this.cmd_connect_ans_d3 = initObj.cmd_connect_ans_d3
      }
      else {
        this.cmd_connect_ans_d3 = 0;
      }
      if (initObj.hasOwnProperty('cmd_connect_ans_d2')) {
        this.cmd_connect_ans_d2 = initObj.cmd_connect_ans_d2
      }
      else {
        this.cmd_connect_ans_d2 = 0;
      }
      if (initObj.hasOwnProperty('cmd_connect_ans_d1')) {
        this.cmd_connect_ans_d1 = initObj.cmd_connect_ans_d1
      }
      else {
        this.cmd_connect_ans_d1 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnsToCmdConnect
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [cmd_connect_ans_d7]
    bufferOffset = _serializer.uint8(obj.cmd_connect_ans_d7, buffer, bufferOffset);
    // Serialize message field [cmd_connect_ans_d6]
    bufferOffset = _serializer.uint8(obj.cmd_connect_ans_d6, buffer, bufferOffset);
    // Serialize message field [cmd_connect_ans_d5]
    bufferOffset = _serializer.uint8(obj.cmd_connect_ans_d5, buffer, bufferOffset);
    // Serialize message field [cmd_connect_ans_d4]
    bufferOffset = _serializer.uint8(obj.cmd_connect_ans_d4, buffer, bufferOffset);
    // Serialize message field [cmd_connect_ans_d3]
    bufferOffset = _serializer.uint8(obj.cmd_connect_ans_d3, buffer, bufferOffset);
    // Serialize message field [cmd_connect_ans_d2]
    bufferOffset = _serializer.uint8(obj.cmd_connect_ans_d2, buffer, bufferOffset);
    // Serialize message field [cmd_connect_ans_d1]
    bufferOffset = _serializer.uint8(obj.cmd_connect_ans_d1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnsToCmdConnect
    let len;
    let data = new AnsToCmdConnect(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cmd_connect_ans_d7]
    data.cmd_connect_ans_d7 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cmd_connect_ans_d6]
    data.cmd_connect_ans_d6 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cmd_connect_ans_d5]
    data.cmd_connect_ans_d5 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cmd_connect_ans_d4]
    data.cmd_connect_ans_d4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cmd_connect_ans_d3]
    data.cmd_connect_ans_d3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cmd_connect_ans_d2]
    data.cmd_connect_ans_d2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cmd_connect_ans_d1]
    data.cmd_connect_ans_d1 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neobotix_usboard_msgs/AnsToCmdConnect';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1ef60b13020f0e599d4fbb33f17b3f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for AnsToCmdConnect
    
    std_msgs/Header header
    
    uint8     command                                 
    uint8     cmd_connect_ans_d7                      
    uint8     cmd_connect_ans_d6                      
    uint8     cmd_connect_ans_d5                      
    uint8     cmd_connect_ans_d4                      
    uint8     cmd_connect_ans_d3                      
    uint8     cmd_connect_ans_d2                      
    uint8     cmd_connect_ans_d1                      
    
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
    const resolved = new AnsToCmdConnect(null);
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

    if (msg.cmd_connect_ans_d7 !== undefined) {
      resolved.cmd_connect_ans_d7 = msg.cmd_connect_ans_d7;
    }
    else {
      resolved.cmd_connect_ans_d7 = 0
    }

    if (msg.cmd_connect_ans_d6 !== undefined) {
      resolved.cmd_connect_ans_d6 = msg.cmd_connect_ans_d6;
    }
    else {
      resolved.cmd_connect_ans_d6 = 0
    }

    if (msg.cmd_connect_ans_d5 !== undefined) {
      resolved.cmd_connect_ans_d5 = msg.cmd_connect_ans_d5;
    }
    else {
      resolved.cmd_connect_ans_d5 = 0
    }

    if (msg.cmd_connect_ans_d4 !== undefined) {
      resolved.cmd_connect_ans_d4 = msg.cmd_connect_ans_d4;
    }
    else {
      resolved.cmd_connect_ans_d4 = 0
    }

    if (msg.cmd_connect_ans_d3 !== undefined) {
      resolved.cmd_connect_ans_d3 = msg.cmd_connect_ans_d3;
    }
    else {
      resolved.cmd_connect_ans_d3 = 0
    }

    if (msg.cmd_connect_ans_d2 !== undefined) {
      resolved.cmd_connect_ans_d2 = msg.cmd_connect_ans_d2;
    }
    else {
      resolved.cmd_connect_ans_d2 = 0
    }

    if (msg.cmd_connect_ans_d1 !== undefined) {
      resolved.cmd_connect_ans_d1 = msg.cmd_connect_ans_d1;
    }
    else {
      resolved.cmd_connect_ans_d1 = 0
    }

    return resolved;
    }
};

module.exports = AnsToCmdConnect;
