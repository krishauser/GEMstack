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

class Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.command = null;
      this.command_data = null;
      this.set_num = null;
      this.paraset_byte6 = null;
      this.paraset_byte5 = null;
      this.paraset_byte4 = null;
      this.paraset_byte3 = null;
      this.paraset_byte2 = null;
      this.paraset_byte1 = null;
      this.set_active_9to16 = null;
      this.set_active_1to8 = null;
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
      if (initObj.hasOwnProperty('command_data')) {
        this.command_data = initObj.command_data
      }
      else {
        this.command_data = 0;
      }
      if (initObj.hasOwnProperty('set_num')) {
        this.set_num = initObj.set_num
      }
      else {
        this.set_num = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte6')) {
        this.paraset_byte6 = initObj.paraset_byte6
      }
      else {
        this.paraset_byte6 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte5')) {
        this.paraset_byte5 = initObj.paraset_byte5
      }
      else {
        this.paraset_byte5 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte4')) {
        this.paraset_byte4 = initObj.paraset_byte4
      }
      else {
        this.paraset_byte4 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte3')) {
        this.paraset_byte3 = initObj.paraset_byte3
      }
      else {
        this.paraset_byte3 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte2')) {
        this.paraset_byte2 = initObj.paraset_byte2
      }
      else {
        this.paraset_byte2 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte1')) {
        this.paraset_byte1 = initObj.paraset_byte1
      }
      else {
        this.paraset_byte1 = 0;
      }
      if (initObj.hasOwnProperty('set_active_9to16')) {
        this.set_active_9to16 = initObj.set_active_9to16
      }
      else {
        this.set_active_9to16 = 0;
      }
      if (initObj.hasOwnProperty('set_active_1to8')) {
        this.set_active_1to8 = initObj.set_active_1to8
      }
      else {
        this.set_active_1to8 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Command
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [command_data]
    bufferOffset = _serializer.uint64(obj.command_data, buffer, bufferOffset);
    // Serialize message field [set_num]
    bufferOffset = _serializer.uint8(obj.set_num, buffer, bufferOffset);
    // Serialize message field [paraset_byte6]
    bufferOffset = _serializer.uint8(obj.paraset_byte6, buffer, bufferOffset);
    // Serialize message field [paraset_byte5]
    bufferOffset = _serializer.uint8(obj.paraset_byte5, buffer, bufferOffset);
    // Serialize message field [paraset_byte4]
    bufferOffset = _serializer.uint8(obj.paraset_byte4, buffer, bufferOffset);
    // Serialize message field [paraset_byte3]
    bufferOffset = _serializer.uint8(obj.paraset_byte3, buffer, bufferOffset);
    // Serialize message field [paraset_byte2]
    bufferOffset = _serializer.uint8(obj.paraset_byte2, buffer, bufferOffset);
    // Serialize message field [paraset_byte1]
    bufferOffset = _serializer.uint8(obj.paraset_byte1, buffer, bufferOffset);
    // Serialize message field [set_active_9to16]
    bufferOffset = _serializer.uint8(obj.set_active_9to16, buffer, bufferOffset);
    // Serialize message field [set_active_1to8]
    bufferOffset = _serializer.uint8(obj.set_active_1to8, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Command
    let len;
    let data = new Command(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [command_data]
    data.command_data = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [set_num]
    data.set_num = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte6]
    data.paraset_byte6 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte5]
    data.paraset_byte5 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte4]
    data.paraset_byte4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte3]
    data.paraset_byte3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte2]
    data.paraset_byte2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte1]
    data.paraset_byte1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [set_active_9to16]
    data.set_active_9to16 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [set_active_1to8]
    data.set_active_1to8 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neobotix_usboard_msgs/Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa2adac976b058480751c90ad9fb67d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for Command
    
    std_msgs/Header header
    
    uint8     command                                 
    uint64    command_data                            
    uint8     set_num                                 
    uint8     paraset_byte6                     
    uint8     paraset_byte5                     
    uint8     paraset_byte4                     
    uint8     paraset_byte3                     
    uint8     paraset_byte2                     
    uint8     paraset_byte1                     
    uint8     set_active_9to16                        
    uint8     set_active_1to8                         
    
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
    const resolved = new Command(null);
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

    if (msg.command_data !== undefined) {
      resolved.command_data = msg.command_data;
    }
    else {
      resolved.command_data = 0
    }

    if (msg.set_num !== undefined) {
      resolved.set_num = msg.set_num;
    }
    else {
      resolved.set_num = 0
    }

    if (msg.paraset_byte6 !== undefined) {
      resolved.paraset_byte6 = msg.paraset_byte6;
    }
    else {
      resolved.paraset_byte6 = 0
    }

    if (msg.paraset_byte5 !== undefined) {
      resolved.paraset_byte5 = msg.paraset_byte5;
    }
    else {
      resolved.paraset_byte5 = 0
    }

    if (msg.paraset_byte4 !== undefined) {
      resolved.paraset_byte4 = msg.paraset_byte4;
    }
    else {
      resolved.paraset_byte4 = 0
    }

    if (msg.paraset_byte3 !== undefined) {
      resolved.paraset_byte3 = msg.paraset_byte3;
    }
    else {
      resolved.paraset_byte3 = 0
    }

    if (msg.paraset_byte2 !== undefined) {
      resolved.paraset_byte2 = msg.paraset_byte2;
    }
    else {
      resolved.paraset_byte2 = 0
    }

    if (msg.paraset_byte1 !== undefined) {
      resolved.paraset_byte1 = msg.paraset_byte1;
    }
    else {
      resolved.paraset_byte1 = 0
    }

    if (msg.set_active_9to16 !== undefined) {
      resolved.set_active_9to16 = msg.set_active_9to16;
    }
    else {
      resolved.set_active_9to16 = 0
    }

    if (msg.set_active_1to8 !== undefined) {
      resolved.set_active_1to8 = msg.set_active_1to8;
    }
    else {
      resolved.set_active_1to8 = 0
    }

    return resolved;
    }
};

module.exports = Command;
