// Auto-generated. Do not edit!

// (in-package kartech_linear_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ReassignCommandIdCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.command_id_index = null;
      this.user_command_id = null;
      this.disable_default_command_id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('confirm')) {
        this.confirm = initObj.confirm
      }
      else {
        this.confirm = false;
      }
      if (initObj.hasOwnProperty('command_id_index')) {
        this.command_id_index = initObj.command_id_index
      }
      else {
        this.command_id_index = 0;
      }
      if (initObj.hasOwnProperty('user_command_id')) {
        this.user_command_id = initObj.user_command_id
      }
      else {
        this.user_command_id = 0;
      }
      if (initObj.hasOwnProperty('disable_default_command_id')) {
        this.disable_default_command_id = initObj.disable_default_command_id
      }
      else {
        this.disable_default_command_id = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReassignCommandIdCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [command_id_index]
    bufferOffset = _serializer.uint8(obj.command_id_index, buffer, bufferOffset);
    // Serialize message field [user_command_id]
    bufferOffset = _serializer.uint32(obj.user_command_id, buffer, bufferOffset);
    // Serialize message field [disable_default_command_id]
    bufferOffset = _serializer.bool(obj.disable_default_command_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReassignCommandIdCmd
    let len;
    let data = new ReassignCommandIdCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [command_id_index]
    data.command_id_index = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [user_command_id]
    data.user_command_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [disable_default_command_id]
    data.disable_default_command_id = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ReassignCommandIdCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41d43df68f42f68725a7567326abaa4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    uint8 command_id_index            # The index of the user-defined command ID (1-4) to change.
    uint32 user_command_id            # The new user-defined command ID to set. 0xFFFEXX and 0xFF00XX are reserved.
                                      # Setting this to 0xFFFFFFFF will change the disable_default_command_id flag without affecting any others.
    bool disable_default_command_id   # Disables (true) or enables (false) the default command ID.
    
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
    const resolved = new ReassignCommandIdCmd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.confirm !== undefined) {
      resolved.confirm = msg.confirm;
    }
    else {
      resolved.confirm = false
    }

    if (msg.command_id_index !== undefined) {
      resolved.command_id_index = msg.command_id_index;
    }
    else {
      resolved.command_id_index = 0
    }

    if (msg.user_command_id !== undefined) {
      resolved.user_command_id = msg.user_command_id;
    }
    else {
      resolved.user_command_id = 0
    }

    if (msg.disable_default_command_id !== undefined) {
      resolved.disable_default_command_id = msg.disable_default_command_id;
    }
    else {
      resolved.disable_default_command_id = false
    }

    return resolved;
    }
};

module.exports = ReassignCommandIdCmd;
