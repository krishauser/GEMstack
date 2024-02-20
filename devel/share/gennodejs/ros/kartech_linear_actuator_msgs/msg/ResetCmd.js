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

class ResetCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.reset_type = null;
      this.reset_user_rpt_id = null;
      this.reset_user_cmd_id_1 = null;
      this.reset_user_cmd_id_2 = null;
      this.reset_user_cmd_id_3 = null;
      this.reset_user_cmd_id_4 = null;
      this.disable_user_rpt_id = null;
      this.reenable_default_cmd_id = null;
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
      if (initObj.hasOwnProperty('reset_type')) {
        this.reset_type = initObj.reset_type
      }
      else {
        this.reset_type = 0;
      }
      if (initObj.hasOwnProperty('reset_user_rpt_id')) {
        this.reset_user_rpt_id = initObj.reset_user_rpt_id
      }
      else {
        this.reset_user_rpt_id = false;
      }
      if (initObj.hasOwnProperty('reset_user_cmd_id_1')) {
        this.reset_user_cmd_id_1 = initObj.reset_user_cmd_id_1
      }
      else {
        this.reset_user_cmd_id_1 = false;
      }
      if (initObj.hasOwnProperty('reset_user_cmd_id_2')) {
        this.reset_user_cmd_id_2 = initObj.reset_user_cmd_id_2
      }
      else {
        this.reset_user_cmd_id_2 = false;
      }
      if (initObj.hasOwnProperty('reset_user_cmd_id_3')) {
        this.reset_user_cmd_id_3 = initObj.reset_user_cmd_id_3
      }
      else {
        this.reset_user_cmd_id_3 = false;
      }
      if (initObj.hasOwnProperty('reset_user_cmd_id_4')) {
        this.reset_user_cmd_id_4 = initObj.reset_user_cmd_id_4
      }
      else {
        this.reset_user_cmd_id_4 = false;
      }
      if (initObj.hasOwnProperty('disable_user_rpt_id')) {
        this.disable_user_rpt_id = initObj.disable_user_rpt_id
      }
      else {
        this.disable_user_rpt_id = false;
      }
      if (initObj.hasOwnProperty('reenable_default_cmd_id')) {
        this.reenable_default_cmd_id = initObj.reenable_default_cmd_id
      }
      else {
        this.reenable_default_cmd_id = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [reset_type]
    bufferOffset = _serializer.uint8(obj.reset_type, buffer, bufferOffset);
    // Serialize message field [reset_user_rpt_id]
    bufferOffset = _serializer.bool(obj.reset_user_rpt_id, buffer, bufferOffset);
    // Serialize message field [reset_user_cmd_id_1]
    bufferOffset = _serializer.bool(obj.reset_user_cmd_id_1, buffer, bufferOffset);
    // Serialize message field [reset_user_cmd_id_2]
    bufferOffset = _serializer.bool(obj.reset_user_cmd_id_2, buffer, bufferOffset);
    // Serialize message field [reset_user_cmd_id_3]
    bufferOffset = _serializer.bool(obj.reset_user_cmd_id_3, buffer, bufferOffset);
    // Serialize message field [reset_user_cmd_id_4]
    bufferOffset = _serializer.bool(obj.reset_user_cmd_id_4, buffer, bufferOffset);
    // Serialize message field [disable_user_rpt_id]
    bufferOffset = _serializer.bool(obj.disable_user_rpt_id, buffer, bufferOffset);
    // Serialize message field [reenable_default_cmd_id]
    bufferOffset = _serializer.bool(obj.reenable_default_cmd_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetCmd
    let len;
    let data = new ResetCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reset_type]
    data.reset_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reset_user_rpt_id]
    data.reset_user_rpt_id = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reset_user_cmd_id_1]
    data.reset_user_cmd_id_1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reset_user_cmd_id_2]
    data.reset_user_cmd_id_2 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reset_user_cmd_id_3]
    data.reset_user_cmd_id_3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reset_user_cmd_id_4]
    data.reset_user_cmd_id_4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [disable_user_rpt_id]
    data.disable_user_rpt_id = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reenable_default_cmd_id]
    data.reenable_default_cmd_id = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ResetCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0599fdf3d91d47c66c1ecb3a5a9d3e0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    
    uint8 RESET_OUTPUTS = 0
    uint8 RESET_USER_DEFINED_IDS = 1
    uint8 RESET_REPORT_RATES = 2
    uint8 RESET_HARDWARE_CONFIGURATIONS = 3
    uint8 RESET_USER_CONFIGURATIONS = 4
    uint8 RESET_EVERYTHING = 5
    uint8 RESET_NONE = 6
    
    uint8 reset_type
    
    bool reset_user_rpt_id
    bool reset_user_cmd_id_1
    bool reset_user_cmd_id_2
    bool reset_user_cmd_id_3
    bool reset_user_cmd_id_4
    bool disable_user_rpt_id
    bool reenable_default_cmd_id
    
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
    const resolved = new ResetCmd(null);
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

    if (msg.reset_type !== undefined) {
      resolved.reset_type = msg.reset_type;
    }
    else {
      resolved.reset_type = 0
    }

    if (msg.reset_user_rpt_id !== undefined) {
      resolved.reset_user_rpt_id = msg.reset_user_rpt_id;
    }
    else {
      resolved.reset_user_rpt_id = false
    }

    if (msg.reset_user_cmd_id_1 !== undefined) {
      resolved.reset_user_cmd_id_1 = msg.reset_user_cmd_id_1;
    }
    else {
      resolved.reset_user_cmd_id_1 = false
    }

    if (msg.reset_user_cmd_id_2 !== undefined) {
      resolved.reset_user_cmd_id_2 = msg.reset_user_cmd_id_2;
    }
    else {
      resolved.reset_user_cmd_id_2 = false
    }

    if (msg.reset_user_cmd_id_3 !== undefined) {
      resolved.reset_user_cmd_id_3 = msg.reset_user_cmd_id_3;
    }
    else {
      resolved.reset_user_cmd_id_3 = false
    }

    if (msg.reset_user_cmd_id_4 !== undefined) {
      resolved.reset_user_cmd_id_4 = msg.reset_user_cmd_id_4;
    }
    else {
      resolved.reset_user_cmd_id_4 = false
    }

    if (msg.disable_user_rpt_id !== undefined) {
      resolved.disable_user_rpt_id = msg.disable_user_rpt_id;
    }
    else {
      resolved.disable_user_rpt_id = false
    }

    if (msg.reenable_default_cmd_id !== undefined) {
      resolved.reenable_default_cmd_id = msg.reenable_default_cmd_id;
    }
    else {
      resolved.reenable_default_cmd_id = false
    }

    return resolved;
    }
};

// Constants for message
ResetCmd.Constants = {
  RESET_OUTPUTS: 0,
  RESET_USER_DEFINED_IDS: 1,
  RESET_REPORT_RATES: 2,
  RESET_HARDWARE_CONFIGURATIONS: 3,
  RESET_USER_CONFIGURATIONS: 4,
  RESET_EVERYTHING: 5,
  RESET_NONE: 6,
}

module.exports = ResetCmd;
