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

class ReassignReportIdCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.user_report_id = null;
      this.use_user_report_id = null;
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
      if (initObj.hasOwnProperty('user_report_id')) {
        this.user_report_id = initObj.user_report_id
      }
      else {
        this.user_report_id = 0;
      }
      if (initObj.hasOwnProperty('use_user_report_id')) {
        this.use_user_report_id = initObj.use_user_report_id
      }
      else {
        this.use_user_report_id = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReassignReportIdCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [user_report_id]
    bufferOffset = _serializer.uint32(obj.user_report_id, buffer, bufferOffset);
    // Serialize message field [use_user_report_id]
    bufferOffset = _serializer.bool(obj.use_user_report_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReassignReportIdCmd
    let len;
    let data = new ReassignReportIdCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [user_report_id]
    data.user_report_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [use_user_report_id]
    data.use_user_report_id = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ReassignReportIdCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fd2b8bcf6e2eaf63371bf7a3445d195c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    uint32 user_report_id     # The new user report ID to use. Values 0xFFFEXX and 0xFF00XX are reserved. Setting this to 0xFFFFFFFF will only change the use_user_report_id flag.
    bool use_user_report_id   # Whether to use the user-defined report ID (true) or the default report ID (false).
    
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
    const resolved = new ReassignReportIdCmd(null);
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

    if (msg.user_report_id !== undefined) {
      resolved.user_report_id = msg.user_report_id;
    }
    else {
      resolved.user_report_id = 0
    }

    if (msg.use_user_report_id !== undefined) {
      resolved.use_user_report_id = msg.use_user_report_id;
    }
    else {
      resolved.use_user_report_id = false
    }

    return resolved;
    }
};

module.exports = ReassignReportIdCmd;
