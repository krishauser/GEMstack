// Auto-generated. Do not edit!

// (in-package kartech_linear_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ReportIndex = require('./ReportIndex.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ReportPollReq {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.report_indices = null;
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
      if (initObj.hasOwnProperty('report_indices')) {
        this.report_indices = initObj.report_indices
      }
      else {
        this.report_indices = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReportPollReq
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [report_indices]
    // Serialize the length for message field [report_indices]
    bufferOffset = _serializer.uint32(obj.report_indices.length, buffer, bufferOffset);
    obj.report_indices.forEach((val) => {
      bufferOffset = ReportIndex.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReportPollReq
    let len;
    let data = new ReportPollReq(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [report_indices]
    // Deserialize array length for message field [report_indices]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.report_indices = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.report_indices[i] = ReportIndex.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.report_indices.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ReportPollReq';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f75ac448280dc0453a2f53fff2ba9c03';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    kartech_linear_actuator_msgs/ReportIndex[] report_indices     # The indicies of the reports that you would like to receive. Up to 6 may be requested at a time.
    
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
    
    ================================================================================
    MSG: kartech_linear_actuator_msgs/ReportIndex
    uint8 POSITION_REPORT_INDEX = 128
    uint8 MOTOR_CURRENT_REPORT_INDEX = 129
    uint8 ENHANCED_POSITION_REPORT_INDEX = 152
    uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167
    uint8 SOFTWARE_REVISION_REPORT_INDEX = 229
    uint8 ZEROING_MESSAGE_REPORT_INDEX = 238
    
    uint8 report_index
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReportPollReq(null);
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

    if (msg.report_indices !== undefined) {
      resolved.report_indices = new Array(msg.report_indices.length);
      for (let i = 0; i < resolved.report_indices.length; ++i) {
        resolved.report_indices[i] = ReportIndex.Resolve(msg.report_indices[i]);
      }
    }
    else {
      resolved.report_indices = []
    }

    return resolved;
    }
};

module.exports = ReportPollReq;
