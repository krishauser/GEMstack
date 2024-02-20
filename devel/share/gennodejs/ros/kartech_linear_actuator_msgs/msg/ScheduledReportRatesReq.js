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

class ScheduledReportRatesReq {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.index_1 = null;
      this.index_1_report_time = null;
      this.index_2 = null;
      this.index_2_report_time = null;
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
      if (initObj.hasOwnProperty('index_1')) {
        this.index_1 = initObj.index_1
      }
      else {
        this.index_1 = new ReportIndex();
      }
      if (initObj.hasOwnProperty('index_1_report_time')) {
        this.index_1_report_time = initObj.index_1_report_time
      }
      else {
        this.index_1_report_time = 0;
      }
      if (initObj.hasOwnProperty('index_2')) {
        this.index_2 = initObj.index_2
      }
      else {
        this.index_2 = new ReportIndex();
      }
      if (initObj.hasOwnProperty('index_2_report_time')) {
        this.index_2_report_time = initObj.index_2_report_time
      }
      else {
        this.index_2_report_time = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ScheduledReportRatesReq
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [index_1]
    bufferOffset = ReportIndex.serialize(obj.index_1, buffer, bufferOffset);
    // Serialize message field [index_1_report_time]
    bufferOffset = _serializer.uint16(obj.index_1_report_time, buffer, bufferOffset);
    // Serialize message field [index_2]
    bufferOffset = ReportIndex.serialize(obj.index_2, buffer, bufferOffset);
    // Serialize message field [index_2_report_time]
    bufferOffset = _serializer.uint16(obj.index_2_report_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ScheduledReportRatesReq
    let len;
    let data = new ScheduledReportRatesReq(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [index_1]
    data.index_1 = ReportIndex.deserialize(buffer, bufferOffset);
    // Deserialize message field [index_1_report_time]
    data.index_1_report_time = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [index_2]
    data.index_2 = ReportIndex.deserialize(buffer, bufferOffset);
    // Deserialize message field [index_2_report_time]
    data.index_2_report_time = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ScheduledReportRatesReq';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26225aeadc02f4f458a0546ea8c99d87';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    kartech_linear_actuator_msgs/ReportIndex index_1
    uint16 index_1_report_time                       # How often to publish the requested report in ms.
    kartech_linear_actuator_msgs/ReportIndex index_2 # If this is set to REPORT_NONE_INDEX then only the first index will be reported.
    uint16 index_2_report_time                       # Ignored if index_2 is set to REPORT_NONE_INDEX.
    
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
    const resolved = new ScheduledReportRatesReq(null);
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

    if (msg.index_1 !== undefined) {
      resolved.index_1 = ReportIndex.Resolve(msg.index_1)
    }
    else {
      resolved.index_1 = new ReportIndex()
    }

    if (msg.index_1_report_time !== undefined) {
      resolved.index_1_report_time = msg.index_1_report_time;
    }
    else {
      resolved.index_1_report_time = 0
    }

    if (msg.index_2 !== undefined) {
      resolved.index_2 = ReportIndex.Resolve(msg.index_2)
    }
    else {
      resolved.index_2 = new ReportIndex()
    }

    if (msg.index_2_report_time !== undefined) {
      resolved.index_2_report_time = msg.index_2_report_time;
    }
    else {
      resolved.index_2_report_time = 0
    }

    return resolved;
    }
};

module.exports = ScheduledReportRatesReq;
