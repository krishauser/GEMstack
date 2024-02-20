// Auto-generated. Do not edit!

// (in-package kartech_linear_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ReportIndex {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.report_index = null;
    }
    else {
      if (initObj.hasOwnProperty('report_index')) {
        this.report_index = initObj.report_index
      }
      else {
        this.report_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReportIndex
    // Serialize message field [report_index]
    bufferOffset = _serializer.uint8(obj.report_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReportIndex
    let len;
    let data = new ReportIndex(null);
    // Deserialize message field [report_index]
    data.report_index = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ReportIndex';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05847e803066ad58819c151b2e8471e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new ReportIndex(null);
    if (msg.report_index !== undefined) {
      resolved.report_index = msg.report_index;
    }
    else {
      resolved.report_index = 0
    }

    return resolved;
    }
};

// Constants for message
ReportIndex.Constants = {
  POSITION_REPORT_INDEX: 128,
  MOTOR_CURRENT_REPORT_INDEX: 129,
  ENHANCED_POSITION_REPORT_INDEX: 152,
  UNIQUE_DEVICE_ID_REPORTS_INDEX: 167,
  SOFTWARE_REVISION_REPORT_INDEX: 229,
  ZEROING_MESSAGE_REPORT_INDEX: 238,
}

module.exports = ReportIndex;
