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

class SoftwareRevisionRpt {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.software_version_0 = null;
      this.software_version_1 = null;
      this.software_version_2 = null;
      this.software_day = null;
      this.software_month_year = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('software_version_0')) {
        this.software_version_0 = initObj.software_version_0
      }
      else {
        this.software_version_0 = 0;
      }
      if (initObj.hasOwnProperty('software_version_1')) {
        this.software_version_1 = initObj.software_version_1
      }
      else {
        this.software_version_1 = 0;
      }
      if (initObj.hasOwnProperty('software_version_2')) {
        this.software_version_2 = initObj.software_version_2
      }
      else {
        this.software_version_2 = 0;
      }
      if (initObj.hasOwnProperty('software_day')) {
        this.software_day = initObj.software_day
      }
      else {
        this.software_day = 0;
      }
      if (initObj.hasOwnProperty('software_month_year')) {
        this.software_month_year = initObj.software_month_year
      }
      else {
        this.software_month_year = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SoftwareRevisionRpt
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [software_version_0]
    bufferOffset = _serializer.uint8(obj.software_version_0, buffer, bufferOffset);
    // Serialize message field [software_version_1]
    bufferOffset = _serializer.uint8(obj.software_version_1, buffer, bufferOffset);
    // Serialize message field [software_version_2]
    bufferOffset = _serializer.uint8(obj.software_version_2, buffer, bufferOffset);
    // Serialize message field [software_day]
    bufferOffset = _serializer.uint8(obj.software_day, buffer, bufferOffset);
    // Serialize message field [software_month_year]
    bufferOffset = _serializer.uint16(obj.software_month_year, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SoftwareRevisionRpt
    let len;
    let data = new SoftwareRevisionRpt(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [software_version_0]
    data.software_version_0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [software_version_1]
    data.software_version_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [software_version_2]
    data.software_version_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [software_day]
    data.software_day = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [software_month_year]
    data.software_month_year = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/SoftwareRevisionRpt';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b4e8937bac2714ef707d040a16f320a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint8 software_version_0
    uint8 software_version_1
    uint8 software_version_2
    uint8 software_day
    uint16 software_month_year
    
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
    const resolved = new SoftwareRevisionRpt(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.software_version_0 !== undefined) {
      resolved.software_version_0 = msg.software_version_0;
    }
    else {
      resolved.software_version_0 = 0
    }

    if (msg.software_version_1 !== undefined) {
      resolved.software_version_1 = msg.software_version_1;
    }
    else {
      resolved.software_version_1 = 0
    }

    if (msg.software_version_2 !== undefined) {
      resolved.software_version_2 = msg.software_version_2;
    }
    else {
      resolved.software_version_2 = 0
    }

    if (msg.software_day !== undefined) {
      resolved.software_day = msg.software_day;
    }
    else {
      resolved.software_day = 0
    }

    if (msg.software_month_year !== undefined) {
      resolved.software_month_year = msg.software_month_year;
    }
    else {
      resolved.software_month_year = 0
    }

    return resolved;
    }
};

module.exports = SoftwareRevisionRpt;
