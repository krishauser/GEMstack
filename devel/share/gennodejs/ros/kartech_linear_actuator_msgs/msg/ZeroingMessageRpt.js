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

class ZeroingMessageRpt {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.chip_1_voltage = null;
      this.chip_2_voltage = null;
      this.chip_error_1 = null;
      this.chip_error_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('chip_1_voltage')) {
        this.chip_1_voltage = initObj.chip_1_voltage
      }
      else {
        this.chip_1_voltage = 0;
      }
      if (initObj.hasOwnProperty('chip_2_voltage')) {
        this.chip_2_voltage = initObj.chip_2_voltage
      }
      else {
        this.chip_2_voltage = 0;
      }
      if (initObj.hasOwnProperty('chip_error_1')) {
        this.chip_error_1 = initObj.chip_error_1
      }
      else {
        this.chip_error_1 = 0;
      }
      if (initObj.hasOwnProperty('chip_error_2')) {
        this.chip_error_2 = initObj.chip_error_2
      }
      else {
        this.chip_error_2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ZeroingMessageRpt
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [chip_1_voltage]
    bufferOffset = _serializer.uint16(obj.chip_1_voltage, buffer, bufferOffset);
    // Serialize message field [chip_2_voltage]
    bufferOffset = _serializer.uint16(obj.chip_2_voltage, buffer, bufferOffset);
    // Serialize message field [chip_error_1]
    bufferOffset = _serializer.uint8(obj.chip_error_1, buffer, bufferOffset);
    // Serialize message field [chip_error_2]
    bufferOffset = _serializer.uint8(obj.chip_error_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ZeroingMessageRpt
    let len;
    let data = new ZeroingMessageRpt(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [chip_1_voltage]
    data.chip_1_voltage = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [chip_2_voltage]
    data.chip_2_voltage = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [chip_error_1]
    data.chip_error_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [chip_error_2]
    data.chip_error_2 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ZeroingMessageRpt';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1be34276909afaf5d9fd5f38a98c32a7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint16 chip_1_voltage
    uint16 chip_2_voltage
    uint8 chip_error_1
    uint8 chip_error_2
    
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
    const resolved = new ZeroingMessageRpt(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.chip_1_voltage !== undefined) {
      resolved.chip_1_voltage = msg.chip_1_voltage;
    }
    else {
      resolved.chip_1_voltage = 0
    }

    if (msg.chip_2_voltage !== undefined) {
      resolved.chip_2_voltage = msg.chip_2_voltage;
    }
    else {
      resolved.chip_2_voltage = 0
    }

    if (msg.chip_error_1 !== undefined) {
      resolved.chip_error_1 = msg.chip_error_1;
    }
    else {
      resolved.chip_error_1 = 0
    }

    if (msg.chip_error_2 !== undefined) {
      resolved.chip_error_2 = msg.chip_error_2;
    }
    else {
      resolved.chip_error_2 = 0
    }

    return resolved;
    }
};

module.exports = ZeroingMessageRpt;
