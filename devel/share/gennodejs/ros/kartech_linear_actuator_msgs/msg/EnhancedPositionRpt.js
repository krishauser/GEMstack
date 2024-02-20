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

class EnhancedPositionRpt {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.shaft_extension = null;
      this.motor_overload_error = null;
      this.clutch_overload_error = null;
      this.motor_open_load_error = null;
      this.clutch_open_load_error = null;
      this.position_reach_error = null;
      this.hardware_warning_1_error = null;
      this.hardware_warning_2_error = null;
      this.motor_current = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('shaft_extension')) {
        this.shaft_extension = initObj.shaft_extension
      }
      else {
        this.shaft_extension = 0.0;
      }
      if (initObj.hasOwnProperty('motor_overload_error')) {
        this.motor_overload_error = initObj.motor_overload_error
      }
      else {
        this.motor_overload_error = false;
      }
      if (initObj.hasOwnProperty('clutch_overload_error')) {
        this.clutch_overload_error = initObj.clutch_overload_error
      }
      else {
        this.clutch_overload_error = false;
      }
      if (initObj.hasOwnProperty('motor_open_load_error')) {
        this.motor_open_load_error = initObj.motor_open_load_error
      }
      else {
        this.motor_open_load_error = false;
      }
      if (initObj.hasOwnProperty('clutch_open_load_error')) {
        this.clutch_open_load_error = initObj.clutch_open_load_error
      }
      else {
        this.clutch_open_load_error = false;
      }
      if (initObj.hasOwnProperty('position_reach_error')) {
        this.position_reach_error = initObj.position_reach_error
      }
      else {
        this.position_reach_error = false;
      }
      if (initObj.hasOwnProperty('hardware_warning_1_error')) {
        this.hardware_warning_1_error = initObj.hardware_warning_1_error
      }
      else {
        this.hardware_warning_1_error = false;
      }
      if (initObj.hasOwnProperty('hardware_warning_2_error')) {
        this.hardware_warning_2_error = initObj.hardware_warning_2_error
      }
      else {
        this.hardware_warning_2_error = false;
      }
      if (initObj.hasOwnProperty('motor_current')) {
        this.motor_current = initObj.motor_current
      }
      else {
        this.motor_current = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EnhancedPositionRpt
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [shaft_extension]
    bufferOffset = _serializer.float64(obj.shaft_extension, buffer, bufferOffset);
    // Serialize message field [motor_overload_error]
    bufferOffset = _serializer.bool(obj.motor_overload_error, buffer, bufferOffset);
    // Serialize message field [clutch_overload_error]
    bufferOffset = _serializer.bool(obj.clutch_overload_error, buffer, bufferOffset);
    // Serialize message field [motor_open_load_error]
    bufferOffset = _serializer.bool(obj.motor_open_load_error, buffer, bufferOffset);
    // Serialize message field [clutch_open_load_error]
    bufferOffset = _serializer.bool(obj.clutch_open_load_error, buffer, bufferOffset);
    // Serialize message field [position_reach_error]
    bufferOffset = _serializer.bool(obj.position_reach_error, buffer, bufferOffset);
    // Serialize message field [hardware_warning_1_error]
    bufferOffset = _serializer.bool(obj.hardware_warning_1_error, buffer, bufferOffset);
    // Serialize message field [hardware_warning_2_error]
    bufferOffset = _serializer.bool(obj.hardware_warning_2_error, buffer, bufferOffset);
    // Serialize message field [motor_current]
    bufferOffset = _serializer.uint16(obj.motor_current, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EnhancedPositionRpt
    let len;
    let data = new EnhancedPositionRpt(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [shaft_extension]
    data.shaft_extension = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [motor_overload_error]
    data.motor_overload_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clutch_overload_error]
    data.clutch_overload_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [motor_open_load_error]
    data.motor_open_load_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clutch_open_load_error]
    data.clutch_open_load_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [position_reach_error]
    data.position_reach_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hardware_warning_1_error]
    data.hardware_warning_1_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hardware_warning_2_error]
    data.hardware_warning_2_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [motor_current]
    data.motor_current = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/EnhancedPositionRpt';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b5d14804230789155d91f65364c956fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float64 shaft_extension     # The current shaft position in 0.001" increments.
    bool motor_overload_error
    bool clutch_overload_error
    bool motor_open_load_error
    bool clutch_open_load_error
    bool position_reach_error
    bool hardware_warning_1_error
    bool hardware_warning_2_error
    uint16 motor_current        # The current motor current in mA.
    
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
    const resolved = new EnhancedPositionRpt(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.shaft_extension !== undefined) {
      resolved.shaft_extension = msg.shaft_extension;
    }
    else {
      resolved.shaft_extension = 0.0
    }

    if (msg.motor_overload_error !== undefined) {
      resolved.motor_overload_error = msg.motor_overload_error;
    }
    else {
      resolved.motor_overload_error = false
    }

    if (msg.clutch_overload_error !== undefined) {
      resolved.clutch_overload_error = msg.clutch_overload_error;
    }
    else {
      resolved.clutch_overload_error = false
    }

    if (msg.motor_open_load_error !== undefined) {
      resolved.motor_open_load_error = msg.motor_open_load_error;
    }
    else {
      resolved.motor_open_load_error = false
    }

    if (msg.clutch_open_load_error !== undefined) {
      resolved.clutch_open_load_error = msg.clutch_open_load_error;
    }
    else {
      resolved.clutch_open_load_error = false
    }

    if (msg.position_reach_error !== undefined) {
      resolved.position_reach_error = msg.position_reach_error;
    }
    else {
      resolved.position_reach_error = false
    }

    if (msg.hardware_warning_1_error !== undefined) {
      resolved.hardware_warning_1_error = msg.hardware_warning_1_error;
    }
    else {
      resolved.hardware_warning_1_error = false
    }

    if (msg.hardware_warning_2_error !== undefined) {
      resolved.hardware_warning_2_error = msg.hardware_warning_2_error;
    }
    else {
      resolved.hardware_warning_2_error = false
    }

    if (msg.motor_current !== undefined) {
      resolved.motor_current = msg.motor_current;
    }
    else {
      resolved.motor_current = 0
    }

    return resolved;
    }
};

module.exports = EnhancedPositionRpt;
