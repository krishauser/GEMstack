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

class ConfigureOutputsPwmFreqCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.min_pwm_pct = null;
      this.max_pwm_pct = null;
      this.pwm_freq = null;
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
      if (initObj.hasOwnProperty('min_pwm_pct')) {
        this.min_pwm_pct = initObj.min_pwm_pct
      }
      else {
        this.min_pwm_pct = 0;
      }
      if (initObj.hasOwnProperty('max_pwm_pct')) {
        this.max_pwm_pct = initObj.max_pwm_pct
      }
      else {
        this.max_pwm_pct = 0;
      }
      if (initObj.hasOwnProperty('pwm_freq')) {
        this.pwm_freq = initObj.pwm_freq
      }
      else {
        this.pwm_freq = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigureOutputsPwmFreqCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [min_pwm_pct]
    bufferOffset = _serializer.uint8(obj.min_pwm_pct, buffer, bufferOffset);
    // Serialize message field [max_pwm_pct]
    bufferOffset = _serializer.uint8(obj.max_pwm_pct, buffer, bufferOffset);
    // Serialize message field [pwm_freq]
    bufferOffset = _serializer.uint16(obj.pwm_freq, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigureOutputsPwmFreqCmd
    let len;
    let data = new ConfigureOutputsPwmFreqCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [min_pwm_pct]
    data.min_pwm_pct = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [max_pwm_pct]
    data.max_pwm_pct = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pwm_freq]
    data.pwm_freq = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ConfigureOutputsPwmFreqCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '177ba95b80ad87cfd885201c32903f9c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    uint8 min_pwm_pct   # The minimum motor drive duty cycle in percent (0-100). Default is 20%.
    uint8 max_pwm_pct   # The maximum motor drive duty cycle in percent (0-100). Default is 90%.
    uint16 pwm_freq     # The frequency of the PWM outputs in Hz. Default is 2000Hz.
    
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
    const resolved = new ConfigureOutputsPwmFreqCmd(null);
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

    if (msg.min_pwm_pct !== undefined) {
      resolved.min_pwm_pct = msg.min_pwm_pct;
    }
    else {
      resolved.min_pwm_pct = 0
    }

    if (msg.max_pwm_pct !== undefined) {
      resolved.max_pwm_pct = msg.max_pwm_pct;
    }
    else {
      resolved.max_pwm_pct = 0
    }

    if (msg.pwm_freq !== undefined) {
      resolved.pwm_freq = msg.pwm_freq;
    }
    else {
      resolved.pwm_freq = 0
    }

    return resolved;
    }
};

module.exports = ConfigureOutputsPwmFreqCmd;
