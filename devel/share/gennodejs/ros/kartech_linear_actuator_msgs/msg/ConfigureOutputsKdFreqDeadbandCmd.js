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

class ConfigureOutputsKdFreqDeadbandCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.kd = null;
      this.closed_loop_freq = null;
      this.error_dead_band = null;
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
      if (initObj.hasOwnProperty('kd')) {
        this.kd = initObj.kd
      }
      else {
        this.kd = 0;
      }
      if (initObj.hasOwnProperty('closed_loop_freq')) {
        this.closed_loop_freq = initObj.closed_loop_freq
      }
      else {
        this.closed_loop_freq = 0;
      }
      if (initObj.hasOwnProperty('error_dead_band')) {
        this.error_dead_band = initObj.error_dead_band
      }
      else {
        this.error_dead_band = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigureOutputsKdFreqDeadbandCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [kd]
    bufferOffset = _serializer.uint16(obj.kd, buffer, bufferOffset);
    // Serialize message field [closed_loop_freq]
    bufferOffset = _serializer.uint8(obj.closed_loop_freq, buffer, bufferOffset);
    // Serialize message field [error_dead_band]
    bufferOffset = _serializer.float64(obj.error_dead_band, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigureOutputsKdFreqDeadbandCmd
    let len;
    let data = new ConfigureOutputsKdFreqDeadbandCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [kd]
    data.kd = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [closed_loop_freq]
    data.closed_loop_freq = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error_dead_band]
    data.error_dead_band = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/ConfigureOutputsKdFreqDeadbandCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '865fff7dce2fec39beac32ec4e1f4638';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    uint16 kd                   # The differential term of the closed-loop control. Default is 10.
    uint8 closed_loop_freq      # The frequency of closed-loop corrections in Hz. The default is 60Hz.
    float64 error_dead_band     # The size of the dead-band for error correction in units of 0.001". The default is 0.05"
    
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
    const resolved = new ConfigureOutputsKdFreqDeadbandCmd(null);
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

    if (msg.kd !== undefined) {
      resolved.kd = msg.kd;
    }
    else {
      resolved.kd = 0
    }

    if (msg.closed_loop_freq !== undefined) {
      resolved.closed_loop_freq = msg.closed_loop_freq;
    }
    else {
      resolved.closed_loop_freq = 0
    }

    if (msg.error_dead_band !== undefined) {
      resolved.error_dead_band = msg.error_dead_band;
    }
    else {
      resolved.error_dead_band = 0.0
    }

    return resolved;
    }
};

module.exports = ConfigureOutputsKdFreqDeadbandCmd;
