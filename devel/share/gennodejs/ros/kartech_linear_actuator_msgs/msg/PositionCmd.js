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

class PositionCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.auto_reply = null;
      this.position = null;
      this.clutch_enable = null;
      this.motor_enable = null;
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
      if (initObj.hasOwnProperty('auto_reply')) {
        this.auto_reply = initObj.auto_reply
      }
      else {
        this.auto_reply = false;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('clutch_enable')) {
        this.clutch_enable = initObj.clutch_enable
      }
      else {
        this.clutch_enable = false;
      }
      if (initObj.hasOwnProperty('motor_enable')) {
        this.motor_enable = initObj.motor_enable
      }
      else {
        this.motor_enable = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PositionCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [auto_reply]
    bufferOffset = _serializer.bool(obj.auto_reply, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float64(obj.position, buffer, bufferOffset);
    // Serialize message field [clutch_enable]
    bufferOffset = _serializer.bool(obj.clutch_enable, buffer, bufferOffset);
    // Serialize message field [motor_enable]
    bufferOffset = _serializer.bool(obj.motor_enable, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PositionCmd
    let len;
    let data = new PositionCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [auto_reply]
    data.auto_reply = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [clutch_enable]
    data.clutch_enable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [motor_enable]
    data.motor_enable = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/PositionCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac9ab77927289195f06ee9b42fabeac2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    bool auto_reply
    float64 position    # Position in 0.001" increments.
    bool clutch_enable  # Disables (false) or enables (true) the built-in clutch after the position has been reached.
    bool motor_enable   # Disables (false) or enables (true) the motor after the position has been reached.
    
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
    const resolved = new PositionCmd(null);
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

    if (msg.auto_reply !== undefined) {
      resolved.auto_reply = msg.auto_reply;
    }
    else {
      resolved.auto_reply = false
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.clutch_enable !== undefined) {
      resolved.clutch_enable = msg.clutch_enable;
    }
    else {
      resolved.clutch_enable = false
    }

    if (msg.motor_enable !== undefined) {
      resolved.motor_enable = msg.motor_enable;
    }
    else {
      resolved.motor_enable = false
    }

    return resolved;
    }
};

module.exports = PositionCmd;
