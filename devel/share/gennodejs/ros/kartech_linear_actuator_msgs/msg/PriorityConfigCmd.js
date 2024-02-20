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

class PriorityConfigCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.confirm = null;
      this.handshake_priority = null;
      this.auto_reply_priority = null;
      this.scheduled_priority = null;
      this.polled_priority = null;
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
      if (initObj.hasOwnProperty('handshake_priority')) {
        this.handshake_priority = initObj.handshake_priority
      }
      else {
        this.handshake_priority = 0;
      }
      if (initObj.hasOwnProperty('auto_reply_priority')) {
        this.auto_reply_priority = initObj.auto_reply_priority
      }
      else {
        this.auto_reply_priority = 0;
      }
      if (initObj.hasOwnProperty('scheduled_priority')) {
        this.scheduled_priority = initObj.scheduled_priority
      }
      else {
        this.scheduled_priority = 0;
      }
      if (initObj.hasOwnProperty('polled_priority')) {
        this.polled_priority = initObj.polled_priority
      }
      else {
        this.polled_priority = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PriorityConfigCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [confirm]
    bufferOffset = _serializer.bool(obj.confirm, buffer, bufferOffset);
    // Serialize message field [handshake_priority]
    bufferOffset = _serializer.uint8(obj.handshake_priority, buffer, bufferOffset);
    // Serialize message field [auto_reply_priority]
    bufferOffset = _serializer.uint8(obj.auto_reply_priority, buffer, bufferOffset);
    // Serialize message field [scheduled_priority]
    bufferOffset = _serializer.uint8(obj.scheduled_priority, buffer, bufferOffset);
    // Serialize message field [polled_priority]
    bufferOffset = _serializer.uint8(obj.polled_priority, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PriorityConfigCmd
    let len;
    let data = new PriorityConfigCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [confirm]
    data.confirm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [handshake_priority]
    data.handshake_priority = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [auto_reply_priority]
    data.auto_reply_priority = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [scheduled_priority]
    data.scheduled_priority = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [polled_priority]
    data.polled_priority = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/PriorityConfigCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04b16f80c8b9d73ef8343b9ba34c9b78';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool confirm
    
    # Priority values for different types of reports. Lower value = higher priority.
    uint8 handshake_priority
    uint8 auto_reply_priority
    uint8 scheduled_priority
    uint8 polled_priority
    
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
    const resolved = new PriorityConfigCmd(null);
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

    if (msg.handshake_priority !== undefined) {
      resolved.handshake_priority = msg.handshake_priority;
    }
    else {
      resolved.handshake_priority = 0
    }

    if (msg.auto_reply_priority !== undefined) {
      resolved.auto_reply_priority = msg.auto_reply_priority;
    }
    else {
      resolved.auto_reply_priority = 0
    }

    if (msg.scheduled_priority !== undefined) {
      resolved.scheduled_priority = msg.scheduled_priority;
    }
    else {
      resolved.scheduled_priority = 0
    }

    if (msg.polled_priority !== undefined) {
      resolved.polled_priority = msg.polled_priority;
    }
    else {
      resolved.polled_priority = 0
    }

    return resolved;
    }
};

module.exports = PriorityConfigCmd;
