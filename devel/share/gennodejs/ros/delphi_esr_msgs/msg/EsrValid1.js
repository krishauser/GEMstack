// Auto-generated. Do not edit!

// (in-package delphi_esr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EsrValid1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.lr_sn = null;
      this.lr_range = null;
      this.lr_range_rate = null;
      this.lr_angle = null;
      this.lr_power = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('canmsg')) {
        this.canmsg = initObj.canmsg
      }
      else {
        this.canmsg = '';
      }
      if (initObj.hasOwnProperty('lr_sn')) {
        this.lr_sn = initObj.lr_sn
      }
      else {
        this.lr_sn = 0;
      }
      if (initObj.hasOwnProperty('lr_range')) {
        this.lr_range = initObj.lr_range
      }
      else {
        this.lr_range = 0.0;
      }
      if (initObj.hasOwnProperty('lr_range_rate')) {
        this.lr_range_rate = initObj.lr_range_rate
      }
      else {
        this.lr_range_rate = 0.0;
      }
      if (initObj.hasOwnProperty('lr_angle')) {
        this.lr_angle = initObj.lr_angle
      }
      else {
        this.lr_angle = 0.0;
      }
      if (initObj.hasOwnProperty('lr_power')) {
        this.lr_power = initObj.lr_power
      }
      else {
        this.lr_power = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrValid1
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [lr_sn]
    bufferOffset = _serializer.uint8(obj.lr_sn, buffer, bufferOffset);
    // Serialize message field [lr_range]
    bufferOffset = _serializer.float32(obj.lr_range, buffer, bufferOffset);
    // Serialize message field [lr_range_rate]
    bufferOffset = _serializer.float32(obj.lr_range_rate, buffer, bufferOffset);
    // Serialize message field [lr_angle]
    bufferOffset = _serializer.float32(obj.lr_angle, buffer, bufferOffset);
    // Serialize message field [lr_power]
    bufferOffset = _serializer.int8(obj.lr_power, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrValid1
    let len;
    let data = new EsrValid1(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lr_sn]
    data.lr_sn = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lr_range]
    data.lr_range = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lr_range_rate]
    data.lr_range_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lr_angle]
    data.lr_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lr_power]
    data.lr_power = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrValid1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41a700597be629966cda02aac94ad2e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Valid1
    string      canmsg
    
    uint8       lr_sn
    float32     lr_range
    float32     lr_range_rate
    float32     lr_angle
    int8        lr_power
    
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
    const resolved = new EsrValid1(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.canmsg !== undefined) {
      resolved.canmsg = msg.canmsg;
    }
    else {
      resolved.canmsg = ''
    }

    if (msg.lr_sn !== undefined) {
      resolved.lr_sn = msg.lr_sn;
    }
    else {
      resolved.lr_sn = 0
    }

    if (msg.lr_range !== undefined) {
      resolved.lr_range = msg.lr_range;
    }
    else {
      resolved.lr_range = 0.0
    }

    if (msg.lr_range_rate !== undefined) {
      resolved.lr_range_rate = msg.lr_range_rate;
    }
    else {
      resolved.lr_range_rate = 0.0
    }

    if (msg.lr_angle !== undefined) {
      resolved.lr_angle = msg.lr_angle;
    }
    else {
      resolved.lr_angle = 0.0
    }

    if (msg.lr_power !== undefined) {
      resolved.lr_power = msg.lr_power;
    }
    else {
      resolved.lr_power = 0
    }

    return resolved;
    }
};

module.exports = EsrValid1;
