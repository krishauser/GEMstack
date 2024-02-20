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

class EsrValid2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.mr_sn = null;
      this.mr_range = null;
      this.mr_range_rate = null;
      this.mr_angle = null;
      this.mr_power = null;
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
      if (initObj.hasOwnProperty('mr_sn')) {
        this.mr_sn = initObj.mr_sn
      }
      else {
        this.mr_sn = 0;
      }
      if (initObj.hasOwnProperty('mr_range')) {
        this.mr_range = initObj.mr_range
      }
      else {
        this.mr_range = 0.0;
      }
      if (initObj.hasOwnProperty('mr_range_rate')) {
        this.mr_range_rate = initObj.mr_range_rate
      }
      else {
        this.mr_range_rate = 0.0;
      }
      if (initObj.hasOwnProperty('mr_angle')) {
        this.mr_angle = initObj.mr_angle
      }
      else {
        this.mr_angle = 0.0;
      }
      if (initObj.hasOwnProperty('mr_power')) {
        this.mr_power = initObj.mr_power
      }
      else {
        this.mr_power = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrValid2
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [mr_sn]
    bufferOffset = _serializer.uint8(obj.mr_sn, buffer, bufferOffset);
    // Serialize message field [mr_range]
    bufferOffset = _serializer.float32(obj.mr_range, buffer, bufferOffset);
    // Serialize message field [mr_range_rate]
    bufferOffset = _serializer.float32(obj.mr_range_rate, buffer, bufferOffset);
    // Serialize message field [mr_angle]
    bufferOffset = _serializer.float32(obj.mr_angle, buffer, bufferOffset);
    // Serialize message field [mr_power]
    bufferOffset = _serializer.int8(obj.mr_power, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrValid2
    let len;
    let data = new EsrValid2(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mr_sn]
    data.mr_sn = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mr_range]
    data.mr_range = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mr_range_rate]
    data.mr_range_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mr_angle]
    data.mr_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mr_power]
    data.mr_power = _deserializer.int8(buffer, bufferOffset);
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
    return 'delphi_esr_msgs/EsrValid2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd38bb2ae1a8306a633e6d233f392ac23';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Valid2
    string      canmsg
    
    uint8       mr_sn
    float32     mr_range
    float32     mr_range_rate
    float32     mr_angle
    int8        mr_power
    
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
    const resolved = new EsrValid2(null);
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

    if (msg.mr_sn !== undefined) {
      resolved.mr_sn = msg.mr_sn;
    }
    else {
      resolved.mr_sn = 0
    }

    if (msg.mr_range !== undefined) {
      resolved.mr_range = msg.mr_range;
    }
    else {
      resolved.mr_range = 0.0
    }

    if (msg.mr_range_rate !== undefined) {
      resolved.mr_range_rate = msg.mr_range_rate;
    }
    else {
      resolved.mr_range_rate = 0.0
    }

    if (msg.mr_angle !== undefined) {
      resolved.mr_angle = msg.mr_angle;
    }
    else {
      resolved.mr_angle = 0.0
    }

    if (msg.mr_power !== undefined) {
      resolved.mr_power = msg.mr_power;
    }
    else {
      resolved.mr_power = 0
    }

    return resolved;
    }
};

module.exports = EsrValid2;
