// Auto-generated. Do not edit!

// (in-package mobileye_560_660_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TsrVisionOnly {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.vision_only_sign_type_display1 = null;
      this.vision_only_supplementary_sign_type_display1 = null;
      this.vision_only_sign_type_display2 = null;
      this.vision_only_supplementary_sign_type_display2 = null;
      this.vision_only_sign_type_display3 = null;
      this.vision_only_supplementary_sign_type_display3 = null;
      this.vision_only_sign_type_display4 = null;
      this.vision_only_supplementary_sign_type_display4 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('vision_only_sign_type_display1')) {
        this.vision_only_sign_type_display1 = initObj.vision_only_sign_type_display1
      }
      else {
        this.vision_only_sign_type_display1 = 0;
      }
      if (initObj.hasOwnProperty('vision_only_supplementary_sign_type_display1')) {
        this.vision_only_supplementary_sign_type_display1 = initObj.vision_only_supplementary_sign_type_display1
      }
      else {
        this.vision_only_supplementary_sign_type_display1 = 0;
      }
      if (initObj.hasOwnProperty('vision_only_sign_type_display2')) {
        this.vision_only_sign_type_display2 = initObj.vision_only_sign_type_display2
      }
      else {
        this.vision_only_sign_type_display2 = 0;
      }
      if (initObj.hasOwnProperty('vision_only_supplementary_sign_type_display2')) {
        this.vision_only_supplementary_sign_type_display2 = initObj.vision_only_supplementary_sign_type_display2
      }
      else {
        this.vision_only_supplementary_sign_type_display2 = 0;
      }
      if (initObj.hasOwnProperty('vision_only_sign_type_display3')) {
        this.vision_only_sign_type_display3 = initObj.vision_only_sign_type_display3
      }
      else {
        this.vision_only_sign_type_display3 = 0;
      }
      if (initObj.hasOwnProperty('vision_only_supplementary_sign_type_display3')) {
        this.vision_only_supplementary_sign_type_display3 = initObj.vision_only_supplementary_sign_type_display3
      }
      else {
        this.vision_only_supplementary_sign_type_display3 = 0;
      }
      if (initObj.hasOwnProperty('vision_only_sign_type_display4')) {
        this.vision_only_sign_type_display4 = initObj.vision_only_sign_type_display4
      }
      else {
        this.vision_only_sign_type_display4 = 0;
      }
      if (initObj.hasOwnProperty('vision_only_supplementary_sign_type_display4')) {
        this.vision_only_supplementary_sign_type_display4 = initObj.vision_only_supplementary_sign_type_display4
      }
      else {
        this.vision_only_supplementary_sign_type_display4 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TsrVisionOnly
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [vision_only_sign_type_display1]
    bufferOffset = _serializer.uint8(obj.vision_only_sign_type_display1, buffer, bufferOffset);
    // Serialize message field [vision_only_supplementary_sign_type_display1]
    bufferOffset = _serializer.uint8(obj.vision_only_supplementary_sign_type_display1, buffer, bufferOffset);
    // Serialize message field [vision_only_sign_type_display2]
    bufferOffset = _serializer.uint8(obj.vision_only_sign_type_display2, buffer, bufferOffset);
    // Serialize message field [vision_only_supplementary_sign_type_display2]
    bufferOffset = _serializer.uint8(obj.vision_only_supplementary_sign_type_display2, buffer, bufferOffset);
    // Serialize message field [vision_only_sign_type_display3]
    bufferOffset = _serializer.uint8(obj.vision_only_sign_type_display3, buffer, bufferOffset);
    // Serialize message field [vision_only_supplementary_sign_type_display3]
    bufferOffset = _serializer.uint8(obj.vision_only_supplementary_sign_type_display3, buffer, bufferOffset);
    // Serialize message field [vision_only_sign_type_display4]
    bufferOffset = _serializer.uint8(obj.vision_only_sign_type_display4, buffer, bufferOffset);
    // Serialize message field [vision_only_supplementary_sign_type_display4]
    bufferOffset = _serializer.uint8(obj.vision_only_supplementary_sign_type_display4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TsrVisionOnly
    let len;
    let data = new TsrVisionOnly(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [vision_only_sign_type_display1]
    data.vision_only_sign_type_display1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_supplementary_sign_type_display1]
    data.vision_only_supplementary_sign_type_display1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_sign_type_display2]
    data.vision_only_sign_type_display2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_supplementary_sign_type_display2]
    data.vision_only_supplementary_sign_type_display2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_sign_type_display3]
    data.vision_only_sign_type_display3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_supplementary_sign_type_display3]
    data.vision_only_supplementary_sign_type_display3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_sign_type_display4]
    data.vision_only_sign_type_display4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_supplementary_sign_type_display4]
    data.vision_only_supplementary_sign_type_display4 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/TsrVisionOnly';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '84f9582e1cda52683c53338cffe795f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 vision_only_sign_type_display1
    uint8 vision_only_supplementary_sign_type_display1
    uint8 vision_only_sign_type_display2
    uint8 vision_only_supplementary_sign_type_display2
    uint8 vision_only_sign_type_display3
    uint8 vision_only_supplementary_sign_type_display3
    uint8 vision_only_sign_type_display4
    uint8 vision_only_supplementary_sign_type_display4
    
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
    const resolved = new TsrVisionOnly(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.vision_only_sign_type_display1 !== undefined) {
      resolved.vision_only_sign_type_display1 = msg.vision_only_sign_type_display1;
    }
    else {
      resolved.vision_only_sign_type_display1 = 0
    }

    if (msg.vision_only_supplementary_sign_type_display1 !== undefined) {
      resolved.vision_only_supplementary_sign_type_display1 = msg.vision_only_supplementary_sign_type_display1;
    }
    else {
      resolved.vision_only_supplementary_sign_type_display1 = 0
    }

    if (msg.vision_only_sign_type_display2 !== undefined) {
      resolved.vision_only_sign_type_display2 = msg.vision_only_sign_type_display2;
    }
    else {
      resolved.vision_only_sign_type_display2 = 0
    }

    if (msg.vision_only_supplementary_sign_type_display2 !== undefined) {
      resolved.vision_only_supplementary_sign_type_display2 = msg.vision_only_supplementary_sign_type_display2;
    }
    else {
      resolved.vision_only_supplementary_sign_type_display2 = 0
    }

    if (msg.vision_only_sign_type_display3 !== undefined) {
      resolved.vision_only_sign_type_display3 = msg.vision_only_sign_type_display3;
    }
    else {
      resolved.vision_only_sign_type_display3 = 0
    }

    if (msg.vision_only_supplementary_sign_type_display3 !== undefined) {
      resolved.vision_only_supplementary_sign_type_display3 = msg.vision_only_supplementary_sign_type_display3;
    }
    else {
      resolved.vision_only_supplementary_sign_type_display3 = 0
    }

    if (msg.vision_only_sign_type_display4 !== undefined) {
      resolved.vision_only_sign_type_display4 = msg.vision_only_sign_type_display4;
    }
    else {
      resolved.vision_only_sign_type_display4 = 0
    }

    if (msg.vision_only_supplementary_sign_type_display4 !== undefined) {
      resolved.vision_only_supplementary_sign_type_display4 = msg.vision_only_supplementary_sign_type_display4;
    }
    else {
      resolved.vision_only_supplementary_sign_type_display4 = 0
    }

    return resolved;
    }
};

module.exports = TsrVisionOnly;
