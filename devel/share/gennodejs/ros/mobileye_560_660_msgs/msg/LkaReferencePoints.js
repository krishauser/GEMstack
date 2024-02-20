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

class LkaReferencePoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ref_point_1_position = null;
      this.ref_point_1_distance = null;
      this.ref_point_1_validity = null;
      this.ref_point_2_position = null;
      this.ref_point_2_distance = null;
      this.ref_point_2_validity = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ref_point_1_position')) {
        this.ref_point_1_position = initObj.ref_point_1_position
      }
      else {
        this.ref_point_1_position = 0.0;
      }
      if (initObj.hasOwnProperty('ref_point_1_distance')) {
        this.ref_point_1_distance = initObj.ref_point_1_distance
      }
      else {
        this.ref_point_1_distance = 0.0;
      }
      if (initObj.hasOwnProperty('ref_point_1_validity')) {
        this.ref_point_1_validity = initObj.ref_point_1_validity
      }
      else {
        this.ref_point_1_validity = false;
      }
      if (initObj.hasOwnProperty('ref_point_2_position')) {
        this.ref_point_2_position = initObj.ref_point_2_position
      }
      else {
        this.ref_point_2_position = 0.0;
      }
      if (initObj.hasOwnProperty('ref_point_2_distance')) {
        this.ref_point_2_distance = initObj.ref_point_2_distance
      }
      else {
        this.ref_point_2_distance = 0.0;
      }
      if (initObj.hasOwnProperty('ref_point_2_validity')) {
        this.ref_point_2_validity = initObj.ref_point_2_validity
      }
      else {
        this.ref_point_2_validity = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LkaReferencePoints
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ref_point_1_position]
    bufferOffset = _serializer.float64(obj.ref_point_1_position, buffer, bufferOffset);
    // Serialize message field [ref_point_1_distance]
    bufferOffset = _serializer.float64(obj.ref_point_1_distance, buffer, bufferOffset);
    // Serialize message field [ref_point_1_validity]
    bufferOffset = _serializer.bool(obj.ref_point_1_validity, buffer, bufferOffset);
    // Serialize message field [ref_point_2_position]
    bufferOffset = _serializer.float64(obj.ref_point_2_position, buffer, bufferOffset);
    // Serialize message field [ref_point_2_distance]
    bufferOffset = _serializer.float64(obj.ref_point_2_distance, buffer, bufferOffset);
    // Serialize message field [ref_point_2_validity]
    bufferOffset = _serializer.bool(obj.ref_point_2_validity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LkaReferencePoints
    let len;
    let data = new LkaReferencePoints(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ref_point_1_position]
    data.ref_point_1_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ref_point_1_distance]
    data.ref_point_1_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ref_point_1_validity]
    data.ref_point_1_validity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ref_point_2_position]
    data.ref_point_2_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ref_point_2_distance]
    data.ref_point_2_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ref_point_2_validity]
    data.ref_point_2_validity = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 34;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/LkaReferencePoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0da833fa4330bb296afc10246a88cb60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float64 ref_point_1_position
    float64 ref_point_1_distance
    bool ref_point_1_validity
    float64 ref_point_2_position
    float64 ref_point_2_distance
    bool ref_point_2_validity
    
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
    const resolved = new LkaReferencePoints(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ref_point_1_position !== undefined) {
      resolved.ref_point_1_position = msg.ref_point_1_position;
    }
    else {
      resolved.ref_point_1_position = 0.0
    }

    if (msg.ref_point_1_distance !== undefined) {
      resolved.ref_point_1_distance = msg.ref_point_1_distance;
    }
    else {
      resolved.ref_point_1_distance = 0.0
    }

    if (msg.ref_point_1_validity !== undefined) {
      resolved.ref_point_1_validity = msg.ref_point_1_validity;
    }
    else {
      resolved.ref_point_1_validity = false
    }

    if (msg.ref_point_2_position !== undefined) {
      resolved.ref_point_2_position = msg.ref_point_2_position;
    }
    else {
      resolved.ref_point_2_position = 0.0
    }

    if (msg.ref_point_2_distance !== undefined) {
      resolved.ref_point_2_distance = msg.ref_point_2_distance;
    }
    else {
      resolved.ref_point_2_distance = 0.0
    }

    if (msg.ref_point_2_validity !== undefined) {
      resolved.ref_point_2_validity = msg.ref_point_2_validity;
    }
    else {
      resolved.ref_point_2_validity = false
    }

    return resolved;
    }
};

module.exports = LkaReferencePoints;
