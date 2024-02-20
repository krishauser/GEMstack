// Auto-generated. Do not edit!

// (in-package delphi_srr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SrrFeatureSwVersion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lcma_sw_version = null;
      this.cta_sw_version = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lcma_sw_version')) {
        this.lcma_sw_version = initObj.lcma_sw_version
      }
      else {
        this.lcma_sw_version = 0;
      }
      if (initObj.hasOwnProperty('cta_sw_version')) {
        this.cta_sw_version = initObj.cta_sw_version
      }
      else {
        this.cta_sw_version = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrFeatureSwVersion
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lcma_sw_version]
    bufferOffset = _serializer.uint8(obj.lcma_sw_version, buffer, bufferOffset);
    // Serialize message field [cta_sw_version]
    bufferOffset = _serializer.uint8(obj.cta_sw_version, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrFeatureSwVersion
    let len;
    let data = new SrrFeatureSwVersion(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lcma_sw_version]
    data.lcma_sw_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cta_sw_version]
    data.cta_sw_version = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrFeatureSwVersion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87f69bbbab65c94e0dda04a9e0914206';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_feature_sw_version
    
    std_msgs/Header header
    
    uint8     lcma_sw_version
    uint8     cta_sw_version
    
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
    const resolved = new SrrFeatureSwVersion(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lcma_sw_version !== undefined) {
      resolved.lcma_sw_version = msg.lcma_sw_version;
    }
    else {
      resolved.lcma_sw_version = 0
    }

    if (msg.cta_sw_version !== undefined) {
      resolved.cta_sw_version = msg.cta_sw_version;
    }
    else {
      resolved.cta_sw_version = 0
    }

    return resolved;
    }
};

module.exports = SrrFeatureSwVersion;
