// Auto-generated. Do not edit!

// (in-package delphi_mrr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MrrStatusCanVersion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_usc_section_compatibility = null;
      this.can_pcan_minor_mrr = null;
      this.can_pcan_major_mrr = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_usc_section_compatibility')) {
        this.can_usc_section_compatibility = initObj.can_usc_section_compatibility
      }
      else {
        this.can_usc_section_compatibility = 0;
      }
      if (initObj.hasOwnProperty('can_pcan_minor_mrr')) {
        this.can_pcan_minor_mrr = initObj.can_pcan_minor_mrr
      }
      else {
        this.can_pcan_minor_mrr = 0;
      }
      if (initObj.hasOwnProperty('can_pcan_major_mrr')) {
        this.can_pcan_major_mrr = initObj.can_pcan_major_mrr
      }
      else {
        this.can_pcan_major_mrr = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrStatusCanVersion
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_usc_section_compatibility]
    bufferOffset = _serializer.uint16(obj.can_usc_section_compatibility, buffer, bufferOffset);
    // Serialize message field [can_pcan_minor_mrr]
    bufferOffset = _serializer.uint8(obj.can_pcan_minor_mrr, buffer, bufferOffset);
    // Serialize message field [can_pcan_major_mrr]
    bufferOffset = _serializer.uint8(obj.can_pcan_major_mrr, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrStatusCanVersion
    let len;
    let data = new MrrStatusCanVersion(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_usc_section_compatibility]
    data.can_usc_section_compatibility = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_pcan_minor_mrr]
    data.can_pcan_minor_mrr = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_pcan_major_mrr]
    data.can_pcan_major_mrr = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrStatusCanVersion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a03c2b40596b39ba528bf17fb334bae6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint16 can_usc_section_compatibility
    uint8  can_pcan_minor_mrr
    uint8  can_pcan_major_mrr
    
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
    const resolved = new MrrStatusCanVersion(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_usc_section_compatibility !== undefined) {
      resolved.can_usc_section_compatibility = msg.can_usc_section_compatibility;
    }
    else {
      resolved.can_usc_section_compatibility = 0
    }

    if (msg.can_pcan_minor_mrr !== undefined) {
      resolved.can_pcan_minor_mrr = msg.can_pcan_minor_mrr;
    }
    else {
      resolved.can_pcan_minor_mrr = 0
    }

    if (msg.can_pcan_major_mrr !== undefined) {
      resolved.can_pcan_major_mrr = msg.can_pcan_major_mrr;
    }
    else {
      resolved.can_pcan_major_mrr = 0
    }

    return resolved;
    }
};

module.exports = MrrStatusCanVersion;
