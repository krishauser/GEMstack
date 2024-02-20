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

class MrrStatusSwVersion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_pbl_field_revision = null;
      this.can_pbl_promote_revision = null;
      this.can_sw_field_revision = null;
      this.can_sw_promote_revision = null;
      this.can_sw_release_revision = null;
      this.can_pbl_release_revision = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_pbl_field_revision')) {
        this.can_pbl_field_revision = initObj.can_pbl_field_revision
      }
      else {
        this.can_pbl_field_revision = 0;
      }
      if (initObj.hasOwnProperty('can_pbl_promote_revision')) {
        this.can_pbl_promote_revision = initObj.can_pbl_promote_revision
      }
      else {
        this.can_pbl_promote_revision = 0;
      }
      if (initObj.hasOwnProperty('can_sw_field_revision')) {
        this.can_sw_field_revision = initObj.can_sw_field_revision
      }
      else {
        this.can_sw_field_revision = 0;
      }
      if (initObj.hasOwnProperty('can_sw_promote_revision')) {
        this.can_sw_promote_revision = initObj.can_sw_promote_revision
      }
      else {
        this.can_sw_promote_revision = 0;
      }
      if (initObj.hasOwnProperty('can_sw_release_revision')) {
        this.can_sw_release_revision = initObj.can_sw_release_revision
      }
      else {
        this.can_sw_release_revision = 0;
      }
      if (initObj.hasOwnProperty('can_pbl_release_revision')) {
        this.can_pbl_release_revision = initObj.can_pbl_release_revision
      }
      else {
        this.can_pbl_release_revision = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrStatusSwVersion
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_pbl_field_revision]
    bufferOffset = _serializer.uint8(obj.can_pbl_field_revision, buffer, bufferOffset);
    // Serialize message field [can_pbl_promote_revision]
    bufferOffset = _serializer.uint8(obj.can_pbl_promote_revision, buffer, bufferOffset);
    // Serialize message field [can_sw_field_revision]
    bufferOffset = _serializer.uint8(obj.can_sw_field_revision, buffer, bufferOffset);
    // Serialize message field [can_sw_promote_revision]
    bufferOffset = _serializer.uint8(obj.can_sw_promote_revision, buffer, bufferOffset);
    // Serialize message field [can_sw_release_revision]
    bufferOffset = _serializer.uint8(obj.can_sw_release_revision, buffer, bufferOffset);
    // Serialize message field [can_pbl_release_revision]
    bufferOffset = _serializer.uint8(obj.can_pbl_release_revision, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrStatusSwVersion
    let len;
    let data = new MrrStatusSwVersion(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_pbl_field_revision]
    data.can_pbl_field_revision = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_pbl_promote_revision]
    data.can_pbl_promote_revision = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_sw_field_revision]
    data.can_sw_field_revision = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_sw_promote_revision]
    data.can_sw_promote_revision = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_sw_release_revision]
    data.can_sw_release_revision = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_pbl_release_revision]
    data.can_pbl_release_revision = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrStatusSwVersion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2cd8dd3a73da2efcca1c60aaa8a7f9c5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 can_pbl_field_revision
    uint8 can_pbl_promote_revision
    uint8 can_sw_field_revision
    uint8 can_sw_promote_revision
    uint8 can_sw_release_revision
    uint8 can_pbl_release_revision
    
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
    const resolved = new MrrStatusSwVersion(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_pbl_field_revision !== undefined) {
      resolved.can_pbl_field_revision = msg.can_pbl_field_revision;
    }
    else {
      resolved.can_pbl_field_revision = 0
    }

    if (msg.can_pbl_promote_revision !== undefined) {
      resolved.can_pbl_promote_revision = msg.can_pbl_promote_revision;
    }
    else {
      resolved.can_pbl_promote_revision = 0
    }

    if (msg.can_sw_field_revision !== undefined) {
      resolved.can_sw_field_revision = msg.can_sw_field_revision;
    }
    else {
      resolved.can_sw_field_revision = 0
    }

    if (msg.can_sw_promote_revision !== undefined) {
      resolved.can_sw_promote_revision = msg.can_sw_promote_revision;
    }
    else {
      resolved.can_sw_promote_revision = 0
    }

    if (msg.can_sw_release_revision !== undefined) {
      resolved.can_sw_release_revision = msg.can_sw_release_revision;
    }
    else {
      resolved.can_sw_release_revision = 0
    }

    if (msg.can_pbl_release_revision !== undefined) {
      resolved.can_pbl_release_revision = msg.can_pbl_release_revision;
    }
    else {
      resolved.can_pbl_release_revision = 0
    }

    return resolved;
    }
};

module.exports = MrrStatusSwVersion;
