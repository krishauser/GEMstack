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

class MrrHeaderAlignmentState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_auto_align_hangle_qf = null;
      this.can_alignment_status = null;
      this.can_alignment_state = null;
      this.can_auto_align_hangle_ref = null;
      this.can_auto_align_hangle = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_auto_align_hangle_qf')) {
        this.can_auto_align_hangle_qf = initObj.can_auto_align_hangle_qf
      }
      else {
        this.can_auto_align_hangle_qf = 0;
      }
      if (initObj.hasOwnProperty('can_alignment_status')) {
        this.can_alignment_status = initObj.can_alignment_status
      }
      else {
        this.can_alignment_status = 0;
      }
      if (initObj.hasOwnProperty('can_alignment_state')) {
        this.can_alignment_state = initObj.can_alignment_state
      }
      else {
        this.can_alignment_state = 0;
      }
      if (initObj.hasOwnProperty('can_auto_align_hangle_ref')) {
        this.can_auto_align_hangle_ref = initObj.can_auto_align_hangle_ref
      }
      else {
        this.can_auto_align_hangle_ref = 0.0;
      }
      if (initObj.hasOwnProperty('can_auto_align_hangle')) {
        this.can_auto_align_hangle = initObj.can_auto_align_hangle
      }
      else {
        this.can_auto_align_hangle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrHeaderAlignmentState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_auto_align_hangle_qf]
    bufferOffset = _serializer.uint8(obj.can_auto_align_hangle_qf, buffer, bufferOffset);
    // Serialize message field [can_alignment_status]
    bufferOffset = _serializer.uint8(obj.can_alignment_status, buffer, bufferOffset);
    // Serialize message field [can_alignment_state]
    bufferOffset = _serializer.uint8(obj.can_alignment_state, buffer, bufferOffset);
    // Serialize message field [can_auto_align_hangle_ref]
    bufferOffset = _serializer.float32(obj.can_auto_align_hangle_ref, buffer, bufferOffset);
    // Serialize message field [can_auto_align_hangle]
    bufferOffset = _serializer.float32(obj.can_auto_align_hangle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrHeaderAlignmentState
    let len;
    let data = new MrrHeaderAlignmentState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_auto_align_hangle_qf]
    data.can_auto_align_hangle_qf = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_alignment_status]
    data.can_alignment_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_alignment_state]
    data.can_alignment_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_auto_align_hangle_ref]
    data.can_auto_align_hangle_ref = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_auto_align_hangle]
    data.can_auto_align_hangle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 11;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrHeaderAlignmentState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed76a328bc6693a98452aedfe696f11a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8   can_auto_align_hangle_qf
    uint8   can_alignment_status
    uint8   can_alignment_state
    float32 can_auto_align_hangle_ref
    float32 can_auto_align_hangle
    
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
    const resolved = new MrrHeaderAlignmentState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_auto_align_hangle_qf !== undefined) {
      resolved.can_auto_align_hangle_qf = msg.can_auto_align_hangle_qf;
    }
    else {
      resolved.can_auto_align_hangle_qf = 0
    }

    if (msg.can_alignment_status !== undefined) {
      resolved.can_alignment_status = msg.can_alignment_status;
    }
    else {
      resolved.can_alignment_status = 0
    }

    if (msg.can_alignment_state !== undefined) {
      resolved.can_alignment_state = msg.can_alignment_state;
    }
    else {
      resolved.can_alignment_state = 0
    }

    if (msg.can_auto_align_hangle_ref !== undefined) {
      resolved.can_auto_align_hangle_ref = msg.can_auto_align_hangle_ref;
    }
    else {
      resolved.can_auto_align_hangle_ref = 0.0
    }

    if (msg.can_auto_align_hangle !== undefined) {
      resolved.can_auto_align_hangle = msg.can_auto_align_hangle;
    }
    else {
      resolved.can_auto_align_hangle = 0.0
    }

    return resolved;
    }
};

module.exports = MrrHeaderAlignmentState;
