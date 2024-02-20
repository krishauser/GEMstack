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

class MrrHeaderInformationDetections {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_align_updates_done = null;
      this.can_scan_index = null;
      this.can_number_of_det = null;
      this.can_look_id = null;
      this.can_look_index = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_align_updates_done')) {
        this.can_align_updates_done = initObj.can_align_updates_done
      }
      else {
        this.can_align_updates_done = 0;
      }
      if (initObj.hasOwnProperty('can_scan_index')) {
        this.can_scan_index = initObj.can_scan_index
      }
      else {
        this.can_scan_index = 0;
      }
      if (initObj.hasOwnProperty('can_number_of_det')) {
        this.can_number_of_det = initObj.can_number_of_det
      }
      else {
        this.can_number_of_det = 0;
      }
      if (initObj.hasOwnProperty('can_look_id')) {
        this.can_look_id = initObj.can_look_id
      }
      else {
        this.can_look_id = 0;
      }
      if (initObj.hasOwnProperty('can_look_index')) {
        this.can_look_index = initObj.can_look_index
      }
      else {
        this.can_look_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrHeaderInformationDetections
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_align_updates_done]
    bufferOffset = _serializer.uint16(obj.can_align_updates_done, buffer, bufferOffset);
    // Serialize message field [can_scan_index]
    bufferOffset = _serializer.uint16(obj.can_scan_index, buffer, bufferOffset);
    // Serialize message field [can_number_of_det]
    bufferOffset = _serializer.uint8(obj.can_number_of_det, buffer, bufferOffset);
    // Serialize message field [can_look_id]
    bufferOffset = _serializer.uint8(obj.can_look_id, buffer, bufferOffset);
    // Serialize message field [can_look_index]
    bufferOffset = _serializer.uint16(obj.can_look_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrHeaderInformationDetections
    let len;
    let data = new MrrHeaderInformationDetections(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_align_updates_done]
    data.can_align_updates_done = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_scan_index]
    data.can_scan_index = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_number_of_det]
    data.can_number_of_det = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_look_id]
    data.can_look_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_look_index]
    data.can_look_index = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrHeaderInformationDetections';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39494a4731101be0be38c8ac22c1c084';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint16 can_align_updates_done
    uint16 can_scan_index
    uint8  can_number_of_det
    uint8  can_look_id
    uint16 can_look_index
    
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
    const resolved = new MrrHeaderInformationDetections(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_align_updates_done !== undefined) {
      resolved.can_align_updates_done = msg.can_align_updates_done;
    }
    else {
      resolved.can_align_updates_done = 0
    }

    if (msg.can_scan_index !== undefined) {
      resolved.can_scan_index = msg.can_scan_index;
    }
    else {
      resolved.can_scan_index = 0
    }

    if (msg.can_number_of_det !== undefined) {
      resolved.can_number_of_det = msg.can_number_of_det;
    }
    else {
      resolved.can_number_of_det = 0
    }

    if (msg.can_look_id !== undefined) {
      resolved.can_look_id = msg.can_look_id;
    }
    else {
      resolved.can_look_id = 0
    }

    if (msg.can_look_index !== undefined) {
      resolved.can_look_index = msg.can_look_index;
    }
    else {
      resolved.can_look_index = 0
    }

    return resolved;
    }
};

module.exports = MrrHeaderInformationDetections;
