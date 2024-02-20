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

class Ahbc {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.high_low_beam_decision = null;
      this.reasons_for_switch_to_low_beam = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('high_low_beam_decision')) {
        this.high_low_beam_decision = initObj.high_low_beam_decision
      }
      else {
        this.high_low_beam_decision = 0;
      }
      if (initObj.hasOwnProperty('reasons_for_switch_to_low_beam')) {
        this.reasons_for_switch_to_low_beam = initObj.reasons_for_switch_to_low_beam
      }
      else {
        this.reasons_for_switch_to_low_beam = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Ahbc
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [high_low_beam_decision]
    bufferOffset = _serializer.uint8(obj.high_low_beam_decision, buffer, bufferOffset);
    // Serialize message field [reasons_for_switch_to_low_beam]
    bufferOffset = _serializer.uint16(obj.reasons_for_switch_to_low_beam, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ahbc
    let len;
    let data = new Ahbc(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [high_low_beam_decision]
    data.high_low_beam_decision = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reasons_for_switch_to_low_beam]
    data.reasons_for_switch_to_low_beam = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/Ahbc';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '475e214fc14bee0ccbbfc2ae7aaea6ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION = 0
    uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF = 1
    uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON = 2
    uint8 HIGH_LOW_BEAM_DECISION_INVALID = 3
    uint8 high_low_beam_decision
    
    uint16 reasons_for_switch_to_low_beam
    
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
    const resolved = new Ahbc(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.high_low_beam_decision !== undefined) {
      resolved.high_low_beam_decision = msg.high_low_beam_decision;
    }
    else {
      resolved.high_low_beam_decision = 0
    }

    if (msg.reasons_for_switch_to_low_beam !== undefined) {
      resolved.reasons_for_switch_to_low_beam = msg.reasons_for_switch_to_low_beam;
    }
    else {
      resolved.reasons_for_switch_to_low_beam = 0
    }

    return resolved;
    }
};

// Constants for message
Ahbc.Constants = {
  HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION: 0,
  HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF: 1,
  HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON: 2,
  HIGH_LOW_BEAM_DECISION_INVALID: 3,
}

module.exports = Ahbc;
