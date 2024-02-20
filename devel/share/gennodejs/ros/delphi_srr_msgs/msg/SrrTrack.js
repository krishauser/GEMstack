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

class SrrTrack {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_tx_detect_valid_level = null;
      this.can_tx_detect_status = null;
      this.can_tx_detect_range_rate = null;
      this.can_tx_detect_range = null;
      this.can_tx_detect_angle = null;
      this.can_tx_detect_amplitude = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_tx_detect_valid_level')) {
        this.can_tx_detect_valid_level = initObj.can_tx_detect_valid_level
      }
      else {
        this.can_tx_detect_valid_level = 0;
      }
      if (initObj.hasOwnProperty('can_tx_detect_status')) {
        this.can_tx_detect_status = initObj.can_tx_detect_status
      }
      else {
        this.can_tx_detect_status = false;
      }
      if (initObj.hasOwnProperty('can_tx_detect_range_rate')) {
        this.can_tx_detect_range_rate = initObj.can_tx_detect_range_rate
      }
      else {
        this.can_tx_detect_range_rate = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_detect_range')) {
        this.can_tx_detect_range = initObj.can_tx_detect_range
      }
      else {
        this.can_tx_detect_range = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_detect_angle')) {
        this.can_tx_detect_angle = initObj.can_tx_detect_angle
      }
      else {
        this.can_tx_detect_angle = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_detect_amplitude')) {
        this.can_tx_detect_amplitude = initObj.can_tx_detect_amplitude
      }
      else {
        this.can_tx_detect_amplitude = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrTrack
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_tx_detect_valid_level]
    bufferOffset = _serializer.uint8(obj.can_tx_detect_valid_level, buffer, bufferOffset);
    // Serialize message field [can_tx_detect_status]
    bufferOffset = _serializer.bool(obj.can_tx_detect_status, buffer, bufferOffset);
    // Serialize message field [can_tx_detect_range_rate]
    bufferOffset = _serializer.float32(obj.can_tx_detect_range_rate, buffer, bufferOffset);
    // Serialize message field [can_tx_detect_range]
    bufferOffset = _serializer.float32(obj.can_tx_detect_range, buffer, bufferOffset);
    // Serialize message field [can_tx_detect_angle]
    bufferOffset = _serializer.float32(obj.can_tx_detect_angle, buffer, bufferOffset);
    // Serialize message field [can_tx_detect_amplitude]
    bufferOffset = _serializer.float32(obj.can_tx_detect_amplitude, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrTrack
    let len;
    let data = new SrrTrack(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_tx_detect_valid_level]
    data.can_tx_detect_valid_level = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_detect_status]
    data.can_tx_detect_status = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_detect_range_rate]
    data.can_tx_detect_range_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_detect_range]
    data.can_tx_detect_range = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_detect_angle]
    data.can_tx_detect_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_detect_amplitude]
    data.can_tx_detect_amplitude = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrTrack';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a689930ba3ce2066d655a7425f6fdbde';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_track
    
    std_msgs/Header header
    
    uint8     can_tx_detect_valid_level
    uint8     CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION=0
    uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_1=1
    uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_2=2
    uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_3=3
    uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_4=4
    uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_5=5
    uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_6=6
    uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_7=7
    
    bool      can_tx_detect_status
    bool      CAN_TX_DETECT_STATUS_NO_DATA=0
    bool      CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT=1
    
    float32   can_tx_detect_range_rate                 # m/s
    float32   can_tx_detect_range                      # m
    float32   can_tx_detect_angle                      # deg
    float32   can_tx_detect_amplitude                  # dbsm
    
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
    const resolved = new SrrTrack(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_tx_detect_valid_level !== undefined) {
      resolved.can_tx_detect_valid_level = msg.can_tx_detect_valid_level;
    }
    else {
      resolved.can_tx_detect_valid_level = 0
    }

    if (msg.can_tx_detect_status !== undefined) {
      resolved.can_tx_detect_status = msg.can_tx_detect_status;
    }
    else {
      resolved.can_tx_detect_status = false
    }

    if (msg.can_tx_detect_range_rate !== undefined) {
      resolved.can_tx_detect_range_rate = msg.can_tx_detect_range_rate;
    }
    else {
      resolved.can_tx_detect_range_rate = 0.0
    }

    if (msg.can_tx_detect_range !== undefined) {
      resolved.can_tx_detect_range = msg.can_tx_detect_range;
    }
    else {
      resolved.can_tx_detect_range = 0.0
    }

    if (msg.can_tx_detect_angle !== undefined) {
      resolved.can_tx_detect_angle = msg.can_tx_detect_angle;
    }
    else {
      resolved.can_tx_detect_angle = 0.0
    }

    if (msg.can_tx_detect_amplitude !== undefined) {
      resolved.can_tx_detect_amplitude = msg.can_tx_detect_amplitude;
    }
    else {
      resolved.can_tx_detect_amplitude = 0.0
    }

    return resolved;
    }
};

// Constants for message
SrrTrack.Constants = {
  CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION: 0,
  CAN_TX_DETECT_VALID_LEVEL_LEVEL_1: 1,
  CAN_TX_DETECT_VALID_LEVEL_LEVEL_2: 2,
  CAN_TX_DETECT_VALID_LEVEL_LEVEL_3: 3,
  CAN_TX_DETECT_VALID_LEVEL_LEVEL_4: 4,
  CAN_TX_DETECT_VALID_LEVEL_LEVEL_5: 5,
  CAN_TX_DETECT_VALID_LEVEL_LEVEL_6: 6,
  CAN_TX_DETECT_VALID_LEVEL_LEVEL_7: 7,
  CAN_TX_DETECT_STATUS_NO_DATA: false,
  CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT: true,
}

module.exports = SrrTrack;
