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

class AftermarketLane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lane_confidence_left = null;
      this.ldw_available_left = null;
      this.lane_type_left = null;
      this.distance_to_left_lane = null;
      this.lane_confidence_right = null;
      this.ldw_available_right = null;
      this.lane_type_right = null;
      this.distance_to_right_lane = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lane_confidence_left')) {
        this.lane_confidence_left = initObj.lane_confidence_left
      }
      else {
        this.lane_confidence_left = 0;
      }
      if (initObj.hasOwnProperty('ldw_available_left')) {
        this.ldw_available_left = initObj.ldw_available_left
      }
      else {
        this.ldw_available_left = false;
      }
      if (initObj.hasOwnProperty('lane_type_left')) {
        this.lane_type_left = initObj.lane_type_left
      }
      else {
        this.lane_type_left = 0;
      }
      if (initObj.hasOwnProperty('distance_to_left_lane')) {
        this.distance_to_left_lane = initObj.distance_to_left_lane
      }
      else {
        this.distance_to_left_lane = 0.0;
      }
      if (initObj.hasOwnProperty('lane_confidence_right')) {
        this.lane_confidence_right = initObj.lane_confidence_right
      }
      else {
        this.lane_confidence_right = 0;
      }
      if (initObj.hasOwnProperty('ldw_available_right')) {
        this.ldw_available_right = initObj.ldw_available_right
      }
      else {
        this.ldw_available_right = false;
      }
      if (initObj.hasOwnProperty('lane_type_right')) {
        this.lane_type_right = initObj.lane_type_right
      }
      else {
        this.lane_type_right = 0;
      }
      if (initObj.hasOwnProperty('distance_to_right_lane')) {
        this.distance_to_right_lane = initObj.distance_to_right_lane
      }
      else {
        this.distance_to_right_lane = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AftermarketLane
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lane_confidence_left]
    bufferOffset = _serializer.uint8(obj.lane_confidence_left, buffer, bufferOffset);
    // Serialize message field [ldw_available_left]
    bufferOffset = _serializer.bool(obj.ldw_available_left, buffer, bufferOffset);
    // Serialize message field [lane_type_left]
    bufferOffset = _serializer.uint8(obj.lane_type_left, buffer, bufferOffset);
    // Serialize message field [distance_to_left_lane]
    bufferOffset = _serializer.float32(obj.distance_to_left_lane, buffer, bufferOffset);
    // Serialize message field [lane_confidence_right]
    bufferOffset = _serializer.uint8(obj.lane_confidence_right, buffer, bufferOffset);
    // Serialize message field [ldw_available_right]
    bufferOffset = _serializer.bool(obj.ldw_available_right, buffer, bufferOffset);
    // Serialize message field [lane_type_right]
    bufferOffset = _serializer.uint8(obj.lane_type_right, buffer, bufferOffset);
    // Serialize message field [distance_to_right_lane]
    bufferOffset = _serializer.float32(obj.distance_to_right_lane, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AftermarketLane
    let len;
    let data = new AftermarketLane(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_confidence_left]
    data.lane_confidence_left = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ldw_available_left]
    data.ldw_available_left = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [lane_type_left]
    data.lane_type_left = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [distance_to_left_lane]
    data.distance_to_left_lane = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lane_confidence_right]
    data.lane_confidence_right = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ldw_available_right]
    data.ldw_available_right = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [lane_type_right]
    data.lane_type_right = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [distance_to_right_lane]
    data.distance_to_right_lane = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/AftermarketLane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8a56b7a5f0247252831a59dfc0910af7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 LANE_CONFIDENCE_NONE = 0
    uint8 LANE_CONFIDENCE_LOW = 1
    uint8 LANE_CONFIDENCE_MED = 2
    uint8 LANE_CONFIDENCE_HIGH = 3
    
    uint8 LANE_TYPE_DASHED = 0
    uint8 LANE_TYPE_SOLID = 1
    uint8 LANE_TYPE_NONE = 2
    uint8 LANE_TYPE_ROAD_EDGE = 3
    uint8 LANE_TYPE_DOUBLE_LANE_MARK = 4
    uint8 LANE_TYPE_BOTTS_DOTS = 5
    uint8 LANE_TYPE_INVALID = 6
    
    uint8 lane_confidence_left
    bool ldw_available_left
    uint8 lane_type_left
    float32 distance_to_left_lane
    uint8 lane_confidence_right
    bool ldw_available_right
    uint8 lane_type_right
    float32 distance_to_right_lane
    
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
    const resolved = new AftermarketLane(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lane_confidence_left !== undefined) {
      resolved.lane_confidence_left = msg.lane_confidence_left;
    }
    else {
      resolved.lane_confidence_left = 0
    }

    if (msg.ldw_available_left !== undefined) {
      resolved.ldw_available_left = msg.ldw_available_left;
    }
    else {
      resolved.ldw_available_left = false
    }

    if (msg.lane_type_left !== undefined) {
      resolved.lane_type_left = msg.lane_type_left;
    }
    else {
      resolved.lane_type_left = 0
    }

    if (msg.distance_to_left_lane !== undefined) {
      resolved.distance_to_left_lane = msg.distance_to_left_lane;
    }
    else {
      resolved.distance_to_left_lane = 0.0
    }

    if (msg.lane_confidence_right !== undefined) {
      resolved.lane_confidence_right = msg.lane_confidence_right;
    }
    else {
      resolved.lane_confidence_right = 0
    }

    if (msg.ldw_available_right !== undefined) {
      resolved.ldw_available_right = msg.ldw_available_right;
    }
    else {
      resolved.ldw_available_right = false
    }

    if (msg.lane_type_right !== undefined) {
      resolved.lane_type_right = msg.lane_type_right;
    }
    else {
      resolved.lane_type_right = 0
    }

    if (msg.distance_to_right_lane !== undefined) {
      resolved.distance_to_right_lane = msg.distance_to_right_lane;
    }
    else {
      resolved.distance_to_right_lane = 0.0
    }

    return resolved;
    }
};

// Constants for message
AftermarketLane.Constants = {
  LANE_CONFIDENCE_NONE: 0,
  LANE_CONFIDENCE_LOW: 1,
  LANE_CONFIDENCE_MED: 2,
  LANE_CONFIDENCE_HIGH: 3,
  LANE_TYPE_DASHED: 0,
  LANE_TYPE_SOLID: 1,
  LANE_TYPE_NONE: 2,
  LANE_TYPE_ROAD_EDGE: 3,
  LANE_TYPE_DOUBLE_LANE_MARK: 4,
  LANE_TYPE_BOTTS_DOTS: 5,
  LANE_TYPE_INVALID: 6,
}

module.exports = AftermarketLane;
