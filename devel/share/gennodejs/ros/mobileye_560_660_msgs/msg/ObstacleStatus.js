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

class ObstacleStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_obstacles = null;
      this.timestamp = null;
      this.application_version = null;
      this.active_version_number_section = null;
      this.left_close_range_cut_in = null;
      this.right_close_range_cut_in = null;
      this.stop_go = null;
      this.protocol_version = null;
      this.close_car = null;
      this.failsafe = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_obstacles')) {
        this.num_obstacles = initObj.num_obstacles
      }
      else {
        this.num_obstacles = 0;
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0;
      }
      if (initObj.hasOwnProperty('application_version')) {
        this.application_version = initObj.application_version
      }
      else {
        this.application_version = 0;
      }
      if (initObj.hasOwnProperty('active_version_number_section')) {
        this.active_version_number_section = initObj.active_version_number_section
      }
      else {
        this.active_version_number_section = 0;
      }
      if (initObj.hasOwnProperty('left_close_range_cut_in')) {
        this.left_close_range_cut_in = initObj.left_close_range_cut_in
      }
      else {
        this.left_close_range_cut_in = false;
      }
      if (initObj.hasOwnProperty('right_close_range_cut_in')) {
        this.right_close_range_cut_in = initObj.right_close_range_cut_in
      }
      else {
        this.right_close_range_cut_in = false;
      }
      if (initObj.hasOwnProperty('stop_go')) {
        this.stop_go = initObj.stop_go
      }
      else {
        this.stop_go = 0;
      }
      if (initObj.hasOwnProperty('protocol_version')) {
        this.protocol_version = initObj.protocol_version
      }
      else {
        this.protocol_version = 0;
      }
      if (initObj.hasOwnProperty('close_car')) {
        this.close_car = initObj.close_car
      }
      else {
        this.close_car = false;
      }
      if (initObj.hasOwnProperty('failsafe')) {
        this.failsafe = initObj.failsafe
      }
      else {
        this.failsafe = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_obstacles]
    bufferOffset = _serializer.uint16(obj.num_obstacles, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.uint16(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [application_version]
    bufferOffset = _serializer.uint16(obj.application_version, buffer, bufferOffset);
    // Serialize message field [active_version_number_section]
    bufferOffset = _serializer.uint16(obj.active_version_number_section, buffer, bufferOffset);
    // Serialize message field [left_close_range_cut_in]
    bufferOffset = _serializer.bool(obj.left_close_range_cut_in, buffer, bufferOffset);
    // Serialize message field [right_close_range_cut_in]
    bufferOffset = _serializer.bool(obj.right_close_range_cut_in, buffer, bufferOffset);
    // Serialize message field [stop_go]
    bufferOffset = _serializer.uint8(obj.stop_go, buffer, bufferOffset);
    // Serialize message field [protocol_version]
    bufferOffset = _serializer.uint8(obj.protocol_version, buffer, bufferOffset);
    // Serialize message field [close_car]
    bufferOffset = _serializer.bool(obj.close_car, buffer, bufferOffset);
    // Serialize message field [failsafe]
    bufferOffset = _serializer.uint8(obj.failsafe, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleStatus
    let len;
    let data = new ObstacleStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_obstacles]
    data.num_obstacles = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [application_version]
    data.application_version = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [active_version_number_section]
    data.active_version_number_section = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [left_close_range_cut_in]
    data.left_close_range_cut_in = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_close_range_cut_in]
    data.right_close_range_cut_in = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stop_go]
    data.stop_go = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [protocol_version]
    data.protocol_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [close_car]
    data.close_car = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [failsafe]
    data.failsafe = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/ObstacleStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b963ecf49d557c90935e49005018b9ff';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint16 num_obstacles
    uint16 timestamp
    uint16 application_version
    uint16 active_version_number_section
    bool left_close_range_cut_in
    bool right_close_range_cut_in
    
    uint8 STOP_GO_STOP = 0
    uint8 STOP_GO_GO = 1
    uint8 STOP_GO_UNDECIDED = 2
    uint8 STOP_GO_DRIVER_DECISION_REQUIRED = 3
    uint8 STOP_GO_NOT_CALCULATED = 15
    uint8 stop_go
    
    uint8 protocol_version
    bool close_car
    
    uint8 FAILSAFE_NONE = 0
    uint8 FAILSAFE_LOW_SUN = 1
    uint8 FAILSAFE_BLUR_IMAGE = 2
    uint8 failsafe
    
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
    const resolved = new ObstacleStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_obstacles !== undefined) {
      resolved.num_obstacles = msg.num_obstacles;
    }
    else {
      resolved.num_obstacles = 0
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0
    }

    if (msg.application_version !== undefined) {
      resolved.application_version = msg.application_version;
    }
    else {
      resolved.application_version = 0
    }

    if (msg.active_version_number_section !== undefined) {
      resolved.active_version_number_section = msg.active_version_number_section;
    }
    else {
      resolved.active_version_number_section = 0
    }

    if (msg.left_close_range_cut_in !== undefined) {
      resolved.left_close_range_cut_in = msg.left_close_range_cut_in;
    }
    else {
      resolved.left_close_range_cut_in = false
    }

    if (msg.right_close_range_cut_in !== undefined) {
      resolved.right_close_range_cut_in = msg.right_close_range_cut_in;
    }
    else {
      resolved.right_close_range_cut_in = false
    }

    if (msg.stop_go !== undefined) {
      resolved.stop_go = msg.stop_go;
    }
    else {
      resolved.stop_go = 0
    }

    if (msg.protocol_version !== undefined) {
      resolved.protocol_version = msg.protocol_version;
    }
    else {
      resolved.protocol_version = 0
    }

    if (msg.close_car !== undefined) {
      resolved.close_car = msg.close_car;
    }
    else {
      resolved.close_car = false
    }

    if (msg.failsafe !== undefined) {
      resolved.failsafe = msg.failsafe;
    }
    else {
      resolved.failsafe = 0
    }

    return resolved;
    }
};

// Constants for message
ObstacleStatus.Constants = {
  STOP_GO_STOP: 0,
  STOP_GO_GO: 1,
  STOP_GO_UNDECIDED: 2,
  STOP_GO_DRIVER_DECISION_REQUIRED: 3,
  STOP_GO_NOT_CALCULATED: 15,
  FAILSAFE_NONE: 0,
  FAILSAFE_LOW_SUN: 1,
  FAILSAFE_BLUR_IMAGE: 2,
}

module.exports = ObstacleStatus;
