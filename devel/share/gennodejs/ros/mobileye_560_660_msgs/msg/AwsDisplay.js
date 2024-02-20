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

class AwsDisplay {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.suppress_sound = null;
      this.night_time = null;
      this.dusk_time = null;
      this.sound_type = null;
      this.headway_valid = null;
      this.headway_measurement = null;
      this.lanes_on = null;
      this.left_ldw_on = null;
      this.right_ldw_on = null;
      this.fcw_on = null;
      this.left_crossing = null;
      this.right_crossing = null;
      this.maintenance = null;
      this.failsafe = null;
      this.ped_fcw = null;
      this.ped_in_dz = null;
      this.headway_warning_level = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('suppress_sound')) {
        this.suppress_sound = initObj.suppress_sound
      }
      else {
        this.suppress_sound = false;
      }
      if (initObj.hasOwnProperty('night_time')) {
        this.night_time = initObj.night_time
      }
      else {
        this.night_time = false;
      }
      if (initObj.hasOwnProperty('dusk_time')) {
        this.dusk_time = initObj.dusk_time
      }
      else {
        this.dusk_time = false;
      }
      if (initObj.hasOwnProperty('sound_type')) {
        this.sound_type = initObj.sound_type
      }
      else {
        this.sound_type = 0;
      }
      if (initObj.hasOwnProperty('headway_valid')) {
        this.headway_valid = initObj.headway_valid
      }
      else {
        this.headway_valid = false;
      }
      if (initObj.hasOwnProperty('headway_measurement')) {
        this.headway_measurement = initObj.headway_measurement
      }
      else {
        this.headway_measurement = 0.0;
      }
      if (initObj.hasOwnProperty('lanes_on')) {
        this.lanes_on = initObj.lanes_on
      }
      else {
        this.lanes_on = false;
      }
      if (initObj.hasOwnProperty('left_ldw_on')) {
        this.left_ldw_on = initObj.left_ldw_on
      }
      else {
        this.left_ldw_on = false;
      }
      if (initObj.hasOwnProperty('right_ldw_on')) {
        this.right_ldw_on = initObj.right_ldw_on
      }
      else {
        this.right_ldw_on = false;
      }
      if (initObj.hasOwnProperty('fcw_on')) {
        this.fcw_on = initObj.fcw_on
      }
      else {
        this.fcw_on = false;
      }
      if (initObj.hasOwnProperty('left_crossing')) {
        this.left_crossing = initObj.left_crossing
      }
      else {
        this.left_crossing = false;
      }
      if (initObj.hasOwnProperty('right_crossing')) {
        this.right_crossing = initObj.right_crossing
      }
      else {
        this.right_crossing = false;
      }
      if (initObj.hasOwnProperty('maintenance')) {
        this.maintenance = initObj.maintenance
      }
      else {
        this.maintenance = false;
      }
      if (initObj.hasOwnProperty('failsafe')) {
        this.failsafe = initObj.failsafe
      }
      else {
        this.failsafe = false;
      }
      if (initObj.hasOwnProperty('ped_fcw')) {
        this.ped_fcw = initObj.ped_fcw
      }
      else {
        this.ped_fcw = false;
      }
      if (initObj.hasOwnProperty('ped_in_dz')) {
        this.ped_in_dz = initObj.ped_in_dz
      }
      else {
        this.ped_in_dz = false;
      }
      if (initObj.hasOwnProperty('headway_warning_level')) {
        this.headway_warning_level = initObj.headway_warning_level
      }
      else {
        this.headway_warning_level = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AwsDisplay
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [suppress_sound]
    bufferOffset = _serializer.bool(obj.suppress_sound, buffer, bufferOffset);
    // Serialize message field [night_time]
    bufferOffset = _serializer.bool(obj.night_time, buffer, bufferOffset);
    // Serialize message field [dusk_time]
    bufferOffset = _serializer.bool(obj.dusk_time, buffer, bufferOffset);
    // Serialize message field [sound_type]
    bufferOffset = _serializer.uint8(obj.sound_type, buffer, bufferOffset);
    // Serialize message field [headway_valid]
    bufferOffset = _serializer.bool(obj.headway_valid, buffer, bufferOffset);
    // Serialize message field [headway_measurement]
    bufferOffset = _serializer.float32(obj.headway_measurement, buffer, bufferOffset);
    // Serialize message field [lanes_on]
    bufferOffset = _serializer.bool(obj.lanes_on, buffer, bufferOffset);
    // Serialize message field [left_ldw_on]
    bufferOffset = _serializer.bool(obj.left_ldw_on, buffer, bufferOffset);
    // Serialize message field [right_ldw_on]
    bufferOffset = _serializer.bool(obj.right_ldw_on, buffer, bufferOffset);
    // Serialize message field [fcw_on]
    bufferOffset = _serializer.bool(obj.fcw_on, buffer, bufferOffset);
    // Serialize message field [left_crossing]
    bufferOffset = _serializer.bool(obj.left_crossing, buffer, bufferOffset);
    // Serialize message field [right_crossing]
    bufferOffset = _serializer.bool(obj.right_crossing, buffer, bufferOffset);
    // Serialize message field [maintenance]
    bufferOffset = _serializer.bool(obj.maintenance, buffer, bufferOffset);
    // Serialize message field [failsafe]
    bufferOffset = _serializer.bool(obj.failsafe, buffer, bufferOffset);
    // Serialize message field [ped_fcw]
    bufferOffset = _serializer.bool(obj.ped_fcw, buffer, bufferOffset);
    // Serialize message field [ped_in_dz]
    bufferOffset = _serializer.bool(obj.ped_in_dz, buffer, bufferOffset);
    // Serialize message field [headway_warning_level]
    bufferOffset = _serializer.uint8(obj.headway_warning_level, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AwsDisplay
    let len;
    let data = new AwsDisplay(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [suppress_sound]
    data.suppress_sound = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [night_time]
    data.night_time = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dusk_time]
    data.dusk_time = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [sound_type]
    data.sound_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [headway_valid]
    data.headway_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [headway_measurement]
    data.headway_measurement = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lanes_on]
    data.lanes_on = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_ldw_on]
    data.left_ldw_on = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_ldw_on]
    data.right_ldw_on = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fcw_on]
    data.fcw_on = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_crossing]
    data.left_crossing = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_crossing]
    data.right_crossing = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [maintenance]
    data.maintenance = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [failsafe]
    data.failsafe = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ped_fcw]
    data.ped_fcw = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ped_in_dz]
    data.ped_in_dz = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [headway_warning_level]
    data.headway_warning_level = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/AwsDisplay';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7aa82a0063aa4c0e719bef3d14c24bf7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool suppress_sound
    bool night_time
    bool dusk_time
    
    uint8 SOUND_SILENT = 0
    uint8 SOUND_LDWL = 1
    uint8 SOUND_LDWR = 2
    uint8 SOUND_FAR_HW = 3
    uint8 SOUND_NEAR_HW = 4
    uint8 SOUND_SOFT_FCW = 5
    uint8 SOUND_HARD_FCW = 6
    uint8 SOUND_RESERVED = 7
    uint8 sound_type
    
    bool headway_valid
    float32 headway_measurement
    bool lanes_on
    bool left_ldw_on
    bool right_ldw_on
    bool fcw_on
    bool left_crossing
    bool right_crossing
    bool maintenance
    bool failsafe
    bool ped_fcw
    bool ped_in_dz
    
    uint8 HEADWAY_LEVEL_OFF = 0
    uint8 HEADWAY_LEVEL_GREEN = 1
    uint8 HEADWAY_LEVEL_ORANGE = 2
    uint8 HEADWAY_LEVEL_RED = 3
    uint8 headway_warning_level
    
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
    const resolved = new AwsDisplay(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.suppress_sound !== undefined) {
      resolved.suppress_sound = msg.suppress_sound;
    }
    else {
      resolved.suppress_sound = false
    }

    if (msg.night_time !== undefined) {
      resolved.night_time = msg.night_time;
    }
    else {
      resolved.night_time = false
    }

    if (msg.dusk_time !== undefined) {
      resolved.dusk_time = msg.dusk_time;
    }
    else {
      resolved.dusk_time = false
    }

    if (msg.sound_type !== undefined) {
      resolved.sound_type = msg.sound_type;
    }
    else {
      resolved.sound_type = 0
    }

    if (msg.headway_valid !== undefined) {
      resolved.headway_valid = msg.headway_valid;
    }
    else {
      resolved.headway_valid = false
    }

    if (msg.headway_measurement !== undefined) {
      resolved.headway_measurement = msg.headway_measurement;
    }
    else {
      resolved.headway_measurement = 0.0
    }

    if (msg.lanes_on !== undefined) {
      resolved.lanes_on = msg.lanes_on;
    }
    else {
      resolved.lanes_on = false
    }

    if (msg.left_ldw_on !== undefined) {
      resolved.left_ldw_on = msg.left_ldw_on;
    }
    else {
      resolved.left_ldw_on = false
    }

    if (msg.right_ldw_on !== undefined) {
      resolved.right_ldw_on = msg.right_ldw_on;
    }
    else {
      resolved.right_ldw_on = false
    }

    if (msg.fcw_on !== undefined) {
      resolved.fcw_on = msg.fcw_on;
    }
    else {
      resolved.fcw_on = false
    }

    if (msg.left_crossing !== undefined) {
      resolved.left_crossing = msg.left_crossing;
    }
    else {
      resolved.left_crossing = false
    }

    if (msg.right_crossing !== undefined) {
      resolved.right_crossing = msg.right_crossing;
    }
    else {
      resolved.right_crossing = false
    }

    if (msg.maintenance !== undefined) {
      resolved.maintenance = msg.maintenance;
    }
    else {
      resolved.maintenance = false
    }

    if (msg.failsafe !== undefined) {
      resolved.failsafe = msg.failsafe;
    }
    else {
      resolved.failsafe = false
    }

    if (msg.ped_fcw !== undefined) {
      resolved.ped_fcw = msg.ped_fcw;
    }
    else {
      resolved.ped_fcw = false
    }

    if (msg.ped_in_dz !== undefined) {
      resolved.ped_in_dz = msg.ped_in_dz;
    }
    else {
      resolved.ped_in_dz = false
    }

    if (msg.headway_warning_level !== undefined) {
      resolved.headway_warning_level = msg.headway_warning_level;
    }
    else {
      resolved.headway_warning_level = 0
    }

    return resolved;
    }
};

// Constants for message
AwsDisplay.Constants = {
  SOUND_SILENT: 0,
  SOUND_LDWL: 1,
  SOUND_LDWR: 2,
  SOUND_FAR_HW: 3,
  SOUND_NEAR_HW: 4,
  SOUND_SOFT_FCW: 5,
  SOUND_HARD_FCW: 6,
  SOUND_RESERVED: 7,
  HEADWAY_LEVEL_OFF: 0,
  HEADWAY_LEVEL_GREEN: 1,
  HEADWAY_LEVEL_ORANGE: 2,
  HEADWAY_LEVEL_RED: 3,
}

module.exports = AwsDisplay;
