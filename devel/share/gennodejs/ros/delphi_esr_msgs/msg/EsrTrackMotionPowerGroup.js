// Auto-generated. Do not edit!

// (in-package delphi_esr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let EsrTrackMotionPowerTrack = require('./EsrTrackMotionPowerTrack.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EsrTrackMotionPowerGroup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.rolling_count_2 = null;
      this.can_id_group = null;
      this.tracks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('canmsg')) {
        this.canmsg = initObj.canmsg
      }
      else {
        this.canmsg = '';
      }
      if (initObj.hasOwnProperty('rolling_count_2')) {
        this.rolling_count_2 = initObj.rolling_count_2
      }
      else {
        this.rolling_count_2 = 0;
      }
      if (initObj.hasOwnProperty('can_id_group')) {
        this.can_id_group = initObj.can_id_group
      }
      else {
        this.can_id_group = 0;
      }
      if (initObj.hasOwnProperty('tracks')) {
        this.tracks = initObj.tracks
      }
      else {
        this.tracks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrTrackMotionPowerGroup
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [rolling_count_2]
    bufferOffset = _serializer.uint8(obj.rolling_count_2, buffer, bufferOffset);
    // Serialize message field [can_id_group]
    bufferOffset = _serializer.uint8(obj.can_id_group, buffer, bufferOffset);
    // Serialize message field [tracks]
    // Serialize the length for message field [tracks]
    bufferOffset = _serializer.uint32(obj.tracks.length, buffer, bufferOffset);
    obj.tracks.forEach((val) => {
      bufferOffset = EsrTrackMotionPowerTrack.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrTrackMotionPowerGroup
    let len;
    let data = new EsrTrackMotionPowerGroup(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [rolling_count_2]
    data.rolling_count_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_id_group]
    data.can_id_group = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tracks]
    // Deserialize array length for message field [tracks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tracks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tracks[i] = EsrTrackMotionPowerTrack.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    length += 5 * object.tracks.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrTrackMotionPowerGroup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '58598630b679d4a2eed0f058be9b1aaa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR TrackMotionPower, information common to a group
    string                                      canmsg
    
    uint8                                       rolling_count_2
    uint8                                       can_id_group
    delphi_esr_msgs/EsrTrackMotionPowerTrack[]  tracks
    
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
    
    ================================================================================
    MSG: delphi_esr_msgs/EsrTrackMotionPowerTrack
    # ESR TrackMotionPower, track-specific information
    uint8  id
    bool   movable_fast
    bool   movable_slow
    bool   moving
    int8   power
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EsrTrackMotionPowerGroup(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.canmsg !== undefined) {
      resolved.canmsg = msg.canmsg;
    }
    else {
      resolved.canmsg = ''
    }

    if (msg.rolling_count_2 !== undefined) {
      resolved.rolling_count_2 = msg.rolling_count_2;
    }
    else {
      resolved.rolling_count_2 = 0
    }

    if (msg.can_id_group !== undefined) {
      resolved.can_id_group = msg.can_id_group;
    }
    else {
      resolved.can_id_group = 0
    }

    if (msg.tracks !== undefined) {
      resolved.tracks = new Array(msg.tracks.length);
      for (let i = 0; i < resolved.tracks.length; ++i) {
        resolved.tracks[i] = EsrTrackMotionPowerTrack.Resolve(msg.tracks[i]);
      }
    }
    else {
      resolved.tracks = []
    }

    return resolved;
    }
};

module.exports = EsrTrackMotionPowerGroup;
