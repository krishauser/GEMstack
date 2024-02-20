// Auto-generated. Do not edit!

// (in-package novatel_gps_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackstatChannel = require('./TrackstatChannel.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Trackstat {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.solution_status = null;
      this.position_type = null;
      this.cutoff = null;
      this.channels = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('solution_status')) {
        this.solution_status = initObj.solution_status
      }
      else {
        this.solution_status = '';
      }
      if (initObj.hasOwnProperty('position_type')) {
        this.position_type = initObj.position_type
      }
      else {
        this.position_type = '';
      }
      if (initObj.hasOwnProperty('cutoff')) {
        this.cutoff = initObj.cutoff
      }
      else {
        this.cutoff = 0.0;
      }
      if (initObj.hasOwnProperty('channels')) {
        this.channels = initObj.channels
      }
      else {
        this.channels = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Trackstat
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [solution_status]
    bufferOffset = _serializer.string(obj.solution_status, buffer, bufferOffset);
    // Serialize message field [position_type]
    bufferOffset = _serializer.string(obj.position_type, buffer, bufferOffset);
    // Serialize message field [cutoff]
    bufferOffset = _serializer.float32(obj.cutoff, buffer, bufferOffset);
    // Serialize message field [channels]
    // Serialize the length for message field [channels]
    bufferOffset = _serializer.uint32(obj.channels.length, buffer, bufferOffset);
    obj.channels.forEach((val) => {
      bufferOffset = TrackstatChannel.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Trackstat
    let len;
    let data = new Trackstat(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [solution_status]
    data.solution_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position_type]
    data.position_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cutoff]
    data.cutoff = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [channels]
    // Deserialize array length for message field [channels]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.channels = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.channels[i] = TrackstatChannel.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.solution_status);
    length += _getByteLength(object.position_type);
    object.channels.forEach((val) => {
      length += TrackstatChannel.getMessageSize(val);
    });
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Trackstat';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '10e52c1ea54daca4de3c8cdda3a79817';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Channel tracking status information for each of the receiver parallel channels
    
    Header header
    
    string solution_status
    string position_type
    
    # Tracking elevation cutff-off angle
    float32 cutoff
    
    TrackstatChannel[] channels
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
    MSG: novatel_gps_msgs/TrackstatChannel
    # A submessage of Trackstat that contains all of the data about a single hardware channel
    
    # Satellite PRN number
    int16 prn
    
    # GLONASS Frequency +7
    int16 glofreq
    
    # Channel tracking status
    uint32 ch_tr_status
    
    # Pseudorange (m)
    float64 psr
    
    # Doppler frequency (Hz)
    float32 doppler
    
    # Carrier to noise density ratio (dB-Hz)
    float32 c_no
    
    # Number of seconds of continuous tracking (no cycle slips)
    float32 locktime
    
    # Pseudorange residual from pseudorange filter (m)
    float32 psr_res
    
    # Range reject code from pseudorange filter
    string reject
    
    # Pseudorange filter weighting
    float32 psr_weight
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Trackstat(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.solution_status !== undefined) {
      resolved.solution_status = msg.solution_status;
    }
    else {
      resolved.solution_status = ''
    }

    if (msg.position_type !== undefined) {
      resolved.position_type = msg.position_type;
    }
    else {
      resolved.position_type = ''
    }

    if (msg.cutoff !== undefined) {
      resolved.cutoff = msg.cutoff;
    }
    else {
      resolved.cutoff = 0.0
    }

    if (msg.channels !== undefined) {
      resolved.channels = new Array(msg.channels.length);
      for (let i = 0; i < resolved.channels.length; ++i) {
        resolved.channels[i] = TrackstatChannel.Resolve(msg.channels[i]);
      }
    }
    else {
      resolved.channels = []
    }

    return resolved;
    }
};

module.exports = Trackstat;
