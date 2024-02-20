// Auto-generated. Do not edit!

// (in-package novatel_gps_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Satellite = require('./Satellite.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Gpgsv {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.message_id = null;
      this.n_msgs = null;
      this.msg_number = null;
      this.n_satellites = null;
      this.satellites = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('message_id')) {
        this.message_id = initObj.message_id
      }
      else {
        this.message_id = '';
      }
      if (initObj.hasOwnProperty('n_msgs')) {
        this.n_msgs = initObj.n_msgs
      }
      else {
        this.n_msgs = 0;
      }
      if (initObj.hasOwnProperty('msg_number')) {
        this.msg_number = initObj.msg_number
      }
      else {
        this.msg_number = 0;
      }
      if (initObj.hasOwnProperty('n_satellites')) {
        this.n_satellites = initObj.n_satellites
      }
      else {
        this.n_satellites = 0;
      }
      if (initObj.hasOwnProperty('satellites')) {
        this.satellites = initObj.satellites
      }
      else {
        this.satellites = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gpgsv
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [message_id]
    bufferOffset = _serializer.string(obj.message_id, buffer, bufferOffset);
    // Serialize message field [n_msgs]
    bufferOffset = _serializer.uint8(obj.n_msgs, buffer, bufferOffset);
    // Serialize message field [msg_number]
    bufferOffset = _serializer.uint8(obj.msg_number, buffer, bufferOffset);
    // Serialize message field [n_satellites]
    bufferOffset = _serializer.uint8(obj.n_satellites, buffer, bufferOffset);
    // Serialize message field [satellites]
    // Serialize the length for message field [satellites]
    bufferOffset = _serializer.uint32(obj.satellites.length, buffer, bufferOffset);
    obj.satellites.forEach((val) => {
      bufferOffset = Satellite.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gpgsv
    let len;
    let data = new Gpgsv(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [message_id]
    data.message_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [n_msgs]
    data.n_msgs = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [msg_number]
    data.msg_number = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [n_satellites]
    data.n_satellites = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [satellites]
    // Deserialize array length for message field [satellites]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.satellites = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.satellites[i] = Satellite.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.message_id);
    length += 5 * object.satellites.length;
    return length + 11;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Gpgsv';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f34bebc32fe085313c942a96fd39c77';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Total number of satellites in view and data about satellites
    # Because the NMEA sentence is limited to 4 satellites per message, several
    # of these messages may need to be synthesized to get data about all visible
    # satellites.
    
    Header header
    
    string message_id
    
    # Number of messages in this sequence
    uint8 n_msgs
    # This messages number in its sequence. The first message is number 1.
    uint8 msg_number
    
    # Number of satellites currently visible
    uint8 n_satellites
    
    # Up to 4 satellites are described in each message
    Satellite[] satellites
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
    MSG: novatel_gps_msgs/Satellite
    # Satellite data structure used in GPGSV messages
    
    # PRN number of the satellite
    # GPS = 1..32
    # SBAS = 33..64
    # GLO = 65..96
    uint8 prn
    
    # Elevation, degrees. Maximum 90
    uint8 elevation
    
    # Azimuth, True North degrees. [0, 359]
    uint16 azimuth
    
    # Signal to noise ratio, 0-99 dB. -1 when null in NMEA sentence (not tracking)
    int8 snr
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Gpgsv(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.message_id !== undefined) {
      resolved.message_id = msg.message_id;
    }
    else {
      resolved.message_id = ''
    }

    if (msg.n_msgs !== undefined) {
      resolved.n_msgs = msg.n_msgs;
    }
    else {
      resolved.n_msgs = 0
    }

    if (msg.msg_number !== undefined) {
      resolved.msg_number = msg.msg_number;
    }
    else {
      resolved.msg_number = 0
    }

    if (msg.n_satellites !== undefined) {
      resolved.n_satellites = msg.n_satellites;
    }
    else {
      resolved.n_satellites = 0
    }

    if (msg.satellites !== undefined) {
      resolved.satellites = new Array(msg.satellites.length);
      for (let i = 0; i < resolved.satellites.length; ++i) {
        resolved.satellites[i] = Satellite.Resolve(msg.satellites[i]);
      }
    }
    else {
      resolved.satellites = []
    }

    return resolved;
    }
};

module.exports = Gpgsv;
