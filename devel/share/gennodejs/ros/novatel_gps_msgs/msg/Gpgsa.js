// Auto-generated. Do not edit!

// (in-package novatel_gps_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Gpgsa {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.message_id = null;
      this.auto_manual_mode = null;
      this.fix_mode = null;
      this.sv_ids = null;
      this.pdop = null;
      this.hdop = null;
      this.vdop = null;
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
      if (initObj.hasOwnProperty('auto_manual_mode')) {
        this.auto_manual_mode = initObj.auto_manual_mode
      }
      else {
        this.auto_manual_mode = '';
      }
      if (initObj.hasOwnProperty('fix_mode')) {
        this.fix_mode = initObj.fix_mode
      }
      else {
        this.fix_mode = 0;
      }
      if (initObj.hasOwnProperty('sv_ids')) {
        this.sv_ids = initObj.sv_ids
      }
      else {
        this.sv_ids = [];
      }
      if (initObj.hasOwnProperty('pdop')) {
        this.pdop = initObj.pdop
      }
      else {
        this.pdop = 0.0;
      }
      if (initObj.hasOwnProperty('hdop')) {
        this.hdop = initObj.hdop
      }
      else {
        this.hdop = 0.0;
      }
      if (initObj.hasOwnProperty('vdop')) {
        this.vdop = initObj.vdop
      }
      else {
        this.vdop = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gpgsa
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [message_id]
    bufferOffset = _serializer.string(obj.message_id, buffer, bufferOffset);
    // Serialize message field [auto_manual_mode]
    bufferOffset = _serializer.string(obj.auto_manual_mode, buffer, bufferOffset);
    // Serialize message field [fix_mode]
    bufferOffset = _serializer.uint8(obj.fix_mode, buffer, bufferOffset);
    // Serialize message field [sv_ids]
    bufferOffset = _arraySerializer.uint8(obj.sv_ids, buffer, bufferOffset, null);
    // Serialize message field [pdop]
    bufferOffset = _serializer.float32(obj.pdop, buffer, bufferOffset);
    // Serialize message field [hdop]
    bufferOffset = _serializer.float32(obj.hdop, buffer, bufferOffset);
    // Serialize message field [vdop]
    bufferOffset = _serializer.float32(obj.vdop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gpgsa
    let len;
    let data = new Gpgsa(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [message_id]
    data.message_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [auto_manual_mode]
    data.auto_manual_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fix_mode]
    data.fix_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sv_ids]
    data.sv_ids = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [pdop]
    data.pdop = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hdop]
    data.hdop = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vdop]
    data.vdop = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.message_id);
    length += _getByteLength(object.auto_manual_mode);
    length += object.sv_ids.length;
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Gpgsa';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '94a6ef4a36d322374b16097a5d03433e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message from GPGSA NMEA String
    Header header
    
    string message_id
    
    string auto_manual_mode
    uint8 fix_mode
    
    uint8[] sv_ids
    
    float32 pdop
    float32 hdop
    float32 vdop
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
    const resolved = new Gpgsa(null);
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

    if (msg.auto_manual_mode !== undefined) {
      resolved.auto_manual_mode = msg.auto_manual_mode;
    }
    else {
      resolved.auto_manual_mode = ''
    }

    if (msg.fix_mode !== undefined) {
      resolved.fix_mode = msg.fix_mode;
    }
    else {
      resolved.fix_mode = 0
    }

    if (msg.sv_ids !== undefined) {
      resolved.sv_ids = msg.sv_ids;
    }
    else {
      resolved.sv_ids = []
    }

    if (msg.pdop !== undefined) {
      resolved.pdop = msg.pdop;
    }
    else {
      resolved.pdop = 0.0
    }

    if (msg.hdop !== undefined) {
      resolved.hdop = msg.hdop;
    }
    else {
      resolved.hdop = 0.0
    }

    if (msg.vdop !== undefined) {
      resolved.vdop = msg.vdop;
    }
    else {
      resolved.vdop = 0.0
    }

    return resolved;
    }
};

module.exports = Gpgsa;
