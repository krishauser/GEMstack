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

class Time {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.clock_status = null;
      this.offset = null;
      this.offset_std = null;
      this.utc_offset = null;
      this.utc_year = null;
      this.utc_month = null;
      this.utc_day = null;
      this.utc_hour = null;
      this.utc_minute = null;
      this.utc_millisecond = null;
      this.utc_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('clock_status')) {
        this.clock_status = initObj.clock_status
      }
      else {
        this.clock_status = '';
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = 0.0;
      }
      if (initObj.hasOwnProperty('offset_std')) {
        this.offset_std = initObj.offset_std
      }
      else {
        this.offset_std = 0.0;
      }
      if (initObj.hasOwnProperty('utc_offset')) {
        this.utc_offset = initObj.utc_offset
      }
      else {
        this.utc_offset = 0.0;
      }
      if (initObj.hasOwnProperty('utc_year')) {
        this.utc_year = initObj.utc_year
      }
      else {
        this.utc_year = 0;
      }
      if (initObj.hasOwnProperty('utc_month')) {
        this.utc_month = initObj.utc_month
      }
      else {
        this.utc_month = 0;
      }
      if (initObj.hasOwnProperty('utc_day')) {
        this.utc_day = initObj.utc_day
      }
      else {
        this.utc_day = 0;
      }
      if (initObj.hasOwnProperty('utc_hour')) {
        this.utc_hour = initObj.utc_hour
      }
      else {
        this.utc_hour = 0;
      }
      if (initObj.hasOwnProperty('utc_minute')) {
        this.utc_minute = initObj.utc_minute
      }
      else {
        this.utc_minute = 0;
      }
      if (initObj.hasOwnProperty('utc_millisecond')) {
        this.utc_millisecond = initObj.utc_millisecond
      }
      else {
        this.utc_millisecond = 0;
      }
      if (initObj.hasOwnProperty('utc_status')) {
        this.utc_status = initObj.utc_status
      }
      else {
        this.utc_status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Time
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [clock_status]
    bufferOffset = _serializer.string(obj.clock_status, buffer, bufferOffset);
    // Serialize message field [offset]
    bufferOffset = _serializer.float64(obj.offset, buffer, bufferOffset);
    // Serialize message field [offset_std]
    bufferOffset = _serializer.float64(obj.offset_std, buffer, bufferOffset);
    // Serialize message field [utc_offset]
    bufferOffset = _serializer.float64(obj.utc_offset, buffer, bufferOffset);
    // Serialize message field [utc_year]
    bufferOffset = _serializer.uint32(obj.utc_year, buffer, bufferOffset);
    // Serialize message field [utc_month]
    bufferOffset = _serializer.uint8(obj.utc_month, buffer, bufferOffset);
    // Serialize message field [utc_day]
    bufferOffset = _serializer.uint8(obj.utc_day, buffer, bufferOffset);
    // Serialize message field [utc_hour]
    bufferOffset = _serializer.uint8(obj.utc_hour, buffer, bufferOffset);
    // Serialize message field [utc_minute]
    bufferOffset = _serializer.uint8(obj.utc_minute, buffer, bufferOffset);
    // Serialize message field [utc_millisecond]
    bufferOffset = _serializer.uint32(obj.utc_millisecond, buffer, bufferOffset);
    // Serialize message field [utc_status]
    bufferOffset = _serializer.string(obj.utc_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Time
    let len;
    let data = new Time(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [clock_status]
    data.clock_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [offset]
    data.offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [offset_std]
    data.offset_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [utc_offset]
    data.utc_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [utc_year]
    data.utc_year = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [utc_month]
    data.utc_month = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [utc_day]
    data.utc_day = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [utc_hour]
    data.utc_hour = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [utc_minute]
    data.utc_minute = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [utc_millisecond]
    data.utc_millisecond = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [utc_status]
    data.utc_status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.clock_status);
    length += _getByteLength(object.utc_status);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Time';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '65d339585d71de8242304ff93e8a4f1a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver
    Header header
    
    # Clock model status
    # see Table 65 on pg 322 of the OEM6 Family Firmware Reference Manual, Rev3
    string clock_status
    
    # Receiver clock offset, in seconds, from GPS reference time. A positive offset
    # implies that the receiver clock is ahead of GPS reference time. To derive
    # GPS reference time, use the following formula:
    # GPS reference time = receiver time - offset
    float64 offset
    
    # Standard deviation of the offset
    float64 offset_std
    
    # The offset of the GPS reference time from UTC time, computed using almanac
    # parameters. UTC time is GPS reference time plus the current UTC offset plus
    # the receiver clock offset:
    # UTC time = GPS reference time + offset + UTC offset
    float64 utc_offset
    
    uint32 utc_year
    uint8 utc_month
    uint8 utc_day
    uint8 utc_hour
    uint8 utc_minute
    uint32 utc_millisecond
    
    string utc_status
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
    const resolved = new Time(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.clock_status !== undefined) {
      resolved.clock_status = msg.clock_status;
    }
    else {
      resolved.clock_status = ''
    }

    if (msg.offset !== undefined) {
      resolved.offset = msg.offset;
    }
    else {
      resolved.offset = 0.0
    }

    if (msg.offset_std !== undefined) {
      resolved.offset_std = msg.offset_std;
    }
    else {
      resolved.offset_std = 0.0
    }

    if (msg.utc_offset !== undefined) {
      resolved.utc_offset = msg.utc_offset;
    }
    else {
      resolved.utc_offset = 0.0
    }

    if (msg.utc_year !== undefined) {
      resolved.utc_year = msg.utc_year;
    }
    else {
      resolved.utc_year = 0
    }

    if (msg.utc_month !== undefined) {
      resolved.utc_month = msg.utc_month;
    }
    else {
      resolved.utc_month = 0
    }

    if (msg.utc_day !== undefined) {
      resolved.utc_day = msg.utc_day;
    }
    else {
      resolved.utc_day = 0
    }

    if (msg.utc_hour !== undefined) {
      resolved.utc_hour = msg.utc_hour;
    }
    else {
      resolved.utc_hour = 0
    }

    if (msg.utc_minute !== undefined) {
      resolved.utc_minute = msg.utc_minute;
    }
    else {
      resolved.utc_minute = 0
    }

    if (msg.utc_millisecond !== undefined) {
      resolved.utc_millisecond = msg.utc_millisecond;
    }
    else {
      resolved.utc_millisecond = 0
    }

    if (msg.utc_status !== undefined) {
      resolved.utc_status = msg.utc_status;
    }
    else {
      resolved.utc_status = ''
    }

    return resolved;
    }
};

module.exports = Time;
