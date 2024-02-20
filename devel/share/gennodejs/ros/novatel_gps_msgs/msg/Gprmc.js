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

class Gprmc {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.message_id = null;
      this.utc_seconds = null;
      this.position_status = null;
      this.lat = null;
      this.lon = null;
      this.lat_dir = null;
      this.lon_dir = null;
      this.speed = null;
      this.track = null;
      this.date = null;
      this.mag_var = null;
      this.mag_var_direction = null;
      this.mode_indicator = null;
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
      if (initObj.hasOwnProperty('utc_seconds')) {
        this.utc_seconds = initObj.utc_seconds
      }
      else {
        this.utc_seconds = 0.0;
      }
      if (initObj.hasOwnProperty('position_status')) {
        this.position_status = initObj.position_status
      }
      else {
        this.position_status = '';
      }
      if (initObj.hasOwnProperty('lat')) {
        this.lat = initObj.lat
      }
      else {
        this.lat = 0.0;
      }
      if (initObj.hasOwnProperty('lon')) {
        this.lon = initObj.lon
      }
      else {
        this.lon = 0.0;
      }
      if (initObj.hasOwnProperty('lat_dir')) {
        this.lat_dir = initObj.lat_dir
      }
      else {
        this.lat_dir = '';
      }
      if (initObj.hasOwnProperty('lon_dir')) {
        this.lon_dir = initObj.lon_dir
      }
      else {
        this.lon_dir = '';
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('track')) {
        this.track = initObj.track
      }
      else {
        this.track = 0.0;
      }
      if (initObj.hasOwnProperty('date')) {
        this.date = initObj.date
      }
      else {
        this.date = '';
      }
      if (initObj.hasOwnProperty('mag_var')) {
        this.mag_var = initObj.mag_var
      }
      else {
        this.mag_var = 0.0;
      }
      if (initObj.hasOwnProperty('mag_var_direction')) {
        this.mag_var_direction = initObj.mag_var_direction
      }
      else {
        this.mag_var_direction = '';
      }
      if (initObj.hasOwnProperty('mode_indicator')) {
        this.mode_indicator = initObj.mode_indicator
      }
      else {
        this.mode_indicator = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gprmc
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [message_id]
    bufferOffset = _serializer.string(obj.message_id, buffer, bufferOffset);
    // Serialize message field [utc_seconds]
    bufferOffset = _serializer.float64(obj.utc_seconds, buffer, bufferOffset);
    // Serialize message field [position_status]
    bufferOffset = _serializer.string(obj.position_status, buffer, bufferOffset);
    // Serialize message field [lat]
    bufferOffset = _serializer.float64(obj.lat, buffer, bufferOffset);
    // Serialize message field [lon]
    bufferOffset = _serializer.float64(obj.lon, buffer, bufferOffset);
    // Serialize message field [lat_dir]
    bufferOffset = _serializer.string(obj.lat_dir, buffer, bufferOffset);
    // Serialize message field [lon_dir]
    bufferOffset = _serializer.string(obj.lon_dir, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [track]
    bufferOffset = _serializer.float32(obj.track, buffer, bufferOffset);
    // Serialize message field [date]
    bufferOffset = _serializer.string(obj.date, buffer, bufferOffset);
    // Serialize message field [mag_var]
    bufferOffset = _serializer.float32(obj.mag_var, buffer, bufferOffset);
    // Serialize message field [mag_var_direction]
    bufferOffset = _serializer.string(obj.mag_var_direction, buffer, bufferOffset);
    // Serialize message field [mode_indicator]
    bufferOffset = _serializer.string(obj.mode_indicator, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gprmc
    let len;
    let data = new Gprmc(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [message_id]
    data.message_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [utc_seconds]
    data.utc_seconds = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position_status]
    data.position_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lat]
    data.lat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lon]
    data.lon = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lat_dir]
    data.lat_dir = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lon_dir]
    data.lon_dir = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track]
    data.track = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [date]
    data.date = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mag_var]
    data.mag_var = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mag_var_direction]
    data.mag_var_direction = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mode_indicator]
    data.mode_indicator = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.message_id);
    length += _getByteLength(object.position_status);
    length += _getByteLength(object.lat_dir);
    length += _getByteLength(object.lon_dir);
    length += _getByteLength(object.date);
    length += _getByteLength(object.mag_var_direction);
    length += _getByteLength(object.mode_indicator);
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Gprmc';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02533bac67f17457b2e3538525ba1aae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message from GPRMC NMEA String
    Header header
    
    string message_id
    
    float64 utc_seconds
    string position_status
    
    float64 lat
    float64 lon
    
    string lat_dir
    string lon_dir
    
    float32 speed
    float32 track
    string date
    float32 mag_var
    string mag_var_direction
    string mode_indicator
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
    const resolved = new Gprmc(null);
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

    if (msg.utc_seconds !== undefined) {
      resolved.utc_seconds = msg.utc_seconds;
    }
    else {
      resolved.utc_seconds = 0.0
    }

    if (msg.position_status !== undefined) {
      resolved.position_status = msg.position_status;
    }
    else {
      resolved.position_status = ''
    }

    if (msg.lat !== undefined) {
      resolved.lat = msg.lat;
    }
    else {
      resolved.lat = 0.0
    }

    if (msg.lon !== undefined) {
      resolved.lon = msg.lon;
    }
    else {
      resolved.lon = 0.0
    }

    if (msg.lat_dir !== undefined) {
      resolved.lat_dir = msg.lat_dir;
    }
    else {
      resolved.lat_dir = ''
    }

    if (msg.lon_dir !== undefined) {
      resolved.lon_dir = msg.lon_dir;
    }
    else {
      resolved.lon_dir = ''
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.track !== undefined) {
      resolved.track = msg.track;
    }
    else {
      resolved.track = 0.0
    }

    if (msg.date !== undefined) {
      resolved.date = msg.date;
    }
    else {
      resolved.date = ''
    }

    if (msg.mag_var !== undefined) {
      resolved.mag_var = msg.mag_var;
    }
    else {
      resolved.mag_var = 0.0
    }

    if (msg.mag_var_direction !== undefined) {
      resolved.mag_var_direction = msg.mag_var_direction;
    }
    else {
      resolved.mag_var_direction = ''
    }

    if (msg.mode_indicator !== undefined) {
      resolved.mode_indicator = msg.mode_indicator;
    }
    else {
      resolved.mode_indicator = ''
    }

    return resolved;
    }
};

module.exports = Gprmc;
