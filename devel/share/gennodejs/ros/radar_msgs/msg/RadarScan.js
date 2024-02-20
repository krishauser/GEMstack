// Auto-generated. Do not edit!

// (in-package radar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RadarReturn = require('./RadarReturn.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RadarScan {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.returns = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('returns')) {
        this.returns = initObj.returns
      }
      else {
        this.returns = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RadarScan
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [returns]
    // Serialize the length for message field [returns]
    bufferOffset = _serializer.uint32(obj.returns.length, buffer, bufferOffset);
    obj.returns.forEach((val) => {
      bufferOffset = RadarReturn.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RadarScan
    let len;
    let data = new RadarScan(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [returns]
    // Deserialize array length for message field [returns]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.returns = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.returns[i] = RadarReturn.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 20 * object.returns.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'radar_msgs/RadarScan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6dfacef1e665538dbd8e159d5ce7a97a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    radar_msgs/RadarReturn[] returns
    
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
    MSG: radar_msgs/RadarReturn
    # All variables below are relative to the radar's frame of reference.
    # This message is not meant to be used alone but as part of a stamped or array message.
    
    float32 range                            # Distance (m) from the sensor to the detected return.
    float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.
                                             #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.
    float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.
                                             #    Negative angles are below the sensor. For 2D radar, this will be 0.
    float32 doppler_velocity                 # The doppler speeds (m/s) of the return.
    float32 amplitude                        # The amplitude of the of the return (dB)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RadarScan(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.returns !== undefined) {
      resolved.returns = new Array(msg.returns.length);
      for (let i = 0; i < resolved.returns.length; ++i) {
        resolved.returns[i] = RadarReturn.Resolve(msg.returns[i]);
      }
    }
    else {
      resolved.returns = []
    }

    return resolved;
    }
};

module.exports = RadarScan;
