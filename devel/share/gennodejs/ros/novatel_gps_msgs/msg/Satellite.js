// Auto-generated. Do not edit!

// (in-package novatel_gps_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Satellite {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prn = null;
      this.elevation = null;
      this.azimuth = null;
      this.snr = null;
    }
    else {
      if (initObj.hasOwnProperty('prn')) {
        this.prn = initObj.prn
      }
      else {
        this.prn = 0;
      }
      if (initObj.hasOwnProperty('elevation')) {
        this.elevation = initObj.elevation
      }
      else {
        this.elevation = 0;
      }
      if (initObj.hasOwnProperty('azimuth')) {
        this.azimuth = initObj.azimuth
      }
      else {
        this.azimuth = 0;
      }
      if (initObj.hasOwnProperty('snr')) {
        this.snr = initObj.snr
      }
      else {
        this.snr = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Satellite
    // Serialize message field [prn]
    bufferOffset = _serializer.uint8(obj.prn, buffer, bufferOffset);
    // Serialize message field [elevation]
    bufferOffset = _serializer.uint8(obj.elevation, buffer, bufferOffset);
    // Serialize message field [azimuth]
    bufferOffset = _serializer.uint16(obj.azimuth, buffer, bufferOffset);
    // Serialize message field [snr]
    bufferOffset = _serializer.int8(obj.snr, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Satellite
    let len;
    let data = new Satellite(null);
    // Deserialize message field [prn]
    data.prn = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [elevation]
    data.elevation = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [azimuth]
    data.azimuth = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [snr]
    data.snr = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Satellite';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd862f2ce05a26a83264a8add99c7b668';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Satellite(null);
    if (msg.prn !== undefined) {
      resolved.prn = msg.prn;
    }
    else {
      resolved.prn = 0
    }

    if (msg.elevation !== undefined) {
      resolved.elevation = msg.elevation;
    }
    else {
      resolved.elevation = 0
    }

    if (msg.azimuth !== undefined) {
      resolved.azimuth = msg.azimuth;
    }
    else {
      resolved.azimuth = 0
    }

    if (msg.snr !== undefined) {
      resolved.snr = msg.snr;
    }
    else {
      resolved.snr = 0
    }

    return resolved;
    }
};

module.exports = Satellite;
