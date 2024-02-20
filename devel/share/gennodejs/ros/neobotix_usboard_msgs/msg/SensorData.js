// Auto-generated. Do not edit!

// (in-package neobotix_usboard_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SensorData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.distance = null;
      this.warn = null;
      this.alarm = null;
      this.active = null;
    }
    else {
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0;
      }
      if (initObj.hasOwnProperty('warn')) {
        this.warn = initObj.warn
      }
      else {
        this.warn = false;
      }
      if (initObj.hasOwnProperty('alarm')) {
        this.alarm = initObj.alarm
      }
      else {
        this.alarm = false;
      }
      if (initObj.hasOwnProperty('active')) {
        this.active = initObj.active
      }
      else {
        this.active = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorData
    // Serialize message field [distance]
    bufferOffset = _serializer.uint8(obj.distance, buffer, bufferOffset);
    // Serialize message field [warn]
    bufferOffset = _serializer.bool(obj.warn, buffer, bufferOffset);
    // Serialize message field [alarm]
    bufferOffset = _serializer.bool(obj.alarm, buffer, bufferOffset);
    // Serialize message field [active]
    bufferOffset = _serializer.bool(obj.active, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorData
    let len;
    let data = new SensorData(null);
    // Deserialize message field [distance]
    data.distance = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [warn]
    data.warn = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm]
    data.alarm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active]
    data.active = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neobotix_usboard_msgs/SensorData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8b4451cc862e6df92992cfa6088c67e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for SensorData
    
    uint8   distance # cm
    bool    warn
    bool    alarm
    bool    active
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorData(null);
    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0
    }

    if (msg.warn !== undefined) {
      resolved.warn = msg.warn;
    }
    else {
      resolved.warn = false
    }

    if (msg.alarm !== undefined) {
      resolved.alarm = msg.alarm;
    }
    else {
      resolved.alarm = false
    }

    if (msg.active !== undefined) {
      resolved.active = msg.active;
    }
    else {
      resolved.active = false
    }

    return resolved;
    }
};

module.exports = SensorData;
