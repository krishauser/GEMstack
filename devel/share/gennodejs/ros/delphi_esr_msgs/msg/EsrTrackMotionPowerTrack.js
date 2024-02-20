// Auto-generated. Do not edit!

// (in-package delphi_esr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class EsrTrackMotionPowerTrack {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.movable_fast = null;
      this.movable_slow = null;
      this.moving = null;
      this.power = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('movable_fast')) {
        this.movable_fast = initObj.movable_fast
      }
      else {
        this.movable_fast = false;
      }
      if (initObj.hasOwnProperty('movable_slow')) {
        this.movable_slow = initObj.movable_slow
      }
      else {
        this.movable_slow = false;
      }
      if (initObj.hasOwnProperty('moving')) {
        this.moving = initObj.moving
      }
      else {
        this.moving = false;
      }
      if (initObj.hasOwnProperty('power')) {
        this.power = initObj.power
      }
      else {
        this.power = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrTrackMotionPowerTrack
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [movable_fast]
    bufferOffset = _serializer.bool(obj.movable_fast, buffer, bufferOffset);
    // Serialize message field [movable_slow]
    bufferOffset = _serializer.bool(obj.movable_slow, buffer, bufferOffset);
    // Serialize message field [moving]
    bufferOffset = _serializer.bool(obj.moving, buffer, bufferOffset);
    // Serialize message field [power]
    bufferOffset = _serializer.int8(obj.power, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrTrackMotionPowerTrack
    let len;
    let data = new EsrTrackMotionPowerTrack(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [movable_fast]
    data.movable_fast = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [movable_slow]
    data.movable_slow = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [moving]
    data.moving = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [power]
    data.power = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrTrackMotionPowerTrack';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3cb7ee3e17f03f833bf47e59a4267646';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new EsrTrackMotionPowerTrack(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.movable_fast !== undefined) {
      resolved.movable_fast = msg.movable_fast;
    }
    else {
      resolved.movable_fast = false
    }

    if (msg.movable_slow !== undefined) {
      resolved.movable_slow = msg.movable_slow;
    }
    else {
      resolved.movable_slow = false
    }

    if (msg.moving !== undefined) {
      resolved.moving = msg.moving;
    }
    else {
      resolved.moving = false
    }

    if (msg.power !== undefined) {
      resolved.power = msg.power;
    }
    else {
      resolved.power = 0
    }

    return resolved;
    }
};

module.exports = EsrTrackMotionPowerTrack;
