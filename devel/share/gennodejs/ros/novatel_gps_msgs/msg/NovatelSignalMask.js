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

class NovatelSignalMask {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.original_mask = null;
      this.gps_L1_used_in_solution = null;
      this.gps_L2_used_in_solution = null;
      this.gps_L3_used_in_solution = null;
      this.glonass_L1_used_in_solution = null;
      this.glonass_L2_used_in_solution = null;
    }
    else {
      if (initObj.hasOwnProperty('original_mask')) {
        this.original_mask = initObj.original_mask
      }
      else {
        this.original_mask = 0;
      }
      if (initObj.hasOwnProperty('gps_L1_used_in_solution')) {
        this.gps_L1_used_in_solution = initObj.gps_L1_used_in_solution
      }
      else {
        this.gps_L1_used_in_solution = false;
      }
      if (initObj.hasOwnProperty('gps_L2_used_in_solution')) {
        this.gps_L2_used_in_solution = initObj.gps_L2_used_in_solution
      }
      else {
        this.gps_L2_used_in_solution = false;
      }
      if (initObj.hasOwnProperty('gps_L3_used_in_solution')) {
        this.gps_L3_used_in_solution = initObj.gps_L3_used_in_solution
      }
      else {
        this.gps_L3_used_in_solution = false;
      }
      if (initObj.hasOwnProperty('glonass_L1_used_in_solution')) {
        this.glonass_L1_used_in_solution = initObj.glonass_L1_used_in_solution
      }
      else {
        this.glonass_L1_used_in_solution = false;
      }
      if (initObj.hasOwnProperty('glonass_L2_used_in_solution')) {
        this.glonass_L2_used_in_solution = initObj.glonass_L2_used_in_solution
      }
      else {
        this.glonass_L2_used_in_solution = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NovatelSignalMask
    // Serialize message field [original_mask]
    bufferOffset = _serializer.uint32(obj.original_mask, buffer, bufferOffset);
    // Serialize message field [gps_L1_used_in_solution]
    bufferOffset = _serializer.bool(obj.gps_L1_used_in_solution, buffer, bufferOffset);
    // Serialize message field [gps_L2_used_in_solution]
    bufferOffset = _serializer.bool(obj.gps_L2_used_in_solution, buffer, bufferOffset);
    // Serialize message field [gps_L3_used_in_solution]
    bufferOffset = _serializer.bool(obj.gps_L3_used_in_solution, buffer, bufferOffset);
    // Serialize message field [glonass_L1_used_in_solution]
    bufferOffset = _serializer.bool(obj.glonass_L1_used_in_solution, buffer, bufferOffset);
    // Serialize message field [glonass_L2_used_in_solution]
    bufferOffset = _serializer.bool(obj.glonass_L2_used_in_solution, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NovatelSignalMask
    let len;
    let data = new NovatelSignalMask(null);
    // Deserialize message field [original_mask]
    data.original_mask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gps_L1_used_in_solution]
    data.gps_L1_used_in_solution = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gps_L2_used_in_solution]
    data.gps_L2_used_in_solution = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gps_L3_used_in_solution]
    data.gps_L3_used_in_solution = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [glonass_L1_used_in_solution]
    data.glonass_L1_used_in_solution = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [glonass_L2_used_in_solution]
    data.glonass_L2_used_in_solution = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/NovatelSignalMask';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '007d687355f8f3c12ea4e18109172710';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Bit    Mask      Description
    #  0     0x01      GPS L1 used in Solution
    #  1     0x02      GPS L2 used in Solution
    #  2     0x04      GPS L5 used in Solution
    #  3     0x08      <Reserved>
    #  4     0x10      GLONASS L1 used in Solution
    #  5     0x20      GLONASS L2 used in Solution
    # 6-7  0x40-0x80   <Reserved>
    uint32 original_mask
    bool gps_L1_used_in_solution
    bool gps_L2_used_in_solution
    bool gps_L3_used_in_solution
    bool glonass_L1_used_in_solution
    bool glonass_L2_used_in_solution
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NovatelSignalMask(null);
    if (msg.original_mask !== undefined) {
      resolved.original_mask = msg.original_mask;
    }
    else {
      resolved.original_mask = 0
    }

    if (msg.gps_L1_used_in_solution !== undefined) {
      resolved.gps_L1_used_in_solution = msg.gps_L1_used_in_solution;
    }
    else {
      resolved.gps_L1_used_in_solution = false
    }

    if (msg.gps_L2_used_in_solution !== undefined) {
      resolved.gps_L2_used_in_solution = msg.gps_L2_used_in_solution;
    }
    else {
      resolved.gps_L2_used_in_solution = false
    }

    if (msg.gps_L3_used_in_solution !== undefined) {
      resolved.gps_L3_used_in_solution = msg.gps_L3_used_in_solution;
    }
    else {
      resolved.gps_L3_used_in_solution = false
    }

    if (msg.glonass_L1_used_in_solution !== undefined) {
      resolved.glonass_L1_used_in_solution = msg.glonass_L1_used_in_solution;
    }
    else {
      resolved.glonass_L1_used_in_solution = false
    }

    if (msg.glonass_L2_used_in_solution !== undefined) {
      resolved.glonass_L2_used_in_solution = msg.glonass_L2_used_in_solution;
    }
    else {
      resolved.glonass_L2_used_in_solution = false
    }

    return resolved;
    }
};

module.exports = NovatelSignalMask;
