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

class NovatelExtendedSolutionStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.original_mask = null;
      this.advance_rtk_verified = null;
      this.psuedorange_iono_correction = null;
    }
    else {
      if (initObj.hasOwnProperty('original_mask')) {
        this.original_mask = initObj.original_mask
      }
      else {
        this.original_mask = 0;
      }
      if (initObj.hasOwnProperty('advance_rtk_verified')) {
        this.advance_rtk_verified = initObj.advance_rtk_verified
      }
      else {
        this.advance_rtk_verified = false;
      }
      if (initObj.hasOwnProperty('psuedorange_iono_correction')) {
        this.psuedorange_iono_correction = initObj.psuedorange_iono_correction
      }
      else {
        this.psuedorange_iono_correction = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NovatelExtendedSolutionStatus
    // Serialize message field [original_mask]
    bufferOffset = _serializer.uint32(obj.original_mask, buffer, bufferOffset);
    // Serialize message field [advance_rtk_verified]
    bufferOffset = _serializer.bool(obj.advance_rtk_verified, buffer, bufferOffset);
    // Serialize message field [psuedorange_iono_correction]
    bufferOffset = _serializer.string(obj.psuedorange_iono_correction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NovatelExtendedSolutionStatus
    let len;
    let data = new NovatelExtendedSolutionStatus(null);
    // Deserialize message field [original_mask]
    data.original_mask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [advance_rtk_verified]
    data.advance_rtk_verified = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [psuedorange_iono_correction]
    data.psuedorange_iono_correction = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.psuedorange_iono_correction);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/NovatelExtendedSolutionStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f0e19d53094c207c4dafdfbde750c4b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Bit    Mask      Description
    #  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)
    # 1-3    0x0E      Pseudorange Ionosphere Correction
    #                    0 = unknown
    #                    1 = Klobuchar Broadcast
    #                    2 = SBAS Broadcast
    #                    3 = Multi-frequency Computed
    #                    4 = PSRDiff Correction
    #                    5 = NovaTel Blended Ionosphere Value
    # 4-7  0xF0        <Reserved>
    uint32 original_mask
    bool advance_rtk_verified
    string psuedorange_iono_correction
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NovatelExtendedSolutionStatus(null);
    if (msg.original_mask !== undefined) {
      resolved.original_mask = msg.original_mask;
    }
    else {
      resolved.original_mask = 0
    }

    if (msg.advance_rtk_verified !== undefined) {
      resolved.advance_rtk_verified = msg.advance_rtk_verified;
    }
    else {
      resolved.advance_rtk_verified = false
    }

    if (msg.psuedorange_iono_correction !== undefined) {
      resolved.psuedorange_iono_correction = msg.psuedorange_iono_correction;
    }
    else {
      resolved.psuedorange_iono_correction = ''
    }

    return resolved;
    }
};

module.exports = NovatelExtendedSolutionStatus;
