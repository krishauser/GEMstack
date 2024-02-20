// Auto-generated. Do not edit!

// (in-package delphi_mrr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MrrDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detection_id = null;
      this.confid_azimuth = null;
      this.super_res_target = null;
      this.nd_target = null;
      this.host_veh_clutter = null;
      this.valid_level = null;
      this.azimuth = null;
      this.range = null;
      this.range_rate = null;
      this.amplitude = null;
      this.index_2lsb = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detection_id')) {
        this.detection_id = initObj.detection_id
      }
      else {
        this.detection_id = 0;
      }
      if (initObj.hasOwnProperty('confid_azimuth')) {
        this.confid_azimuth = initObj.confid_azimuth
      }
      else {
        this.confid_azimuth = 0;
      }
      if (initObj.hasOwnProperty('super_res_target')) {
        this.super_res_target = initObj.super_res_target
      }
      else {
        this.super_res_target = false;
      }
      if (initObj.hasOwnProperty('nd_target')) {
        this.nd_target = initObj.nd_target
      }
      else {
        this.nd_target = false;
      }
      if (initObj.hasOwnProperty('host_veh_clutter')) {
        this.host_veh_clutter = initObj.host_veh_clutter
      }
      else {
        this.host_veh_clutter = false;
      }
      if (initObj.hasOwnProperty('valid_level')) {
        this.valid_level = initObj.valid_level
      }
      else {
        this.valid_level = false;
      }
      if (initObj.hasOwnProperty('azimuth')) {
        this.azimuth = initObj.azimuth
      }
      else {
        this.azimuth = 0.0;
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('range_rate')) {
        this.range_rate = initObj.range_rate
      }
      else {
        this.range_rate = 0.0;
      }
      if (initObj.hasOwnProperty('amplitude')) {
        this.amplitude = initObj.amplitude
      }
      else {
        this.amplitude = 0;
      }
      if (initObj.hasOwnProperty('index_2lsb')) {
        this.index_2lsb = initObj.index_2lsb
      }
      else {
        this.index_2lsb = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrDetection
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detection_id]
    bufferOffset = _serializer.uint8(obj.detection_id, buffer, bufferOffset);
    // Serialize message field [confid_azimuth]
    bufferOffset = _serializer.uint8(obj.confid_azimuth, buffer, bufferOffset);
    // Serialize message field [super_res_target]
    bufferOffset = _serializer.bool(obj.super_res_target, buffer, bufferOffset);
    // Serialize message field [nd_target]
    bufferOffset = _serializer.bool(obj.nd_target, buffer, bufferOffset);
    // Serialize message field [host_veh_clutter]
    bufferOffset = _serializer.bool(obj.host_veh_clutter, buffer, bufferOffset);
    // Serialize message field [valid_level]
    bufferOffset = _serializer.bool(obj.valid_level, buffer, bufferOffset);
    // Serialize message field [azimuth]
    bufferOffset = _serializer.float32(obj.azimuth, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float32(obj.range, buffer, bufferOffset);
    // Serialize message field [range_rate]
    bufferOffset = _serializer.float32(obj.range_rate, buffer, bufferOffset);
    // Serialize message field [amplitude]
    bufferOffset = _serializer.int8(obj.amplitude, buffer, bufferOffset);
    // Serialize message field [index_2lsb]
    bufferOffset = _serializer.uint8(obj.index_2lsb, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrDetection
    let len;
    let data = new MrrDetection(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detection_id]
    data.detection_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [confid_azimuth]
    data.confid_azimuth = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [super_res_target]
    data.super_res_target = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [nd_target]
    data.nd_target = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [host_veh_clutter]
    data.host_veh_clutter = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [valid_level]
    data.valid_level = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [azimuth]
    data.azimuth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [range_rate]
    data.range_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [amplitude]
    data.amplitude = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [index_2lsb]
    data.index_2lsb = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'beed6f988400c44635b6b62be6463175';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8   detection_id
    uint8   confid_azimuth
    bool    super_res_target
    bool    nd_target
    bool    host_veh_clutter
    bool    valid_level
    float32 azimuth
    float32 range
    float32 range_rate
    int8    amplitude
    uint8   index_2lsb
    
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
    const resolved = new MrrDetection(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detection_id !== undefined) {
      resolved.detection_id = msg.detection_id;
    }
    else {
      resolved.detection_id = 0
    }

    if (msg.confid_azimuth !== undefined) {
      resolved.confid_azimuth = msg.confid_azimuth;
    }
    else {
      resolved.confid_azimuth = 0
    }

    if (msg.super_res_target !== undefined) {
      resolved.super_res_target = msg.super_res_target;
    }
    else {
      resolved.super_res_target = false
    }

    if (msg.nd_target !== undefined) {
      resolved.nd_target = msg.nd_target;
    }
    else {
      resolved.nd_target = false
    }

    if (msg.host_veh_clutter !== undefined) {
      resolved.host_veh_clutter = msg.host_veh_clutter;
    }
    else {
      resolved.host_veh_clutter = false
    }

    if (msg.valid_level !== undefined) {
      resolved.valid_level = msg.valid_level;
    }
    else {
      resolved.valid_level = false
    }

    if (msg.azimuth !== undefined) {
      resolved.azimuth = msg.azimuth;
    }
    else {
      resolved.azimuth = 0.0
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.range_rate !== undefined) {
      resolved.range_rate = msg.range_rate;
    }
    else {
      resolved.range_rate = 0.0
    }

    if (msg.amplitude !== undefined) {
      resolved.amplitude = msg.amplitude;
    }
    else {
      resolved.amplitude = 0
    }

    if (msg.index_2lsb !== undefined) {
      resolved.index_2lsb = msg.index_2lsb;
    }
    else {
      resolved.index_2lsb = 0
    }

    return resolved;
    }
};

module.exports = MrrDetection;
