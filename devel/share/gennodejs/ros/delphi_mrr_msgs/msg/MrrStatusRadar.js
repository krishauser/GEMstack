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

class MrrStatusRadar {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_interference_type = null;
      this.can_recommend_unconverge = null;
      this.can_blockage_sidelobe_filter_val = null;
      this.can_radar_align_incomplete = null;
      this.can_blockage_sidelobe = null;
      this.can_blockage_mnr = null;
      this.can_radar_ext_cond_nok = null;
      this.can_radar_align_out_range = null;
      this.can_radar_align_not_start = null;
      this.can_radar_overheat_error = null;
      this.can_radar_not_op = null;
      this.can_xcvr_operational = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_interference_type')) {
        this.can_interference_type = initObj.can_interference_type
      }
      else {
        this.can_interference_type = 0;
      }
      if (initObj.hasOwnProperty('can_recommend_unconverge')) {
        this.can_recommend_unconverge = initObj.can_recommend_unconverge
      }
      else {
        this.can_recommend_unconverge = false;
      }
      if (initObj.hasOwnProperty('can_blockage_sidelobe_filter_val')) {
        this.can_blockage_sidelobe_filter_val = initObj.can_blockage_sidelobe_filter_val
      }
      else {
        this.can_blockage_sidelobe_filter_val = 0;
      }
      if (initObj.hasOwnProperty('can_radar_align_incomplete')) {
        this.can_radar_align_incomplete = initObj.can_radar_align_incomplete
      }
      else {
        this.can_radar_align_incomplete = false;
      }
      if (initObj.hasOwnProperty('can_blockage_sidelobe')) {
        this.can_blockage_sidelobe = initObj.can_blockage_sidelobe
      }
      else {
        this.can_blockage_sidelobe = false;
      }
      if (initObj.hasOwnProperty('can_blockage_mnr')) {
        this.can_blockage_mnr = initObj.can_blockage_mnr
      }
      else {
        this.can_blockage_mnr = false;
      }
      if (initObj.hasOwnProperty('can_radar_ext_cond_nok')) {
        this.can_radar_ext_cond_nok = initObj.can_radar_ext_cond_nok
      }
      else {
        this.can_radar_ext_cond_nok = false;
      }
      if (initObj.hasOwnProperty('can_radar_align_out_range')) {
        this.can_radar_align_out_range = initObj.can_radar_align_out_range
      }
      else {
        this.can_radar_align_out_range = false;
      }
      if (initObj.hasOwnProperty('can_radar_align_not_start')) {
        this.can_radar_align_not_start = initObj.can_radar_align_not_start
      }
      else {
        this.can_radar_align_not_start = false;
      }
      if (initObj.hasOwnProperty('can_radar_overheat_error')) {
        this.can_radar_overheat_error = initObj.can_radar_overheat_error
      }
      else {
        this.can_radar_overheat_error = false;
      }
      if (initObj.hasOwnProperty('can_radar_not_op')) {
        this.can_radar_not_op = initObj.can_radar_not_op
      }
      else {
        this.can_radar_not_op = false;
      }
      if (initObj.hasOwnProperty('can_xcvr_operational')) {
        this.can_xcvr_operational = initObj.can_xcvr_operational
      }
      else {
        this.can_xcvr_operational = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrStatusRadar
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_interference_type]
    bufferOffset = _serializer.uint8(obj.can_interference_type, buffer, bufferOffset);
    // Serialize message field [can_recommend_unconverge]
    bufferOffset = _serializer.bool(obj.can_recommend_unconverge, buffer, bufferOffset);
    // Serialize message field [can_blockage_sidelobe_filter_val]
    bufferOffset = _serializer.uint8(obj.can_blockage_sidelobe_filter_val, buffer, bufferOffset);
    // Serialize message field [can_radar_align_incomplete]
    bufferOffset = _serializer.bool(obj.can_radar_align_incomplete, buffer, bufferOffset);
    // Serialize message field [can_blockage_sidelobe]
    bufferOffset = _serializer.bool(obj.can_blockage_sidelobe, buffer, bufferOffset);
    // Serialize message field [can_blockage_mnr]
    bufferOffset = _serializer.bool(obj.can_blockage_mnr, buffer, bufferOffset);
    // Serialize message field [can_radar_ext_cond_nok]
    bufferOffset = _serializer.bool(obj.can_radar_ext_cond_nok, buffer, bufferOffset);
    // Serialize message field [can_radar_align_out_range]
    bufferOffset = _serializer.bool(obj.can_radar_align_out_range, buffer, bufferOffset);
    // Serialize message field [can_radar_align_not_start]
    bufferOffset = _serializer.bool(obj.can_radar_align_not_start, buffer, bufferOffset);
    // Serialize message field [can_radar_overheat_error]
    bufferOffset = _serializer.bool(obj.can_radar_overheat_error, buffer, bufferOffset);
    // Serialize message field [can_radar_not_op]
    bufferOffset = _serializer.bool(obj.can_radar_not_op, buffer, bufferOffset);
    // Serialize message field [can_xcvr_operational]
    bufferOffset = _serializer.bool(obj.can_xcvr_operational, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrStatusRadar
    let len;
    let data = new MrrStatusRadar(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_interference_type]
    data.can_interference_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_recommend_unconverge]
    data.can_recommend_unconverge = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_blockage_sidelobe_filter_val]
    data.can_blockage_sidelobe_filter_val = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_radar_align_incomplete]
    data.can_radar_align_incomplete = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_blockage_sidelobe]
    data.can_blockage_sidelobe = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_blockage_mnr]
    data.can_blockage_mnr = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_radar_ext_cond_nok]
    data.can_radar_ext_cond_nok = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_radar_align_out_range]
    data.can_radar_align_out_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_radar_align_not_start]
    data.can_radar_align_not_start = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_radar_overheat_error]
    data.can_radar_overheat_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_radar_not_op]
    data.can_radar_not_op = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_xcvr_operational]
    data.can_xcvr_operational = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrStatusRadar';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3dbdaa8c61c744a4f2863586bf997cac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 can_interference_type
    bool  can_recommend_unconverge
    uint8 can_blockage_sidelobe_filter_val
    bool  can_radar_align_incomplete
    bool  can_blockage_sidelobe
    bool  can_blockage_mnr
    bool  can_radar_ext_cond_nok
    bool  can_radar_align_out_range
    bool  can_radar_align_not_start
    bool  can_radar_overheat_error
    bool  can_radar_not_op
    bool  can_xcvr_operational
    
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
    const resolved = new MrrStatusRadar(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_interference_type !== undefined) {
      resolved.can_interference_type = msg.can_interference_type;
    }
    else {
      resolved.can_interference_type = 0
    }

    if (msg.can_recommend_unconverge !== undefined) {
      resolved.can_recommend_unconverge = msg.can_recommend_unconverge;
    }
    else {
      resolved.can_recommend_unconverge = false
    }

    if (msg.can_blockage_sidelobe_filter_val !== undefined) {
      resolved.can_blockage_sidelobe_filter_val = msg.can_blockage_sidelobe_filter_val;
    }
    else {
      resolved.can_blockage_sidelobe_filter_val = 0
    }

    if (msg.can_radar_align_incomplete !== undefined) {
      resolved.can_radar_align_incomplete = msg.can_radar_align_incomplete;
    }
    else {
      resolved.can_radar_align_incomplete = false
    }

    if (msg.can_blockage_sidelobe !== undefined) {
      resolved.can_blockage_sidelobe = msg.can_blockage_sidelobe;
    }
    else {
      resolved.can_blockage_sidelobe = false
    }

    if (msg.can_blockage_mnr !== undefined) {
      resolved.can_blockage_mnr = msg.can_blockage_mnr;
    }
    else {
      resolved.can_blockage_mnr = false
    }

    if (msg.can_radar_ext_cond_nok !== undefined) {
      resolved.can_radar_ext_cond_nok = msg.can_radar_ext_cond_nok;
    }
    else {
      resolved.can_radar_ext_cond_nok = false
    }

    if (msg.can_radar_align_out_range !== undefined) {
      resolved.can_radar_align_out_range = msg.can_radar_align_out_range;
    }
    else {
      resolved.can_radar_align_out_range = false
    }

    if (msg.can_radar_align_not_start !== undefined) {
      resolved.can_radar_align_not_start = msg.can_radar_align_not_start;
    }
    else {
      resolved.can_radar_align_not_start = false
    }

    if (msg.can_radar_overheat_error !== undefined) {
      resolved.can_radar_overheat_error = msg.can_radar_overheat_error;
    }
    else {
      resolved.can_radar_overheat_error = false
    }

    if (msg.can_radar_not_op !== undefined) {
      resolved.can_radar_not_op = msg.can_radar_not_op;
    }
    else {
      resolved.can_radar_not_op = false
    }

    if (msg.can_xcvr_operational !== undefined) {
      resolved.can_xcvr_operational = msg.can_xcvr_operational;
    }
    else {
      resolved.can_xcvr_operational = false
    }

    return resolved;
    }
};

module.exports = MrrStatusRadar;
