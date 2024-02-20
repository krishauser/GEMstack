// Auto-generated. Do not edit!

// (in-package delphi_esr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EsrStatus4 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.truck_target_det = null;
      this.lr_only_grating_lobe_det = null;
      this.sidelobe_blockage = null;
      this.partial_blockage = null;
      this.mr_lr_mode = null;
      this.rolling_count_3 = null;
      this.path_id_acc = null;
      this.path_id_cmbb_move = null;
      this.path_id_cmbb_stat = null;
      this.path_id_fcw_move = null;
      this.path_id_fcw_stat = null;
      this.auto_align_angle = null;
      this.path_id_acc_stat = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('canmsg')) {
        this.canmsg = initObj.canmsg
      }
      else {
        this.canmsg = '';
      }
      if (initObj.hasOwnProperty('truck_target_det')) {
        this.truck_target_det = initObj.truck_target_det
      }
      else {
        this.truck_target_det = false;
      }
      if (initObj.hasOwnProperty('lr_only_grating_lobe_det')) {
        this.lr_only_grating_lobe_det = initObj.lr_only_grating_lobe_det
      }
      else {
        this.lr_only_grating_lobe_det = false;
      }
      if (initObj.hasOwnProperty('sidelobe_blockage')) {
        this.sidelobe_blockage = initObj.sidelobe_blockage
      }
      else {
        this.sidelobe_blockage = false;
      }
      if (initObj.hasOwnProperty('partial_blockage')) {
        this.partial_blockage = initObj.partial_blockage
      }
      else {
        this.partial_blockage = false;
      }
      if (initObj.hasOwnProperty('mr_lr_mode')) {
        this.mr_lr_mode = initObj.mr_lr_mode
      }
      else {
        this.mr_lr_mode = 0;
      }
      if (initObj.hasOwnProperty('rolling_count_3')) {
        this.rolling_count_3 = initObj.rolling_count_3
      }
      else {
        this.rolling_count_3 = 0;
      }
      if (initObj.hasOwnProperty('path_id_acc')) {
        this.path_id_acc = initObj.path_id_acc
      }
      else {
        this.path_id_acc = 0;
      }
      if (initObj.hasOwnProperty('path_id_cmbb_move')) {
        this.path_id_cmbb_move = initObj.path_id_cmbb_move
      }
      else {
        this.path_id_cmbb_move = 0;
      }
      if (initObj.hasOwnProperty('path_id_cmbb_stat')) {
        this.path_id_cmbb_stat = initObj.path_id_cmbb_stat
      }
      else {
        this.path_id_cmbb_stat = 0;
      }
      if (initObj.hasOwnProperty('path_id_fcw_move')) {
        this.path_id_fcw_move = initObj.path_id_fcw_move
      }
      else {
        this.path_id_fcw_move = 0;
      }
      if (initObj.hasOwnProperty('path_id_fcw_stat')) {
        this.path_id_fcw_stat = initObj.path_id_fcw_stat
      }
      else {
        this.path_id_fcw_stat = 0;
      }
      if (initObj.hasOwnProperty('auto_align_angle')) {
        this.auto_align_angle = initObj.auto_align_angle
      }
      else {
        this.auto_align_angle = 0.0;
      }
      if (initObj.hasOwnProperty('path_id_acc_stat')) {
        this.path_id_acc_stat = initObj.path_id_acc_stat
      }
      else {
        this.path_id_acc_stat = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrStatus4
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [truck_target_det]
    bufferOffset = _serializer.bool(obj.truck_target_det, buffer, bufferOffset);
    // Serialize message field [lr_only_grating_lobe_det]
    bufferOffset = _serializer.bool(obj.lr_only_grating_lobe_det, buffer, bufferOffset);
    // Serialize message field [sidelobe_blockage]
    bufferOffset = _serializer.bool(obj.sidelobe_blockage, buffer, bufferOffset);
    // Serialize message field [partial_blockage]
    bufferOffset = _serializer.bool(obj.partial_blockage, buffer, bufferOffset);
    // Serialize message field [mr_lr_mode]
    bufferOffset = _serializer.uint8(obj.mr_lr_mode, buffer, bufferOffset);
    // Serialize message field [rolling_count_3]
    bufferOffset = _serializer.uint8(obj.rolling_count_3, buffer, bufferOffset);
    // Serialize message field [path_id_acc]
    bufferOffset = _serializer.uint8(obj.path_id_acc, buffer, bufferOffset);
    // Serialize message field [path_id_cmbb_move]
    bufferOffset = _serializer.uint8(obj.path_id_cmbb_move, buffer, bufferOffset);
    // Serialize message field [path_id_cmbb_stat]
    bufferOffset = _serializer.uint8(obj.path_id_cmbb_stat, buffer, bufferOffset);
    // Serialize message field [path_id_fcw_move]
    bufferOffset = _serializer.uint8(obj.path_id_fcw_move, buffer, bufferOffset);
    // Serialize message field [path_id_fcw_stat]
    bufferOffset = _serializer.uint8(obj.path_id_fcw_stat, buffer, bufferOffset);
    // Serialize message field [auto_align_angle]
    bufferOffset = _serializer.float32(obj.auto_align_angle, buffer, bufferOffset);
    // Serialize message field [path_id_acc_stat]
    bufferOffset = _serializer.uint8(obj.path_id_acc_stat, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrStatus4
    let len;
    let data = new EsrStatus4(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [truck_target_det]
    data.truck_target_det = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [lr_only_grating_lobe_det]
    data.lr_only_grating_lobe_det = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [sidelobe_blockage]
    data.sidelobe_blockage = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [partial_blockage]
    data.partial_blockage = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mr_lr_mode]
    data.mr_lr_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rolling_count_3]
    data.rolling_count_3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [path_id_acc]
    data.path_id_acc = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [path_id_cmbb_move]
    data.path_id_cmbb_move = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [path_id_cmbb_stat]
    data.path_id_cmbb_stat = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [path_id_fcw_move]
    data.path_id_fcw_move = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [path_id_fcw_stat]
    data.path_id_fcw_stat = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [auto_align_angle]
    data.auto_align_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [path_id_acc_stat]
    data.path_id_acc_stat = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrStatus4';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6d073b78c0d621fce59ffa9fb7c576de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Status4
    string      canmsg
    
    bool        truck_target_det
    bool        lr_only_grating_lobe_det
    bool        sidelobe_blockage
    bool        partial_blockage
    uint8       mr_lr_mode
    uint8       rolling_count_3
    uint8       path_id_acc
    uint8       path_id_cmbb_move
    uint8       path_id_cmbb_stat
    uint8       path_id_fcw_move
    uint8       path_id_fcw_stat
    float32     auto_align_angle
    uint8       path_id_acc_stat
    
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
    const resolved = new EsrStatus4(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.canmsg !== undefined) {
      resolved.canmsg = msg.canmsg;
    }
    else {
      resolved.canmsg = ''
    }

    if (msg.truck_target_det !== undefined) {
      resolved.truck_target_det = msg.truck_target_det;
    }
    else {
      resolved.truck_target_det = false
    }

    if (msg.lr_only_grating_lobe_det !== undefined) {
      resolved.lr_only_grating_lobe_det = msg.lr_only_grating_lobe_det;
    }
    else {
      resolved.lr_only_grating_lobe_det = false
    }

    if (msg.sidelobe_blockage !== undefined) {
      resolved.sidelobe_blockage = msg.sidelobe_blockage;
    }
    else {
      resolved.sidelobe_blockage = false
    }

    if (msg.partial_blockage !== undefined) {
      resolved.partial_blockage = msg.partial_blockage;
    }
    else {
      resolved.partial_blockage = false
    }

    if (msg.mr_lr_mode !== undefined) {
      resolved.mr_lr_mode = msg.mr_lr_mode;
    }
    else {
      resolved.mr_lr_mode = 0
    }

    if (msg.rolling_count_3 !== undefined) {
      resolved.rolling_count_3 = msg.rolling_count_3;
    }
    else {
      resolved.rolling_count_3 = 0
    }

    if (msg.path_id_acc !== undefined) {
      resolved.path_id_acc = msg.path_id_acc;
    }
    else {
      resolved.path_id_acc = 0
    }

    if (msg.path_id_cmbb_move !== undefined) {
      resolved.path_id_cmbb_move = msg.path_id_cmbb_move;
    }
    else {
      resolved.path_id_cmbb_move = 0
    }

    if (msg.path_id_cmbb_stat !== undefined) {
      resolved.path_id_cmbb_stat = msg.path_id_cmbb_stat;
    }
    else {
      resolved.path_id_cmbb_stat = 0
    }

    if (msg.path_id_fcw_move !== undefined) {
      resolved.path_id_fcw_move = msg.path_id_fcw_move;
    }
    else {
      resolved.path_id_fcw_move = 0
    }

    if (msg.path_id_fcw_stat !== undefined) {
      resolved.path_id_fcw_stat = msg.path_id_fcw_stat;
    }
    else {
      resolved.path_id_fcw_stat = 0
    }

    if (msg.auto_align_angle !== undefined) {
      resolved.auto_align_angle = msg.auto_align_angle;
    }
    else {
      resolved.auto_align_angle = 0.0
    }

    if (msg.path_id_acc_stat !== undefined) {
      resolved.path_id_acc_stat = msg.path_id_acc_stat;
    }
    else {
      resolved.path_id_acc_stat = 0
    }

    return resolved;
    }
};

module.exports = EsrStatus4;
