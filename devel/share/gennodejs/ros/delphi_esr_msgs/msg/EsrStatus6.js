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

class EsrStatus6 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.supply_1p8v_a2d = null;
      this.supply_n5v_a2d = null;
      this.wave_diff_a2d = null;
      this.sw_version_dsp_3rd_byte = null;
      this.vertical_align_updated = null;
      this.system_power_mode = null;
      this.found_target = null;
      this.recommend_unconverge = null;
      this.factory_align_status_1 = null;
      this.factory_align_status_2 = null;
      this.factory_misalignment = null;
      this.serv_align_updates_done = null;
      this.vertical_misalignment = null;
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
      if (initObj.hasOwnProperty('supply_1p8v_a2d')) {
        this.supply_1p8v_a2d = initObj.supply_1p8v_a2d
      }
      else {
        this.supply_1p8v_a2d = 0;
      }
      if (initObj.hasOwnProperty('supply_n5v_a2d')) {
        this.supply_n5v_a2d = initObj.supply_n5v_a2d
      }
      else {
        this.supply_n5v_a2d = 0;
      }
      if (initObj.hasOwnProperty('wave_diff_a2d')) {
        this.wave_diff_a2d = initObj.wave_diff_a2d
      }
      else {
        this.wave_diff_a2d = 0;
      }
      if (initObj.hasOwnProperty('sw_version_dsp_3rd_byte')) {
        this.sw_version_dsp_3rd_byte = initObj.sw_version_dsp_3rd_byte
      }
      else {
        this.sw_version_dsp_3rd_byte = 0;
      }
      if (initObj.hasOwnProperty('vertical_align_updated')) {
        this.vertical_align_updated = initObj.vertical_align_updated
      }
      else {
        this.vertical_align_updated = false;
      }
      if (initObj.hasOwnProperty('system_power_mode')) {
        this.system_power_mode = initObj.system_power_mode
      }
      else {
        this.system_power_mode = 0;
      }
      if (initObj.hasOwnProperty('found_target')) {
        this.found_target = initObj.found_target
      }
      else {
        this.found_target = false;
      }
      if (initObj.hasOwnProperty('recommend_unconverge')) {
        this.recommend_unconverge = initObj.recommend_unconverge
      }
      else {
        this.recommend_unconverge = false;
      }
      if (initObj.hasOwnProperty('factory_align_status_1')) {
        this.factory_align_status_1 = initObj.factory_align_status_1
      }
      else {
        this.factory_align_status_1 = 0;
      }
      if (initObj.hasOwnProperty('factory_align_status_2')) {
        this.factory_align_status_2 = initObj.factory_align_status_2
      }
      else {
        this.factory_align_status_2 = 0;
      }
      if (initObj.hasOwnProperty('factory_misalignment')) {
        this.factory_misalignment = initObj.factory_misalignment
      }
      else {
        this.factory_misalignment = 0.0;
      }
      if (initObj.hasOwnProperty('serv_align_updates_done')) {
        this.serv_align_updates_done = initObj.serv_align_updates_done
      }
      else {
        this.serv_align_updates_done = 0;
      }
      if (initObj.hasOwnProperty('vertical_misalignment')) {
        this.vertical_misalignment = initObj.vertical_misalignment
      }
      else {
        this.vertical_misalignment = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrStatus6
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [supply_1p8v_a2d]
    bufferOffset = _serializer.uint8(obj.supply_1p8v_a2d, buffer, bufferOffset);
    // Serialize message field [supply_n5v_a2d]
    bufferOffset = _serializer.uint8(obj.supply_n5v_a2d, buffer, bufferOffset);
    // Serialize message field [wave_diff_a2d]
    bufferOffset = _serializer.uint8(obj.wave_diff_a2d, buffer, bufferOffset);
    // Serialize message field [sw_version_dsp_3rd_byte]
    bufferOffset = _serializer.uint8(obj.sw_version_dsp_3rd_byte, buffer, bufferOffset);
    // Serialize message field [vertical_align_updated]
    bufferOffset = _serializer.bool(obj.vertical_align_updated, buffer, bufferOffset);
    // Serialize message field [system_power_mode]
    bufferOffset = _serializer.uint8(obj.system_power_mode, buffer, bufferOffset);
    // Serialize message field [found_target]
    bufferOffset = _serializer.bool(obj.found_target, buffer, bufferOffset);
    // Serialize message field [recommend_unconverge]
    bufferOffset = _serializer.bool(obj.recommend_unconverge, buffer, bufferOffset);
    // Serialize message field [factory_align_status_1]
    bufferOffset = _serializer.uint8(obj.factory_align_status_1, buffer, bufferOffset);
    // Serialize message field [factory_align_status_2]
    bufferOffset = _serializer.uint8(obj.factory_align_status_2, buffer, bufferOffset);
    // Serialize message field [factory_misalignment]
    bufferOffset = _serializer.float32(obj.factory_misalignment, buffer, bufferOffset);
    // Serialize message field [serv_align_updates_done]
    bufferOffset = _serializer.uint8(obj.serv_align_updates_done, buffer, bufferOffset);
    // Serialize message field [vertical_misalignment]
    bufferOffset = _serializer.float32(obj.vertical_misalignment, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrStatus6
    let len;
    let data = new EsrStatus6(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [supply_1p8v_a2d]
    data.supply_1p8v_a2d = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [supply_n5v_a2d]
    data.supply_n5v_a2d = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [wave_diff_a2d]
    data.wave_diff_a2d = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sw_version_dsp_3rd_byte]
    data.sw_version_dsp_3rd_byte = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vertical_align_updated]
    data.vertical_align_updated = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [system_power_mode]
    data.system_power_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [found_target]
    data.found_target = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [recommend_unconverge]
    data.recommend_unconverge = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [factory_align_status_1]
    data.factory_align_status_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [factory_align_status_2]
    data.factory_align_status_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [factory_misalignment]
    data.factory_misalignment = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [serv_align_updates_done]
    data.serv_align_updates_done = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vertical_misalignment]
    data.vertical_misalignment = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    return length + 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrStatus6';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd37d9b3519a6461cdf385184e58e8259';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Status6
    string      canmsg
    
    uint8       supply_1p8v_a2d
    uint8       supply_n5v_a2d
    uint8       wave_diff_a2d
    uint8       sw_version_dsp_3rd_byte
    bool        vertical_align_updated
    uint8       system_power_mode
    bool        found_target
    bool        recommend_unconverge
    uint8       factory_align_status_1
    uint8       factory_align_status_2
    float32     factory_misalignment
    uint8       serv_align_updates_done
    float32     vertical_misalignment
    
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
    const resolved = new EsrStatus6(null);
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

    if (msg.supply_1p8v_a2d !== undefined) {
      resolved.supply_1p8v_a2d = msg.supply_1p8v_a2d;
    }
    else {
      resolved.supply_1p8v_a2d = 0
    }

    if (msg.supply_n5v_a2d !== undefined) {
      resolved.supply_n5v_a2d = msg.supply_n5v_a2d;
    }
    else {
      resolved.supply_n5v_a2d = 0
    }

    if (msg.wave_diff_a2d !== undefined) {
      resolved.wave_diff_a2d = msg.wave_diff_a2d;
    }
    else {
      resolved.wave_diff_a2d = 0
    }

    if (msg.sw_version_dsp_3rd_byte !== undefined) {
      resolved.sw_version_dsp_3rd_byte = msg.sw_version_dsp_3rd_byte;
    }
    else {
      resolved.sw_version_dsp_3rd_byte = 0
    }

    if (msg.vertical_align_updated !== undefined) {
      resolved.vertical_align_updated = msg.vertical_align_updated;
    }
    else {
      resolved.vertical_align_updated = false
    }

    if (msg.system_power_mode !== undefined) {
      resolved.system_power_mode = msg.system_power_mode;
    }
    else {
      resolved.system_power_mode = 0
    }

    if (msg.found_target !== undefined) {
      resolved.found_target = msg.found_target;
    }
    else {
      resolved.found_target = false
    }

    if (msg.recommend_unconverge !== undefined) {
      resolved.recommend_unconverge = msg.recommend_unconverge;
    }
    else {
      resolved.recommend_unconverge = false
    }

    if (msg.factory_align_status_1 !== undefined) {
      resolved.factory_align_status_1 = msg.factory_align_status_1;
    }
    else {
      resolved.factory_align_status_1 = 0
    }

    if (msg.factory_align_status_2 !== undefined) {
      resolved.factory_align_status_2 = msg.factory_align_status_2;
    }
    else {
      resolved.factory_align_status_2 = 0
    }

    if (msg.factory_misalignment !== undefined) {
      resolved.factory_misalignment = msg.factory_misalignment;
    }
    else {
      resolved.factory_misalignment = 0.0
    }

    if (msg.serv_align_updates_done !== undefined) {
      resolved.serv_align_updates_done = msg.serv_align_updates_done;
    }
    else {
      resolved.serv_align_updates_done = 0
    }

    if (msg.vertical_misalignment !== undefined) {
      resolved.vertical_misalignment = msg.vertical_misalignment;
    }
    else {
      resolved.vertical_misalignment = 0.0
    }

    return resolved;
    }
};

module.exports = EsrStatus6;
