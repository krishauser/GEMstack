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

class EsrStatus9 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.avg_pwr_cwblkg = null;
      this.sideslip_angle = null;
      this.serial_num_3rd_byte = null;
      this.water_spray_target_id = null;
      this.filtered_xohp_acc_cipv = null;
      this.path_id_acc_2 = null;
      this.path_id_acc_3 = null;
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
      if (initObj.hasOwnProperty('avg_pwr_cwblkg')) {
        this.avg_pwr_cwblkg = initObj.avg_pwr_cwblkg
      }
      else {
        this.avg_pwr_cwblkg = 0;
      }
      if (initObj.hasOwnProperty('sideslip_angle')) {
        this.sideslip_angle = initObj.sideslip_angle
      }
      else {
        this.sideslip_angle = 0.0;
      }
      if (initObj.hasOwnProperty('serial_num_3rd_byte')) {
        this.serial_num_3rd_byte = initObj.serial_num_3rd_byte
      }
      else {
        this.serial_num_3rd_byte = 0;
      }
      if (initObj.hasOwnProperty('water_spray_target_id')) {
        this.water_spray_target_id = initObj.water_spray_target_id
      }
      else {
        this.water_spray_target_id = 0;
      }
      if (initObj.hasOwnProperty('filtered_xohp_acc_cipv')) {
        this.filtered_xohp_acc_cipv = initObj.filtered_xohp_acc_cipv
      }
      else {
        this.filtered_xohp_acc_cipv = 0.0;
      }
      if (initObj.hasOwnProperty('path_id_acc_2')) {
        this.path_id_acc_2 = initObj.path_id_acc_2
      }
      else {
        this.path_id_acc_2 = 0;
      }
      if (initObj.hasOwnProperty('path_id_acc_3')) {
        this.path_id_acc_3 = initObj.path_id_acc_3
      }
      else {
        this.path_id_acc_3 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrStatus9
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [avg_pwr_cwblkg]
    bufferOffset = _serializer.uint16(obj.avg_pwr_cwblkg, buffer, bufferOffset);
    // Serialize message field [sideslip_angle]
    bufferOffset = _serializer.float32(obj.sideslip_angle, buffer, bufferOffset);
    // Serialize message field [serial_num_3rd_byte]
    bufferOffset = _serializer.uint8(obj.serial_num_3rd_byte, buffer, bufferOffset);
    // Serialize message field [water_spray_target_id]
    bufferOffset = _serializer.uint8(obj.water_spray_target_id, buffer, bufferOffset);
    // Serialize message field [filtered_xohp_acc_cipv]
    bufferOffset = _serializer.float32(obj.filtered_xohp_acc_cipv, buffer, bufferOffset);
    // Serialize message field [path_id_acc_2]
    bufferOffset = _serializer.uint8(obj.path_id_acc_2, buffer, bufferOffset);
    // Serialize message field [path_id_acc_3]
    bufferOffset = _serializer.uint8(obj.path_id_acc_3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrStatus9
    let len;
    let data = new EsrStatus9(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [avg_pwr_cwblkg]
    data.avg_pwr_cwblkg = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [sideslip_angle]
    data.sideslip_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [serial_num_3rd_byte]
    data.serial_num_3rd_byte = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [water_spray_target_id]
    data.water_spray_target_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [filtered_xohp_acc_cipv]
    data.filtered_xohp_acc_cipv = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [path_id_acc_2]
    data.path_id_acc_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [path_id_acc_3]
    data.path_id_acc_3 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrStatus9';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '303ebfcbdd8866a01d6c2acd5e4df496';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Status9
    string      canmsg
    
    uint16      avg_pwr_cwblkg
    float32     sideslip_angle
    uint8       serial_num_3rd_byte
    uint8       water_spray_target_id
    float32     filtered_xohp_acc_cipv
    uint8       path_id_acc_2
    uint8       path_id_acc_3
    
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
    const resolved = new EsrStatus9(null);
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

    if (msg.avg_pwr_cwblkg !== undefined) {
      resolved.avg_pwr_cwblkg = msg.avg_pwr_cwblkg;
    }
    else {
      resolved.avg_pwr_cwblkg = 0
    }

    if (msg.sideslip_angle !== undefined) {
      resolved.sideslip_angle = msg.sideslip_angle;
    }
    else {
      resolved.sideslip_angle = 0.0
    }

    if (msg.serial_num_3rd_byte !== undefined) {
      resolved.serial_num_3rd_byte = msg.serial_num_3rd_byte;
    }
    else {
      resolved.serial_num_3rd_byte = 0
    }

    if (msg.water_spray_target_id !== undefined) {
      resolved.water_spray_target_id = msg.water_spray_target_id;
    }
    else {
      resolved.water_spray_target_id = 0
    }

    if (msg.filtered_xohp_acc_cipv !== undefined) {
      resolved.filtered_xohp_acc_cipv = msg.filtered_xohp_acc_cipv;
    }
    else {
      resolved.filtered_xohp_acc_cipv = 0.0
    }

    if (msg.path_id_acc_2 !== undefined) {
      resolved.path_id_acc_2 = msg.path_id_acc_2;
    }
    else {
      resolved.path_id_acc_2 = 0
    }

    if (msg.path_id_acc_3 !== undefined) {
      resolved.path_id_acc_3 = msg.path_id_acc_3;
    }
    else {
      resolved.path_id_acc_3 = 0
    }

    return resolved;
    }
};

module.exports = EsrStatus9;
