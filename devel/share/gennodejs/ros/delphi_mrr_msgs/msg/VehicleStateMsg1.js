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

class VehicleStateMsg1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_fcw_sensitivity_level = null;
      this.can_vehicle_stationary = null;
      this.can_intf_minor_version = null;
      this.can_intf_major_version = null;
      this.can_brake_pedal = null;
      this.can_high_wheel_slip = null;
      this.can_turn_signal_status = null;
      this.can_washer_front_cmd = null;
      this.can_wiper_front_cmd = null;
      this.can_wiper_speed_info = null;
      this.can_reverse_gear = null;
      this.can_beam_shape_actual_right = null;
      this.can_beam_shape_actual_left = null;
      this.can_main_beam_indication = null;
      this.can_vehicle_index = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_fcw_sensitivity_level')) {
        this.can_fcw_sensitivity_level = initObj.can_fcw_sensitivity_level
      }
      else {
        this.can_fcw_sensitivity_level = 0;
      }
      if (initObj.hasOwnProperty('can_vehicle_stationary')) {
        this.can_vehicle_stationary = initObj.can_vehicle_stationary
      }
      else {
        this.can_vehicle_stationary = false;
      }
      if (initObj.hasOwnProperty('can_intf_minor_version')) {
        this.can_intf_minor_version = initObj.can_intf_minor_version
      }
      else {
        this.can_intf_minor_version = 0;
      }
      if (initObj.hasOwnProperty('can_intf_major_version')) {
        this.can_intf_major_version = initObj.can_intf_major_version
      }
      else {
        this.can_intf_major_version = 0;
      }
      if (initObj.hasOwnProperty('can_brake_pedal')) {
        this.can_brake_pedal = initObj.can_brake_pedal
      }
      else {
        this.can_brake_pedal = 0;
      }
      if (initObj.hasOwnProperty('can_high_wheel_slip')) {
        this.can_high_wheel_slip = initObj.can_high_wheel_slip
      }
      else {
        this.can_high_wheel_slip = false;
      }
      if (initObj.hasOwnProperty('can_turn_signal_status')) {
        this.can_turn_signal_status = initObj.can_turn_signal_status
      }
      else {
        this.can_turn_signal_status = 0;
      }
      if (initObj.hasOwnProperty('can_washer_front_cmd')) {
        this.can_washer_front_cmd = initObj.can_washer_front_cmd
      }
      else {
        this.can_washer_front_cmd = false;
      }
      if (initObj.hasOwnProperty('can_wiper_front_cmd')) {
        this.can_wiper_front_cmd = initObj.can_wiper_front_cmd
      }
      else {
        this.can_wiper_front_cmd = false;
      }
      if (initObj.hasOwnProperty('can_wiper_speed_info')) {
        this.can_wiper_speed_info = initObj.can_wiper_speed_info
      }
      else {
        this.can_wiper_speed_info = 0;
      }
      if (initObj.hasOwnProperty('can_reverse_gear')) {
        this.can_reverse_gear = initObj.can_reverse_gear
      }
      else {
        this.can_reverse_gear = false;
      }
      if (initObj.hasOwnProperty('can_beam_shape_actual_right')) {
        this.can_beam_shape_actual_right = initObj.can_beam_shape_actual_right
      }
      else {
        this.can_beam_shape_actual_right = 0;
      }
      if (initObj.hasOwnProperty('can_beam_shape_actual_left')) {
        this.can_beam_shape_actual_left = initObj.can_beam_shape_actual_left
      }
      else {
        this.can_beam_shape_actual_left = 0;
      }
      if (initObj.hasOwnProperty('can_main_beam_indication')) {
        this.can_main_beam_indication = initObj.can_main_beam_indication
      }
      else {
        this.can_main_beam_indication = false;
      }
      if (initObj.hasOwnProperty('can_vehicle_index')) {
        this.can_vehicle_index = initObj.can_vehicle_index
      }
      else {
        this.can_vehicle_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleStateMsg1
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_fcw_sensitivity_level]
    bufferOffset = _serializer.uint8(obj.can_fcw_sensitivity_level, buffer, bufferOffset);
    // Serialize message field [can_vehicle_stationary]
    bufferOffset = _serializer.bool(obj.can_vehicle_stationary, buffer, bufferOffset);
    // Serialize message field [can_intf_minor_version]
    bufferOffset = _serializer.uint8(obj.can_intf_minor_version, buffer, bufferOffset);
    // Serialize message field [can_intf_major_version]
    bufferOffset = _serializer.uint8(obj.can_intf_major_version, buffer, bufferOffset);
    // Serialize message field [can_brake_pedal]
    bufferOffset = _serializer.uint8(obj.can_brake_pedal, buffer, bufferOffset);
    // Serialize message field [can_high_wheel_slip]
    bufferOffset = _serializer.bool(obj.can_high_wheel_slip, buffer, bufferOffset);
    // Serialize message field [can_turn_signal_status]
    bufferOffset = _serializer.uint8(obj.can_turn_signal_status, buffer, bufferOffset);
    // Serialize message field [can_washer_front_cmd]
    bufferOffset = _serializer.bool(obj.can_washer_front_cmd, buffer, bufferOffset);
    // Serialize message field [can_wiper_front_cmd]
    bufferOffset = _serializer.bool(obj.can_wiper_front_cmd, buffer, bufferOffset);
    // Serialize message field [can_wiper_speed_info]
    bufferOffset = _serializer.uint8(obj.can_wiper_speed_info, buffer, bufferOffset);
    // Serialize message field [can_reverse_gear]
    bufferOffset = _serializer.bool(obj.can_reverse_gear, buffer, bufferOffset);
    // Serialize message field [can_beam_shape_actual_right]
    bufferOffset = _serializer.uint8(obj.can_beam_shape_actual_right, buffer, bufferOffset);
    // Serialize message field [can_beam_shape_actual_left]
    bufferOffset = _serializer.uint8(obj.can_beam_shape_actual_left, buffer, bufferOffset);
    // Serialize message field [can_main_beam_indication]
    bufferOffset = _serializer.bool(obj.can_main_beam_indication, buffer, bufferOffset);
    // Serialize message field [can_vehicle_index]
    bufferOffset = _serializer.uint16(obj.can_vehicle_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleStateMsg1
    let len;
    let data = new VehicleStateMsg1(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_fcw_sensitivity_level]
    data.can_fcw_sensitivity_level = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_vehicle_stationary]
    data.can_vehicle_stationary = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_intf_minor_version]
    data.can_intf_minor_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_intf_major_version]
    data.can_intf_major_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_brake_pedal]
    data.can_brake_pedal = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_high_wheel_slip]
    data.can_high_wheel_slip = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_turn_signal_status]
    data.can_turn_signal_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_washer_front_cmd]
    data.can_washer_front_cmd = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_wiper_front_cmd]
    data.can_wiper_front_cmd = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_wiper_speed_info]
    data.can_wiper_speed_info = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_reverse_gear]
    data.can_reverse_gear = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_beam_shape_actual_right]
    data.can_beam_shape_actual_right = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_beam_shape_actual_left]
    data.can_beam_shape_actual_left = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_main_beam_indication]
    data.can_main_beam_indication = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_vehicle_index]
    data.can_vehicle_index = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/VehicleStateMsg1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9bd8d57bd02218fdeffdb6496e73cbe0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8  can_fcw_sensitivity_level
    bool   can_vehicle_stationary
    uint8  can_intf_minor_version
    uint8  can_intf_major_version
    uint8  can_brake_pedal
    bool   can_high_wheel_slip
    uint8  can_turn_signal_status
    bool   can_washer_front_cmd
    bool   can_wiper_front_cmd
    uint8  can_wiper_speed_info
    bool   can_reverse_gear
    uint8  can_beam_shape_actual_right
    uint8  can_beam_shape_actual_left
    bool   can_main_beam_indication
    uint16 can_vehicle_index
    
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
    const resolved = new VehicleStateMsg1(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_fcw_sensitivity_level !== undefined) {
      resolved.can_fcw_sensitivity_level = msg.can_fcw_sensitivity_level;
    }
    else {
      resolved.can_fcw_sensitivity_level = 0
    }

    if (msg.can_vehicle_stationary !== undefined) {
      resolved.can_vehicle_stationary = msg.can_vehicle_stationary;
    }
    else {
      resolved.can_vehicle_stationary = false
    }

    if (msg.can_intf_minor_version !== undefined) {
      resolved.can_intf_minor_version = msg.can_intf_minor_version;
    }
    else {
      resolved.can_intf_minor_version = 0
    }

    if (msg.can_intf_major_version !== undefined) {
      resolved.can_intf_major_version = msg.can_intf_major_version;
    }
    else {
      resolved.can_intf_major_version = 0
    }

    if (msg.can_brake_pedal !== undefined) {
      resolved.can_brake_pedal = msg.can_brake_pedal;
    }
    else {
      resolved.can_brake_pedal = 0
    }

    if (msg.can_high_wheel_slip !== undefined) {
      resolved.can_high_wheel_slip = msg.can_high_wheel_slip;
    }
    else {
      resolved.can_high_wheel_slip = false
    }

    if (msg.can_turn_signal_status !== undefined) {
      resolved.can_turn_signal_status = msg.can_turn_signal_status;
    }
    else {
      resolved.can_turn_signal_status = 0
    }

    if (msg.can_washer_front_cmd !== undefined) {
      resolved.can_washer_front_cmd = msg.can_washer_front_cmd;
    }
    else {
      resolved.can_washer_front_cmd = false
    }

    if (msg.can_wiper_front_cmd !== undefined) {
      resolved.can_wiper_front_cmd = msg.can_wiper_front_cmd;
    }
    else {
      resolved.can_wiper_front_cmd = false
    }

    if (msg.can_wiper_speed_info !== undefined) {
      resolved.can_wiper_speed_info = msg.can_wiper_speed_info;
    }
    else {
      resolved.can_wiper_speed_info = 0
    }

    if (msg.can_reverse_gear !== undefined) {
      resolved.can_reverse_gear = msg.can_reverse_gear;
    }
    else {
      resolved.can_reverse_gear = false
    }

    if (msg.can_beam_shape_actual_right !== undefined) {
      resolved.can_beam_shape_actual_right = msg.can_beam_shape_actual_right;
    }
    else {
      resolved.can_beam_shape_actual_right = 0
    }

    if (msg.can_beam_shape_actual_left !== undefined) {
      resolved.can_beam_shape_actual_left = msg.can_beam_shape_actual_left;
    }
    else {
      resolved.can_beam_shape_actual_left = 0
    }

    if (msg.can_main_beam_indication !== undefined) {
      resolved.can_main_beam_indication = msg.can_main_beam_indication;
    }
    else {
      resolved.can_main_beam_indication = false
    }

    if (msg.can_vehicle_index !== undefined) {
      resolved.can_vehicle_index = msg.can_vehicle_index;
    }
    else {
      resolved.can_vehicle_index = 0
    }

    return resolved;
    }
};

module.exports = VehicleStateMsg1;
