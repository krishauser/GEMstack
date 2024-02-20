// Auto-generated. Do not edit!

// (in-package delphi_srr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SrrStatus2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_tx_alignment_status = null;
      this.can_tx_comm_error = null;
      this.can_tx_steering_angle_sign = null;
      this.can_tx_yaw_rate_bias = null;
      this.can_tx_veh_spd_comp_factor = null;
      this.can_tx_sw_version_dsp = null;
      this.can_tx_temperature = null;
      this.can_tx_range_perf_error = null;
      this.can_tx_overheat_error = null;
      this.can_tx_internal_error = null;
      this.can_tx_xcvr_operational = null;
      this.can_tx_steering_angle = null;
      this.can_tx_rolling_count_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_tx_alignment_status')) {
        this.can_tx_alignment_status = initObj.can_tx_alignment_status
      }
      else {
        this.can_tx_alignment_status = 0;
      }
      if (initObj.hasOwnProperty('can_tx_comm_error')) {
        this.can_tx_comm_error = initObj.can_tx_comm_error
      }
      else {
        this.can_tx_comm_error = false;
      }
      if (initObj.hasOwnProperty('can_tx_steering_angle_sign')) {
        this.can_tx_steering_angle_sign = initObj.can_tx_steering_angle_sign
      }
      else {
        this.can_tx_steering_angle_sign = false;
      }
      if (initObj.hasOwnProperty('can_tx_yaw_rate_bias')) {
        this.can_tx_yaw_rate_bias = initObj.can_tx_yaw_rate_bias
      }
      else {
        this.can_tx_yaw_rate_bias = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_veh_spd_comp_factor')) {
        this.can_tx_veh_spd_comp_factor = initObj.can_tx_veh_spd_comp_factor
      }
      else {
        this.can_tx_veh_spd_comp_factor = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_sw_version_dsp')) {
        this.can_tx_sw_version_dsp = initObj.can_tx_sw_version_dsp
      }
      else {
        this.can_tx_sw_version_dsp = 0;
      }
      if (initObj.hasOwnProperty('can_tx_temperature')) {
        this.can_tx_temperature = initObj.can_tx_temperature
      }
      else {
        this.can_tx_temperature = 0;
      }
      if (initObj.hasOwnProperty('can_tx_range_perf_error')) {
        this.can_tx_range_perf_error = initObj.can_tx_range_perf_error
      }
      else {
        this.can_tx_range_perf_error = false;
      }
      if (initObj.hasOwnProperty('can_tx_overheat_error')) {
        this.can_tx_overheat_error = initObj.can_tx_overheat_error
      }
      else {
        this.can_tx_overheat_error = false;
      }
      if (initObj.hasOwnProperty('can_tx_internal_error')) {
        this.can_tx_internal_error = initObj.can_tx_internal_error
      }
      else {
        this.can_tx_internal_error = false;
      }
      if (initObj.hasOwnProperty('can_tx_xcvr_operational')) {
        this.can_tx_xcvr_operational = initObj.can_tx_xcvr_operational
      }
      else {
        this.can_tx_xcvr_operational = false;
      }
      if (initObj.hasOwnProperty('can_tx_steering_angle')) {
        this.can_tx_steering_angle = initObj.can_tx_steering_angle
      }
      else {
        this.can_tx_steering_angle = 0;
      }
      if (initObj.hasOwnProperty('can_tx_rolling_count_2')) {
        this.can_tx_rolling_count_2 = initObj.can_tx_rolling_count_2
      }
      else {
        this.can_tx_rolling_count_2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrStatus2
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_tx_alignment_status]
    bufferOffset = _serializer.uint8(obj.can_tx_alignment_status, buffer, bufferOffset);
    // Serialize message field [can_tx_comm_error]
    bufferOffset = _serializer.bool(obj.can_tx_comm_error, buffer, bufferOffset);
    // Serialize message field [can_tx_steering_angle_sign]
    bufferOffset = _serializer.bool(obj.can_tx_steering_angle_sign, buffer, bufferOffset);
    // Serialize message field [can_tx_yaw_rate_bias]
    bufferOffset = _serializer.float32(obj.can_tx_yaw_rate_bias, buffer, bufferOffset);
    // Serialize message field [can_tx_veh_spd_comp_factor]
    bufferOffset = _serializer.float32(obj.can_tx_veh_spd_comp_factor, buffer, bufferOffset);
    // Serialize message field [can_tx_sw_version_dsp]
    bufferOffset = _serializer.uint16(obj.can_tx_sw_version_dsp, buffer, bufferOffset);
    // Serialize message field [can_tx_temperature]
    bufferOffset = _serializer.int16(obj.can_tx_temperature, buffer, bufferOffset);
    // Serialize message field [can_tx_range_perf_error]
    bufferOffset = _serializer.bool(obj.can_tx_range_perf_error, buffer, bufferOffset);
    // Serialize message field [can_tx_overheat_error]
    bufferOffset = _serializer.bool(obj.can_tx_overheat_error, buffer, bufferOffset);
    // Serialize message field [can_tx_internal_error]
    bufferOffset = _serializer.bool(obj.can_tx_internal_error, buffer, bufferOffset);
    // Serialize message field [can_tx_xcvr_operational]
    bufferOffset = _serializer.bool(obj.can_tx_xcvr_operational, buffer, bufferOffset);
    // Serialize message field [can_tx_steering_angle]
    bufferOffset = _serializer.uint16(obj.can_tx_steering_angle, buffer, bufferOffset);
    // Serialize message field [can_tx_rolling_count_2]
    bufferOffset = _serializer.uint8(obj.can_tx_rolling_count_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrStatus2
    let len;
    let data = new SrrStatus2(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_tx_alignment_status]
    data.can_tx_alignment_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_comm_error]
    data.can_tx_comm_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_steering_angle_sign]
    data.can_tx_steering_angle_sign = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_yaw_rate_bias]
    data.can_tx_yaw_rate_bias = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_veh_spd_comp_factor]
    data.can_tx_veh_spd_comp_factor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_sw_version_dsp]
    data.can_tx_sw_version_dsp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_tx_temperature]
    data.can_tx_temperature = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [can_tx_range_perf_error]
    data.can_tx_range_perf_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_overheat_error]
    data.can_tx_overheat_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_internal_error]
    data.can_tx_internal_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_xcvr_operational]
    data.can_tx_xcvr_operational = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_steering_angle]
    data.can_tx_steering_angle = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [can_tx_rolling_count_2]
    data.can_tx_rolling_count_2 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrStatus2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2b05d1c3cfa8185e9616806113ff9b8c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_status2
    
    std_msgs/Header header
    
    uint8     can_tx_alignment_status
    uint8     CAN_TX_ALIGNMENT_STATUS_UNKNOWN=0
    uint8     CAN_TX_ALIGNMENT_STATUS_CONVERGED=1
    uint8     CAN_TX_ALIGNMENT_STATUS_FAILED=2
    uint8     CAN_TX_ALIGNMENT_STATUS_RESERVED=3
    
    bool      can_tx_comm_error
    bool      can_tx_steering_angle_sign
    float32   can_tx_yaw_rate_bias
    float32   can_tx_veh_spd_comp_factor
    uint16    can_tx_sw_version_dsp
    int16     can_tx_temperature                       # degc
    bool      can_tx_range_perf_error
    bool      can_tx_overheat_error
    bool      can_tx_internal_error
    bool      can_tx_xcvr_operational
    uint16    can_tx_steering_angle                    # deg
    uint8     can_tx_rolling_count_2
    
    
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
    const resolved = new SrrStatus2(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_tx_alignment_status !== undefined) {
      resolved.can_tx_alignment_status = msg.can_tx_alignment_status;
    }
    else {
      resolved.can_tx_alignment_status = 0
    }

    if (msg.can_tx_comm_error !== undefined) {
      resolved.can_tx_comm_error = msg.can_tx_comm_error;
    }
    else {
      resolved.can_tx_comm_error = false
    }

    if (msg.can_tx_steering_angle_sign !== undefined) {
      resolved.can_tx_steering_angle_sign = msg.can_tx_steering_angle_sign;
    }
    else {
      resolved.can_tx_steering_angle_sign = false
    }

    if (msg.can_tx_yaw_rate_bias !== undefined) {
      resolved.can_tx_yaw_rate_bias = msg.can_tx_yaw_rate_bias;
    }
    else {
      resolved.can_tx_yaw_rate_bias = 0.0
    }

    if (msg.can_tx_veh_spd_comp_factor !== undefined) {
      resolved.can_tx_veh_spd_comp_factor = msg.can_tx_veh_spd_comp_factor;
    }
    else {
      resolved.can_tx_veh_spd_comp_factor = 0.0
    }

    if (msg.can_tx_sw_version_dsp !== undefined) {
      resolved.can_tx_sw_version_dsp = msg.can_tx_sw_version_dsp;
    }
    else {
      resolved.can_tx_sw_version_dsp = 0
    }

    if (msg.can_tx_temperature !== undefined) {
      resolved.can_tx_temperature = msg.can_tx_temperature;
    }
    else {
      resolved.can_tx_temperature = 0
    }

    if (msg.can_tx_range_perf_error !== undefined) {
      resolved.can_tx_range_perf_error = msg.can_tx_range_perf_error;
    }
    else {
      resolved.can_tx_range_perf_error = false
    }

    if (msg.can_tx_overheat_error !== undefined) {
      resolved.can_tx_overheat_error = msg.can_tx_overheat_error;
    }
    else {
      resolved.can_tx_overheat_error = false
    }

    if (msg.can_tx_internal_error !== undefined) {
      resolved.can_tx_internal_error = msg.can_tx_internal_error;
    }
    else {
      resolved.can_tx_internal_error = false
    }

    if (msg.can_tx_xcvr_operational !== undefined) {
      resolved.can_tx_xcvr_operational = msg.can_tx_xcvr_operational;
    }
    else {
      resolved.can_tx_xcvr_operational = false
    }

    if (msg.can_tx_steering_angle !== undefined) {
      resolved.can_tx_steering_angle = msg.can_tx_steering_angle;
    }
    else {
      resolved.can_tx_steering_angle = 0
    }

    if (msg.can_tx_rolling_count_2 !== undefined) {
      resolved.can_tx_rolling_count_2 = msg.can_tx_rolling_count_2;
    }
    else {
      resolved.can_tx_rolling_count_2 = 0
    }

    return resolved;
    }
};

// Constants for message
SrrStatus2.Constants = {
  CAN_TX_ALIGNMENT_STATUS_UNKNOWN: 0,
  CAN_TX_ALIGNMENT_STATUS_CONVERGED: 1,
  CAN_TX_ALIGNMENT_STATUS_FAILED: 2,
  CAN_TX_ALIGNMENT_STATUS_RESERVED: 3,
}

module.exports = SrrStatus2;
