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

class SrrStatus5 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.disable_auto_align = null;
      this.can_tx_yaw_rate_ref_qf = null;
      this.can_tx_yaw_rate_raw_qf = null;
      this.can_tx_yaw_rate_reference = null;
      this.can_tx_yaw_rate_raw = null;
      this.can_tx_system_status = null;
      this.can_tx_outside_temperature = null;
      this.can_blockage_mnr_blocked = null;
      this.can_blockage_bb_blocked = null;
      this.can_blockage_radar_blocked = null;
      this.can_td_blocked = null;
      this.radar_tx_power_error = null;
      this.radar_lo_power_error = null;
      this.radar_data_sync_error = null;
      this.linearizer_spi_transfer_error = null;
      this.saturated_tuning_freq_error = null;
      this.rtn_spi_transfer_error = null;
      this.rrn_spi_transfer_error = null;
      this.video_port_capture_error = null;
      this.vertical_misalignment_error = null;
      this.tx_temperature_fault = null;
      this.transmitter_id_error = null;
      this.dsp_unit_cal_checksum_error = null;
      this.dsp_unit_cal_block_chcksm_error = null;
      this.dsp_tuning_sensitivity_error = null;
      this.dsp_loop_overrun_error = null;
      this.adc_spi_transfer_error = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('disable_auto_align')) {
        this.disable_auto_align = initObj.disable_auto_align
      }
      else {
        this.disable_auto_align = false;
      }
      if (initObj.hasOwnProperty('can_tx_yaw_rate_ref_qf')) {
        this.can_tx_yaw_rate_ref_qf = initObj.can_tx_yaw_rate_ref_qf
      }
      else {
        this.can_tx_yaw_rate_ref_qf = 0;
      }
      if (initObj.hasOwnProperty('can_tx_yaw_rate_raw_qf')) {
        this.can_tx_yaw_rate_raw_qf = initObj.can_tx_yaw_rate_raw_qf
      }
      else {
        this.can_tx_yaw_rate_raw_qf = 0;
      }
      if (initObj.hasOwnProperty('can_tx_yaw_rate_reference')) {
        this.can_tx_yaw_rate_reference = initObj.can_tx_yaw_rate_reference
      }
      else {
        this.can_tx_yaw_rate_reference = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_yaw_rate_raw')) {
        this.can_tx_yaw_rate_raw = initObj.can_tx_yaw_rate_raw
      }
      else {
        this.can_tx_yaw_rate_raw = 0.0;
      }
      if (initObj.hasOwnProperty('can_tx_system_status')) {
        this.can_tx_system_status = initObj.can_tx_system_status
      }
      else {
        this.can_tx_system_status = 0;
      }
      if (initObj.hasOwnProperty('can_tx_outside_temperature')) {
        this.can_tx_outside_temperature = initObj.can_tx_outside_temperature
      }
      else {
        this.can_tx_outside_temperature = 0;
      }
      if (initObj.hasOwnProperty('can_blockage_mnr_blocked')) {
        this.can_blockage_mnr_blocked = initObj.can_blockage_mnr_blocked
      }
      else {
        this.can_blockage_mnr_blocked = false;
      }
      if (initObj.hasOwnProperty('can_blockage_bb_blocked')) {
        this.can_blockage_bb_blocked = initObj.can_blockage_bb_blocked
      }
      else {
        this.can_blockage_bb_blocked = false;
      }
      if (initObj.hasOwnProperty('can_blockage_radar_blocked')) {
        this.can_blockage_radar_blocked = initObj.can_blockage_radar_blocked
      }
      else {
        this.can_blockage_radar_blocked = false;
      }
      if (initObj.hasOwnProperty('can_td_blocked')) {
        this.can_td_blocked = initObj.can_td_blocked
      }
      else {
        this.can_td_blocked = false;
      }
      if (initObj.hasOwnProperty('radar_tx_power_error')) {
        this.radar_tx_power_error = initObj.radar_tx_power_error
      }
      else {
        this.radar_tx_power_error = false;
      }
      if (initObj.hasOwnProperty('radar_lo_power_error')) {
        this.radar_lo_power_error = initObj.radar_lo_power_error
      }
      else {
        this.radar_lo_power_error = false;
      }
      if (initObj.hasOwnProperty('radar_data_sync_error')) {
        this.radar_data_sync_error = initObj.radar_data_sync_error
      }
      else {
        this.radar_data_sync_error = false;
      }
      if (initObj.hasOwnProperty('linearizer_spi_transfer_error')) {
        this.linearizer_spi_transfer_error = initObj.linearizer_spi_transfer_error
      }
      else {
        this.linearizer_spi_transfer_error = false;
      }
      if (initObj.hasOwnProperty('saturated_tuning_freq_error')) {
        this.saturated_tuning_freq_error = initObj.saturated_tuning_freq_error
      }
      else {
        this.saturated_tuning_freq_error = false;
      }
      if (initObj.hasOwnProperty('rtn_spi_transfer_error')) {
        this.rtn_spi_transfer_error = initObj.rtn_spi_transfer_error
      }
      else {
        this.rtn_spi_transfer_error = false;
      }
      if (initObj.hasOwnProperty('rrn_spi_transfer_error')) {
        this.rrn_spi_transfer_error = initObj.rrn_spi_transfer_error
      }
      else {
        this.rrn_spi_transfer_error = false;
      }
      if (initObj.hasOwnProperty('video_port_capture_error')) {
        this.video_port_capture_error = initObj.video_port_capture_error
      }
      else {
        this.video_port_capture_error = false;
      }
      if (initObj.hasOwnProperty('vertical_misalignment_error')) {
        this.vertical_misalignment_error = initObj.vertical_misalignment_error
      }
      else {
        this.vertical_misalignment_error = false;
      }
      if (initObj.hasOwnProperty('tx_temperature_fault')) {
        this.tx_temperature_fault = initObj.tx_temperature_fault
      }
      else {
        this.tx_temperature_fault = false;
      }
      if (initObj.hasOwnProperty('transmitter_id_error')) {
        this.transmitter_id_error = initObj.transmitter_id_error
      }
      else {
        this.transmitter_id_error = false;
      }
      if (initObj.hasOwnProperty('dsp_unit_cal_checksum_error')) {
        this.dsp_unit_cal_checksum_error = initObj.dsp_unit_cal_checksum_error
      }
      else {
        this.dsp_unit_cal_checksum_error = false;
      }
      if (initObj.hasOwnProperty('dsp_unit_cal_block_chcksm_error')) {
        this.dsp_unit_cal_block_chcksm_error = initObj.dsp_unit_cal_block_chcksm_error
      }
      else {
        this.dsp_unit_cal_block_chcksm_error = false;
      }
      if (initObj.hasOwnProperty('dsp_tuning_sensitivity_error')) {
        this.dsp_tuning_sensitivity_error = initObj.dsp_tuning_sensitivity_error
      }
      else {
        this.dsp_tuning_sensitivity_error = false;
      }
      if (initObj.hasOwnProperty('dsp_loop_overrun_error')) {
        this.dsp_loop_overrun_error = initObj.dsp_loop_overrun_error
      }
      else {
        this.dsp_loop_overrun_error = false;
      }
      if (initObj.hasOwnProperty('adc_spi_transfer_error')) {
        this.adc_spi_transfer_error = initObj.adc_spi_transfer_error
      }
      else {
        this.adc_spi_transfer_error = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrStatus5
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [disable_auto_align]
    bufferOffset = _serializer.bool(obj.disable_auto_align, buffer, bufferOffset);
    // Serialize message field [can_tx_yaw_rate_ref_qf]
    bufferOffset = _serializer.uint8(obj.can_tx_yaw_rate_ref_qf, buffer, bufferOffset);
    // Serialize message field [can_tx_yaw_rate_raw_qf]
    bufferOffset = _serializer.uint8(obj.can_tx_yaw_rate_raw_qf, buffer, bufferOffset);
    // Serialize message field [can_tx_yaw_rate_reference]
    bufferOffset = _serializer.float32(obj.can_tx_yaw_rate_reference, buffer, bufferOffset);
    // Serialize message field [can_tx_yaw_rate_raw]
    bufferOffset = _serializer.float32(obj.can_tx_yaw_rate_raw, buffer, bufferOffset);
    // Serialize message field [can_tx_system_status]
    bufferOffset = _serializer.uint8(obj.can_tx_system_status, buffer, bufferOffset);
    // Serialize message field [can_tx_outside_temperature]
    bufferOffset = _serializer.int16(obj.can_tx_outside_temperature, buffer, bufferOffset);
    // Serialize message field [can_blockage_mnr_blocked]
    bufferOffset = _serializer.bool(obj.can_blockage_mnr_blocked, buffer, bufferOffset);
    // Serialize message field [can_blockage_bb_blocked]
    bufferOffset = _serializer.bool(obj.can_blockage_bb_blocked, buffer, bufferOffset);
    // Serialize message field [can_blockage_radar_blocked]
    bufferOffset = _serializer.bool(obj.can_blockage_radar_blocked, buffer, bufferOffset);
    // Serialize message field [can_td_blocked]
    bufferOffset = _serializer.bool(obj.can_td_blocked, buffer, bufferOffset);
    // Serialize message field [radar_tx_power_error]
    bufferOffset = _serializer.bool(obj.radar_tx_power_error, buffer, bufferOffset);
    // Serialize message field [radar_lo_power_error]
    bufferOffset = _serializer.bool(obj.radar_lo_power_error, buffer, bufferOffset);
    // Serialize message field [radar_data_sync_error]
    bufferOffset = _serializer.bool(obj.radar_data_sync_error, buffer, bufferOffset);
    // Serialize message field [linearizer_spi_transfer_error]
    bufferOffset = _serializer.bool(obj.linearizer_spi_transfer_error, buffer, bufferOffset);
    // Serialize message field [saturated_tuning_freq_error]
    bufferOffset = _serializer.bool(obj.saturated_tuning_freq_error, buffer, bufferOffset);
    // Serialize message field [rtn_spi_transfer_error]
    bufferOffset = _serializer.bool(obj.rtn_spi_transfer_error, buffer, bufferOffset);
    // Serialize message field [rrn_spi_transfer_error]
    bufferOffset = _serializer.bool(obj.rrn_spi_transfer_error, buffer, bufferOffset);
    // Serialize message field [video_port_capture_error]
    bufferOffset = _serializer.bool(obj.video_port_capture_error, buffer, bufferOffset);
    // Serialize message field [vertical_misalignment_error]
    bufferOffset = _serializer.bool(obj.vertical_misalignment_error, buffer, bufferOffset);
    // Serialize message field [tx_temperature_fault]
    bufferOffset = _serializer.bool(obj.tx_temperature_fault, buffer, bufferOffset);
    // Serialize message field [transmitter_id_error]
    bufferOffset = _serializer.bool(obj.transmitter_id_error, buffer, bufferOffset);
    // Serialize message field [dsp_unit_cal_checksum_error]
    bufferOffset = _serializer.bool(obj.dsp_unit_cal_checksum_error, buffer, bufferOffset);
    // Serialize message field [dsp_unit_cal_block_chcksm_error]
    bufferOffset = _serializer.bool(obj.dsp_unit_cal_block_chcksm_error, buffer, bufferOffset);
    // Serialize message field [dsp_tuning_sensitivity_error]
    bufferOffset = _serializer.bool(obj.dsp_tuning_sensitivity_error, buffer, bufferOffset);
    // Serialize message field [dsp_loop_overrun_error]
    bufferOffset = _serializer.bool(obj.dsp_loop_overrun_error, buffer, bufferOffset);
    // Serialize message field [adc_spi_transfer_error]
    bufferOffset = _serializer.bool(obj.adc_spi_transfer_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrStatus5
    let len;
    let data = new SrrStatus5(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [disable_auto_align]
    data.disable_auto_align = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_tx_yaw_rate_ref_qf]
    data.can_tx_yaw_rate_ref_qf = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_yaw_rate_raw_qf]
    data.can_tx_yaw_rate_raw_qf = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_yaw_rate_reference]
    data.can_tx_yaw_rate_reference = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_yaw_rate_raw]
    data.can_tx_yaw_rate_raw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_tx_system_status]
    data.can_tx_system_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [can_tx_outside_temperature]
    data.can_tx_outside_temperature = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [can_blockage_mnr_blocked]
    data.can_blockage_mnr_blocked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_blockage_bb_blocked]
    data.can_blockage_bb_blocked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_blockage_radar_blocked]
    data.can_blockage_radar_blocked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_td_blocked]
    data.can_td_blocked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [radar_tx_power_error]
    data.radar_tx_power_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [radar_lo_power_error]
    data.radar_lo_power_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [radar_data_sync_error]
    data.radar_data_sync_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [linearizer_spi_transfer_error]
    data.linearizer_spi_transfer_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [saturated_tuning_freq_error]
    data.saturated_tuning_freq_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rtn_spi_transfer_error]
    data.rtn_spi_transfer_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rrn_spi_transfer_error]
    data.rrn_spi_transfer_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [video_port_capture_error]
    data.video_port_capture_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [vertical_misalignment_error]
    data.vertical_misalignment_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tx_temperature_fault]
    data.tx_temperature_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [transmitter_id_error]
    data.transmitter_id_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_unit_cal_checksum_error]
    data.dsp_unit_cal_checksum_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_unit_cal_block_chcksm_error]
    data.dsp_unit_cal_block_chcksm_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_tuning_sensitivity_error]
    data.dsp_tuning_sensitivity_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_loop_overrun_error]
    data.dsp_loop_overrun_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [adc_spi_transfer_error]
    data.adc_spi_transfer_error = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 34;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrStatus5';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cd86757abd1063dffe9941dbe3f4362f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_status5
    
    std_msgs/Header header
    
    bool      disable_auto_align
    
    uint8     can_tx_yaw_rate_ref_qf
    uint8     CAN_TX_YAW_RATE_REF_QF_UNDEFINED=0
    uint8     CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED=1
    uint8     CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE=2
    uint8     CAN_TX_YAW_RATE_REF_QF_ACCURATE=3
    
    uint8     can_tx_yaw_rate_raw_qf
    uint8     CAN_TX_YAW_RATE_RAW_QF_UNDEFINED=0
    uint8     CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED=1
    uint8     CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE=2
    uint8     CAN_TX_YAW_RATE_RAW_QF_ACCURATE=3
    
    float32   can_tx_yaw_rate_reference                # deg/s
    float32   can_tx_yaw_rate_raw                      # deg/s
    
    uint8     can_tx_system_status
    uint8     CAN_TX_SYSTEM_STATUS_CONFIGURATION=0
    uint8     CAN_TX_SYSTEM_STATUS_STARTUP=1
    uint8     CAN_TX_SYSTEM_STATUS_RUNNING=2
    uint8     CAN_TX_SYSTEM_STATUS_BLOCKED=3
    uint8     CAN_TX_SYSTEM_STATUS_FAULTY=4
    uint8     CAN_TX_SYSTEM_STATUS_SHUTDOWN=5
    uint8     CAN_TX_SYSTEM_STATUS_HOT=6
    
    int16     can_tx_outside_temperature               # degc
    bool      can_blockage_mnr_blocked
    bool      can_blockage_bb_blocked
    bool      can_blockage_radar_blocked
    bool      can_td_blocked
    bool      radar_tx_power_error
    bool      radar_lo_power_error
    bool      radar_data_sync_error
    bool      linearizer_spi_transfer_error
    bool      saturated_tuning_freq_error
    bool      rtn_spi_transfer_error
    bool      rrn_spi_transfer_error
    bool      video_port_capture_error
    bool      vertical_misalignment_error
    bool      tx_temperature_fault
    bool      transmitter_id_error
    bool      dsp_unit_cal_checksum_error
    bool      dsp_unit_cal_block_chcksm_error
    bool      dsp_tuning_sensitivity_error
    bool      dsp_loop_overrun_error
    bool      adc_spi_transfer_error
    
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
    const resolved = new SrrStatus5(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.disable_auto_align !== undefined) {
      resolved.disable_auto_align = msg.disable_auto_align;
    }
    else {
      resolved.disable_auto_align = false
    }

    if (msg.can_tx_yaw_rate_ref_qf !== undefined) {
      resolved.can_tx_yaw_rate_ref_qf = msg.can_tx_yaw_rate_ref_qf;
    }
    else {
      resolved.can_tx_yaw_rate_ref_qf = 0
    }

    if (msg.can_tx_yaw_rate_raw_qf !== undefined) {
      resolved.can_tx_yaw_rate_raw_qf = msg.can_tx_yaw_rate_raw_qf;
    }
    else {
      resolved.can_tx_yaw_rate_raw_qf = 0
    }

    if (msg.can_tx_yaw_rate_reference !== undefined) {
      resolved.can_tx_yaw_rate_reference = msg.can_tx_yaw_rate_reference;
    }
    else {
      resolved.can_tx_yaw_rate_reference = 0.0
    }

    if (msg.can_tx_yaw_rate_raw !== undefined) {
      resolved.can_tx_yaw_rate_raw = msg.can_tx_yaw_rate_raw;
    }
    else {
      resolved.can_tx_yaw_rate_raw = 0.0
    }

    if (msg.can_tx_system_status !== undefined) {
      resolved.can_tx_system_status = msg.can_tx_system_status;
    }
    else {
      resolved.can_tx_system_status = 0
    }

    if (msg.can_tx_outside_temperature !== undefined) {
      resolved.can_tx_outside_temperature = msg.can_tx_outside_temperature;
    }
    else {
      resolved.can_tx_outside_temperature = 0
    }

    if (msg.can_blockage_mnr_blocked !== undefined) {
      resolved.can_blockage_mnr_blocked = msg.can_blockage_mnr_blocked;
    }
    else {
      resolved.can_blockage_mnr_blocked = false
    }

    if (msg.can_blockage_bb_blocked !== undefined) {
      resolved.can_blockage_bb_blocked = msg.can_blockage_bb_blocked;
    }
    else {
      resolved.can_blockage_bb_blocked = false
    }

    if (msg.can_blockage_radar_blocked !== undefined) {
      resolved.can_blockage_radar_blocked = msg.can_blockage_radar_blocked;
    }
    else {
      resolved.can_blockage_radar_blocked = false
    }

    if (msg.can_td_blocked !== undefined) {
      resolved.can_td_blocked = msg.can_td_blocked;
    }
    else {
      resolved.can_td_blocked = false
    }

    if (msg.radar_tx_power_error !== undefined) {
      resolved.radar_tx_power_error = msg.radar_tx_power_error;
    }
    else {
      resolved.radar_tx_power_error = false
    }

    if (msg.radar_lo_power_error !== undefined) {
      resolved.radar_lo_power_error = msg.radar_lo_power_error;
    }
    else {
      resolved.radar_lo_power_error = false
    }

    if (msg.radar_data_sync_error !== undefined) {
      resolved.radar_data_sync_error = msg.radar_data_sync_error;
    }
    else {
      resolved.radar_data_sync_error = false
    }

    if (msg.linearizer_spi_transfer_error !== undefined) {
      resolved.linearizer_spi_transfer_error = msg.linearizer_spi_transfer_error;
    }
    else {
      resolved.linearizer_spi_transfer_error = false
    }

    if (msg.saturated_tuning_freq_error !== undefined) {
      resolved.saturated_tuning_freq_error = msg.saturated_tuning_freq_error;
    }
    else {
      resolved.saturated_tuning_freq_error = false
    }

    if (msg.rtn_spi_transfer_error !== undefined) {
      resolved.rtn_spi_transfer_error = msg.rtn_spi_transfer_error;
    }
    else {
      resolved.rtn_spi_transfer_error = false
    }

    if (msg.rrn_spi_transfer_error !== undefined) {
      resolved.rrn_spi_transfer_error = msg.rrn_spi_transfer_error;
    }
    else {
      resolved.rrn_spi_transfer_error = false
    }

    if (msg.video_port_capture_error !== undefined) {
      resolved.video_port_capture_error = msg.video_port_capture_error;
    }
    else {
      resolved.video_port_capture_error = false
    }

    if (msg.vertical_misalignment_error !== undefined) {
      resolved.vertical_misalignment_error = msg.vertical_misalignment_error;
    }
    else {
      resolved.vertical_misalignment_error = false
    }

    if (msg.tx_temperature_fault !== undefined) {
      resolved.tx_temperature_fault = msg.tx_temperature_fault;
    }
    else {
      resolved.tx_temperature_fault = false
    }

    if (msg.transmitter_id_error !== undefined) {
      resolved.transmitter_id_error = msg.transmitter_id_error;
    }
    else {
      resolved.transmitter_id_error = false
    }

    if (msg.dsp_unit_cal_checksum_error !== undefined) {
      resolved.dsp_unit_cal_checksum_error = msg.dsp_unit_cal_checksum_error;
    }
    else {
      resolved.dsp_unit_cal_checksum_error = false
    }

    if (msg.dsp_unit_cal_block_chcksm_error !== undefined) {
      resolved.dsp_unit_cal_block_chcksm_error = msg.dsp_unit_cal_block_chcksm_error;
    }
    else {
      resolved.dsp_unit_cal_block_chcksm_error = false
    }

    if (msg.dsp_tuning_sensitivity_error !== undefined) {
      resolved.dsp_tuning_sensitivity_error = msg.dsp_tuning_sensitivity_error;
    }
    else {
      resolved.dsp_tuning_sensitivity_error = false
    }

    if (msg.dsp_loop_overrun_error !== undefined) {
      resolved.dsp_loop_overrun_error = msg.dsp_loop_overrun_error;
    }
    else {
      resolved.dsp_loop_overrun_error = false
    }

    if (msg.adc_spi_transfer_error !== undefined) {
      resolved.adc_spi_transfer_error = msg.adc_spi_transfer_error;
    }
    else {
      resolved.adc_spi_transfer_error = false
    }

    return resolved;
    }
};

// Constants for message
SrrStatus5.Constants = {
  CAN_TX_YAW_RATE_REF_QF_UNDEFINED: 0,
  CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED: 1,
  CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE: 2,
  CAN_TX_YAW_RATE_REF_QF_ACCURATE: 3,
  CAN_TX_YAW_RATE_RAW_QF_UNDEFINED: 0,
  CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED: 1,
  CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE: 2,
  CAN_TX_YAW_RATE_RAW_QF_ACCURATE: 3,
  CAN_TX_SYSTEM_STATUS_CONFIGURATION: 0,
  CAN_TX_SYSTEM_STATUS_STARTUP: 1,
  CAN_TX_SYSTEM_STATUS_RUNNING: 2,
  CAN_TX_SYSTEM_STATUS_BLOCKED: 3,
  CAN_TX_SYSTEM_STATUS_FAULTY: 4,
  CAN_TX_SYSTEM_STATUS_SHUTDOWN: 5,
  CAN_TX_SYSTEM_STATUS_HOT: 6,
}

module.exports = SrrStatus5;
