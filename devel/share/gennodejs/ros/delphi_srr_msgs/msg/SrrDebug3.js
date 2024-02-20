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

class SrrDebug3 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.timer_create_error = null;
      this.thread_create_error = null;
      this.arm_calibration_error = null;
      this.spi_fee_error = null;
      this.spi_comm_error = null;
      this.socket_write_error = null;
      this.dsp_cal_obsolete_62_error = null;
      this.socket_read_error = null;
      this.socket_init_error = null;
      this.signal_wait_error = null;
      this.signal_send_error = null;
      this.signal_create_error = null;
      this.shared_mem_write_error = null;
      this.shared_mem_read_error = null;
      this.shared_mem_config_error = null;
      this.share_mem_init_error = null;
      this.ram_test_error = null;
      this.num_errors = null;
      this.mmap_memory_error = null;
      this.isr_attach_error = null;
      this.ipc_drv_write_error = null;
      this.ipc_drv_trigger_error = null;
      this.ipc_drv_sync_error = null;
      this.ipc_drv_read_error = null;
      this.ipc_drv_init_error = null;
      this.interrupt_enable_error = null;
      this.hil_format_error = null;
      this.flash_filesystem_error = null;
      this.error_none = null;
      this.dsp_load_read_error = null;
      this.dsp_load_open_error = null;
      this.dsp_load_address_error = null;
      this.dsp_isp_write_error = null;
      this.dsp_ipc_read_error = null;
      this.dsp_ipc_init = null;
      this.dsp_init_error = null;
      this.dsp_drv_start_error = null;
      this.dsp_drv_load_error = null;
      this.dsp_drv_init_error = null;
      this.dsp_drv_init2_error = null;
      this.dsp_drv_init1_error = null;
      this.dsp_calibration_error = null;
      this.can_xmt_error = null;
      this.can_rcv_error = null;
      this.can_hardware_error = null;
      this.always_true = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('timer_create_error')) {
        this.timer_create_error = initObj.timer_create_error
      }
      else {
        this.timer_create_error = false;
      }
      if (initObj.hasOwnProperty('thread_create_error')) {
        this.thread_create_error = initObj.thread_create_error
      }
      else {
        this.thread_create_error = false;
      }
      if (initObj.hasOwnProperty('arm_calibration_error')) {
        this.arm_calibration_error = initObj.arm_calibration_error
      }
      else {
        this.arm_calibration_error = false;
      }
      if (initObj.hasOwnProperty('spi_fee_error')) {
        this.spi_fee_error = initObj.spi_fee_error
      }
      else {
        this.spi_fee_error = false;
      }
      if (initObj.hasOwnProperty('spi_comm_error')) {
        this.spi_comm_error = initObj.spi_comm_error
      }
      else {
        this.spi_comm_error = false;
      }
      if (initObj.hasOwnProperty('socket_write_error')) {
        this.socket_write_error = initObj.socket_write_error
      }
      else {
        this.socket_write_error = false;
      }
      if (initObj.hasOwnProperty('dsp_cal_obsolete_62_error')) {
        this.dsp_cal_obsolete_62_error = initObj.dsp_cal_obsolete_62_error
      }
      else {
        this.dsp_cal_obsolete_62_error = false;
      }
      if (initObj.hasOwnProperty('socket_read_error')) {
        this.socket_read_error = initObj.socket_read_error
      }
      else {
        this.socket_read_error = false;
      }
      if (initObj.hasOwnProperty('socket_init_error')) {
        this.socket_init_error = initObj.socket_init_error
      }
      else {
        this.socket_init_error = false;
      }
      if (initObj.hasOwnProperty('signal_wait_error')) {
        this.signal_wait_error = initObj.signal_wait_error
      }
      else {
        this.signal_wait_error = false;
      }
      if (initObj.hasOwnProperty('signal_send_error')) {
        this.signal_send_error = initObj.signal_send_error
      }
      else {
        this.signal_send_error = false;
      }
      if (initObj.hasOwnProperty('signal_create_error')) {
        this.signal_create_error = initObj.signal_create_error
      }
      else {
        this.signal_create_error = false;
      }
      if (initObj.hasOwnProperty('shared_mem_write_error')) {
        this.shared_mem_write_error = initObj.shared_mem_write_error
      }
      else {
        this.shared_mem_write_error = false;
      }
      if (initObj.hasOwnProperty('shared_mem_read_error')) {
        this.shared_mem_read_error = initObj.shared_mem_read_error
      }
      else {
        this.shared_mem_read_error = false;
      }
      if (initObj.hasOwnProperty('shared_mem_config_error')) {
        this.shared_mem_config_error = initObj.shared_mem_config_error
      }
      else {
        this.shared_mem_config_error = false;
      }
      if (initObj.hasOwnProperty('share_mem_init_error')) {
        this.share_mem_init_error = initObj.share_mem_init_error
      }
      else {
        this.share_mem_init_error = false;
      }
      if (initObj.hasOwnProperty('ram_test_error')) {
        this.ram_test_error = initObj.ram_test_error
      }
      else {
        this.ram_test_error = false;
      }
      if (initObj.hasOwnProperty('num_errors')) {
        this.num_errors = initObj.num_errors
      }
      else {
        this.num_errors = false;
      }
      if (initObj.hasOwnProperty('mmap_memory_error')) {
        this.mmap_memory_error = initObj.mmap_memory_error
      }
      else {
        this.mmap_memory_error = false;
      }
      if (initObj.hasOwnProperty('isr_attach_error')) {
        this.isr_attach_error = initObj.isr_attach_error
      }
      else {
        this.isr_attach_error = false;
      }
      if (initObj.hasOwnProperty('ipc_drv_write_error')) {
        this.ipc_drv_write_error = initObj.ipc_drv_write_error
      }
      else {
        this.ipc_drv_write_error = false;
      }
      if (initObj.hasOwnProperty('ipc_drv_trigger_error')) {
        this.ipc_drv_trigger_error = initObj.ipc_drv_trigger_error
      }
      else {
        this.ipc_drv_trigger_error = false;
      }
      if (initObj.hasOwnProperty('ipc_drv_sync_error')) {
        this.ipc_drv_sync_error = initObj.ipc_drv_sync_error
      }
      else {
        this.ipc_drv_sync_error = false;
      }
      if (initObj.hasOwnProperty('ipc_drv_read_error')) {
        this.ipc_drv_read_error = initObj.ipc_drv_read_error
      }
      else {
        this.ipc_drv_read_error = false;
      }
      if (initObj.hasOwnProperty('ipc_drv_init_error')) {
        this.ipc_drv_init_error = initObj.ipc_drv_init_error
      }
      else {
        this.ipc_drv_init_error = false;
      }
      if (initObj.hasOwnProperty('interrupt_enable_error')) {
        this.interrupt_enable_error = initObj.interrupt_enable_error
      }
      else {
        this.interrupt_enable_error = false;
      }
      if (initObj.hasOwnProperty('hil_format_error')) {
        this.hil_format_error = initObj.hil_format_error
      }
      else {
        this.hil_format_error = false;
      }
      if (initObj.hasOwnProperty('flash_filesystem_error')) {
        this.flash_filesystem_error = initObj.flash_filesystem_error
      }
      else {
        this.flash_filesystem_error = false;
      }
      if (initObj.hasOwnProperty('error_none')) {
        this.error_none = initObj.error_none
      }
      else {
        this.error_none = false;
      }
      if (initObj.hasOwnProperty('dsp_load_read_error')) {
        this.dsp_load_read_error = initObj.dsp_load_read_error
      }
      else {
        this.dsp_load_read_error = false;
      }
      if (initObj.hasOwnProperty('dsp_load_open_error')) {
        this.dsp_load_open_error = initObj.dsp_load_open_error
      }
      else {
        this.dsp_load_open_error = false;
      }
      if (initObj.hasOwnProperty('dsp_load_address_error')) {
        this.dsp_load_address_error = initObj.dsp_load_address_error
      }
      else {
        this.dsp_load_address_error = false;
      }
      if (initObj.hasOwnProperty('dsp_isp_write_error')) {
        this.dsp_isp_write_error = initObj.dsp_isp_write_error
      }
      else {
        this.dsp_isp_write_error = false;
      }
      if (initObj.hasOwnProperty('dsp_ipc_read_error')) {
        this.dsp_ipc_read_error = initObj.dsp_ipc_read_error
      }
      else {
        this.dsp_ipc_read_error = false;
      }
      if (initObj.hasOwnProperty('dsp_ipc_init')) {
        this.dsp_ipc_init = initObj.dsp_ipc_init
      }
      else {
        this.dsp_ipc_init = false;
      }
      if (initObj.hasOwnProperty('dsp_init_error')) {
        this.dsp_init_error = initObj.dsp_init_error
      }
      else {
        this.dsp_init_error = false;
      }
      if (initObj.hasOwnProperty('dsp_drv_start_error')) {
        this.dsp_drv_start_error = initObj.dsp_drv_start_error
      }
      else {
        this.dsp_drv_start_error = false;
      }
      if (initObj.hasOwnProperty('dsp_drv_load_error')) {
        this.dsp_drv_load_error = initObj.dsp_drv_load_error
      }
      else {
        this.dsp_drv_load_error = false;
      }
      if (initObj.hasOwnProperty('dsp_drv_init_error')) {
        this.dsp_drv_init_error = initObj.dsp_drv_init_error
      }
      else {
        this.dsp_drv_init_error = false;
      }
      if (initObj.hasOwnProperty('dsp_drv_init2_error')) {
        this.dsp_drv_init2_error = initObj.dsp_drv_init2_error
      }
      else {
        this.dsp_drv_init2_error = false;
      }
      if (initObj.hasOwnProperty('dsp_drv_init1_error')) {
        this.dsp_drv_init1_error = initObj.dsp_drv_init1_error
      }
      else {
        this.dsp_drv_init1_error = false;
      }
      if (initObj.hasOwnProperty('dsp_calibration_error')) {
        this.dsp_calibration_error = initObj.dsp_calibration_error
      }
      else {
        this.dsp_calibration_error = false;
      }
      if (initObj.hasOwnProperty('can_xmt_error')) {
        this.can_xmt_error = initObj.can_xmt_error
      }
      else {
        this.can_xmt_error = false;
      }
      if (initObj.hasOwnProperty('can_rcv_error')) {
        this.can_rcv_error = initObj.can_rcv_error
      }
      else {
        this.can_rcv_error = false;
      }
      if (initObj.hasOwnProperty('can_hardware_error')) {
        this.can_hardware_error = initObj.can_hardware_error
      }
      else {
        this.can_hardware_error = false;
      }
      if (initObj.hasOwnProperty('always_true')) {
        this.always_true = initObj.always_true
      }
      else {
        this.always_true = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrDebug3
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [timer_create_error]
    bufferOffset = _serializer.bool(obj.timer_create_error, buffer, bufferOffset);
    // Serialize message field [thread_create_error]
    bufferOffset = _serializer.bool(obj.thread_create_error, buffer, bufferOffset);
    // Serialize message field [arm_calibration_error]
    bufferOffset = _serializer.bool(obj.arm_calibration_error, buffer, bufferOffset);
    // Serialize message field [spi_fee_error]
    bufferOffset = _serializer.bool(obj.spi_fee_error, buffer, bufferOffset);
    // Serialize message field [spi_comm_error]
    bufferOffset = _serializer.bool(obj.spi_comm_error, buffer, bufferOffset);
    // Serialize message field [socket_write_error]
    bufferOffset = _serializer.bool(obj.socket_write_error, buffer, bufferOffset);
    // Serialize message field [dsp_cal_obsolete_62_error]
    bufferOffset = _serializer.bool(obj.dsp_cal_obsolete_62_error, buffer, bufferOffset);
    // Serialize message field [socket_read_error]
    bufferOffset = _serializer.bool(obj.socket_read_error, buffer, bufferOffset);
    // Serialize message field [socket_init_error]
    bufferOffset = _serializer.bool(obj.socket_init_error, buffer, bufferOffset);
    // Serialize message field [signal_wait_error]
    bufferOffset = _serializer.bool(obj.signal_wait_error, buffer, bufferOffset);
    // Serialize message field [signal_send_error]
    bufferOffset = _serializer.bool(obj.signal_send_error, buffer, bufferOffset);
    // Serialize message field [signal_create_error]
    bufferOffset = _serializer.bool(obj.signal_create_error, buffer, bufferOffset);
    // Serialize message field [shared_mem_write_error]
    bufferOffset = _serializer.bool(obj.shared_mem_write_error, buffer, bufferOffset);
    // Serialize message field [shared_mem_read_error]
    bufferOffset = _serializer.bool(obj.shared_mem_read_error, buffer, bufferOffset);
    // Serialize message field [shared_mem_config_error]
    bufferOffset = _serializer.bool(obj.shared_mem_config_error, buffer, bufferOffset);
    // Serialize message field [share_mem_init_error]
    bufferOffset = _serializer.bool(obj.share_mem_init_error, buffer, bufferOffset);
    // Serialize message field [ram_test_error]
    bufferOffset = _serializer.bool(obj.ram_test_error, buffer, bufferOffset);
    // Serialize message field [num_errors]
    bufferOffset = _serializer.bool(obj.num_errors, buffer, bufferOffset);
    // Serialize message field [mmap_memory_error]
    bufferOffset = _serializer.bool(obj.mmap_memory_error, buffer, bufferOffset);
    // Serialize message field [isr_attach_error]
    bufferOffset = _serializer.bool(obj.isr_attach_error, buffer, bufferOffset);
    // Serialize message field [ipc_drv_write_error]
    bufferOffset = _serializer.bool(obj.ipc_drv_write_error, buffer, bufferOffset);
    // Serialize message field [ipc_drv_trigger_error]
    bufferOffset = _serializer.bool(obj.ipc_drv_trigger_error, buffer, bufferOffset);
    // Serialize message field [ipc_drv_sync_error]
    bufferOffset = _serializer.bool(obj.ipc_drv_sync_error, buffer, bufferOffset);
    // Serialize message field [ipc_drv_read_error]
    bufferOffset = _serializer.bool(obj.ipc_drv_read_error, buffer, bufferOffset);
    // Serialize message field [ipc_drv_init_error]
    bufferOffset = _serializer.bool(obj.ipc_drv_init_error, buffer, bufferOffset);
    // Serialize message field [interrupt_enable_error]
    bufferOffset = _serializer.bool(obj.interrupt_enable_error, buffer, bufferOffset);
    // Serialize message field [hil_format_error]
    bufferOffset = _serializer.bool(obj.hil_format_error, buffer, bufferOffset);
    // Serialize message field [flash_filesystem_error]
    bufferOffset = _serializer.bool(obj.flash_filesystem_error, buffer, bufferOffset);
    // Serialize message field [error_none]
    bufferOffset = _serializer.bool(obj.error_none, buffer, bufferOffset);
    // Serialize message field [dsp_load_read_error]
    bufferOffset = _serializer.bool(obj.dsp_load_read_error, buffer, bufferOffset);
    // Serialize message field [dsp_load_open_error]
    bufferOffset = _serializer.bool(obj.dsp_load_open_error, buffer, bufferOffset);
    // Serialize message field [dsp_load_address_error]
    bufferOffset = _serializer.bool(obj.dsp_load_address_error, buffer, bufferOffset);
    // Serialize message field [dsp_isp_write_error]
    bufferOffset = _serializer.bool(obj.dsp_isp_write_error, buffer, bufferOffset);
    // Serialize message field [dsp_ipc_read_error]
    bufferOffset = _serializer.bool(obj.dsp_ipc_read_error, buffer, bufferOffset);
    // Serialize message field [dsp_ipc_init]
    bufferOffset = _serializer.bool(obj.dsp_ipc_init, buffer, bufferOffset);
    // Serialize message field [dsp_init_error]
    bufferOffset = _serializer.bool(obj.dsp_init_error, buffer, bufferOffset);
    // Serialize message field [dsp_drv_start_error]
    bufferOffset = _serializer.bool(obj.dsp_drv_start_error, buffer, bufferOffset);
    // Serialize message field [dsp_drv_load_error]
    bufferOffset = _serializer.bool(obj.dsp_drv_load_error, buffer, bufferOffset);
    // Serialize message field [dsp_drv_init_error]
    bufferOffset = _serializer.bool(obj.dsp_drv_init_error, buffer, bufferOffset);
    // Serialize message field [dsp_drv_init2_error]
    bufferOffset = _serializer.bool(obj.dsp_drv_init2_error, buffer, bufferOffset);
    // Serialize message field [dsp_drv_init1_error]
    bufferOffset = _serializer.bool(obj.dsp_drv_init1_error, buffer, bufferOffset);
    // Serialize message field [dsp_calibration_error]
    bufferOffset = _serializer.bool(obj.dsp_calibration_error, buffer, bufferOffset);
    // Serialize message field [can_xmt_error]
    bufferOffset = _serializer.bool(obj.can_xmt_error, buffer, bufferOffset);
    // Serialize message field [can_rcv_error]
    bufferOffset = _serializer.bool(obj.can_rcv_error, buffer, bufferOffset);
    // Serialize message field [can_hardware_error]
    bufferOffset = _serializer.bool(obj.can_hardware_error, buffer, bufferOffset);
    // Serialize message field [always_true]
    bufferOffset = _serializer.bool(obj.always_true, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrDebug3
    let len;
    let data = new SrrDebug3(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [timer_create_error]
    data.timer_create_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [thread_create_error]
    data.thread_create_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [arm_calibration_error]
    data.arm_calibration_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [spi_fee_error]
    data.spi_fee_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [spi_comm_error]
    data.spi_comm_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [socket_write_error]
    data.socket_write_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_cal_obsolete_62_error]
    data.dsp_cal_obsolete_62_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [socket_read_error]
    data.socket_read_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [socket_init_error]
    data.socket_init_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [signal_wait_error]
    data.signal_wait_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [signal_send_error]
    data.signal_send_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [signal_create_error]
    data.signal_create_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [shared_mem_write_error]
    data.shared_mem_write_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [shared_mem_read_error]
    data.shared_mem_read_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [shared_mem_config_error]
    data.shared_mem_config_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [share_mem_init_error]
    data.share_mem_init_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ram_test_error]
    data.ram_test_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [num_errors]
    data.num_errors = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mmap_memory_error]
    data.mmap_memory_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [isr_attach_error]
    data.isr_attach_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ipc_drv_write_error]
    data.ipc_drv_write_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ipc_drv_trigger_error]
    data.ipc_drv_trigger_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ipc_drv_sync_error]
    data.ipc_drv_sync_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ipc_drv_read_error]
    data.ipc_drv_read_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ipc_drv_init_error]
    data.ipc_drv_init_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [interrupt_enable_error]
    data.interrupt_enable_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hil_format_error]
    data.hil_format_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flash_filesystem_error]
    data.flash_filesystem_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error_none]
    data.error_none = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_load_read_error]
    data.dsp_load_read_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_load_open_error]
    data.dsp_load_open_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_load_address_error]
    data.dsp_load_address_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_isp_write_error]
    data.dsp_isp_write_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_ipc_read_error]
    data.dsp_ipc_read_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_ipc_init]
    data.dsp_ipc_init = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_init_error]
    data.dsp_init_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_drv_start_error]
    data.dsp_drv_start_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_drv_load_error]
    data.dsp_drv_load_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_drv_init_error]
    data.dsp_drv_init_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_drv_init2_error]
    data.dsp_drv_init2_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_drv_init1_error]
    data.dsp_drv_init1_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_calibration_error]
    data.dsp_calibration_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_xmt_error]
    data.can_xmt_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_rcv_error]
    data.can_rcv_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_hardware_error]
    data.can_hardware_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [always_true]
    data.always_true = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 46;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrDebug3';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c0ece44351bdc580e837fa3403929592';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_debug3
    
    std_msgs/Header header
    
    bool      timer_create_error
    bool      thread_create_error
    bool      arm_calibration_error
    bool      spi_fee_error
    bool      spi_comm_error
    bool      socket_write_error
    bool      dsp_cal_obsolete_62_error
    bool      socket_read_error
    bool      socket_init_error
    bool      signal_wait_error
    bool      signal_send_error
    bool      signal_create_error
    bool      shared_mem_write_error
    bool      shared_mem_read_error
    bool      shared_mem_config_error
    bool      share_mem_init_error
    bool      ram_test_error
    bool      num_errors
    bool      mmap_memory_error
    bool      isr_attach_error
    bool      ipc_drv_write_error
    bool      ipc_drv_trigger_error
    bool      ipc_drv_sync_error
    bool      ipc_drv_read_error
    bool      ipc_drv_init_error
    bool      interrupt_enable_error
    bool      hil_format_error
    bool      flash_filesystem_error
    bool      error_none
    bool      dsp_load_read_error
    bool      dsp_load_open_error
    bool      dsp_load_address_error
    bool      dsp_isp_write_error
    bool      dsp_ipc_read_error
    bool      dsp_ipc_init
    bool      dsp_init_error
    bool      dsp_drv_start_error
    bool      dsp_drv_load_error
    bool      dsp_drv_init_error
    bool      dsp_drv_init2_error
    bool      dsp_drv_init1_error
    bool      dsp_calibration_error
    bool      can_xmt_error
    bool      can_rcv_error
    bool      can_hardware_error
    bool      always_true
    
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
    const resolved = new SrrDebug3(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.timer_create_error !== undefined) {
      resolved.timer_create_error = msg.timer_create_error;
    }
    else {
      resolved.timer_create_error = false
    }

    if (msg.thread_create_error !== undefined) {
      resolved.thread_create_error = msg.thread_create_error;
    }
    else {
      resolved.thread_create_error = false
    }

    if (msg.arm_calibration_error !== undefined) {
      resolved.arm_calibration_error = msg.arm_calibration_error;
    }
    else {
      resolved.arm_calibration_error = false
    }

    if (msg.spi_fee_error !== undefined) {
      resolved.spi_fee_error = msg.spi_fee_error;
    }
    else {
      resolved.spi_fee_error = false
    }

    if (msg.spi_comm_error !== undefined) {
      resolved.spi_comm_error = msg.spi_comm_error;
    }
    else {
      resolved.spi_comm_error = false
    }

    if (msg.socket_write_error !== undefined) {
      resolved.socket_write_error = msg.socket_write_error;
    }
    else {
      resolved.socket_write_error = false
    }

    if (msg.dsp_cal_obsolete_62_error !== undefined) {
      resolved.dsp_cal_obsolete_62_error = msg.dsp_cal_obsolete_62_error;
    }
    else {
      resolved.dsp_cal_obsolete_62_error = false
    }

    if (msg.socket_read_error !== undefined) {
      resolved.socket_read_error = msg.socket_read_error;
    }
    else {
      resolved.socket_read_error = false
    }

    if (msg.socket_init_error !== undefined) {
      resolved.socket_init_error = msg.socket_init_error;
    }
    else {
      resolved.socket_init_error = false
    }

    if (msg.signal_wait_error !== undefined) {
      resolved.signal_wait_error = msg.signal_wait_error;
    }
    else {
      resolved.signal_wait_error = false
    }

    if (msg.signal_send_error !== undefined) {
      resolved.signal_send_error = msg.signal_send_error;
    }
    else {
      resolved.signal_send_error = false
    }

    if (msg.signal_create_error !== undefined) {
      resolved.signal_create_error = msg.signal_create_error;
    }
    else {
      resolved.signal_create_error = false
    }

    if (msg.shared_mem_write_error !== undefined) {
      resolved.shared_mem_write_error = msg.shared_mem_write_error;
    }
    else {
      resolved.shared_mem_write_error = false
    }

    if (msg.shared_mem_read_error !== undefined) {
      resolved.shared_mem_read_error = msg.shared_mem_read_error;
    }
    else {
      resolved.shared_mem_read_error = false
    }

    if (msg.shared_mem_config_error !== undefined) {
      resolved.shared_mem_config_error = msg.shared_mem_config_error;
    }
    else {
      resolved.shared_mem_config_error = false
    }

    if (msg.share_mem_init_error !== undefined) {
      resolved.share_mem_init_error = msg.share_mem_init_error;
    }
    else {
      resolved.share_mem_init_error = false
    }

    if (msg.ram_test_error !== undefined) {
      resolved.ram_test_error = msg.ram_test_error;
    }
    else {
      resolved.ram_test_error = false
    }

    if (msg.num_errors !== undefined) {
      resolved.num_errors = msg.num_errors;
    }
    else {
      resolved.num_errors = false
    }

    if (msg.mmap_memory_error !== undefined) {
      resolved.mmap_memory_error = msg.mmap_memory_error;
    }
    else {
      resolved.mmap_memory_error = false
    }

    if (msg.isr_attach_error !== undefined) {
      resolved.isr_attach_error = msg.isr_attach_error;
    }
    else {
      resolved.isr_attach_error = false
    }

    if (msg.ipc_drv_write_error !== undefined) {
      resolved.ipc_drv_write_error = msg.ipc_drv_write_error;
    }
    else {
      resolved.ipc_drv_write_error = false
    }

    if (msg.ipc_drv_trigger_error !== undefined) {
      resolved.ipc_drv_trigger_error = msg.ipc_drv_trigger_error;
    }
    else {
      resolved.ipc_drv_trigger_error = false
    }

    if (msg.ipc_drv_sync_error !== undefined) {
      resolved.ipc_drv_sync_error = msg.ipc_drv_sync_error;
    }
    else {
      resolved.ipc_drv_sync_error = false
    }

    if (msg.ipc_drv_read_error !== undefined) {
      resolved.ipc_drv_read_error = msg.ipc_drv_read_error;
    }
    else {
      resolved.ipc_drv_read_error = false
    }

    if (msg.ipc_drv_init_error !== undefined) {
      resolved.ipc_drv_init_error = msg.ipc_drv_init_error;
    }
    else {
      resolved.ipc_drv_init_error = false
    }

    if (msg.interrupt_enable_error !== undefined) {
      resolved.interrupt_enable_error = msg.interrupt_enable_error;
    }
    else {
      resolved.interrupt_enable_error = false
    }

    if (msg.hil_format_error !== undefined) {
      resolved.hil_format_error = msg.hil_format_error;
    }
    else {
      resolved.hil_format_error = false
    }

    if (msg.flash_filesystem_error !== undefined) {
      resolved.flash_filesystem_error = msg.flash_filesystem_error;
    }
    else {
      resolved.flash_filesystem_error = false
    }

    if (msg.error_none !== undefined) {
      resolved.error_none = msg.error_none;
    }
    else {
      resolved.error_none = false
    }

    if (msg.dsp_load_read_error !== undefined) {
      resolved.dsp_load_read_error = msg.dsp_load_read_error;
    }
    else {
      resolved.dsp_load_read_error = false
    }

    if (msg.dsp_load_open_error !== undefined) {
      resolved.dsp_load_open_error = msg.dsp_load_open_error;
    }
    else {
      resolved.dsp_load_open_error = false
    }

    if (msg.dsp_load_address_error !== undefined) {
      resolved.dsp_load_address_error = msg.dsp_load_address_error;
    }
    else {
      resolved.dsp_load_address_error = false
    }

    if (msg.dsp_isp_write_error !== undefined) {
      resolved.dsp_isp_write_error = msg.dsp_isp_write_error;
    }
    else {
      resolved.dsp_isp_write_error = false
    }

    if (msg.dsp_ipc_read_error !== undefined) {
      resolved.dsp_ipc_read_error = msg.dsp_ipc_read_error;
    }
    else {
      resolved.dsp_ipc_read_error = false
    }

    if (msg.dsp_ipc_init !== undefined) {
      resolved.dsp_ipc_init = msg.dsp_ipc_init;
    }
    else {
      resolved.dsp_ipc_init = false
    }

    if (msg.dsp_init_error !== undefined) {
      resolved.dsp_init_error = msg.dsp_init_error;
    }
    else {
      resolved.dsp_init_error = false
    }

    if (msg.dsp_drv_start_error !== undefined) {
      resolved.dsp_drv_start_error = msg.dsp_drv_start_error;
    }
    else {
      resolved.dsp_drv_start_error = false
    }

    if (msg.dsp_drv_load_error !== undefined) {
      resolved.dsp_drv_load_error = msg.dsp_drv_load_error;
    }
    else {
      resolved.dsp_drv_load_error = false
    }

    if (msg.dsp_drv_init_error !== undefined) {
      resolved.dsp_drv_init_error = msg.dsp_drv_init_error;
    }
    else {
      resolved.dsp_drv_init_error = false
    }

    if (msg.dsp_drv_init2_error !== undefined) {
      resolved.dsp_drv_init2_error = msg.dsp_drv_init2_error;
    }
    else {
      resolved.dsp_drv_init2_error = false
    }

    if (msg.dsp_drv_init1_error !== undefined) {
      resolved.dsp_drv_init1_error = msg.dsp_drv_init1_error;
    }
    else {
      resolved.dsp_drv_init1_error = false
    }

    if (msg.dsp_calibration_error !== undefined) {
      resolved.dsp_calibration_error = msg.dsp_calibration_error;
    }
    else {
      resolved.dsp_calibration_error = false
    }

    if (msg.can_xmt_error !== undefined) {
      resolved.can_xmt_error = msg.can_xmt_error;
    }
    else {
      resolved.can_xmt_error = false
    }

    if (msg.can_rcv_error !== undefined) {
      resolved.can_rcv_error = msg.can_rcv_error;
    }
    else {
      resolved.can_rcv_error = false
    }

    if (msg.can_hardware_error !== undefined) {
      resolved.can_hardware_error = msg.can_hardware_error;
    }
    else {
      resolved.can_hardware_error = false
    }

    if (msg.always_true !== undefined) {
      resolved.always_true = msg.always_true;
    }
    else {
      resolved.always_true = false
    }

    return resolved;
    }
};

module.exports = SrrDebug3;
