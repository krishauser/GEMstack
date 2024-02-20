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

class NovatelReceiverStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.original_status_code = null;
      this.error_flag = null;
      this.temperature_flag = null;
      this.voltage_supply_flag = null;
      this.antenna_powered = null;
      this.antenna_is_open = null;
      this.antenna_is_shorted = null;
      this.cpu_overload_flag = null;
      this.com1_buffer_overrun = null;
      this.com2_buffer_overrun = null;
      this.com3_buffer_overrun = null;
      this.usb_buffer_overrun = null;
      this.rf1_agc_flag = null;
      this.rf2_agc_flag = null;
      this.almanac_flag = null;
      this.position_solution_flag = null;
      this.position_fixed_flag = null;
      this.clock_steering_status_enabled = null;
      this.clock_model_flag = null;
      this.oemv_external_oscillator_flag = null;
      this.software_resource_flag = null;
      this.aux1_status_event_flag = null;
      this.aux2_status_event_flag = null;
      this.aux3_status_event_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('original_status_code')) {
        this.original_status_code = initObj.original_status_code
      }
      else {
        this.original_status_code = 0;
      }
      if (initObj.hasOwnProperty('error_flag')) {
        this.error_flag = initObj.error_flag
      }
      else {
        this.error_flag = false;
      }
      if (initObj.hasOwnProperty('temperature_flag')) {
        this.temperature_flag = initObj.temperature_flag
      }
      else {
        this.temperature_flag = false;
      }
      if (initObj.hasOwnProperty('voltage_supply_flag')) {
        this.voltage_supply_flag = initObj.voltage_supply_flag
      }
      else {
        this.voltage_supply_flag = false;
      }
      if (initObj.hasOwnProperty('antenna_powered')) {
        this.antenna_powered = initObj.antenna_powered
      }
      else {
        this.antenna_powered = false;
      }
      if (initObj.hasOwnProperty('antenna_is_open')) {
        this.antenna_is_open = initObj.antenna_is_open
      }
      else {
        this.antenna_is_open = false;
      }
      if (initObj.hasOwnProperty('antenna_is_shorted')) {
        this.antenna_is_shorted = initObj.antenna_is_shorted
      }
      else {
        this.antenna_is_shorted = false;
      }
      if (initObj.hasOwnProperty('cpu_overload_flag')) {
        this.cpu_overload_flag = initObj.cpu_overload_flag
      }
      else {
        this.cpu_overload_flag = false;
      }
      if (initObj.hasOwnProperty('com1_buffer_overrun')) {
        this.com1_buffer_overrun = initObj.com1_buffer_overrun
      }
      else {
        this.com1_buffer_overrun = false;
      }
      if (initObj.hasOwnProperty('com2_buffer_overrun')) {
        this.com2_buffer_overrun = initObj.com2_buffer_overrun
      }
      else {
        this.com2_buffer_overrun = false;
      }
      if (initObj.hasOwnProperty('com3_buffer_overrun')) {
        this.com3_buffer_overrun = initObj.com3_buffer_overrun
      }
      else {
        this.com3_buffer_overrun = false;
      }
      if (initObj.hasOwnProperty('usb_buffer_overrun')) {
        this.usb_buffer_overrun = initObj.usb_buffer_overrun
      }
      else {
        this.usb_buffer_overrun = false;
      }
      if (initObj.hasOwnProperty('rf1_agc_flag')) {
        this.rf1_agc_flag = initObj.rf1_agc_flag
      }
      else {
        this.rf1_agc_flag = false;
      }
      if (initObj.hasOwnProperty('rf2_agc_flag')) {
        this.rf2_agc_flag = initObj.rf2_agc_flag
      }
      else {
        this.rf2_agc_flag = false;
      }
      if (initObj.hasOwnProperty('almanac_flag')) {
        this.almanac_flag = initObj.almanac_flag
      }
      else {
        this.almanac_flag = false;
      }
      if (initObj.hasOwnProperty('position_solution_flag')) {
        this.position_solution_flag = initObj.position_solution_flag
      }
      else {
        this.position_solution_flag = false;
      }
      if (initObj.hasOwnProperty('position_fixed_flag')) {
        this.position_fixed_flag = initObj.position_fixed_flag
      }
      else {
        this.position_fixed_flag = false;
      }
      if (initObj.hasOwnProperty('clock_steering_status_enabled')) {
        this.clock_steering_status_enabled = initObj.clock_steering_status_enabled
      }
      else {
        this.clock_steering_status_enabled = false;
      }
      if (initObj.hasOwnProperty('clock_model_flag')) {
        this.clock_model_flag = initObj.clock_model_flag
      }
      else {
        this.clock_model_flag = false;
      }
      if (initObj.hasOwnProperty('oemv_external_oscillator_flag')) {
        this.oemv_external_oscillator_flag = initObj.oemv_external_oscillator_flag
      }
      else {
        this.oemv_external_oscillator_flag = false;
      }
      if (initObj.hasOwnProperty('software_resource_flag')) {
        this.software_resource_flag = initObj.software_resource_flag
      }
      else {
        this.software_resource_flag = false;
      }
      if (initObj.hasOwnProperty('aux1_status_event_flag')) {
        this.aux1_status_event_flag = initObj.aux1_status_event_flag
      }
      else {
        this.aux1_status_event_flag = false;
      }
      if (initObj.hasOwnProperty('aux2_status_event_flag')) {
        this.aux2_status_event_flag = initObj.aux2_status_event_flag
      }
      else {
        this.aux2_status_event_flag = false;
      }
      if (initObj.hasOwnProperty('aux3_status_event_flag')) {
        this.aux3_status_event_flag = initObj.aux3_status_event_flag
      }
      else {
        this.aux3_status_event_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NovatelReceiverStatus
    // Serialize message field [original_status_code]
    bufferOffset = _serializer.uint32(obj.original_status_code, buffer, bufferOffset);
    // Serialize message field [error_flag]
    bufferOffset = _serializer.bool(obj.error_flag, buffer, bufferOffset);
    // Serialize message field [temperature_flag]
    bufferOffset = _serializer.bool(obj.temperature_flag, buffer, bufferOffset);
    // Serialize message field [voltage_supply_flag]
    bufferOffset = _serializer.bool(obj.voltage_supply_flag, buffer, bufferOffset);
    // Serialize message field [antenna_powered]
    bufferOffset = _serializer.bool(obj.antenna_powered, buffer, bufferOffset);
    // Serialize message field [antenna_is_open]
    bufferOffset = _serializer.bool(obj.antenna_is_open, buffer, bufferOffset);
    // Serialize message field [antenna_is_shorted]
    bufferOffset = _serializer.bool(obj.antenna_is_shorted, buffer, bufferOffset);
    // Serialize message field [cpu_overload_flag]
    bufferOffset = _serializer.bool(obj.cpu_overload_flag, buffer, bufferOffset);
    // Serialize message field [com1_buffer_overrun]
    bufferOffset = _serializer.bool(obj.com1_buffer_overrun, buffer, bufferOffset);
    // Serialize message field [com2_buffer_overrun]
    bufferOffset = _serializer.bool(obj.com2_buffer_overrun, buffer, bufferOffset);
    // Serialize message field [com3_buffer_overrun]
    bufferOffset = _serializer.bool(obj.com3_buffer_overrun, buffer, bufferOffset);
    // Serialize message field [usb_buffer_overrun]
    bufferOffset = _serializer.bool(obj.usb_buffer_overrun, buffer, bufferOffset);
    // Serialize message field [rf1_agc_flag]
    bufferOffset = _serializer.bool(obj.rf1_agc_flag, buffer, bufferOffset);
    // Serialize message field [rf2_agc_flag]
    bufferOffset = _serializer.bool(obj.rf2_agc_flag, buffer, bufferOffset);
    // Serialize message field [almanac_flag]
    bufferOffset = _serializer.bool(obj.almanac_flag, buffer, bufferOffset);
    // Serialize message field [position_solution_flag]
    bufferOffset = _serializer.bool(obj.position_solution_flag, buffer, bufferOffset);
    // Serialize message field [position_fixed_flag]
    bufferOffset = _serializer.bool(obj.position_fixed_flag, buffer, bufferOffset);
    // Serialize message field [clock_steering_status_enabled]
    bufferOffset = _serializer.bool(obj.clock_steering_status_enabled, buffer, bufferOffset);
    // Serialize message field [clock_model_flag]
    bufferOffset = _serializer.bool(obj.clock_model_flag, buffer, bufferOffset);
    // Serialize message field [oemv_external_oscillator_flag]
    bufferOffset = _serializer.bool(obj.oemv_external_oscillator_flag, buffer, bufferOffset);
    // Serialize message field [software_resource_flag]
    bufferOffset = _serializer.bool(obj.software_resource_flag, buffer, bufferOffset);
    // Serialize message field [aux1_status_event_flag]
    bufferOffset = _serializer.bool(obj.aux1_status_event_flag, buffer, bufferOffset);
    // Serialize message field [aux2_status_event_flag]
    bufferOffset = _serializer.bool(obj.aux2_status_event_flag, buffer, bufferOffset);
    // Serialize message field [aux3_status_event_flag]
    bufferOffset = _serializer.bool(obj.aux3_status_event_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NovatelReceiverStatus
    let len;
    let data = new NovatelReceiverStatus(null);
    // Deserialize message field [original_status_code]
    data.original_status_code = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [error_flag]
    data.error_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [temperature_flag]
    data.temperature_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [voltage_supply_flag]
    data.voltage_supply_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [antenna_powered]
    data.antenna_powered = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [antenna_is_open]
    data.antenna_is_open = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [antenna_is_shorted]
    data.antenna_is_shorted = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cpu_overload_flag]
    data.cpu_overload_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [com1_buffer_overrun]
    data.com1_buffer_overrun = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [com2_buffer_overrun]
    data.com2_buffer_overrun = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [com3_buffer_overrun]
    data.com3_buffer_overrun = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [usb_buffer_overrun]
    data.usb_buffer_overrun = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rf1_agc_flag]
    data.rf1_agc_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rf2_agc_flag]
    data.rf2_agc_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [almanac_flag]
    data.almanac_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [position_solution_flag]
    data.position_solution_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [position_fixed_flag]
    data.position_fixed_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clock_steering_status_enabled]
    data.clock_steering_status_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clock_model_flag]
    data.clock_model_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [oemv_external_oscillator_flag]
    data.oemv_external_oscillator_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [software_resource_flag]
    data.software_resource_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [aux1_status_event_flag]
    data.aux1_status_event_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [aux2_status_event_flag]
    data.aux2_status_event_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [aux3_status_event_flag]
    data.aux3_status_event_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 27;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/NovatelReceiverStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf2774401808a6dde392e2ebdb09ca15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # From the original Novatel receiver status message bitfield
    #  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)
    #  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)
    #  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)
    #  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)
    #  4     0x00000010   <Reserved>
    #  5     0x00000020   Antenna open flag (0: OK, 1: Open)
    #  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)
    #  7     0x00000080   CPU overload flag
    #  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)
    #  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)
    #  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)
    #  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)
    #  12    0x00001000   <Reserved>
    #  13    0x00002000   <Reserved>
    #  14    0x00004000   <Reserved>
    #  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)
    #  16    0x00010000   <Reserverd>
    #  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)
    #  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)
    #  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)
    #  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)
    #  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)
    #  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)
    #  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)
    #  24    0x01000000   Software resource (0: OK, 1: Warning)
    #  25    0x02000000   <Reserved>
    #  26    0x04000000   <Reserved>
    #  27    0x08000000   <Reserved>
    #  28    0x10000000   <Reserved>
    #  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)
    #  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)
    #  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)
    uint32 original_status_code
    bool error_flag
    bool temperature_flag
    bool voltage_supply_flag
    bool antenna_powered
    bool antenna_is_open
    bool antenna_is_shorted
    bool cpu_overload_flag
    bool com1_buffer_overrun
    bool com2_buffer_overrun
    bool com3_buffer_overrun
    bool usb_buffer_overrun
    bool rf1_agc_flag
    bool rf2_agc_flag
    bool almanac_flag
    bool position_solution_flag
    bool position_fixed_flag
    bool clock_steering_status_enabled
    bool clock_model_flag
    bool oemv_external_oscillator_flag
    bool software_resource_flag
    bool aux1_status_event_flag
    bool aux2_status_event_flag
    bool aux3_status_event_flag
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NovatelReceiverStatus(null);
    if (msg.original_status_code !== undefined) {
      resolved.original_status_code = msg.original_status_code;
    }
    else {
      resolved.original_status_code = 0
    }

    if (msg.error_flag !== undefined) {
      resolved.error_flag = msg.error_flag;
    }
    else {
      resolved.error_flag = false
    }

    if (msg.temperature_flag !== undefined) {
      resolved.temperature_flag = msg.temperature_flag;
    }
    else {
      resolved.temperature_flag = false
    }

    if (msg.voltage_supply_flag !== undefined) {
      resolved.voltage_supply_flag = msg.voltage_supply_flag;
    }
    else {
      resolved.voltage_supply_flag = false
    }

    if (msg.antenna_powered !== undefined) {
      resolved.antenna_powered = msg.antenna_powered;
    }
    else {
      resolved.antenna_powered = false
    }

    if (msg.antenna_is_open !== undefined) {
      resolved.antenna_is_open = msg.antenna_is_open;
    }
    else {
      resolved.antenna_is_open = false
    }

    if (msg.antenna_is_shorted !== undefined) {
      resolved.antenna_is_shorted = msg.antenna_is_shorted;
    }
    else {
      resolved.antenna_is_shorted = false
    }

    if (msg.cpu_overload_flag !== undefined) {
      resolved.cpu_overload_flag = msg.cpu_overload_flag;
    }
    else {
      resolved.cpu_overload_flag = false
    }

    if (msg.com1_buffer_overrun !== undefined) {
      resolved.com1_buffer_overrun = msg.com1_buffer_overrun;
    }
    else {
      resolved.com1_buffer_overrun = false
    }

    if (msg.com2_buffer_overrun !== undefined) {
      resolved.com2_buffer_overrun = msg.com2_buffer_overrun;
    }
    else {
      resolved.com2_buffer_overrun = false
    }

    if (msg.com3_buffer_overrun !== undefined) {
      resolved.com3_buffer_overrun = msg.com3_buffer_overrun;
    }
    else {
      resolved.com3_buffer_overrun = false
    }

    if (msg.usb_buffer_overrun !== undefined) {
      resolved.usb_buffer_overrun = msg.usb_buffer_overrun;
    }
    else {
      resolved.usb_buffer_overrun = false
    }

    if (msg.rf1_agc_flag !== undefined) {
      resolved.rf1_agc_flag = msg.rf1_agc_flag;
    }
    else {
      resolved.rf1_agc_flag = false
    }

    if (msg.rf2_agc_flag !== undefined) {
      resolved.rf2_agc_flag = msg.rf2_agc_flag;
    }
    else {
      resolved.rf2_agc_flag = false
    }

    if (msg.almanac_flag !== undefined) {
      resolved.almanac_flag = msg.almanac_flag;
    }
    else {
      resolved.almanac_flag = false
    }

    if (msg.position_solution_flag !== undefined) {
      resolved.position_solution_flag = msg.position_solution_flag;
    }
    else {
      resolved.position_solution_flag = false
    }

    if (msg.position_fixed_flag !== undefined) {
      resolved.position_fixed_flag = msg.position_fixed_flag;
    }
    else {
      resolved.position_fixed_flag = false
    }

    if (msg.clock_steering_status_enabled !== undefined) {
      resolved.clock_steering_status_enabled = msg.clock_steering_status_enabled;
    }
    else {
      resolved.clock_steering_status_enabled = false
    }

    if (msg.clock_model_flag !== undefined) {
      resolved.clock_model_flag = msg.clock_model_flag;
    }
    else {
      resolved.clock_model_flag = false
    }

    if (msg.oemv_external_oscillator_flag !== undefined) {
      resolved.oemv_external_oscillator_flag = msg.oemv_external_oscillator_flag;
    }
    else {
      resolved.oemv_external_oscillator_flag = false
    }

    if (msg.software_resource_flag !== undefined) {
      resolved.software_resource_flag = msg.software_resource_flag;
    }
    else {
      resolved.software_resource_flag = false
    }

    if (msg.aux1_status_event_flag !== undefined) {
      resolved.aux1_status_event_flag = msg.aux1_status_event_flag;
    }
    else {
      resolved.aux1_status_event_flag = false
    }

    if (msg.aux2_status_event_flag !== undefined) {
      resolved.aux2_status_event_flag = msg.aux2_status_event_flag;
    }
    else {
      resolved.aux2_status_event_flag = false
    }

    if (msg.aux3_status_event_flag !== undefined) {
      resolved.aux3_status_event_flag = msg.aux3_status_event_flag;
    }
    else {
      resolved.aux3_status_event_flag = false
    }

    return resolved;
    }
};

module.exports = NovatelReceiverStatus;
