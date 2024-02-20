// Auto-generated. Do not edit!

// (in-package novatel_gps_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NovatelReceiverStatus = require('./NovatelReceiverStatus.js');

//-----------------------------------------------------------

class NovatelMessageHeader {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.message_name = null;
      this.port = null;
      this.sequence_num = null;
      this.percent_idle_time = null;
      this.gps_time_status = null;
      this.gps_week_num = null;
      this.gps_seconds = null;
      this.receiver_status = null;
      this.receiver_software_version = null;
    }
    else {
      if (initObj.hasOwnProperty('message_name')) {
        this.message_name = initObj.message_name
      }
      else {
        this.message_name = '';
      }
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = '';
      }
      if (initObj.hasOwnProperty('sequence_num')) {
        this.sequence_num = initObj.sequence_num
      }
      else {
        this.sequence_num = 0;
      }
      if (initObj.hasOwnProperty('percent_idle_time')) {
        this.percent_idle_time = initObj.percent_idle_time
      }
      else {
        this.percent_idle_time = 0.0;
      }
      if (initObj.hasOwnProperty('gps_time_status')) {
        this.gps_time_status = initObj.gps_time_status
      }
      else {
        this.gps_time_status = '';
      }
      if (initObj.hasOwnProperty('gps_week_num')) {
        this.gps_week_num = initObj.gps_week_num
      }
      else {
        this.gps_week_num = 0;
      }
      if (initObj.hasOwnProperty('gps_seconds')) {
        this.gps_seconds = initObj.gps_seconds
      }
      else {
        this.gps_seconds = 0.0;
      }
      if (initObj.hasOwnProperty('receiver_status')) {
        this.receiver_status = initObj.receiver_status
      }
      else {
        this.receiver_status = new NovatelReceiverStatus();
      }
      if (initObj.hasOwnProperty('receiver_software_version')) {
        this.receiver_software_version = initObj.receiver_software_version
      }
      else {
        this.receiver_software_version = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NovatelMessageHeader
    // Serialize message field [message_name]
    bufferOffset = _serializer.string(obj.message_name, buffer, bufferOffset);
    // Serialize message field [port]
    bufferOffset = _serializer.string(obj.port, buffer, bufferOffset);
    // Serialize message field [sequence_num]
    bufferOffset = _serializer.uint32(obj.sequence_num, buffer, bufferOffset);
    // Serialize message field [percent_idle_time]
    bufferOffset = _serializer.float32(obj.percent_idle_time, buffer, bufferOffset);
    // Serialize message field [gps_time_status]
    bufferOffset = _serializer.string(obj.gps_time_status, buffer, bufferOffset);
    // Serialize message field [gps_week_num]
    bufferOffset = _serializer.uint32(obj.gps_week_num, buffer, bufferOffset);
    // Serialize message field [gps_seconds]
    bufferOffset = _serializer.float64(obj.gps_seconds, buffer, bufferOffset);
    // Serialize message field [receiver_status]
    bufferOffset = NovatelReceiverStatus.serialize(obj.receiver_status, buffer, bufferOffset);
    // Serialize message field [receiver_software_version]
    bufferOffset = _serializer.uint32(obj.receiver_software_version, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NovatelMessageHeader
    let len;
    let data = new NovatelMessageHeader(null);
    // Deserialize message field [message_name]
    data.message_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [port]
    data.port = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sequence_num]
    data.sequence_num = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [percent_idle_time]
    data.percent_idle_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gps_time_status]
    data.gps_time_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [gps_week_num]
    data.gps_week_num = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gps_seconds]
    data.gps_seconds = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [receiver_status]
    data.receiver_status = NovatelReceiverStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [receiver_software_version]
    data.receiver_software_version = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message_name);
    length += _getByteLength(object.port);
    length += _getByteLength(object.gps_time_status);
    return length + 63;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/NovatelMessageHeader';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43b536606c527a56309297282bb7adef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Novatel Message Header
    
    string message_name
    string port
    uint32 sequence_num
    float32 percent_idle_time
    string gps_time_status
    uint32 gps_week_num
    float64 gps_seconds
    
    # Bit       Mask      Description
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
    NovatelReceiverStatus receiver_status
    
    # Receiver build number (0-65535)
    uint32 receiver_software_version
    ================================================================================
    MSG: novatel_gps_msgs/NovatelReceiverStatus
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
    const resolved = new NovatelMessageHeader(null);
    if (msg.message_name !== undefined) {
      resolved.message_name = msg.message_name;
    }
    else {
      resolved.message_name = ''
    }

    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = ''
    }

    if (msg.sequence_num !== undefined) {
      resolved.sequence_num = msg.sequence_num;
    }
    else {
      resolved.sequence_num = 0
    }

    if (msg.percent_idle_time !== undefined) {
      resolved.percent_idle_time = msg.percent_idle_time;
    }
    else {
      resolved.percent_idle_time = 0.0
    }

    if (msg.gps_time_status !== undefined) {
      resolved.gps_time_status = msg.gps_time_status;
    }
    else {
      resolved.gps_time_status = ''
    }

    if (msg.gps_week_num !== undefined) {
      resolved.gps_week_num = msg.gps_week_num;
    }
    else {
      resolved.gps_week_num = 0
    }

    if (msg.gps_seconds !== undefined) {
      resolved.gps_seconds = msg.gps_seconds;
    }
    else {
      resolved.gps_seconds = 0.0
    }

    if (msg.receiver_status !== undefined) {
      resolved.receiver_status = NovatelReceiverStatus.Resolve(msg.receiver_status)
    }
    else {
      resolved.receiver_status = new NovatelReceiverStatus()
    }

    if (msg.receiver_software_version !== undefined) {
      resolved.receiver_software_version = msg.receiver_software_version;
    }
    else {
      resolved.receiver_software_version = 0
    }

    return resolved;
    }
};

module.exports = NovatelMessageHeader;
