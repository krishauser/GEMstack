// Auto-generated. Do not edit!

// (in-package novatel_gps_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NovatelMessageHeader = require('./NovatelMessageHeader.js');
let NovatelExtendedSolutionStatus = require('./NovatelExtendedSolutionStatus.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Insstdev {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.novatel_msg_header = null;
      this.latitude_dev = null;
      this.longitude_dev = null;
      this.height_dev = null;
      this.north_velocity_dev = null;
      this.east_velocity_dev = null;
      this.up_velocity_dev = null;
      this.roll_dev = null;
      this.pitch_dev = null;
      this.azimuth_dev = null;
      this.extended_solution_status = null;
      this.time_since_update = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('novatel_msg_header')) {
        this.novatel_msg_header = initObj.novatel_msg_header
      }
      else {
        this.novatel_msg_header = new NovatelMessageHeader();
      }
      if (initObj.hasOwnProperty('latitude_dev')) {
        this.latitude_dev = initObj.latitude_dev
      }
      else {
        this.latitude_dev = 0.0;
      }
      if (initObj.hasOwnProperty('longitude_dev')) {
        this.longitude_dev = initObj.longitude_dev
      }
      else {
        this.longitude_dev = 0.0;
      }
      if (initObj.hasOwnProperty('height_dev')) {
        this.height_dev = initObj.height_dev
      }
      else {
        this.height_dev = 0.0;
      }
      if (initObj.hasOwnProperty('north_velocity_dev')) {
        this.north_velocity_dev = initObj.north_velocity_dev
      }
      else {
        this.north_velocity_dev = 0.0;
      }
      if (initObj.hasOwnProperty('east_velocity_dev')) {
        this.east_velocity_dev = initObj.east_velocity_dev
      }
      else {
        this.east_velocity_dev = 0.0;
      }
      if (initObj.hasOwnProperty('up_velocity_dev')) {
        this.up_velocity_dev = initObj.up_velocity_dev
      }
      else {
        this.up_velocity_dev = 0.0;
      }
      if (initObj.hasOwnProperty('roll_dev')) {
        this.roll_dev = initObj.roll_dev
      }
      else {
        this.roll_dev = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_dev')) {
        this.pitch_dev = initObj.pitch_dev
      }
      else {
        this.pitch_dev = 0.0;
      }
      if (initObj.hasOwnProperty('azimuth_dev')) {
        this.azimuth_dev = initObj.azimuth_dev
      }
      else {
        this.azimuth_dev = 0.0;
      }
      if (initObj.hasOwnProperty('extended_solution_status')) {
        this.extended_solution_status = initObj.extended_solution_status
      }
      else {
        this.extended_solution_status = new NovatelExtendedSolutionStatus();
      }
      if (initObj.hasOwnProperty('time_since_update')) {
        this.time_since_update = initObj.time_since_update
      }
      else {
        this.time_since_update = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Insstdev
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [novatel_msg_header]
    bufferOffset = NovatelMessageHeader.serialize(obj.novatel_msg_header, buffer, bufferOffset);
    // Serialize message field [latitude_dev]
    bufferOffset = _serializer.float32(obj.latitude_dev, buffer, bufferOffset);
    // Serialize message field [longitude_dev]
    bufferOffset = _serializer.float32(obj.longitude_dev, buffer, bufferOffset);
    // Serialize message field [height_dev]
    bufferOffset = _serializer.float32(obj.height_dev, buffer, bufferOffset);
    // Serialize message field [north_velocity_dev]
    bufferOffset = _serializer.float32(obj.north_velocity_dev, buffer, bufferOffset);
    // Serialize message field [east_velocity_dev]
    bufferOffset = _serializer.float32(obj.east_velocity_dev, buffer, bufferOffset);
    // Serialize message field [up_velocity_dev]
    bufferOffset = _serializer.float32(obj.up_velocity_dev, buffer, bufferOffset);
    // Serialize message field [roll_dev]
    bufferOffset = _serializer.float32(obj.roll_dev, buffer, bufferOffset);
    // Serialize message field [pitch_dev]
    bufferOffset = _serializer.float32(obj.pitch_dev, buffer, bufferOffset);
    // Serialize message field [azimuth_dev]
    bufferOffset = _serializer.float32(obj.azimuth_dev, buffer, bufferOffset);
    // Serialize message field [extended_solution_status]
    bufferOffset = NovatelExtendedSolutionStatus.serialize(obj.extended_solution_status, buffer, bufferOffset);
    // Serialize message field [time_since_update]
    bufferOffset = _serializer.uint16(obj.time_since_update, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Insstdev
    let len;
    let data = new Insstdev(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [novatel_msg_header]
    data.novatel_msg_header = NovatelMessageHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [latitude_dev]
    data.latitude_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude_dev]
    data.longitude_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [height_dev]
    data.height_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [north_velocity_dev]
    data.north_velocity_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [east_velocity_dev]
    data.east_velocity_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [up_velocity_dev]
    data.up_velocity_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_dev]
    data.roll_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_dev]
    data.pitch_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [azimuth_dev]
    data.azimuth_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [extended_solution_status]
    data.extended_solution_status = NovatelExtendedSolutionStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [time_since_update]
    data.time_since_update = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += NovatelMessageHeader.getMessageSize(object.novatel_msg_header);
    length += NovatelExtendedSolutionStatus.getMessageSize(object.extended_solution_status);
    return length + 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Insstdev';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5a3ffc9969b49cd107b55c9843133d1c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # INS PVA standard deviations
    
    Header header
    
    NovatelMessageHeader novatel_msg_header
    
    float32 latitude_dev
    float32 longitude_dev
    float32 height_dev
    float32 north_velocity_dev
    float32 east_velocity_dev
    float32 up_velocity_dev
    float32 roll_dev
    float32 pitch_dev
    float32 azimuth_dev
    NovatelExtendedSolutionStatus extended_solution_status
    uint16 time_since_update
    
    
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
    
    ================================================================================
    MSG: novatel_gps_msgs/NovatelMessageHeader
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
    
    
    ================================================================================
    MSG: novatel_gps_msgs/NovatelExtendedSolutionStatus
    # Bit    Mask      Description
    #  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)
    # 1-3    0x0E      Pseudorange Ionosphere Correction
    #                    0 = unknown
    #                    1 = Klobuchar Broadcast
    #                    2 = SBAS Broadcast
    #                    3 = Multi-frequency Computed
    #                    4 = PSRDiff Correction
    #                    5 = NovaTel Blended Ionosphere Value
    # 4-7  0xF0        <Reserved>
    uint32 original_mask
    bool advance_rtk_verified
    string psuedorange_iono_correction
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Insstdev(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.novatel_msg_header !== undefined) {
      resolved.novatel_msg_header = NovatelMessageHeader.Resolve(msg.novatel_msg_header)
    }
    else {
      resolved.novatel_msg_header = new NovatelMessageHeader()
    }

    if (msg.latitude_dev !== undefined) {
      resolved.latitude_dev = msg.latitude_dev;
    }
    else {
      resolved.latitude_dev = 0.0
    }

    if (msg.longitude_dev !== undefined) {
      resolved.longitude_dev = msg.longitude_dev;
    }
    else {
      resolved.longitude_dev = 0.0
    }

    if (msg.height_dev !== undefined) {
      resolved.height_dev = msg.height_dev;
    }
    else {
      resolved.height_dev = 0.0
    }

    if (msg.north_velocity_dev !== undefined) {
      resolved.north_velocity_dev = msg.north_velocity_dev;
    }
    else {
      resolved.north_velocity_dev = 0.0
    }

    if (msg.east_velocity_dev !== undefined) {
      resolved.east_velocity_dev = msg.east_velocity_dev;
    }
    else {
      resolved.east_velocity_dev = 0.0
    }

    if (msg.up_velocity_dev !== undefined) {
      resolved.up_velocity_dev = msg.up_velocity_dev;
    }
    else {
      resolved.up_velocity_dev = 0.0
    }

    if (msg.roll_dev !== undefined) {
      resolved.roll_dev = msg.roll_dev;
    }
    else {
      resolved.roll_dev = 0.0
    }

    if (msg.pitch_dev !== undefined) {
      resolved.pitch_dev = msg.pitch_dev;
    }
    else {
      resolved.pitch_dev = 0.0
    }

    if (msg.azimuth_dev !== undefined) {
      resolved.azimuth_dev = msg.azimuth_dev;
    }
    else {
      resolved.azimuth_dev = 0.0
    }

    if (msg.extended_solution_status !== undefined) {
      resolved.extended_solution_status = NovatelExtendedSolutionStatus.Resolve(msg.extended_solution_status)
    }
    else {
      resolved.extended_solution_status = new NovatelExtendedSolutionStatus()
    }

    if (msg.time_since_update !== undefined) {
      resolved.time_since_update = msg.time_since_update;
    }
    else {
      resolved.time_since_update = 0
    }

    return resolved;
    }
};

module.exports = Insstdev;
