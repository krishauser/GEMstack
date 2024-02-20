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
let NovatelSignalMask = require('./NovatelSignalMask.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class NovatelXYZ {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.novatel_msg_header = null;
      this.solution_status = null;
      this.position_type = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.x_sigma = null;
      this.y_sigma = null;
      this.z_sigma = null;
      this.velocity_solution_status = null;
      this.velocity_type = null;
      this.x_vel = null;
      this.y_vel = null;
      this.z_vel = null;
      this.x_vel_sigma = null;
      this.y_vel_sigma = null;
      this.z_vel_sigma = null;
      this.base_station_id = null;
      this.velocity_latency = null;
      this.diff_age = null;
      this.solution_age = null;
      this.num_satellites_tracked = null;
      this.num_satellites_used_in_solution = null;
      this.num_gps_and_glonass_l1_used_in_solution = null;
      this.num_gps_and_glonass_l1_and_l2_used_in_solution = null;
      this.extended_solution_status = null;
      this.signal_mask = null;
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
      if (initObj.hasOwnProperty('solution_status')) {
        this.solution_status = initObj.solution_status
      }
      else {
        this.solution_status = '';
      }
      if (initObj.hasOwnProperty('position_type')) {
        this.position_type = initObj.position_type
      }
      else {
        this.position_type = '';
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('x_sigma')) {
        this.x_sigma = initObj.x_sigma
      }
      else {
        this.x_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('y_sigma')) {
        this.y_sigma = initObj.y_sigma
      }
      else {
        this.y_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('z_sigma')) {
        this.z_sigma = initObj.z_sigma
      }
      else {
        this.z_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_solution_status')) {
        this.velocity_solution_status = initObj.velocity_solution_status
      }
      else {
        this.velocity_solution_status = '';
      }
      if (initObj.hasOwnProperty('velocity_type')) {
        this.velocity_type = initObj.velocity_type
      }
      else {
        this.velocity_type = '';
      }
      if (initObj.hasOwnProperty('x_vel')) {
        this.x_vel = initObj.x_vel
      }
      else {
        this.x_vel = 0.0;
      }
      if (initObj.hasOwnProperty('y_vel')) {
        this.y_vel = initObj.y_vel
      }
      else {
        this.y_vel = 0.0;
      }
      if (initObj.hasOwnProperty('z_vel')) {
        this.z_vel = initObj.z_vel
      }
      else {
        this.z_vel = 0.0;
      }
      if (initObj.hasOwnProperty('x_vel_sigma')) {
        this.x_vel_sigma = initObj.x_vel_sigma
      }
      else {
        this.x_vel_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('y_vel_sigma')) {
        this.y_vel_sigma = initObj.y_vel_sigma
      }
      else {
        this.y_vel_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('z_vel_sigma')) {
        this.z_vel_sigma = initObj.z_vel_sigma
      }
      else {
        this.z_vel_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('base_station_id')) {
        this.base_station_id = initObj.base_station_id
      }
      else {
        this.base_station_id = '';
      }
      if (initObj.hasOwnProperty('velocity_latency')) {
        this.velocity_latency = initObj.velocity_latency
      }
      else {
        this.velocity_latency = 0.0;
      }
      if (initObj.hasOwnProperty('diff_age')) {
        this.diff_age = initObj.diff_age
      }
      else {
        this.diff_age = 0.0;
      }
      if (initObj.hasOwnProperty('solution_age')) {
        this.solution_age = initObj.solution_age
      }
      else {
        this.solution_age = 0.0;
      }
      if (initObj.hasOwnProperty('num_satellites_tracked')) {
        this.num_satellites_tracked = initObj.num_satellites_tracked
      }
      else {
        this.num_satellites_tracked = 0;
      }
      if (initObj.hasOwnProperty('num_satellites_used_in_solution')) {
        this.num_satellites_used_in_solution = initObj.num_satellites_used_in_solution
      }
      else {
        this.num_satellites_used_in_solution = 0;
      }
      if (initObj.hasOwnProperty('num_gps_and_glonass_l1_used_in_solution')) {
        this.num_gps_and_glonass_l1_used_in_solution = initObj.num_gps_and_glonass_l1_used_in_solution
      }
      else {
        this.num_gps_and_glonass_l1_used_in_solution = 0;
      }
      if (initObj.hasOwnProperty('num_gps_and_glonass_l1_and_l2_used_in_solution')) {
        this.num_gps_and_glonass_l1_and_l2_used_in_solution = initObj.num_gps_and_glonass_l1_and_l2_used_in_solution
      }
      else {
        this.num_gps_and_glonass_l1_and_l2_used_in_solution = 0;
      }
      if (initObj.hasOwnProperty('extended_solution_status')) {
        this.extended_solution_status = initObj.extended_solution_status
      }
      else {
        this.extended_solution_status = new NovatelExtendedSolutionStatus();
      }
      if (initObj.hasOwnProperty('signal_mask')) {
        this.signal_mask = initObj.signal_mask
      }
      else {
        this.signal_mask = new NovatelSignalMask();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NovatelXYZ
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [novatel_msg_header]
    bufferOffset = NovatelMessageHeader.serialize(obj.novatel_msg_header, buffer, bufferOffset);
    // Serialize message field [solution_status]
    bufferOffset = _serializer.string(obj.solution_status, buffer, bufferOffset);
    // Serialize message field [position_type]
    bufferOffset = _serializer.string(obj.position_type, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [x_sigma]
    bufferOffset = _serializer.float32(obj.x_sigma, buffer, bufferOffset);
    // Serialize message field [y_sigma]
    bufferOffset = _serializer.float32(obj.y_sigma, buffer, bufferOffset);
    // Serialize message field [z_sigma]
    bufferOffset = _serializer.float32(obj.z_sigma, buffer, bufferOffset);
    // Serialize message field [velocity_solution_status]
    bufferOffset = _serializer.string(obj.velocity_solution_status, buffer, bufferOffset);
    // Serialize message field [velocity_type]
    bufferOffset = _serializer.string(obj.velocity_type, buffer, bufferOffset);
    // Serialize message field [x_vel]
    bufferOffset = _serializer.float64(obj.x_vel, buffer, bufferOffset);
    // Serialize message field [y_vel]
    bufferOffset = _serializer.float64(obj.y_vel, buffer, bufferOffset);
    // Serialize message field [z_vel]
    bufferOffset = _serializer.float64(obj.z_vel, buffer, bufferOffset);
    // Serialize message field [x_vel_sigma]
    bufferOffset = _serializer.float32(obj.x_vel_sigma, buffer, bufferOffset);
    // Serialize message field [y_vel_sigma]
    bufferOffset = _serializer.float32(obj.y_vel_sigma, buffer, bufferOffset);
    // Serialize message field [z_vel_sigma]
    bufferOffset = _serializer.float32(obj.z_vel_sigma, buffer, bufferOffset);
    // Serialize message field [base_station_id]
    bufferOffset = _serializer.string(obj.base_station_id, buffer, bufferOffset);
    // Serialize message field [velocity_latency]
    bufferOffset = _serializer.float32(obj.velocity_latency, buffer, bufferOffset);
    // Serialize message field [diff_age]
    bufferOffset = _serializer.float32(obj.diff_age, buffer, bufferOffset);
    // Serialize message field [solution_age]
    bufferOffset = _serializer.float32(obj.solution_age, buffer, bufferOffset);
    // Serialize message field [num_satellites_tracked]
    bufferOffset = _serializer.uint8(obj.num_satellites_tracked, buffer, bufferOffset);
    // Serialize message field [num_satellites_used_in_solution]
    bufferOffset = _serializer.uint8(obj.num_satellites_used_in_solution, buffer, bufferOffset);
    // Serialize message field [num_gps_and_glonass_l1_used_in_solution]
    bufferOffset = _serializer.uint8(obj.num_gps_and_glonass_l1_used_in_solution, buffer, bufferOffset);
    // Serialize message field [num_gps_and_glonass_l1_and_l2_used_in_solution]
    bufferOffset = _serializer.uint8(obj.num_gps_and_glonass_l1_and_l2_used_in_solution, buffer, bufferOffset);
    // Serialize message field [extended_solution_status]
    bufferOffset = NovatelExtendedSolutionStatus.serialize(obj.extended_solution_status, buffer, bufferOffset);
    // Serialize message field [signal_mask]
    bufferOffset = NovatelSignalMask.serialize(obj.signal_mask, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NovatelXYZ
    let len;
    let data = new NovatelXYZ(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [novatel_msg_header]
    data.novatel_msg_header = NovatelMessageHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [solution_status]
    data.solution_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position_type]
    data.position_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_sigma]
    data.x_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_sigma]
    data.y_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_sigma]
    data.z_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_solution_status]
    data.velocity_solution_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [velocity_type]
    data.velocity_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x_vel]
    data.x_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_vel]
    data.y_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_vel]
    data.z_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_vel_sigma]
    data.x_vel_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_vel_sigma]
    data.y_vel_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_vel_sigma]
    data.z_vel_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [base_station_id]
    data.base_station_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [velocity_latency]
    data.velocity_latency = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [diff_age]
    data.diff_age = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [solution_age]
    data.solution_age = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [num_satellites_tracked]
    data.num_satellites_tracked = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_satellites_used_in_solution]
    data.num_satellites_used_in_solution = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_gps_and_glonass_l1_used_in_solution]
    data.num_gps_and_glonass_l1_used_in_solution = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_gps_and_glonass_l1_and_l2_used_in_solution]
    data.num_gps_and_glonass_l1_and_l2_used_in_solution = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [extended_solution_status]
    data.extended_solution_status = NovatelExtendedSolutionStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [signal_mask]
    data.signal_mask = NovatelSignalMask.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += NovatelMessageHeader.getMessageSize(object.novatel_msg_header);
    length += _getByteLength(object.solution_status);
    length += _getByteLength(object.position_type);
    length += _getByteLength(object.velocity_solution_status);
    length += _getByteLength(object.velocity_type);
    length += _getByteLength(object.base_station_id);
    length += NovatelExtendedSolutionStatus.getMessageSize(object.extended_solution_status);
    return length + 117;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/NovatelXYZ';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1ffc3181aa742b6133febb9d8f77d12';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Parsed Best Position in the WGS-84 ECEF frame from Novatel receiver
    Header header
    
    NovatelMessageHeader novatel_msg_header
    
    # Position Data (m)
    string solution_status
    string position_type
    
    float64 x
    float64 y
    float64 z
    
    # Position Standard Deviation (m)
    float32 x_sigma
    float32 y_sigma
    float32 z_sigma
    
    # Velocity Data
    string velocity_solution_status
    string velocity_type
    
    float64 x_vel
    float64 y_vel
    float64 z_vel
    
    # Velocity Standard Deviation (m/s)
    float32 x_vel_sigma
    float32 y_vel_sigma
    float32 z_vel_sigma
    
    string base_station_id
    float32 velocity_latency
    
    # Data Ages (sec)
    float32 diff_age
    float32 solution_age
    
    # Satellite Usage
    uint8 num_satellites_tracked
    uint8 num_satellites_used_in_solution
    uint8 num_gps_and_glonass_l1_used_in_solution
    uint8 num_gps_and_glonass_l1_and_l2_used_in_solution
    
    # Extended Solution Status
    NovatelExtendedSolutionStatus extended_solution_status
    
    # Signal Masks
    NovatelSignalMask signal_mask
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
    
    ================================================================================
    MSG: novatel_gps_msgs/NovatelSignalMask
    # Bit    Mask      Description
    #  0     0x01      GPS L1 used in Solution
    #  1     0x02      GPS L2 used in Solution
    #  2     0x04      GPS L5 used in Solution
    #  3     0x08      <Reserved>
    #  4     0x10      GLONASS L1 used in Solution
    #  5     0x20      GLONASS L2 used in Solution
    # 6-7  0x40-0x80   <Reserved>
    uint32 original_mask
    bool gps_L1_used_in_solution
    bool gps_L2_used_in_solution
    bool gps_L3_used_in_solution
    bool glonass_L1_used_in_solution
    bool glonass_L2_used_in_solution
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NovatelXYZ(null);
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

    if (msg.solution_status !== undefined) {
      resolved.solution_status = msg.solution_status;
    }
    else {
      resolved.solution_status = ''
    }

    if (msg.position_type !== undefined) {
      resolved.position_type = msg.position_type;
    }
    else {
      resolved.position_type = ''
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.x_sigma !== undefined) {
      resolved.x_sigma = msg.x_sigma;
    }
    else {
      resolved.x_sigma = 0.0
    }

    if (msg.y_sigma !== undefined) {
      resolved.y_sigma = msg.y_sigma;
    }
    else {
      resolved.y_sigma = 0.0
    }

    if (msg.z_sigma !== undefined) {
      resolved.z_sigma = msg.z_sigma;
    }
    else {
      resolved.z_sigma = 0.0
    }

    if (msg.velocity_solution_status !== undefined) {
      resolved.velocity_solution_status = msg.velocity_solution_status;
    }
    else {
      resolved.velocity_solution_status = ''
    }

    if (msg.velocity_type !== undefined) {
      resolved.velocity_type = msg.velocity_type;
    }
    else {
      resolved.velocity_type = ''
    }

    if (msg.x_vel !== undefined) {
      resolved.x_vel = msg.x_vel;
    }
    else {
      resolved.x_vel = 0.0
    }

    if (msg.y_vel !== undefined) {
      resolved.y_vel = msg.y_vel;
    }
    else {
      resolved.y_vel = 0.0
    }

    if (msg.z_vel !== undefined) {
      resolved.z_vel = msg.z_vel;
    }
    else {
      resolved.z_vel = 0.0
    }

    if (msg.x_vel_sigma !== undefined) {
      resolved.x_vel_sigma = msg.x_vel_sigma;
    }
    else {
      resolved.x_vel_sigma = 0.0
    }

    if (msg.y_vel_sigma !== undefined) {
      resolved.y_vel_sigma = msg.y_vel_sigma;
    }
    else {
      resolved.y_vel_sigma = 0.0
    }

    if (msg.z_vel_sigma !== undefined) {
      resolved.z_vel_sigma = msg.z_vel_sigma;
    }
    else {
      resolved.z_vel_sigma = 0.0
    }

    if (msg.base_station_id !== undefined) {
      resolved.base_station_id = msg.base_station_id;
    }
    else {
      resolved.base_station_id = ''
    }

    if (msg.velocity_latency !== undefined) {
      resolved.velocity_latency = msg.velocity_latency;
    }
    else {
      resolved.velocity_latency = 0.0
    }

    if (msg.diff_age !== undefined) {
      resolved.diff_age = msg.diff_age;
    }
    else {
      resolved.diff_age = 0.0
    }

    if (msg.solution_age !== undefined) {
      resolved.solution_age = msg.solution_age;
    }
    else {
      resolved.solution_age = 0.0
    }

    if (msg.num_satellites_tracked !== undefined) {
      resolved.num_satellites_tracked = msg.num_satellites_tracked;
    }
    else {
      resolved.num_satellites_tracked = 0
    }

    if (msg.num_satellites_used_in_solution !== undefined) {
      resolved.num_satellites_used_in_solution = msg.num_satellites_used_in_solution;
    }
    else {
      resolved.num_satellites_used_in_solution = 0
    }

    if (msg.num_gps_and_glonass_l1_used_in_solution !== undefined) {
      resolved.num_gps_and_glonass_l1_used_in_solution = msg.num_gps_and_glonass_l1_used_in_solution;
    }
    else {
      resolved.num_gps_and_glonass_l1_used_in_solution = 0
    }

    if (msg.num_gps_and_glonass_l1_and_l2_used_in_solution !== undefined) {
      resolved.num_gps_and_glonass_l1_and_l2_used_in_solution = msg.num_gps_and_glonass_l1_and_l2_used_in_solution;
    }
    else {
      resolved.num_gps_and_glonass_l1_and_l2_used_in_solution = 0
    }

    if (msg.extended_solution_status !== undefined) {
      resolved.extended_solution_status = NovatelExtendedSolutionStatus.Resolve(msg.extended_solution_status)
    }
    else {
      resolved.extended_solution_status = new NovatelExtendedSolutionStatus()
    }

    if (msg.signal_mask !== undefined) {
      resolved.signal_mask = NovatelSignalMask.Resolve(msg.signal_mask)
    }
    else {
      resolved.signal_mask = new NovatelSignalMask()
    }

    return resolved;
    }
};

module.exports = NovatelXYZ;
