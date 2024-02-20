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

class NovatelHeading2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.novatel_msg_header = null;
      this.solution_status = null;
      this.position_type = null;
      this.baseline_length = null;
      this.heading = null;
      this.pitch = null;
      this.heading_sigma = null;
      this.pitch_sigma = null;
      this.rover_station_id = null;
      this.master_station_id = null;
      this.num_satellites_tracked = null;
      this.num_satellites_used_in_solution = null;
      this.num_satellites_above_elevation_mask_angle = null;
      this.num_satellites_above_elevation_mask_angle_l2 = null;
      this.solution_source = null;
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
      if (initObj.hasOwnProperty('baseline_length')) {
        this.baseline_length = initObj.baseline_length
      }
      else {
        this.baseline_length = 0.0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('heading_sigma')) {
        this.heading_sigma = initObj.heading_sigma
      }
      else {
        this.heading_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_sigma')) {
        this.pitch_sigma = initObj.pitch_sigma
      }
      else {
        this.pitch_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('rover_station_id')) {
        this.rover_station_id = initObj.rover_station_id
      }
      else {
        this.rover_station_id = '';
      }
      if (initObj.hasOwnProperty('master_station_id')) {
        this.master_station_id = initObj.master_station_id
      }
      else {
        this.master_station_id = '';
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
      if (initObj.hasOwnProperty('num_satellites_above_elevation_mask_angle')) {
        this.num_satellites_above_elevation_mask_angle = initObj.num_satellites_above_elevation_mask_angle
      }
      else {
        this.num_satellites_above_elevation_mask_angle = 0;
      }
      if (initObj.hasOwnProperty('num_satellites_above_elevation_mask_angle_l2')) {
        this.num_satellites_above_elevation_mask_angle_l2 = initObj.num_satellites_above_elevation_mask_angle_l2
      }
      else {
        this.num_satellites_above_elevation_mask_angle_l2 = 0;
      }
      if (initObj.hasOwnProperty('solution_source')) {
        this.solution_source = initObj.solution_source
      }
      else {
        this.solution_source = 0;
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
    // Serializes a message object of type NovatelHeading2
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [novatel_msg_header]
    bufferOffset = NovatelMessageHeader.serialize(obj.novatel_msg_header, buffer, bufferOffset);
    // Serialize message field [solution_status]
    bufferOffset = _serializer.string(obj.solution_status, buffer, bufferOffset);
    // Serialize message field [position_type]
    bufferOffset = _serializer.string(obj.position_type, buffer, bufferOffset);
    // Serialize message field [baseline_length]
    bufferOffset = _serializer.float32(obj.baseline_length, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float32(obj.heading, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [heading_sigma]
    bufferOffset = _serializer.float32(obj.heading_sigma, buffer, bufferOffset);
    // Serialize message field [pitch_sigma]
    bufferOffset = _serializer.float32(obj.pitch_sigma, buffer, bufferOffset);
    // Serialize message field [rover_station_id]
    bufferOffset = _serializer.string(obj.rover_station_id, buffer, bufferOffset);
    // Serialize message field [master_station_id]
    bufferOffset = _serializer.string(obj.master_station_id, buffer, bufferOffset);
    // Serialize message field [num_satellites_tracked]
    bufferOffset = _serializer.uint8(obj.num_satellites_tracked, buffer, bufferOffset);
    // Serialize message field [num_satellites_used_in_solution]
    bufferOffset = _serializer.uint8(obj.num_satellites_used_in_solution, buffer, bufferOffset);
    // Serialize message field [num_satellites_above_elevation_mask_angle]
    bufferOffset = _serializer.uint8(obj.num_satellites_above_elevation_mask_angle, buffer, bufferOffset);
    // Serialize message field [num_satellites_above_elevation_mask_angle_l2]
    bufferOffset = _serializer.uint8(obj.num_satellites_above_elevation_mask_angle_l2, buffer, bufferOffset);
    // Serialize message field [solution_source]
    bufferOffset = _serializer.uint8(obj.solution_source, buffer, bufferOffset);
    // Serialize message field [extended_solution_status]
    bufferOffset = NovatelExtendedSolutionStatus.serialize(obj.extended_solution_status, buffer, bufferOffset);
    // Serialize message field [signal_mask]
    bufferOffset = NovatelSignalMask.serialize(obj.signal_mask, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NovatelHeading2
    let len;
    let data = new NovatelHeading2(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [novatel_msg_header]
    data.novatel_msg_header = NovatelMessageHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [solution_status]
    data.solution_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position_type]
    data.position_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [baseline_length]
    data.baseline_length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_sigma]
    data.heading_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_sigma]
    data.pitch_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rover_station_id]
    data.rover_station_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [master_station_id]
    data.master_station_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [num_satellites_tracked]
    data.num_satellites_tracked = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_satellites_used_in_solution]
    data.num_satellites_used_in_solution = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_satellites_above_elevation_mask_angle]
    data.num_satellites_above_elevation_mask_angle = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_satellites_above_elevation_mask_angle_l2]
    data.num_satellites_above_elevation_mask_angle_l2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [solution_source]
    data.solution_source = _deserializer.uint8(buffer, bufferOffset);
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
    length += _getByteLength(object.rover_station_id);
    length += _getByteLength(object.master_station_id);
    length += NovatelExtendedSolutionStatus.getMessageSize(object.extended_solution_status);
    return length + 50;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/NovatelHeading2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1195c3bddd7a9ddbaf770e688a2f354a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Parsed Heading East of North from Novatel receiver. Only ALIGN capable recievers can publish this
    Header header
    
    NovatelMessageHeader novatel_msg_header
    
    # Solution Status
    string solution_status
    string position_type
    
    # Baseline length (m)
    float32 baseline_length
    
    # Heading in degrees [0,360)
    float32 heading
    
    # Pitch in degrees +- 90
    float32 pitch
    
    # Orientation Standard Deviations (deg)
    float32 heading_sigma
    float32 pitch_sigma
    
    # Station ids
    string rover_station_id
    string master_station_id
    
    # Satellite Usage
    uint8 num_satellites_tracked
    uint8 num_satellites_used_in_solution
    uint8 num_satellites_above_elevation_mask_angle
    uint8 num_satellites_above_elevation_mask_angle_l2
    
    # Enum for solution source
    uint8 solution_source
    
    uint8 SOURCE_PRIMARY_ANTENNA=0
    uint8 SOURCE_SECONDARY_ANTENNA=1
    
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
    const resolved = new NovatelHeading2(null);
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

    if (msg.baseline_length !== undefined) {
      resolved.baseline_length = msg.baseline_length;
    }
    else {
      resolved.baseline_length = 0.0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.heading_sigma !== undefined) {
      resolved.heading_sigma = msg.heading_sigma;
    }
    else {
      resolved.heading_sigma = 0.0
    }

    if (msg.pitch_sigma !== undefined) {
      resolved.pitch_sigma = msg.pitch_sigma;
    }
    else {
      resolved.pitch_sigma = 0.0
    }

    if (msg.rover_station_id !== undefined) {
      resolved.rover_station_id = msg.rover_station_id;
    }
    else {
      resolved.rover_station_id = ''
    }

    if (msg.master_station_id !== undefined) {
      resolved.master_station_id = msg.master_station_id;
    }
    else {
      resolved.master_station_id = ''
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

    if (msg.num_satellites_above_elevation_mask_angle !== undefined) {
      resolved.num_satellites_above_elevation_mask_angle = msg.num_satellites_above_elevation_mask_angle;
    }
    else {
      resolved.num_satellites_above_elevation_mask_angle = 0
    }

    if (msg.num_satellites_above_elevation_mask_angle_l2 !== undefined) {
      resolved.num_satellites_above_elevation_mask_angle_l2 = msg.num_satellites_above_elevation_mask_angle_l2;
    }
    else {
      resolved.num_satellites_above_elevation_mask_angle_l2 = 0
    }

    if (msg.solution_source !== undefined) {
      resolved.solution_source = msg.solution_source;
    }
    else {
      resolved.solution_source = 0
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

// Constants for message
NovatelHeading2.Constants = {
  SOURCE_PRIMARY_ANTENNA: 0,
  SOURCE_SECONDARY_ANTENNA: 1,
}

module.exports = NovatelHeading2;
