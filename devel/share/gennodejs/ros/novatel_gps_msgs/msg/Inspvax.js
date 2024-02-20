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

class Inspvax {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.novatel_msg_header = null;
      this.ins_status = null;
      this.position_type = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.undulation = null;
      this.north_velocity = null;
      this.east_velocity = null;
      this.up_velocity = null;
      this.roll = null;
      this.pitch = null;
      this.azimuth = null;
      this.latitude_std = null;
      this.longitude_std = null;
      this.altitude_std = null;
      this.north_velocity_std = null;
      this.east_velocity_std = null;
      this.up_velocity_std = null;
      this.roll_std = null;
      this.pitch_std = null;
      this.azimuth_std = null;
      this.extended_status = null;
      this.seconds_since_update = null;
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
      if (initObj.hasOwnProperty('ins_status')) {
        this.ins_status = initObj.ins_status
      }
      else {
        this.ins_status = '';
      }
      if (initObj.hasOwnProperty('position_type')) {
        this.position_type = initObj.position_type
      }
      else {
        this.position_type = '';
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('undulation')) {
        this.undulation = initObj.undulation
      }
      else {
        this.undulation = 0.0;
      }
      if (initObj.hasOwnProperty('north_velocity')) {
        this.north_velocity = initObj.north_velocity
      }
      else {
        this.north_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('east_velocity')) {
        this.east_velocity = initObj.east_velocity
      }
      else {
        this.east_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('up_velocity')) {
        this.up_velocity = initObj.up_velocity
      }
      else {
        this.up_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('azimuth')) {
        this.azimuth = initObj.azimuth
      }
      else {
        this.azimuth = 0.0;
      }
      if (initObj.hasOwnProperty('latitude_std')) {
        this.latitude_std = initObj.latitude_std
      }
      else {
        this.latitude_std = 0.0;
      }
      if (initObj.hasOwnProperty('longitude_std')) {
        this.longitude_std = initObj.longitude_std
      }
      else {
        this.longitude_std = 0.0;
      }
      if (initObj.hasOwnProperty('altitude_std')) {
        this.altitude_std = initObj.altitude_std
      }
      else {
        this.altitude_std = 0.0;
      }
      if (initObj.hasOwnProperty('north_velocity_std')) {
        this.north_velocity_std = initObj.north_velocity_std
      }
      else {
        this.north_velocity_std = 0.0;
      }
      if (initObj.hasOwnProperty('east_velocity_std')) {
        this.east_velocity_std = initObj.east_velocity_std
      }
      else {
        this.east_velocity_std = 0.0;
      }
      if (initObj.hasOwnProperty('up_velocity_std')) {
        this.up_velocity_std = initObj.up_velocity_std
      }
      else {
        this.up_velocity_std = 0.0;
      }
      if (initObj.hasOwnProperty('roll_std')) {
        this.roll_std = initObj.roll_std
      }
      else {
        this.roll_std = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_std')) {
        this.pitch_std = initObj.pitch_std
      }
      else {
        this.pitch_std = 0.0;
      }
      if (initObj.hasOwnProperty('azimuth_std')) {
        this.azimuth_std = initObj.azimuth_std
      }
      else {
        this.azimuth_std = 0.0;
      }
      if (initObj.hasOwnProperty('extended_status')) {
        this.extended_status = initObj.extended_status
      }
      else {
        this.extended_status = new NovatelExtendedSolutionStatus();
      }
      if (initObj.hasOwnProperty('seconds_since_update')) {
        this.seconds_since_update = initObj.seconds_since_update
      }
      else {
        this.seconds_since_update = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Inspvax
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [novatel_msg_header]
    bufferOffset = NovatelMessageHeader.serialize(obj.novatel_msg_header, buffer, bufferOffset);
    // Serialize message field [ins_status]
    bufferOffset = _serializer.string(obj.ins_status, buffer, bufferOffset);
    // Serialize message field [position_type]
    bufferOffset = _serializer.string(obj.position_type, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    // Serialize message field [undulation]
    bufferOffset = _serializer.float32(obj.undulation, buffer, bufferOffset);
    // Serialize message field [north_velocity]
    bufferOffset = _serializer.float64(obj.north_velocity, buffer, bufferOffset);
    // Serialize message field [east_velocity]
    bufferOffset = _serializer.float64(obj.east_velocity, buffer, bufferOffset);
    // Serialize message field [up_velocity]
    bufferOffset = _serializer.float64(obj.up_velocity, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [azimuth]
    bufferOffset = _serializer.float64(obj.azimuth, buffer, bufferOffset);
    // Serialize message field [latitude_std]
    bufferOffset = _serializer.float32(obj.latitude_std, buffer, bufferOffset);
    // Serialize message field [longitude_std]
    bufferOffset = _serializer.float32(obj.longitude_std, buffer, bufferOffset);
    // Serialize message field [altitude_std]
    bufferOffset = _serializer.float32(obj.altitude_std, buffer, bufferOffset);
    // Serialize message field [north_velocity_std]
    bufferOffset = _serializer.float32(obj.north_velocity_std, buffer, bufferOffset);
    // Serialize message field [east_velocity_std]
    bufferOffset = _serializer.float32(obj.east_velocity_std, buffer, bufferOffset);
    // Serialize message field [up_velocity_std]
    bufferOffset = _serializer.float32(obj.up_velocity_std, buffer, bufferOffset);
    // Serialize message field [roll_std]
    bufferOffset = _serializer.float32(obj.roll_std, buffer, bufferOffset);
    // Serialize message field [pitch_std]
    bufferOffset = _serializer.float32(obj.pitch_std, buffer, bufferOffset);
    // Serialize message field [azimuth_std]
    bufferOffset = _serializer.float32(obj.azimuth_std, buffer, bufferOffset);
    // Serialize message field [extended_status]
    bufferOffset = NovatelExtendedSolutionStatus.serialize(obj.extended_status, buffer, bufferOffset);
    // Serialize message field [seconds_since_update]
    bufferOffset = _serializer.uint16(obj.seconds_since_update, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Inspvax
    let len;
    let data = new Inspvax(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [novatel_msg_header]
    data.novatel_msg_header = NovatelMessageHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [ins_status]
    data.ins_status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position_type]
    data.position_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [undulation]
    data.undulation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [north_velocity]
    data.north_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [east_velocity]
    data.east_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [up_velocity]
    data.up_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [azimuth]
    data.azimuth = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [latitude_std]
    data.latitude_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude_std]
    data.longitude_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude_std]
    data.altitude_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [north_velocity_std]
    data.north_velocity_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [east_velocity_std]
    data.east_velocity_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [up_velocity_std]
    data.up_velocity_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_std]
    data.roll_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_std]
    data.pitch_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [azimuth_std]
    data.azimuth_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [extended_status]
    data.extended_status = NovatelExtendedSolutionStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [seconds_since_update]
    data.seconds_since_update = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += NovatelMessageHeader.getMessageSize(object.novatel_msg_header);
    length += _getByteLength(object.ins_status);
    length += _getByteLength(object.position_type);
    length += NovatelExtendedSolutionStatus.getMessageSize(object.extended_status);
    return length + 122;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/Inspvax';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cebf3b182479d01907e3894361b97eba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message 1465
    
    std_msgs/Header header
    
    NovatelMessageHeader novatel_msg_header
    
    # Table 29 in the SPAN on OEM6 manual:
    # See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=121
    string ins_status
    
    
    # Table 30 in the SPAN on OEM6 manual:
    # See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=124
    string position_type
    
    
    float64 latitude
    float64 longitude
    float64 altitude
    
    float32 undulation
    
    float64 north_velocity
    float64 east_velocity
    float64 up_velocity
    
    float64 roll
    float64 pitch
    float64 azimuth
    
    float32 latitude_std
    float32 longitude_std
    float32 altitude_std
    
    float32 north_velocity_std
    float32 east_velocity_std
    float32 up_velocity_std
    
    float32 roll_std
    float32 pitch_std
    float32 azimuth_std
    
    NovatelExtendedSolutionStatus extended_status
    
    uint16 seconds_since_update
    
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
    const resolved = new Inspvax(null);
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

    if (msg.ins_status !== undefined) {
      resolved.ins_status = msg.ins_status;
    }
    else {
      resolved.ins_status = ''
    }

    if (msg.position_type !== undefined) {
      resolved.position_type = msg.position_type;
    }
    else {
      resolved.position_type = ''
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.undulation !== undefined) {
      resolved.undulation = msg.undulation;
    }
    else {
      resolved.undulation = 0.0
    }

    if (msg.north_velocity !== undefined) {
      resolved.north_velocity = msg.north_velocity;
    }
    else {
      resolved.north_velocity = 0.0
    }

    if (msg.east_velocity !== undefined) {
      resolved.east_velocity = msg.east_velocity;
    }
    else {
      resolved.east_velocity = 0.0
    }

    if (msg.up_velocity !== undefined) {
      resolved.up_velocity = msg.up_velocity;
    }
    else {
      resolved.up_velocity = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.azimuth !== undefined) {
      resolved.azimuth = msg.azimuth;
    }
    else {
      resolved.azimuth = 0.0
    }

    if (msg.latitude_std !== undefined) {
      resolved.latitude_std = msg.latitude_std;
    }
    else {
      resolved.latitude_std = 0.0
    }

    if (msg.longitude_std !== undefined) {
      resolved.longitude_std = msg.longitude_std;
    }
    else {
      resolved.longitude_std = 0.0
    }

    if (msg.altitude_std !== undefined) {
      resolved.altitude_std = msg.altitude_std;
    }
    else {
      resolved.altitude_std = 0.0
    }

    if (msg.north_velocity_std !== undefined) {
      resolved.north_velocity_std = msg.north_velocity_std;
    }
    else {
      resolved.north_velocity_std = 0.0
    }

    if (msg.east_velocity_std !== undefined) {
      resolved.east_velocity_std = msg.east_velocity_std;
    }
    else {
      resolved.east_velocity_std = 0.0
    }

    if (msg.up_velocity_std !== undefined) {
      resolved.up_velocity_std = msg.up_velocity_std;
    }
    else {
      resolved.up_velocity_std = 0.0
    }

    if (msg.roll_std !== undefined) {
      resolved.roll_std = msg.roll_std;
    }
    else {
      resolved.roll_std = 0.0
    }

    if (msg.pitch_std !== undefined) {
      resolved.pitch_std = msg.pitch_std;
    }
    else {
      resolved.pitch_std = 0.0
    }

    if (msg.azimuth_std !== undefined) {
      resolved.azimuth_std = msg.azimuth_std;
    }
    else {
      resolved.azimuth_std = 0.0
    }

    if (msg.extended_status !== undefined) {
      resolved.extended_status = NovatelExtendedSolutionStatus.Resolve(msg.extended_status)
    }
    else {
      resolved.extended_status = new NovatelExtendedSolutionStatus()
    }

    if (msg.seconds_since_update !== undefined) {
      resolved.seconds_since_update = msg.seconds_since_update;
    }
    else {
      resolved.seconds_since_update = 0
    }

    return resolved;
    }
};

module.exports = Inspvax;
