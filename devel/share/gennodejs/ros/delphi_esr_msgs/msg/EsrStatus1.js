// Auto-generated. Do not edit!

// (in-package delphi_esr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EsrStatus1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.rolling_count_1 = null;
      this.dsp_timestamp = null;
      this.comm_error = null;
      this.radius_curvature_calc = null;
      this.scan_index = null;
      this.yaw_rate_calc = null;
      this.vehicle_speed_calc = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('canmsg')) {
        this.canmsg = initObj.canmsg
      }
      else {
        this.canmsg = '';
      }
      if (initObj.hasOwnProperty('rolling_count_1')) {
        this.rolling_count_1 = initObj.rolling_count_1
      }
      else {
        this.rolling_count_1 = 0;
      }
      if (initObj.hasOwnProperty('dsp_timestamp')) {
        this.dsp_timestamp = initObj.dsp_timestamp
      }
      else {
        this.dsp_timestamp = 0;
      }
      if (initObj.hasOwnProperty('comm_error')) {
        this.comm_error = initObj.comm_error
      }
      else {
        this.comm_error = false;
      }
      if (initObj.hasOwnProperty('radius_curvature_calc')) {
        this.radius_curvature_calc = initObj.radius_curvature_calc
      }
      else {
        this.radius_curvature_calc = 0;
      }
      if (initObj.hasOwnProperty('scan_index')) {
        this.scan_index = initObj.scan_index
      }
      else {
        this.scan_index = 0;
      }
      if (initObj.hasOwnProperty('yaw_rate_calc')) {
        this.yaw_rate_calc = initObj.yaw_rate_calc
      }
      else {
        this.yaw_rate_calc = 0.0;
      }
      if (initObj.hasOwnProperty('vehicle_speed_calc')) {
        this.vehicle_speed_calc = initObj.vehicle_speed_calc
      }
      else {
        this.vehicle_speed_calc = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrStatus1
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [rolling_count_1]
    bufferOffset = _serializer.uint8(obj.rolling_count_1, buffer, bufferOffset);
    // Serialize message field [dsp_timestamp]
    bufferOffset = _serializer.uint8(obj.dsp_timestamp, buffer, bufferOffset);
    // Serialize message field [comm_error]
    bufferOffset = _serializer.bool(obj.comm_error, buffer, bufferOffset);
    // Serialize message field [radius_curvature_calc]
    bufferOffset = _serializer.int16(obj.radius_curvature_calc, buffer, bufferOffset);
    // Serialize message field [scan_index]
    bufferOffset = _serializer.uint16(obj.scan_index, buffer, bufferOffset);
    // Serialize message field [yaw_rate_calc]
    bufferOffset = _serializer.float32(obj.yaw_rate_calc, buffer, bufferOffset);
    // Serialize message field [vehicle_speed_calc]
    bufferOffset = _serializer.float32(obj.vehicle_speed_calc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrStatus1
    let len;
    let data = new EsrStatus1(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [rolling_count_1]
    data.rolling_count_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dsp_timestamp]
    data.dsp_timestamp = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [comm_error]
    data.comm_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [radius_curvature_calc]
    data.radius_curvature_calc = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [scan_index]
    data.scan_index = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [yaw_rate_calc]
    data.yaw_rate_calc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vehicle_speed_calc]
    data.vehicle_speed_calc = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrStatus1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f3f440bdd87b7ce3da2d8d915a5970b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Status1
    string      canmsg
    
    uint8       rolling_count_1
    uint8       dsp_timestamp
    bool        comm_error
    int16       radius_curvature_calc
    uint16      scan_index
    float32     yaw_rate_calc
    float32     vehicle_speed_calc
    
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
    const resolved = new EsrStatus1(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.canmsg !== undefined) {
      resolved.canmsg = msg.canmsg;
    }
    else {
      resolved.canmsg = ''
    }

    if (msg.rolling_count_1 !== undefined) {
      resolved.rolling_count_1 = msg.rolling_count_1;
    }
    else {
      resolved.rolling_count_1 = 0
    }

    if (msg.dsp_timestamp !== undefined) {
      resolved.dsp_timestamp = msg.dsp_timestamp;
    }
    else {
      resolved.dsp_timestamp = 0
    }

    if (msg.comm_error !== undefined) {
      resolved.comm_error = msg.comm_error;
    }
    else {
      resolved.comm_error = false
    }

    if (msg.radius_curvature_calc !== undefined) {
      resolved.radius_curvature_calc = msg.radius_curvature_calc;
    }
    else {
      resolved.radius_curvature_calc = 0
    }

    if (msg.scan_index !== undefined) {
      resolved.scan_index = msg.scan_index;
    }
    else {
      resolved.scan_index = 0
    }

    if (msg.yaw_rate_calc !== undefined) {
      resolved.yaw_rate_calc = msg.yaw_rate_calc;
    }
    else {
      resolved.yaw_rate_calc = 0.0
    }

    if (msg.vehicle_speed_calc !== undefined) {
      resolved.vehicle_speed_calc = msg.vehicle_speed_calc;
    }
    else {
      resolved.vehicle_speed_calc = 0.0
    }

    return resolved;
    }
};

module.exports = EsrStatus1;
