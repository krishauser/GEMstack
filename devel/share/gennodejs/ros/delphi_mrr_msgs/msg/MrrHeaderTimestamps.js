// Auto-generated. Do not edit!

// (in-package delphi_mrr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MrrHeaderTimestamps {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_det_time_since_meas = null;
      this.can_sensor_time_stamp = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_det_time_since_meas')) {
        this.can_det_time_since_meas = initObj.can_det_time_since_meas
      }
      else {
        this.can_det_time_since_meas = 0.0;
      }
      if (initObj.hasOwnProperty('can_sensor_time_stamp')) {
        this.can_sensor_time_stamp = initObj.can_sensor_time_stamp
      }
      else {
        this.can_sensor_time_stamp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrHeaderTimestamps
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_det_time_since_meas]
    bufferOffset = _serializer.float32(obj.can_det_time_since_meas, buffer, bufferOffset);
    // Serialize message field [can_sensor_time_stamp]
    bufferOffset = _serializer.float32(obj.can_sensor_time_stamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrHeaderTimestamps
    let len;
    let data = new MrrHeaderTimestamps(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_det_time_since_meas]
    data.can_det_time_since_meas = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_sensor_time_stamp]
    data.can_sensor_time_stamp = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrHeaderTimestamps';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '31560a809bee8d977f1d25fd94db961e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float32 can_det_time_since_meas
    float32 can_sensor_time_stamp
    
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
    const resolved = new MrrHeaderTimestamps(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_det_time_since_meas !== undefined) {
      resolved.can_det_time_since_meas = msg.can_det_time_since_meas;
    }
    else {
      resolved.can_det_time_since_meas = 0.0
    }

    if (msg.can_sensor_time_stamp !== undefined) {
      resolved.can_sensor_time_stamp = msg.can_sensor_time_stamp;
    }
    else {
      resolved.can_sensor_time_stamp = 0.0
    }

    return resolved;
    }
};

module.exports = MrrHeaderTimestamps;
