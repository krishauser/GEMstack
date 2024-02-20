// Auto-generated. Do not edit!

// (in-package kartech_linear_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class UniqueDeviceIdRpt {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.actuator_id_first_6 = null;
      this.actuator_id_last_6 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('actuator_id_first_6')) {
        this.actuator_id_first_6 = initObj.actuator_id_first_6
      }
      else {
        this.actuator_id_first_6 = 0;
      }
      if (initObj.hasOwnProperty('actuator_id_last_6')) {
        this.actuator_id_last_6 = initObj.actuator_id_last_6
      }
      else {
        this.actuator_id_last_6 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UniqueDeviceIdRpt
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [actuator_id_first_6]
    bufferOffset = _serializer.uint64(obj.actuator_id_first_6, buffer, bufferOffset);
    // Serialize message field [actuator_id_last_6]
    bufferOffset = _serializer.uint64(obj.actuator_id_last_6, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UniqueDeviceIdRpt
    let len;
    let data = new UniqueDeviceIdRpt(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [actuator_id_first_6]
    data.actuator_id_first_6 = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [actuator_id_last_6]
    data.actuator_id_last_6 = _deserializer.uint64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kartech_linear_actuator_msgs/UniqueDeviceIdRpt';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ea8eb311cb86c91d9fa6aff8968d0ee0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint64 actuator_id_first_6    # The first six bytes of the unique ID of this actuator.
    uint64 actuator_id_last_6     # The last six bytes of the unique ID of this actuator.
    
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
    const resolved = new UniqueDeviceIdRpt(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.actuator_id_first_6 !== undefined) {
      resolved.actuator_id_first_6 = msg.actuator_id_first_6;
    }
    else {
      resolved.actuator_id_first_6 = 0
    }

    if (msg.actuator_id_last_6 !== undefined) {
      resolved.actuator_id_last_6 = msg.actuator_id_last_6;
    }
    else {
      resolved.actuator_id_last_6 = 0
    }

    return resolved;
    }
};

module.exports = UniqueDeviceIdRpt;
