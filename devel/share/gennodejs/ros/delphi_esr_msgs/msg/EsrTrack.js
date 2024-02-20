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

class EsrTrack {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.canmsg = null;
      this.id = null;
      this.lat_rate = null;
      this.grouping_changed = null;
      this.oncoming = null;
      this.status = null;
      this.angle = null;
      this.range = null;
      this.bridge_object = null;
      this.rolling_count = null;
      this.width = null;
      this.range_accel = null;
      this.med_range_mode = null;
      this.range_rate = null;
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
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('lat_rate')) {
        this.lat_rate = initObj.lat_rate
      }
      else {
        this.lat_rate = 0.0;
      }
      if (initObj.hasOwnProperty('grouping_changed')) {
        this.grouping_changed = initObj.grouping_changed
      }
      else {
        this.grouping_changed = false;
      }
      if (initObj.hasOwnProperty('oncoming')) {
        this.oncoming = initObj.oncoming
      }
      else {
        this.oncoming = false;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('bridge_object')) {
        this.bridge_object = initObj.bridge_object
      }
      else {
        this.bridge_object = false;
      }
      if (initObj.hasOwnProperty('rolling_count')) {
        this.rolling_count = initObj.rolling_count
      }
      else {
        this.rolling_count = false;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
      if (initObj.hasOwnProperty('range_accel')) {
        this.range_accel = initObj.range_accel
      }
      else {
        this.range_accel = 0.0;
      }
      if (initObj.hasOwnProperty('med_range_mode')) {
        this.med_range_mode = initObj.med_range_mode
      }
      else {
        this.med_range_mode = 0;
      }
      if (initObj.hasOwnProperty('range_rate')) {
        this.range_rate = initObj.range_rate
      }
      else {
        this.range_rate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EsrTrack
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [canmsg]
    bufferOffset = _serializer.string(obj.canmsg, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [lat_rate]
    bufferOffset = _serializer.float32(obj.lat_rate, buffer, bufferOffset);
    // Serialize message field [grouping_changed]
    bufferOffset = _serializer.bool(obj.grouping_changed, buffer, bufferOffset);
    // Serialize message field [oncoming]
    bufferOffset = _serializer.bool(obj.oncoming, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float32(obj.angle, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float32(obj.range, buffer, bufferOffset);
    // Serialize message field [bridge_object]
    bufferOffset = _serializer.bool(obj.bridge_object, buffer, bufferOffset);
    // Serialize message field [rolling_count]
    bufferOffset = _serializer.bool(obj.rolling_count, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float32(obj.width, buffer, bufferOffset);
    // Serialize message field [range_accel]
    bufferOffset = _serializer.float32(obj.range_accel, buffer, bufferOffset);
    // Serialize message field [med_range_mode]
    bufferOffset = _serializer.uint8(obj.med_range_mode, buffer, bufferOffset);
    // Serialize message field [range_rate]
    bufferOffset = _serializer.float32(obj.range_rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EsrTrack
    let len;
    let data = new EsrTrack(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [canmsg]
    data.canmsg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lat_rate]
    data.lat_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grouping_changed]
    data.grouping_changed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [oncoming]
    data.oncoming = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bridge_object]
    data.bridge_object = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rolling_count]
    data.rolling_count = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [range_accel]
    data.range_accel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [med_range_mode]
    data.med_range_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [range_rate]
    data.range_rate = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.canmsg);
    return length + 35;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_esr_msgs/EsrTrack';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dbcd2eea001ab20b27c9a37e555910ae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # ESR Track
    string        canmsg
    
    uint8         id
    float32       lat_rate
    bool          grouping_changed
    bool          oncoming
    uint8         status
    float32       angle
    float32       range
    bool          bridge_object
    bool          rolling_count
    float32       width
    float32       range_accel
    uint8         med_range_mode
    float32       range_rate
    
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
    const resolved = new EsrTrack(null);
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

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.lat_rate !== undefined) {
      resolved.lat_rate = msg.lat_rate;
    }
    else {
      resolved.lat_rate = 0.0
    }

    if (msg.grouping_changed !== undefined) {
      resolved.grouping_changed = msg.grouping_changed;
    }
    else {
      resolved.grouping_changed = false
    }

    if (msg.oncoming !== undefined) {
      resolved.oncoming = msg.oncoming;
    }
    else {
      resolved.oncoming = false
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.bridge_object !== undefined) {
      resolved.bridge_object = msg.bridge_object;
    }
    else {
      resolved.bridge_object = false
    }

    if (msg.rolling_count !== undefined) {
      resolved.rolling_count = msg.rolling_count;
    }
    else {
      resolved.rolling_count = false
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    if (msg.range_accel !== undefined) {
      resolved.range_accel = msg.range_accel;
    }
    else {
      resolved.range_accel = 0.0
    }

    if (msg.med_range_mode !== undefined) {
      resolved.med_range_mode = msg.med_range_mode;
    }
    else {
      resolved.med_range_mode = 0
    }

    if (msg.range_rate !== undefined) {
      resolved.range_rate = msg.range_rate;
    }
    else {
      resolved.range_rate = 0.0
    }

    return resolved;
    }
};

module.exports = EsrTrack;
