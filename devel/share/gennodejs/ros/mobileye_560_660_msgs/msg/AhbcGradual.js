// Auto-generated. Do not edit!

// (in-package mobileye_560_660_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AhbcGradual {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.boundary_domain_bottom_non_glare_hlb = null;
      this.boundary_domain_non_glare_left_hand_hlb = null;
      this.boundary_domain_non_glare_right_hand_hlb = null;
      this.object_distance_hlb = null;
      this.status_boundary_domain_bottom_non_glare_hlb = null;
      this.status_boundary_domain_non_glare_left_hand_hlb = null;
      this.status_boundary_domain_non_glare_right_hand_hlb = null;
      this.status_object_distance_hlb = null;
      this.left_target_change = null;
      this.right_target_change = null;
      this.too_many_cars = null;
      this.busy_scene = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('boundary_domain_bottom_non_glare_hlb')) {
        this.boundary_domain_bottom_non_glare_hlb = initObj.boundary_domain_bottom_non_glare_hlb
      }
      else {
        this.boundary_domain_bottom_non_glare_hlb = 0.0;
      }
      if (initObj.hasOwnProperty('boundary_domain_non_glare_left_hand_hlb')) {
        this.boundary_domain_non_glare_left_hand_hlb = initObj.boundary_domain_non_glare_left_hand_hlb
      }
      else {
        this.boundary_domain_non_glare_left_hand_hlb = 0.0;
      }
      if (initObj.hasOwnProperty('boundary_domain_non_glare_right_hand_hlb')) {
        this.boundary_domain_non_glare_right_hand_hlb = initObj.boundary_domain_non_glare_right_hand_hlb
      }
      else {
        this.boundary_domain_non_glare_right_hand_hlb = 0.0;
      }
      if (initObj.hasOwnProperty('object_distance_hlb')) {
        this.object_distance_hlb = initObj.object_distance_hlb
      }
      else {
        this.object_distance_hlb = 0;
      }
      if (initObj.hasOwnProperty('status_boundary_domain_bottom_non_glare_hlb')) {
        this.status_boundary_domain_bottom_non_glare_hlb = initObj.status_boundary_domain_bottom_non_glare_hlb
      }
      else {
        this.status_boundary_domain_bottom_non_glare_hlb = 0;
      }
      if (initObj.hasOwnProperty('status_boundary_domain_non_glare_left_hand_hlb')) {
        this.status_boundary_domain_non_glare_left_hand_hlb = initObj.status_boundary_domain_non_glare_left_hand_hlb
      }
      else {
        this.status_boundary_domain_non_glare_left_hand_hlb = 0;
      }
      if (initObj.hasOwnProperty('status_boundary_domain_non_glare_right_hand_hlb')) {
        this.status_boundary_domain_non_glare_right_hand_hlb = initObj.status_boundary_domain_non_glare_right_hand_hlb
      }
      else {
        this.status_boundary_domain_non_glare_right_hand_hlb = 0;
      }
      if (initObj.hasOwnProperty('status_object_distance_hlb')) {
        this.status_object_distance_hlb = initObj.status_object_distance_hlb
      }
      else {
        this.status_object_distance_hlb = 0;
      }
      if (initObj.hasOwnProperty('left_target_change')) {
        this.left_target_change = initObj.left_target_change
      }
      else {
        this.left_target_change = false;
      }
      if (initObj.hasOwnProperty('right_target_change')) {
        this.right_target_change = initObj.right_target_change
      }
      else {
        this.right_target_change = false;
      }
      if (initObj.hasOwnProperty('too_many_cars')) {
        this.too_many_cars = initObj.too_many_cars
      }
      else {
        this.too_many_cars = false;
      }
      if (initObj.hasOwnProperty('busy_scene')) {
        this.busy_scene = initObj.busy_scene
      }
      else {
        this.busy_scene = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AhbcGradual
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [boundary_domain_bottom_non_glare_hlb]
    bufferOffset = _serializer.float32(obj.boundary_domain_bottom_non_glare_hlb, buffer, bufferOffset);
    // Serialize message field [boundary_domain_non_glare_left_hand_hlb]
    bufferOffset = _serializer.float32(obj.boundary_domain_non_glare_left_hand_hlb, buffer, bufferOffset);
    // Serialize message field [boundary_domain_non_glare_right_hand_hlb]
    bufferOffset = _serializer.float32(obj.boundary_domain_non_glare_right_hand_hlb, buffer, bufferOffset);
    // Serialize message field [object_distance_hlb]
    bufferOffset = _serializer.uint16(obj.object_distance_hlb, buffer, bufferOffset);
    // Serialize message field [status_boundary_domain_bottom_non_glare_hlb]
    bufferOffset = _serializer.uint8(obj.status_boundary_domain_bottom_non_glare_hlb, buffer, bufferOffset);
    // Serialize message field [status_boundary_domain_non_glare_left_hand_hlb]
    bufferOffset = _serializer.uint8(obj.status_boundary_domain_non_glare_left_hand_hlb, buffer, bufferOffset);
    // Serialize message field [status_boundary_domain_non_glare_right_hand_hlb]
    bufferOffset = _serializer.uint8(obj.status_boundary_domain_non_glare_right_hand_hlb, buffer, bufferOffset);
    // Serialize message field [status_object_distance_hlb]
    bufferOffset = _serializer.uint8(obj.status_object_distance_hlb, buffer, bufferOffset);
    // Serialize message field [left_target_change]
    bufferOffset = _serializer.bool(obj.left_target_change, buffer, bufferOffset);
    // Serialize message field [right_target_change]
    bufferOffset = _serializer.bool(obj.right_target_change, buffer, bufferOffset);
    // Serialize message field [too_many_cars]
    bufferOffset = _serializer.bool(obj.too_many_cars, buffer, bufferOffset);
    // Serialize message field [busy_scene]
    bufferOffset = _serializer.bool(obj.busy_scene, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AhbcGradual
    let len;
    let data = new AhbcGradual(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [boundary_domain_bottom_non_glare_hlb]
    data.boundary_domain_bottom_non_glare_hlb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [boundary_domain_non_glare_left_hand_hlb]
    data.boundary_domain_non_glare_left_hand_hlb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [boundary_domain_non_glare_right_hand_hlb]
    data.boundary_domain_non_glare_right_hand_hlb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [object_distance_hlb]
    data.object_distance_hlb = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [status_boundary_domain_bottom_non_glare_hlb]
    data.status_boundary_domain_bottom_non_glare_hlb = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [status_boundary_domain_non_glare_left_hand_hlb]
    data.status_boundary_domain_non_glare_left_hand_hlb = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [status_boundary_domain_non_glare_right_hand_hlb]
    data.status_boundary_domain_non_glare_right_hand_hlb = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [status_object_distance_hlb]
    data.status_object_distance_hlb = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [left_target_change]
    data.left_target_change = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_target_change]
    data.right_target_change = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [too_many_cars]
    data.too_many_cars = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [busy_scene]
    data.busy_scene = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/AhbcGradual';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06801ea66cd7dc52de9867c12dbfa5bf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float32 boundary_domain_bottom_non_glare_hlb
    float32 boundary_domain_non_glare_left_hand_hlb
    float32 boundary_domain_non_glare_right_hand_hlb
    uint16 object_distance_hlb
    uint8 status_boundary_domain_bottom_non_glare_hlb
    uint8 status_boundary_domain_non_glare_left_hand_hlb
    uint8 status_boundary_domain_non_glare_right_hand_hlb
    uint8 status_object_distance_hlb
    bool left_target_change
    bool right_target_change
    bool too_many_cars
    bool busy_scene
    
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
    const resolved = new AhbcGradual(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.boundary_domain_bottom_non_glare_hlb !== undefined) {
      resolved.boundary_domain_bottom_non_glare_hlb = msg.boundary_domain_bottom_non_glare_hlb;
    }
    else {
      resolved.boundary_domain_bottom_non_glare_hlb = 0.0
    }

    if (msg.boundary_domain_non_glare_left_hand_hlb !== undefined) {
      resolved.boundary_domain_non_glare_left_hand_hlb = msg.boundary_domain_non_glare_left_hand_hlb;
    }
    else {
      resolved.boundary_domain_non_glare_left_hand_hlb = 0.0
    }

    if (msg.boundary_domain_non_glare_right_hand_hlb !== undefined) {
      resolved.boundary_domain_non_glare_right_hand_hlb = msg.boundary_domain_non_glare_right_hand_hlb;
    }
    else {
      resolved.boundary_domain_non_glare_right_hand_hlb = 0.0
    }

    if (msg.object_distance_hlb !== undefined) {
      resolved.object_distance_hlb = msg.object_distance_hlb;
    }
    else {
      resolved.object_distance_hlb = 0
    }

    if (msg.status_boundary_domain_bottom_non_glare_hlb !== undefined) {
      resolved.status_boundary_domain_bottom_non_glare_hlb = msg.status_boundary_domain_bottom_non_glare_hlb;
    }
    else {
      resolved.status_boundary_domain_bottom_non_glare_hlb = 0
    }

    if (msg.status_boundary_domain_non_glare_left_hand_hlb !== undefined) {
      resolved.status_boundary_domain_non_glare_left_hand_hlb = msg.status_boundary_domain_non_glare_left_hand_hlb;
    }
    else {
      resolved.status_boundary_domain_non_glare_left_hand_hlb = 0
    }

    if (msg.status_boundary_domain_non_glare_right_hand_hlb !== undefined) {
      resolved.status_boundary_domain_non_glare_right_hand_hlb = msg.status_boundary_domain_non_glare_right_hand_hlb;
    }
    else {
      resolved.status_boundary_domain_non_glare_right_hand_hlb = 0
    }

    if (msg.status_object_distance_hlb !== undefined) {
      resolved.status_object_distance_hlb = msg.status_object_distance_hlb;
    }
    else {
      resolved.status_object_distance_hlb = 0
    }

    if (msg.left_target_change !== undefined) {
      resolved.left_target_change = msg.left_target_change;
    }
    else {
      resolved.left_target_change = false
    }

    if (msg.right_target_change !== undefined) {
      resolved.right_target_change = msg.right_target_change;
    }
    else {
      resolved.right_target_change = false
    }

    if (msg.too_many_cars !== undefined) {
      resolved.too_many_cars = msg.too_many_cars;
    }
    else {
      resolved.too_many_cars = false
    }

    if (msg.busy_scene !== undefined) {
      resolved.busy_scene = msg.busy_scene;
    }
    else {
      resolved.busy_scene = false
    }

    return resolved;
    }
};

module.exports = AhbcGradual;
