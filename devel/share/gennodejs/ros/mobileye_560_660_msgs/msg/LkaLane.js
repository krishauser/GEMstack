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

class LkaLane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lane_type = null;
      this.quality = null;
      this.model_degree = null;
      this.position_parameter_c0 = null;
      this.curvature_parameter_c2 = null;
      this.curvature_derivative_parameter_c3 = null;
      this.marking_width = null;
      this.heading_angle_parameter_c1 = null;
      this.view_range = null;
      this.view_range_availability = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lane_type')) {
        this.lane_type = initObj.lane_type
      }
      else {
        this.lane_type = 0;
      }
      if (initObj.hasOwnProperty('quality')) {
        this.quality = initObj.quality
      }
      else {
        this.quality = 0;
      }
      if (initObj.hasOwnProperty('model_degree')) {
        this.model_degree = initObj.model_degree
      }
      else {
        this.model_degree = 0;
      }
      if (initObj.hasOwnProperty('position_parameter_c0')) {
        this.position_parameter_c0 = initObj.position_parameter_c0
      }
      else {
        this.position_parameter_c0 = 0.0;
      }
      if (initObj.hasOwnProperty('curvature_parameter_c2')) {
        this.curvature_parameter_c2 = initObj.curvature_parameter_c2
      }
      else {
        this.curvature_parameter_c2 = 0.0;
      }
      if (initObj.hasOwnProperty('curvature_derivative_parameter_c3')) {
        this.curvature_derivative_parameter_c3 = initObj.curvature_derivative_parameter_c3
      }
      else {
        this.curvature_derivative_parameter_c3 = 0.0;
      }
      if (initObj.hasOwnProperty('marking_width')) {
        this.marking_width = initObj.marking_width
      }
      else {
        this.marking_width = 0.0;
      }
      if (initObj.hasOwnProperty('heading_angle_parameter_c1')) {
        this.heading_angle_parameter_c1 = initObj.heading_angle_parameter_c1
      }
      else {
        this.heading_angle_parameter_c1 = 0.0;
      }
      if (initObj.hasOwnProperty('view_range')) {
        this.view_range = initObj.view_range
      }
      else {
        this.view_range = 0.0;
      }
      if (initObj.hasOwnProperty('view_range_availability')) {
        this.view_range_availability = initObj.view_range_availability
      }
      else {
        this.view_range_availability = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LkaLane
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lane_type]
    bufferOffset = _serializer.uint8(obj.lane_type, buffer, bufferOffset);
    // Serialize message field [quality]
    bufferOffset = _serializer.uint8(obj.quality, buffer, bufferOffset);
    // Serialize message field [model_degree]
    bufferOffset = _serializer.uint8(obj.model_degree, buffer, bufferOffset);
    // Serialize message field [position_parameter_c0]
    bufferOffset = _serializer.float64(obj.position_parameter_c0, buffer, bufferOffset);
    // Serialize message field [curvature_parameter_c2]
    bufferOffset = _serializer.float64(obj.curvature_parameter_c2, buffer, bufferOffset);
    // Serialize message field [curvature_derivative_parameter_c3]
    bufferOffset = _serializer.float64(obj.curvature_derivative_parameter_c3, buffer, bufferOffset);
    // Serialize message field [marking_width]
    bufferOffset = _serializer.float32(obj.marking_width, buffer, bufferOffset);
    // Serialize message field [heading_angle_parameter_c1]
    bufferOffset = _serializer.float64(obj.heading_angle_parameter_c1, buffer, bufferOffset);
    // Serialize message field [view_range]
    bufferOffset = _serializer.float32(obj.view_range, buffer, bufferOffset);
    // Serialize message field [view_range_availability]
    bufferOffset = _serializer.bool(obj.view_range_availability, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LkaLane
    let len;
    let data = new LkaLane(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_type]
    data.lane_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [quality]
    data.quality = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [model_degree]
    data.model_degree = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [position_parameter_c0]
    data.position_parameter_c0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [curvature_parameter_c2]
    data.curvature_parameter_c2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [curvature_derivative_parameter_c3]
    data.curvature_derivative_parameter_c3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [marking_width]
    data.marking_width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_angle_parameter_c1]
    data.heading_angle_parameter_c1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [view_range]
    data.view_range = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [view_range_availability]
    data.view_range_availability = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/LkaLane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13c7b357c14488be92473cab7e5461ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 LANE_CONFIDENCE_NONE = 0
    uint8 LANE_CONFIDENCE_LOW = 1
    uint8 LANE_CONFIDENCE_MED = 2
    uint8 LANE_CONFIDENCE_HIGH = 3
    
    uint8 LANE_TYPE_DASHED = 0
    uint8 LANE_TYPE_SOLID = 1
    uint8 LANE_TYPE_NONE = 2
    uint8 LANE_TYPE_ROAD_EDGE = 3
    uint8 LANE_TYPE_DOUBLE_LANE_MARK = 4
    uint8 LANE_TYPE_BOTTS_DOTS = 5
    uint8 LANE_TYPE_INVALID = 6
    
    uint8 lane_type
    uint8 quality
    uint8 model_degree
    float64 position_parameter_c0
    float64 curvature_parameter_c2
    float64 curvature_derivative_parameter_c3
    float32 marking_width
    float64 heading_angle_parameter_c1
    float32 view_range
    bool view_range_availability
    
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
    const resolved = new LkaLane(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lane_type !== undefined) {
      resolved.lane_type = msg.lane_type;
    }
    else {
      resolved.lane_type = 0
    }

    if (msg.quality !== undefined) {
      resolved.quality = msg.quality;
    }
    else {
      resolved.quality = 0
    }

    if (msg.model_degree !== undefined) {
      resolved.model_degree = msg.model_degree;
    }
    else {
      resolved.model_degree = 0
    }

    if (msg.position_parameter_c0 !== undefined) {
      resolved.position_parameter_c0 = msg.position_parameter_c0;
    }
    else {
      resolved.position_parameter_c0 = 0.0
    }

    if (msg.curvature_parameter_c2 !== undefined) {
      resolved.curvature_parameter_c2 = msg.curvature_parameter_c2;
    }
    else {
      resolved.curvature_parameter_c2 = 0.0
    }

    if (msg.curvature_derivative_parameter_c3 !== undefined) {
      resolved.curvature_derivative_parameter_c3 = msg.curvature_derivative_parameter_c3;
    }
    else {
      resolved.curvature_derivative_parameter_c3 = 0.0
    }

    if (msg.marking_width !== undefined) {
      resolved.marking_width = msg.marking_width;
    }
    else {
      resolved.marking_width = 0.0
    }

    if (msg.heading_angle_parameter_c1 !== undefined) {
      resolved.heading_angle_parameter_c1 = msg.heading_angle_parameter_c1;
    }
    else {
      resolved.heading_angle_parameter_c1 = 0.0
    }

    if (msg.view_range !== undefined) {
      resolved.view_range = msg.view_range;
    }
    else {
      resolved.view_range = 0.0
    }

    if (msg.view_range_availability !== undefined) {
      resolved.view_range_availability = msg.view_range_availability;
    }
    else {
      resolved.view_range_availability = false
    }

    return resolved;
    }
};

// Constants for message
LkaLane.Constants = {
  LANE_CONFIDENCE_NONE: 0,
  LANE_CONFIDENCE_LOW: 1,
  LANE_CONFIDENCE_MED: 2,
  LANE_CONFIDENCE_HIGH: 3,
  LANE_TYPE_DASHED: 0,
  LANE_TYPE_SOLID: 1,
  LANE_TYPE_NONE: 2,
  LANE_TYPE_ROAD_EDGE: 3,
  LANE_TYPE_DOUBLE_LANE_MARK: 4,
  LANE_TYPE_BOTTS_DOTS: 5,
  LANE_TYPE_INVALID: 6,
}

module.exports = LkaLane;
