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

class VehicleStateMsg2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.fsm_yaw_rate_valid = null;
      this.fsm_yaw_rate = null;
      this.can_vehicle_index_4fa = null;
      this.fsm_vehicle_velocity = null;
      this.can_steering_whl_angle_qf = null;
      this.fsm_vehicle_velocity_valid = null;
      this.can_steering_whl_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('fsm_yaw_rate_valid')) {
        this.fsm_yaw_rate_valid = initObj.fsm_yaw_rate_valid
      }
      else {
        this.fsm_yaw_rate_valid = false;
      }
      if (initObj.hasOwnProperty('fsm_yaw_rate')) {
        this.fsm_yaw_rate = initObj.fsm_yaw_rate
      }
      else {
        this.fsm_yaw_rate = 0.0;
      }
      if (initObj.hasOwnProperty('can_vehicle_index_4fa')) {
        this.can_vehicle_index_4fa = initObj.can_vehicle_index_4fa
      }
      else {
        this.can_vehicle_index_4fa = 0;
      }
      if (initObj.hasOwnProperty('fsm_vehicle_velocity')) {
        this.fsm_vehicle_velocity = initObj.fsm_vehicle_velocity
      }
      else {
        this.fsm_vehicle_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('can_steering_whl_angle_qf')) {
        this.can_steering_whl_angle_qf = initObj.can_steering_whl_angle_qf
      }
      else {
        this.can_steering_whl_angle_qf = false;
      }
      if (initObj.hasOwnProperty('fsm_vehicle_velocity_valid')) {
        this.fsm_vehicle_velocity_valid = initObj.fsm_vehicle_velocity_valid
      }
      else {
        this.fsm_vehicle_velocity_valid = false;
      }
      if (initObj.hasOwnProperty('can_steering_whl_angle')) {
        this.can_steering_whl_angle = initObj.can_steering_whl_angle
      }
      else {
        this.can_steering_whl_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleStateMsg2
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [fsm_yaw_rate_valid]
    bufferOffset = _serializer.bool(obj.fsm_yaw_rate_valid, buffer, bufferOffset);
    // Serialize message field [fsm_yaw_rate]
    bufferOffset = _serializer.float32(obj.fsm_yaw_rate, buffer, bufferOffset);
    // Serialize message field [can_vehicle_index_4fa]
    bufferOffset = _serializer.uint16(obj.can_vehicle_index_4fa, buffer, bufferOffset);
    // Serialize message field [fsm_vehicle_velocity]
    bufferOffset = _serializer.float32(obj.fsm_vehicle_velocity, buffer, bufferOffset);
    // Serialize message field [can_steering_whl_angle_qf]
    bufferOffset = _serializer.bool(obj.can_steering_whl_angle_qf, buffer, bufferOffset);
    // Serialize message field [fsm_vehicle_velocity_valid]
    bufferOffset = _serializer.bool(obj.fsm_vehicle_velocity_valid, buffer, bufferOffset);
    // Serialize message field [can_steering_whl_angle]
    bufferOffset = _serializer.float32(obj.can_steering_whl_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleStateMsg2
    let len;
    let data = new VehicleStateMsg2(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [fsm_yaw_rate_valid]
    data.fsm_yaw_rate_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fsm_yaw_rate]
    data.fsm_yaw_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_vehicle_index_4fa]
    data.can_vehicle_index_4fa = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [fsm_vehicle_velocity]
    data.fsm_vehicle_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_steering_whl_angle_qf]
    data.can_steering_whl_angle_qf = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fsm_vehicle_velocity_valid]
    data.fsm_vehicle_velocity_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [can_steering_whl_angle]
    data.can_steering_whl_angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/VehicleStateMsg2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54463690e9fc3e6c8708c99c08ba6c62';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool    fsm_yaw_rate_valid
    float32 fsm_yaw_rate
    uint16  can_vehicle_index_4fa
    float32 fsm_vehicle_velocity
    bool    can_steering_whl_angle_qf
    bool    fsm_vehicle_velocity_valid
    float32 can_steering_whl_angle
    
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
    const resolved = new VehicleStateMsg2(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.fsm_yaw_rate_valid !== undefined) {
      resolved.fsm_yaw_rate_valid = msg.fsm_yaw_rate_valid;
    }
    else {
      resolved.fsm_yaw_rate_valid = false
    }

    if (msg.fsm_yaw_rate !== undefined) {
      resolved.fsm_yaw_rate = msg.fsm_yaw_rate;
    }
    else {
      resolved.fsm_yaw_rate = 0.0
    }

    if (msg.can_vehicle_index_4fa !== undefined) {
      resolved.can_vehicle_index_4fa = msg.can_vehicle_index_4fa;
    }
    else {
      resolved.can_vehicle_index_4fa = 0
    }

    if (msg.fsm_vehicle_velocity !== undefined) {
      resolved.fsm_vehicle_velocity = msg.fsm_vehicle_velocity;
    }
    else {
      resolved.fsm_vehicle_velocity = 0.0
    }

    if (msg.can_steering_whl_angle_qf !== undefined) {
      resolved.can_steering_whl_angle_qf = msg.can_steering_whl_angle_qf;
    }
    else {
      resolved.can_steering_whl_angle_qf = false
    }

    if (msg.fsm_vehicle_velocity_valid !== undefined) {
      resolved.fsm_vehicle_velocity_valid = msg.fsm_vehicle_velocity_valid;
    }
    else {
      resolved.fsm_vehicle_velocity_valid = false
    }

    if (msg.can_steering_whl_angle !== undefined) {
      resolved.can_steering_whl_angle = msg.can_steering_whl_angle;
    }
    else {
      resolved.can_steering_whl_angle = 0.0
    }

    return resolved;
    }
};

module.exports = VehicleStateMsg2;
