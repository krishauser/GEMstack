// Auto-generated. Do not edit!

// (in-package novatel_gps_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ClockSteering {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.source = null;
      this.steering_state = null;
      this.period = null;
      this.pulse_width = null;
      this.bandwidth = null;
      this.slope = null;
      this.offset = null;
      this.drift_rate = null;
    }
    else {
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = '';
      }
      if (initObj.hasOwnProperty('steering_state')) {
        this.steering_state = initObj.steering_state
      }
      else {
        this.steering_state = '';
      }
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = 0;
      }
      if (initObj.hasOwnProperty('pulse_width')) {
        this.pulse_width = initObj.pulse_width
      }
      else {
        this.pulse_width = 0.0;
      }
      if (initObj.hasOwnProperty('bandwidth')) {
        this.bandwidth = initObj.bandwidth
      }
      else {
        this.bandwidth = 0.0;
      }
      if (initObj.hasOwnProperty('slope')) {
        this.slope = initObj.slope
      }
      else {
        this.slope = 0.0;
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = 0.0;
      }
      if (initObj.hasOwnProperty('drift_rate')) {
        this.drift_rate = initObj.drift_rate
      }
      else {
        this.drift_rate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ClockSteering
    // Serialize message field [source]
    bufferOffset = _serializer.string(obj.source, buffer, bufferOffset);
    // Serialize message field [steering_state]
    bufferOffset = _serializer.string(obj.steering_state, buffer, bufferOffset);
    // Serialize message field [period]
    bufferOffset = _serializer.uint32(obj.period, buffer, bufferOffset);
    // Serialize message field [pulse_width]
    bufferOffset = _serializer.float64(obj.pulse_width, buffer, bufferOffset);
    // Serialize message field [bandwidth]
    bufferOffset = _serializer.float64(obj.bandwidth, buffer, bufferOffset);
    // Serialize message field [slope]
    bufferOffset = _serializer.float32(obj.slope, buffer, bufferOffset);
    // Serialize message field [offset]
    bufferOffset = _serializer.float64(obj.offset, buffer, bufferOffset);
    // Serialize message field [drift_rate]
    bufferOffset = _serializer.float64(obj.drift_rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClockSteering
    let len;
    let data = new ClockSteering(null);
    // Deserialize message field [source]
    data.source = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [steering_state]
    data.steering_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [period]
    data.period = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pulse_width]
    data.pulse_width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bandwidth]
    data.bandwidth = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [slope]
    data.slope = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [offset]
    data.offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [drift_rate]
    data.drift_rate = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.source);
    length += _getByteLength(object.steering_state);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/ClockSteering';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03024ea60365b742dd5e56411830735e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The CLOCKSTEERING log is used to monitor the current state of the clock steering process.
    
    int8 INTERNAL_SOURCE=0
    int8 EXTERNAL_SOURCE=1
    
    int8 FIRST_ORDER_STEERING_STATE=0
    int8 SECOND_ORDER_STEERING_STATE=1
    int8 CALIBRATE_HIGH_STEERING_STATE=2
    int8 CALIBRATE_LOW_STEERING_STATE=3
    int8 CALIBRATE_CENTER_STEERING_STATE=4
    
    # Clock source
    string source
    
    # Steering state
    string steering_state
    
    # Period of the FREQUENCYOUT signal used to control the oscillator
    uint32 period
    
    # Current pulse width of the FREQUENCYOUT signal. 
    float64 pulse_width
    
    # The current band width of the clock steering tracking loop in Hz.
    float64 bandwidth
    
    # The current clock drift change in m/s/bit for a 1 LSB pulse width. 
    float32 slope
    
    # The last valid receiver clock offset computed (m).
    float64 offset
    
    # The last valid receiver clock drift rate received (m/s).
    float64 drift_rate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ClockSteering(null);
    if (msg.source !== undefined) {
      resolved.source = msg.source;
    }
    else {
      resolved.source = ''
    }

    if (msg.steering_state !== undefined) {
      resolved.steering_state = msg.steering_state;
    }
    else {
      resolved.steering_state = ''
    }

    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = 0
    }

    if (msg.pulse_width !== undefined) {
      resolved.pulse_width = msg.pulse_width;
    }
    else {
      resolved.pulse_width = 0.0
    }

    if (msg.bandwidth !== undefined) {
      resolved.bandwidth = msg.bandwidth;
    }
    else {
      resolved.bandwidth = 0.0
    }

    if (msg.slope !== undefined) {
      resolved.slope = msg.slope;
    }
    else {
      resolved.slope = 0.0
    }

    if (msg.offset !== undefined) {
      resolved.offset = msg.offset;
    }
    else {
      resolved.offset = 0.0
    }

    if (msg.drift_rate !== undefined) {
      resolved.drift_rate = msg.drift_rate;
    }
    else {
      resolved.drift_rate = 0.0
    }

    return resolved;
    }
};

// Constants for message
ClockSteering.Constants = {
  INTERNAL_SOURCE: 0,
  EXTERNAL_SOURCE: 1,
  FIRST_ORDER_STEERING_STATE: 0,
  SECOND_ORDER_STEERING_STATE: 1,
  CALIBRATE_HIGH_STEERING_STATE: 2,
  CALIBRATE_LOW_STEERING_STATE: 3,
  CALIBRATE_CENTER_STEERING_STATE: 4,
}

module.exports = ClockSteering;
