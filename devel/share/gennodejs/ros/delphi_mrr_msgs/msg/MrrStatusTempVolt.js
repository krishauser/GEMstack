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

class MrrStatusTempVolt {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.can_batt_volts = null;
      this.can_1_25_v = null;
      this.can_5_v = null;
      this.can_3_3_v_raw = null;
      this.can_3_3_v_dac = null;
      this.can_mmic_temp1 = null;
      this.can_processor_thermistor = null;
      this.can_processor_temp1 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('can_batt_volts')) {
        this.can_batt_volts = initObj.can_batt_volts
      }
      else {
        this.can_batt_volts = 0.0;
      }
      if (initObj.hasOwnProperty('can_1_25_v')) {
        this.can_1_25_v = initObj.can_1_25_v
      }
      else {
        this.can_1_25_v = 0.0;
      }
      if (initObj.hasOwnProperty('can_5_v')) {
        this.can_5_v = initObj.can_5_v
      }
      else {
        this.can_5_v = 0.0;
      }
      if (initObj.hasOwnProperty('can_3_3_v_raw')) {
        this.can_3_3_v_raw = initObj.can_3_3_v_raw
      }
      else {
        this.can_3_3_v_raw = 0.0;
      }
      if (initObj.hasOwnProperty('can_3_3_v_dac')) {
        this.can_3_3_v_dac = initObj.can_3_3_v_dac
      }
      else {
        this.can_3_3_v_dac = 0.0;
      }
      if (initObj.hasOwnProperty('can_mmic_temp1')) {
        this.can_mmic_temp1 = initObj.can_mmic_temp1
      }
      else {
        this.can_mmic_temp1 = 0;
      }
      if (initObj.hasOwnProperty('can_processor_thermistor')) {
        this.can_processor_thermistor = initObj.can_processor_thermistor
      }
      else {
        this.can_processor_thermistor = 0;
      }
      if (initObj.hasOwnProperty('can_processor_temp1')) {
        this.can_processor_temp1 = initObj.can_processor_temp1
      }
      else {
        this.can_processor_temp1 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MrrStatusTempVolt
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [can_batt_volts]
    bufferOffset = _serializer.float32(obj.can_batt_volts, buffer, bufferOffset);
    // Serialize message field [can_1_25_v]
    bufferOffset = _serializer.float32(obj.can_1_25_v, buffer, bufferOffset);
    // Serialize message field [can_5_v]
    bufferOffset = _serializer.float32(obj.can_5_v, buffer, bufferOffset);
    // Serialize message field [can_3_3_v_raw]
    bufferOffset = _serializer.float32(obj.can_3_3_v_raw, buffer, bufferOffset);
    // Serialize message field [can_3_3_v_dac]
    bufferOffset = _serializer.float32(obj.can_3_3_v_dac, buffer, bufferOffset);
    // Serialize message field [can_mmic_temp1]
    bufferOffset = _serializer.int8(obj.can_mmic_temp1, buffer, bufferOffset);
    // Serialize message field [can_processor_thermistor]
    bufferOffset = _serializer.int8(obj.can_processor_thermistor, buffer, bufferOffset);
    // Serialize message field [can_processor_temp1]
    bufferOffset = _serializer.int8(obj.can_processor_temp1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MrrStatusTempVolt
    let len;
    let data = new MrrStatusTempVolt(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_batt_volts]
    data.can_batt_volts = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_1_25_v]
    data.can_1_25_v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_5_v]
    data.can_5_v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_3_3_v_raw]
    data.can_3_3_v_raw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_3_3_v_dac]
    data.can_3_3_v_dac = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [can_mmic_temp1]
    data.can_mmic_temp1 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [can_processor_thermistor]
    data.can_processor_thermistor = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [can_processor_temp1]
    data.can_processor_temp1 = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/MrrStatusTempVolt';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4200163646320431c4493e783122961d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float32 can_batt_volts
    float32 can_1_25_v
    float32 can_5_v
    float32 can_3_3_v_raw
    float32 can_3_3_v_dac
    int8    can_mmic_temp1
    int8    can_processor_thermistor
    int8    can_processor_temp1
    
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
    const resolved = new MrrStatusTempVolt(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.can_batt_volts !== undefined) {
      resolved.can_batt_volts = msg.can_batt_volts;
    }
    else {
      resolved.can_batt_volts = 0.0
    }

    if (msg.can_1_25_v !== undefined) {
      resolved.can_1_25_v = msg.can_1_25_v;
    }
    else {
      resolved.can_1_25_v = 0.0
    }

    if (msg.can_5_v !== undefined) {
      resolved.can_5_v = msg.can_5_v;
    }
    else {
      resolved.can_5_v = 0.0
    }

    if (msg.can_3_3_v_raw !== undefined) {
      resolved.can_3_3_v_raw = msg.can_3_3_v_raw;
    }
    else {
      resolved.can_3_3_v_raw = 0.0
    }

    if (msg.can_3_3_v_dac !== undefined) {
      resolved.can_3_3_v_dac = msg.can_3_3_v_dac;
    }
    else {
      resolved.can_3_3_v_dac = 0.0
    }

    if (msg.can_mmic_temp1 !== undefined) {
      resolved.can_mmic_temp1 = msg.can_mmic_temp1;
    }
    else {
      resolved.can_mmic_temp1 = 0
    }

    if (msg.can_processor_thermistor !== undefined) {
      resolved.can_processor_thermistor = msg.can_processor_thermistor;
    }
    else {
      resolved.can_processor_thermistor = 0
    }

    if (msg.can_processor_temp1 !== undefined) {
      resolved.can_processor_temp1 = msg.can_processor_temp1;
    }
    else {
      resolved.can_processor_temp1 = 0
    }

    return resolved;
    }
};

module.exports = MrrStatusTempVolt;
