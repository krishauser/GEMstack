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

class RangeInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prn_number = null;
      this.glofreq = null;
      this.psr = null;
      this.psr_std = null;
      this.adr = null;
      this.adr_std = null;
      this.dopp = null;
      this.noise_density_ratio = null;
      this.locktime = null;
      this.tracking_status = null;
    }
    else {
      if (initObj.hasOwnProperty('prn_number')) {
        this.prn_number = initObj.prn_number
      }
      else {
        this.prn_number = 0;
      }
      if (initObj.hasOwnProperty('glofreq')) {
        this.glofreq = initObj.glofreq
      }
      else {
        this.glofreq = 0;
      }
      if (initObj.hasOwnProperty('psr')) {
        this.psr = initObj.psr
      }
      else {
        this.psr = 0.0;
      }
      if (initObj.hasOwnProperty('psr_std')) {
        this.psr_std = initObj.psr_std
      }
      else {
        this.psr_std = 0.0;
      }
      if (initObj.hasOwnProperty('adr')) {
        this.adr = initObj.adr
      }
      else {
        this.adr = 0.0;
      }
      if (initObj.hasOwnProperty('adr_std')) {
        this.adr_std = initObj.adr_std
      }
      else {
        this.adr_std = 0.0;
      }
      if (initObj.hasOwnProperty('dopp')) {
        this.dopp = initObj.dopp
      }
      else {
        this.dopp = 0.0;
      }
      if (initObj.hasOwnProperty('noise_density_ratio')) {
        this.noise_density_ratio = initObj.noise_density_ratio
      }
      else {
        this.noise_density_ratio = 0.0;
      }
      if (initObj.hasOwnProperty('locktime')) {
        this.locktime = initObj.locktime
      }
      else {
        this.locktime = 0.0;
      }
      if (initObj.hasOwnProperty('tracking_status')) {
        this.tracking_status = initObj.tracking_status
      }
      else {
        this.tracking_status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RangeInformation
    // Serialize message field [prn_number]
    bufferOffset = _serializer.uint16(obj.prn_number, buffer, bufferOffset);
    // Serialize message field [glofreq]
    bufferOffset = _serializer.uint16(obj.glofreq, buffer, bufferOffset);
    // Serialize message field [psr]
    bufferOffset = _serializer.float64(obj.psr, buffer, bufferOffset);
    // Serialize message field [psr_std]
    bufferOffset = _serializer.float32(obj.psr_std, buffer, bufferOffset);
    // Serialize message field [adr]
    bufferOffset = _serializer.float64(obj.adr, buffer, bufferOffset);
    // Serialize message field [adr_std]
    bufferOffset = _serializer.float32(obj.adr_std, buffer, bufferOffset);
    // Serialize message field [dopp]
    bufferOffset = _serializer.float32(obj.dopp, buffer, bufferOffset);
    // Serialize message field [noise_density_ratio]
    bufferOffset = _serializer.float32(obj.noise_density_ratio, buffer, bufferOffset);
    // Serialize message field [locktime]
    bufferOffset = _serializer.float32(obj.locktime, buffer, bufferOffset);
    // Serialize message field [tracking_status]
    bufferOffset = _serializer.uint32(obj.tracking_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RangeInformation
    let len;
    let data = new RangeInformation(null);
    // Deserialize message field [prn_number]
    data.prn_number = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [glofreq]
    data.glofreq = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [psr]
    data.psr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [psr_std]
    data.psr_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [adr]
    data.adr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [adr_std]
    data.adr_std = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dopp]
    data.dopp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [noise_density_ratio]
    data.noise_density_ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [locktime]
    data.locktime = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tracking_status]
    data.tracking_status = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/RangeInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c29299d245fc707e8f7544af871f110';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Satellite Range information structure used in range messages
    
    #Satelite PRN number of range measurement
    uint16 prn_number
    
    #GLONASS Frequency
    uint16 glofreq
    
    #Pseudorange measurement(m)
    float64 psr
    
    #Pseudorange measurement standard deviation(m)
    float32 psr_std
    
    #Carrier phase, in cycles
    float64 adr
    
    #Estimated carrier phase standard deviation(cycles)
    float32 adr_std
    
    #Instantaneous carrier Doppler frequency(Hz)
    float32 dopp
    
    #Carrier to noise density ratio
    float32 noise_density_ratio
    
    ## of seconds of continous tracking
    float32 locktime
    
    #Tracking status
    uint32 tracking_status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RangeInformation(null);
    if (msg.prn_number !== undefined) {
      resolved.prn_number = msg.prn_number;
    }
    else {
      resolved.prn_number = 0
    }

    if (msg.glofreq !== undefined) {
      resolved.glofreq = msg.glofreq;
    }
    else {
      resolved.glofreq = 0
    }

    if (msg.psr !== undefined) {
      resolved.psr = msg.psr;
    }
    else {
      resolved.psr = 0.0
    }

    if (msg.psr_std !== undefined) {
      resolved.psr_std = msg.psr_std;
    }
    else {
      resolved.psr_std = 0.0
    }

    if (msg.adr !== undefined) {
      resolved.adr = msg.adr;
    }
    else {
      resolved.adr = 0.0
    }

    if (msg.adr_std !== undefined) {
      resolved.adr_std = msg.adr_std;
    }
    else {
      resolved.adr_std = 0.0
    }

    if (msg.dopp !== undefined) {
      resolved.dopp = msg.dopp;
    }
    else {
      resolved.dopp = 0.0
    }

    if (msg.noise_density_ratio !== undefined) {
      resolved.noise_density_ratio = msg.noise_density_ratio;
    }
    else {
      resolved.noise_density_ratio = 0.0
    }

    if (msg.locktime !== undefined) {
      resolved.locktime = msg.locktime;
    }
    else {
      resolved.locktime = 0.0
    }

    if (msg.tracking_status !== undefined) {
      resolved.tracking_status = msg.tracking_status;
    }
    else {
      resolved.tracking_status = 0
    }

    return resolved;
    }
};

module.exports = RangeInformation;
