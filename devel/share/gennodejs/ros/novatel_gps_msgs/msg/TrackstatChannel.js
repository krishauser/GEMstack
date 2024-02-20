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

class TrackstatChannel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prn = null;
      this.glofreq = null;
      this.ch_tr_status = null;
      this.psr = null;
      this.doppler = null;
      this.c_no = null;
      this.locktime = null;
      this.psr_res = null;
      this.reject = null;
      this.psr_weight = null;
    }
    else {
      if (initObj.hasOwnProperty('prn')) {
        this.prn = initObj.prn
      }
      else {
        this.prn = 0;
      }
      if (initObj.hasOwnProperty('glofreq')) {
        this.glofreq = initObj.glofreq
      }
      else {
        this.glofreq = 0;
      }
      if (initObj.hasOwnProperty('ch_tr_status')) {
        this.ch_tr_status = initObj.ch_tr_status
      }
      else {
        this.ch_tr_status = 0;
      }
      if (initObj.hasOwnProperty('psr')) {
        this.psr = initObj.psr
      }
      else {
        this.psr = 0.0;
      }
      if (initObj.hasOwnProperty('doppler')) {
        this.doppler = initObj.doppler
      }
      else {
        this.doppler = 0.0;
      }
      if (initObj.hasOwnProperty('c_no')) {
        this.c_no = initObj.c_no
      }
      else {
        this.c_no = 0.0;
      }
      if (initObj.hasOwnProperty('locktime')) {
        this.locktime = initObj.locktime
      }
      else {
        this.locktime = 0.0;
      }
      if (initObj.hasOwnProperty('psr_res')) {
        this.psr_res = initObj.psr_res
      }
      else {
        this.psr_res = 0.0;
      }
      if (initObj.hasOwnProperty('reject')) {
        this.reject = initObj.reject
      }
      else {
        this.reject = '';
      }
      if (initObj.hasOwnProperty('psr_weight')) {
        this.psr_weight = initObj.psr_weight
      }
      else {
        this.psr_weight = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackstatChannel
    // Serialize message field [prn]
    bufferOffset = _serializer.int16(obj.prn, buffer, bufferOffset);
    // Serialize message field [glofreq]
    bufferOffset = _serializer.int16(obj.glofreq, buffer, bufferOffset);
    // Serialize message field [ch_tr_status]
    bufferOffset = _serializer.uint32(obj.ch_tr_status, buffer, bufferOffset);
    // Serialize message field [psr]
    bufferOffset = _serializer.float64(obj.psr, buffer, bufferOffset);
    // Serialize message field [doppler]
    bufferOffset = _serializer.float32(obj.doppler, buffer, bufferOffset);
    // Serialize message field [c_no]
    bufferOffset = _serializer.float32(obj.c_no, buffer, bufferOffset);
    // Serialize message field [locktime]
    bufferOffset = _serializer.float32(obj.locktime, buffer, bufferOffset);
    // Serialize message field [psr_res]
    bufferOffset = _serializer.float32(obj.psr_res, buffer, bufferOffset);
    // Serialize message field [reject]
    bufferOffset = _serializer.string(obj.reject, buffer, bufferOffset);
    // Serialize message field [psr_weight]
    bufferOffset = _serializer.float32(obj.psr_weight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackstatChannel
    let len;
    let data = new TrackstatChannel(null);
    // Deserialize message field [prn]
    data.prn = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [glofreq]
    data.glofreq = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [ch_tr_status]
    data.ch_tr_status = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [psr]
    data.psr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [doppler]
    data.doppler = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [c_no]
    data.c_no = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [locktime]
    data.locktime = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psr_res]
    data.psr_res = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reject]
    data.reject = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [psr_weight]
    data.psr_weight = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.reject);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'novatel_gps_msgs/TrackstatChannel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '295831118c5ddfb83ac5b655586ae7ef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # A submessage of Trackstat that contains all of the data about a single hardware channel
    
    # Satellite PRN number
    int16 prn
    
    # GLONASS Frequency +7
    int16 glofreq
    
    # Channel tracking status
    uint32 ch_tr_status
    
    # Pseudorange (m)
    float64 psr
    
    # Doppler frequency (Hz)
    float32 doppler
    
    # Carrier to noise density ratio (dB-Hz)
    float32 c_no
    
    # Number of seconds of continuous tracking (no cycle slips)
    float32 locktime
    
    # Pseudorange residual from pseudorange filter (m)
    float32 psr_res
    
    # Range reject code from pseudorange filter
    string reject
    
    # Pseudorange filter weighting
    float32 psr_weight
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackstatChannel(null);
    if (msg.prn !== undefined) {
      resolved.prn = msg.prn;
    }
    else {
      resolved.prn = 0
    }

    if (msg.glofreq !== undefined) {
      resolved.glofreq = msg.glofreq;
    }
    else {
      resolved.glofreq = 0
    }

    if (msg.ch_tr_status !== undefined) {
      resolved.ch_tr_status = msg.ch_tr_status;
    }
    else {
      resolved.ch_tr_status = 0
    }

    if (msg.psr !== undefined) {
      resolved.psr = msg.psr;
    }
    else {
      resolved.psr = 0.0
    }

    if (msg.doppler !== undefined) {
      resolved.doppler = msg.doppler;
    }
    else {
      resolved.doppler = 0.0
    }

    if (msg.c_no !== undefined) {
      resolved.c_no = msg.c_no;
    }
    else {
      resolved.c_no = 0.0
    }

    if (msg.locktime !== undefined) {
      resolved.locktime = msg.locktime;
    }
    else {
      resolved.locktime = 0.0
    }

    if (msg.psr_res !== undefined) {
      resolved.psr_res = msg.psr_res;
    }
    else {
      resolved.psr_res = 0.0
    }

    if (msg.reject !== undefined) {
      resolved.reject = msg.reject;
    }
    else {
      resolved.reject = ''
    }

    if (msg.psr_weight !== undefined) {
      resolved.psr_weight = msg.psr_weight;
    }
    else {
      resolved.psr_weight = 0.0
    }

    return resolved;
    }
};

module.exports = TrackstatChannel;
