// Auto-generated. Do not edit!

// (in-package delphi_srr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SrrFeatureAlert {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lcma_blis_ignored_track_id = null;
      this.lcma_blis_track_id = null;
      this.lcma_cvw_ttc = null;
      this.cta_ttc_alert = null;
      this.cta_selected_track_ttc = null;
      this.cta_selected_track = null;
      this.cta_alert = null;
      this.cta_active = null;
      this.lcma_cvw_cipv = null;
      this.lcma_cvw_alert_state = null;
      this.lcma_blis_alert_state = null;
      this.lcma_active = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lcma_blis_ignored_track_id')) {
        this.lcma_blis_ignored_track_id = initObj.lcma_blis_ignored_track_id
      }
      else {
        this.lcma_blis_ignored_track_id = 0;
      }
      if (initObj.hasOwnProperty('lcma_blis_track_id')) {
        this.lcma_blis_track_id = initObj.lcma_blis_track_id
      }
      else {
        this.lcma_blis_track_id = 0;
      }
      if (initObj.hasOwnProperty('lcma_cvw_ttc')) {
        this.lcma_cvw_ttc = initObj.lcma_cvw_ttc
      }
      else {
        this.lcma_cvw_ttc = 0.0;
      }
      if (initObj.hasOwnProperty('cta_ttc_alert')) {
        this.cta_ttc_alert = initObj.cta_ttc_alert
      }
      else {
        this.cta_ttc_alert = false;
      }
      if (initObj.hasOwnProperty('cta_selected_track_ttc')) {
        this.cta_selected_track_ttc = initObj.cta_selected_track_ttc
      }
      else {
        this.cta_selected_track_ttc = 0.0;
      }
      if (initObj.hasOwnProperty('cta_selected_track')) {
        this.cta_selected_track = initObj.cta_selected_track
      }
      else {
        this.cta_selected_track = 0;
      }
      if (initObj.hasOwnProperty('cta_alert')) {
        this.cta_alert = initObj.cta_alert
      }
      else {
        this.cta_alert = 0;
      }
      if (initObj.hasOwnProperty('cta_active')) {
        this.cta_active = initObj.cta_active
      }
      else {
        this.cta_active = false;
      }
      if (initObj.hasOwnProperty('lcma_cvw_cipv')) {
        this.lcma_cvw_cipv = initObj.lcma_cvw_cipv
      }
      else {
        this.lcma_cvw_cipv = 0;
      }
      if (initObj.hasOwnProperty('lcma_cvw_alert_state')) {
        this.lcma_cvw_alert_state = initObj.lcma_cvw_alert_state
      }
      else {
        this.lcma_cvw_alert_state = 0;
      }
      if (initObj.hasOwnProperty('lcma_blis_alert_state')) {
        this.lcma_blis_alert_state = initObj.lcma_blis_alert_state
      }
      else {
        this.lcma_blis_alert_state = 0;
      }
      if (initObj.hasOwnProperty('lcma_active')) {
        this.lcma_active = initObj.lcma_active
      }
      else {
        this.lcma_active = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrrFeatureAlert
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lcma_blis_ignored_track_id]
    bufferOffset = _serializer.uint8(obj.lcma_blis_ignored_track_id, buffer, bufferOffset);
    // Serialize message field [lcma_blis_track_id]
    bufferOffset = _serializer.uint8(obj.lcma_blis_track_id, buffer, bufferOffset);
    // Serialize message field [lcma_cvw_ttc]
    bufferOffset = _serializer.float32(obj.lcma_cvw_ttc, buffer, bufferOffset);
    // Serialize message field [cta_ttc_alert]
    bufferOffset = _serializer.bool(obj.cta_ttc_alert, buffer, bufferOffset);
    // Serialize message field [cta_selected_track_ttc]
    bufferOffset = _serializer.float32(obj.cta_selected_track_ttc, buffer, bufferOffset);
    // Serialize message field [cta_selected_track]
    bufferOffset = _serializer.uint16(obj.cta_selected_track, buffer, bufferOffset);
    // Serialize message field [cta_alert]
    bufferOffset = _serializer.uint8(obj.cta_alert, buffer, bufferOffset);
    // Serialize message field [cta_active]
    bufferOffset = _serializer.bool(obj.cta_active, buffer, bufferOffset);
    // Serialize message field [lcma_cvw_cipv]
    bufferOffset = _serializer.uint8(obj.lcma_cvw_cipv, buffer, bufferOffset);
    // Serialize message field [lcma_cvw_alert_state]
    bufferOffset = _serializer.uint8(obj.lcma_cvw_alert_state, buffer, bufferOffset);
    // Serialize message field [lcma_blis_alert_state]
    bufferOffset = _serializer.uint8(obj.lcma_blis_alert_state, buffer, bufferOffset);
    // Serialize message field [lcma_active]
    bufferOffset = _serializer.bool(obj.lcma_active, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrrFeatureAlert
    let len;
    let data = new SrrFeatureAlert(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lcma_blis_ignored_track_id]
    data.lcma_blis_ignored_track_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lcma_blis_track_id]
    data.lcma_blis_track_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lcma_cvw_ttc]
    data.lcma_cvw_ttc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cta_ttc_alert]
    data.cta_ttc_alert = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cta_selected_track_ttc]
    data.cta_selected_track_ttc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cta_selected_track]
    data.cta_selected_track = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [cta_alert]
    data.cta_alert = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cta_active]
    data.cta_active = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [lcma_cvw_cipv]
    data.lcma_cvw_cipv = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lcma_cvw_alert_state]
    data.lcma_cvw_alert_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lcma_blis_alert_state]
    data.lcma_blis_alert_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lcma_active]
    data.lcma_active = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_srr_msgs/SrrFeatureAlert';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '721bc54767b8d837fd2e98fc870215ce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for srr_feature_alert
    
    std_msgs/Header header
    
    uint8     lcma_blis_ignored_track_id
    uint8     lcma_blis_track_id
    float32   lcma_cvw_ttc                             # seconds
    bool      cta_ttc_alert
    float32   cta_selected_track_ttc                   # seconds
    uint16    cta_selected_track
    uint8     cta_alert                                # binary
    bool      cta_active                               # binary
    uint8     lcma_cvw_cipv
    uint8     lcma_cvw_alert_state
    uint8     lcma_blis_alert_state
    bool      lcma_active
    
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
    const resolved = new SrrFeatureAlert(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lcma_blis_ignored_track_id !== undefined) {
      resolved.lcma_blis_ignored_track_id = msg.lcma_blis_ignored_track_id;
    }
    else {
      resolved.lcma_blis_ignored_track_id = 0
    }

    if (msg.lcma_blis_track_id !== undefined) {
      resolved.lcma_blis_track_id = msg.lcma_blis_track_id;
    }
    else {
      resolved.lcma_blis_track_id = 0
    }

    if (msg.lcma_cvw_ttc !== undefined) {
      resolved.lcma_cvw_ttc = msg.lcma_cvw_ttc;
    }
    else {
      resolved.lcma_cvw_ttc = 0.0
    }

    if (msg.cta_ttc_alert !== undefined) {
      resolved.cta_ttc_alert = msg.cta_ttc_alert;
    }
    else {
      resolved.cta_ttc_alert = false
    }

    if (msg.cta_selected_track_ttc !== undefined) {
      resolved.cta_selected_track_ttc = msg.cta_selected_track_ttc;
    }
    else {
      resolved.cta_selected_track_ttc = 0.0
    }

    if (msg.cta_selected_track !== undefined) {
      resolved.cta_selected_track = msg.cta_selected_track;
    }
    else {
      resolved.cta_selected_track = 0
    }

    if (msg.cta_alert !== undefined) {
      resolved.cta_alert = msg.cta_alert;
    }
    else {
      resolved.cta_alert = 0
    }

    if (msg.cta_active !== undefined) {
      resolved.cta_active = msg.cta_active;
    }
    else {
      resolved.cta_active = false
    }

    if (msg.lcma_cvw_cipv !== undefined) {
      resolved.lcma_cvw_cipv = msg.lcma_cvw_cipv;
    }
    else {
      resolved.lcma_cvw_cipv = 0
    }

    if (msg.lcma_cvw_alert_state !== undefined) {
      resolved.lcma_cvw_alert_state = msg.lcma_cvw_alert_state;
    }
    else {
      resolved.lcma_cvw_alert_state = 0
    }

    if (msg.lcma_blis_alert_state !== undefined) {
      resolved.lcma_blis_alert_state = msg.lcma_blis_alert_state;
    }
    else {
      resolved.lcma_blis_alert_state = 0
    }

    if (msg.lcma_active !== undefined) {
      resolved.lcma_active = msg.lcma_active;
    }
    else {
      resolved.lcma_active = false
    }

    return resolved;
    }
};

module.exports = SrrFeatureAlert;
