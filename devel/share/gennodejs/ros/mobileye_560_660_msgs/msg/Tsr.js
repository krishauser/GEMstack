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

class Tsr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.vision_only_sign_type = null;
      this.vision_only_supplementary_sign_type = null;
      this.sign_position_x = null;
      this.sign_position_y = null;
      this.sign_position_z = null;
      this.filter_type = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('vision_only_sign_type')) {
        this.vision_only_sign_type = initObj.vision_only_sign_type
      }
      else {
        this.vision_only_sign_type = 0;
      }
      if (initObj.hasOwnProperty('vision_only_supplementary_sign_type')) {
        this.vision_only_supplementary_sign_type = initObj.vision_only_supplementary_sign_type
      }
      else {
        this.vision_only_supplementary_sign_type = 0;
      }
      if (initObj.hasOwnProperty('sign_position_x')) {
        this.sign_position_x = initObj.sign_position_x
      }
      else {
        this.sign_position_x = 0.0;
      }
      if (initObj.hasOwnProperty('sign_position_y')) {
        this.sign_position_y = initObj.sign_position_y
      }
      else {
        this.sign_position_y = 0.0;
      }
      if (initObj.hasOwnProperty('sign_position_z')) {
        this.sign_position_z = initObj.sign_position_z
      }
      else {
        this.sign_position_z = 0.0;
      }
      if (initObj.hasOwnProperty('filter_type')) {
        this.filter_type = initObj.filter_type
      }
      else {
        this.filter_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Tsr
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [vision_only_sign_type]
    bufferOffset = _serializer.uint8(obj.vision_only_sign_type, buffer, bufferOffset);
    // Serialize message field [vision_only_supplementary_sign_type]
    bufferOffset = _serializer.uint8(obj.vision_only_supplementary_sign_type, buffer, bufferOffset);
    // Serialize message field [sign_position_x]
    bufferOffset = _serializer.float32(obj.sign_position_x, buffer, bufferOffset);
    // Serialize message field [sign_position_y]
    bufferOffset = _serializer.float32(obj.sign_position_y, buffer, bufferOffset);
    // Serialize message field [sign_position_z]
    bufferOffset = _serializer.float32(obj.sign_position_z, buffer, bufferOffset);
    // Serialize message field [filter_type]
    bufferOffset = _serializer.uint8(obj.filter_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Tsr
    let len;
    let data = new Tsr(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [vision_only_sign_type]
    data.vision_only_sign_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vision_only_supplementary_sign_type]
    data.vision_only_supplementary_sign_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sign_position_x]
    data.sign_position_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sign_position_y]
    data.sign_position_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sign_position_z]
    data.sign_position_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [filter_type]
    data.filter_type = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobileye_560_660_msgs/Tsr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6181cda0894c479426a7c686589123b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 SIGN_TYPE_REGULAR_10 = 0
    uint8 SIGN_TYPE_REGULAR_20 = 1
    uint8 SIGN_TYPE_REGULAR_30 = 2
    uint8 SIGN_TYPE_REGULAR_40 = 3
    uint8 SIGN_TYPE_REGULAR_50 = 4
    uint8 SIGN_TYPE_REGULAR_60 = 5
    uint8 SIGN_TYPE_REGULAR_70 = 6
    uint8 SIGN_TYPE_REGULAR_80 = 7
    uint8 SIGN_TYPE_REGULAR_90 = 8
    uint8 SIGN_TYPE_REGULAR_100 = 9
    uint8 SIGN_TYPE_REGULAR_110 = 10
    uint8 SIGN_TYPE_REGULAR_120 = 11
    uint8 SIGN_TYPE_REGULAR_130 = 12
    uint8 SIGN_TYPE_REGULAR_140 = 13
    uint8 SIGN_TYPE_REGULAR_END_RESTRICTION_OF_NUMBER = 20
    uint8 SIGN_TYPE_ELECTRONIC_10 = 28
    uint8 SIGN_TYPE_ELECTRONIC_20 = 29
    uint8 SIGN_TYPE_ELECTRONIC_30 = 30
    uint8 SIGN_TYPE_ELECTRONIC_40 = 31
    uint8 SIGN_TYPE_ELECTRONIC_50 = 32
    uint8 SIGN_TYPE_ELECTRONIC_60 = 33
    uint8 SIGN_TYPE_ELECTRONIC_70 = 34
    uint8 SIGN_TYPE_ELECTRONIC_80 = 35
    uint8 SIGN_TYPE_ELECTRONIC_90 = 36
    uint8 SIGN_TYPE_ELECTRONIC_100 = 37
    uint8 SIGN_TYPE_ELECTRONIC_110 = 38
    uint8 SIGN_TYPE_ELECTRONIC_120 = 39
    uint8 SIGN_TYPE_ELECTRONIC_130 = 40
    uint8 SIGN_TYPE_ELECTRONIC_140 = 41
    uint8 SIGN_TYPE_ELECTRONIC_END_RESTRICTION_OF_NUMBER = 50
    uint8 SIGN_TYPE_REGULAR_GENERAL_END_ALL_RESTRICTION = 64
    uint8 SIGN_TYPE_ELECTRONIC_GENERAL_END_ALL_RESTRICTION = 65
    uint8 SIGN_TYPE_REGULAR_5 = 100
    uint8 SIGN_TYPE_REGULAR_15 = 101
    uint8 SIGN_TYPE_REGULAR_25 = 102
    uint8 SIGN_TYPE_REGULAR_35 = 103
    uint8 SIGN_TYPE_REGULAR_45 = 104
    uint8 SIGN_TYPE_REGULAR_55 = 105
    uint8 SIGN_TYPE_REGULAR_65 = 106
    uint8 SIGN_TYPE_REGULAR_75 = 107
    uint8 SIGN_TYPE_REGULAR_85 = 108
    uint8 SIGN_TYPE_REGULAR_95 = 109
    uint8 SIGN_TYPE_REGULAR_105 = 110
    uint8 SIGN_TYPE_REGULAR_115 = 111
    uint8 SIGN_TYPE_REGULAR_125 = 112
    uint8 SIGN_TYPE_REGULAR_135 = 113
    uint8 SIGN_TYPE_REGULAR_145 = 114
    uint8 SIGN_TYPE_ELECTRONIC_5 = 115
    uint8 SIGN_TYPE_ELECTRONIC_15 = 116
    uint8 SIGN_TYPE_ELECTRONIC_25 = 117
    uint8 SIGN_TYPE_ELECTRONIC_35 = 118
    uint8 SIGN_TYPE_ELECTRONIC_45 = 119
    uint8 SIGN_TYPE_ELECTRONIC_55 = 120
    uint8 SIGN_TYPE_ELECTRONIC_65 = 121
    uint8 SIGN_TYPE_ELECTRONIC_75 = 122
    uint8 SIGN_TYPE_ELECTRONIC_85 = 123
    uint8 SIGN_TYPE_ELECTRONIC_95 = 124
    uint8 SIGN_TYPE_ELECTRONIC_105 = 125
    uint8 SIGN_TYPE_ELECTRONIC_115 = 126
    uint8 SIGN_TYPE_ELECTRONIC_125 = 127
    uint8 SIGN_TYPE_ELECTRONIC_135 = 128
    uint8 SIGN_TYPE_ELECTRONIC_145 = 129
    uint8 SIGN_TYPE_REGULAR_MOTORWAY_BEGIN = 171
    uint8 SIGN_TYPE_REGULAR_END_OF_MOTORWAY = 172
    uint8 SIGN_TYPE_REGULAR_EXPRESSWAY_BEGIN = 173
    uint8 SIGN_TYPE_REGULAR_END_OF_EXPRESSWAY = 174
    uint8 SIGN_TYPE_REGULAR_PLAYGROUND_AREA_BEGIN = 175
    uint8 SIGN_TYPE_REGULAR_END_OF_PLAYGROUND_AREA = 176
    uint8 SIGN_TYPE_REGULAR_NO_PASSING_START = 200
    uint8 SIGN_TYPE_REGULAR_END_OF_NO_PASSING = 201
    uint8 SIGN_TYPE_ELECTRONIC_NO_PASSING_START = 220
    uint8 SIGN_TYPE_ELECTRONIC_END_OF_NO_PASSING = 221
    uint8 SIGN_TYPE_NONE_DETECTED = 254
    uint8 SIGN_TYPE_INVALID = 255
    uint8 vision_only_sign_type
    
    uint8 SUPP_SIGN_TYPE_NONE = 0
    uint8 SUPP_SIGN_TYPE_RAIN = 1
    uint8 SUPP_SIGN_TYPE_SNOW = 2
    uint8 SUPP_SIGN_TYPE_TRAILER = 3
    uint8 SUPP_SIGN_TYPE_TIME = 4
    uint8 SUPP_SIGN_TYPE_ARROW_LEFT = 5
    uint8 SUPP_SIGN_TYPE_ARROW_RIGHT = 6
    uint8 SUPP_SIGN_TYPE_BEND_ARROW_LEFT = 7
    uint8 SUPP_SIGN_TYPE_BEND_ARROW_RIGHT = 8
    uint8 SUPP_SIGN_TYPE_TRUCK = 9
    uint8 SUPP_SIGN_TYPE_DISTANCE_ARROW = 10
    uint8 SUPP_SIGN_TYPE_WEIGHT = 11
    uint8 SUPP_SIGN_TYPE_DISTANCE_IN = 12
    uint8 SUPP_SIGN_TYPE_TRACTOR = 13
    uint8 SUPP_SIGN_TYPE_SNOW_RAIN = 14
    uint8 SUPP_SIGN_TYPE_SCHOOL = 15
    uint8 SUPP_SIGN_TYPE_RAIN_CLOUD = 16
    uint8 SUPP_SIGN_TYPE_FOG = 17
    uint8 SUPP_SIGN_TYPE_HAZARDOUS_MATERIALS = 18
    uint8 SUPP_SIGN_TYPE_NIGHT = 19
    uint8 SUPP_SIGN_TYPE_GENERIC = 20
    uint8 SUPP_SIGN_TYPE_RAPPEL = 21
    uint8 SUPP_SIGN_TYPE_ZONE = 22
    uint8 SUPP_SIGN_TYPE_INVALID = 255
    uint8 vision_only_supplementary_sign_type
    
    float32 sign_position_x
    float32 sign_position_y
    float32 sign_position_z
    
    uint8 FILTER_TYPE_NOT_FILTERED = 0
    uint8 FILTER_TYPE_IRRELEVANT_TO_DRIVER = 1
    uint8 FILTER_TYPE_ON_VEHICLE = 2
    uint8 FILTER_TYPE_EMBEDDED = 3
    uint8 filter_type
    
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
    const resolved = new Tsr(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.vision_only_sign_type !== undefined) {
      resolved.vision_only_sign_type = msg.vision_only_sign_type;
    }
    else {
      resolved.vision_only_sign_type = 0
    }

    if (msg.vision_only_supplementary_sign_type !== undefined) {
      resolved.vision_only_supplementary_sign_type = msg.vision_only_supplementary_sign_type;
    }
    else {
      resolved.vision_only_supplementary_sign_type = 0
    }

    if (msg.sign_position_x !== undefined) {
      resolved.sign_position_x = msg.sign_position_x;
    }
    else {
      resolved.sign_position_x = 0.0
    }

    if (msg.sign_position_y !== undefined) {
      resolved.sign_position_y = msg.sign_position_y;
    }
    else {
      resolved.sign_position_y = 0.0
    }

    if (msg.sign_position_z !== undefined) {
      resolved.sign_position_z = msg.sign_position_z;
    }
    else {
      resolved.sign_position_z = 0.0
    }

    if (msg.filter_type !== undefined) {
      resolved.filter_type = msg.filter_type;
    }
    else {
      resolved.filter_type = 0
    }

    return resolved;
    }
};

// Constants for message
Tsr.Constants = {
  SIGN_TYPE_REGULAR_10: 0,
  SIGN_TYPE_REGULAR_20: 1,
  SIGN_TYPE_REGULAR_30: 2,
  SIGN_TYPE_REGULAR_40: 3,
  SIGN_TYPE_REGULAR_50: 4,
  SIGN_TYPE_REGULAR_60: 5,
  SIGN_TYPE_REGULAR_70: 6,
  SIGN_TYPE_REGULAR_80: 7,
  SIGN_TYPE_REGULAR_90: 8,
  SIGN_TYPE_REGULAR_100: 9,
  SIGN_TYPE_REGULAR_110: 10,
  SIGN_TYPE_REGULAR_120: 11,
  SIGN_TYPE_REGULAR_130: 12,
  SIGN_TYPE_REGULAR_140: 13,
  SIGN_TYPE_REGULAR_END_RESTRICTION_OF_NUMBER: 20,
  SIGN_TYPE_ELECTRONIC_10: 28,
  SIGN_TYPE_ELECTRONIC_20: 29,
  SIGN_TYPE_ELECTRONIC_30: 30,
  SIGN_TYPE_ELECTRONIC_40: 31,
  SIGN_TYPE_ELECTRONIC_50: 32,
  SIGN_TYPE_ELECTRONIC_60: 33,
  SIGN_TYPE_ELECTRONIC_70: 34,
  SIGN_TYPE_ELECTRONIC_80: 35,
  SIGN_TYPE_ELECTRONIC_90: 36,
  SIGN_TYPE_ELECTRONIC_100: 37,
  SIGN_TYPE_ELECTRONIC_110: 38,
  SIGN_TYPE_ELECTRONIC_120: 39,
  SIGN_TYPE_ELECTRONIC_130: 40,
  SIGN_TYPE_ELECTRONIC_140: 41,
  SIGN_TYPE_ELECTRONIC_END_RESTRICTION_OF_NUMBER: 50,
  SIGN_TYPE_REGULAR_GENERAL_END_ALL_RESTRICTION: 64,
  SIGN_TYPE_ELECTRONIC_GENERAL_END_ALL_RESTRICTION: 65,
  SIGN_TYPE_REGULAR_5: 100,
  SIGN_TYPE_REGULAR_15: 101,
  SIGN_TYPE_REGULAR_25: 102,
  SIGN_TYPE_REGULAR_35: 103,
  SIGN_TYPE_REGULAR_45: 104,
  SIGN_TYPE_REGULAR_55: 105,
  SIGN_TYPE_REGULAR_65: 106,
  SIGN_TYPE_REGULAR_75: 107,
  SIGN_TYPE_REGULAR_85: 108,
  SIGN_TYPE_REGULAR_95: 109,
  SIGN_TYPE_REGULAR_105: 110,
  SIGN_TYPE_REGULAR_115: 111,
  SIGN_TYPE_REGULAR_125: 112,
  SIGN_TYPE_REGULAR_135: 113,
  SIGN_TYPE_REGULAR_145: 114,
  SIGN_TYPE_ELECTRONIC_5: 115,
  SIGN_TYPE_ELECTRONIC_15: 116,
  SIGN_TYPE_ELECTRONIC_25: 117,
  SIGN_TYPE_ELECTRONIC_35: 118,
  SIGN_TYPE_ELECTRONIC_45: 119,
  SIGN_TYPE_ELECTRONIC_55: 120,
  SIGN_TYPE_ELECTRONIC_65: 121,
  SIGN_TYPE_ELECTRONIC_75: 122,
  SIGN_TYPE_ELECTRONIC_85: 123,
  SIGN_TYPE_ELECTRONIC_95: 124,
  SIGN_TYPE_ELECTRONIC_105: 125,
  SIGN_TYPE_ELECTRONIC_115: 126,
  SIGN_TYPE_ELECTRONIC_125: 127,
  SIGN_TYPE_ELECTRONIC_135: 128,
  SIGN_TYPE_ELECTRONIC_145: 129,
  SIGN_TYPE_REGULAR_MOTORWAY_BEGIN: 171,
  SIGN_TYPE_REGULAR_END_OF_MOTORWAY: 172,
  SIGN_TYPE_REGULAR_EXPRESSWAY_BEGIN: 173,
  SIGN_TYPE_REGULAR_END_OF_EXPRESSWAY: 174,
  SIGN_TYPE_REGULAR_PLAYGROUND_AREA_BEGIN: 175,
  SIGN_TYPE_REGULAR_END_OF_PLAYGROUND_AREA: 176,
  SIGN_TYPE_REGULAR_NO_PASSING_START: 200,
  SIGN_TYPE_REGULAR_END_OF_NO_PASSING: 201,
  SIGN_TYPE_ELECTRONIC_NO_PASSING_START: 220,
  SIGN_TYPE_ELECTRONIC_END_OF_NO_PASSING: 221,
  SIGN_TYPE_NONE_DETECTED: 254,
  SIGN_TYPE_INVALID: 255,
  SUPP_SIGN_TYPE_NONE: 0,
  SUPP_SIGN_TYPE_RAIN: 1,
  SUPP_SIGN_TYPE_SNOW: 2,
  SUPP_SIGN_TYPE_TRAILER: 3,
  SUPP_SIGN_TYPE_TIME: 4,
  SUPP_SIGN_TYPE_ARROW_LEFT: 5,
  SUPP_SIGN_TYPE_ARROW_RIGHT: 6,
  SUPP_SIGN_TYPE_BEND_ARROW_LEFT: 7,
  SUPP_SIGN_TYPE_BEND_ARROW_RIGHT: 8,
  SUPP_SIGN_TYPE_TRUCK: 9,
  SUPP_SIGN_TYPE_DISTANCE_ARROW: 10,
  SUPP_SIGN_TYPE_WEIGHT: 11,
  SUPP_SIGN_TYPE_DISTANCE_IN: 12,
  SUPP_SIGN_TYPE_TRACTOR: 13,
  SUPP_SIGN_TYPE_SNOW_RAIN: 14,
  SUPP_SIGN_TYPE_SCHOOL: 15,
  SUPP_SIGN_TYPE_RAIN_CLOUD: 16,
  SUPP_SIGN_TYPE_FOG: 17,
  SUPP_SIGN_TYPE_HAZARDOUS_MATERIALS: 18,
  SUPP_SIGN_TYPE_NIGHT: 19,
  SUPP_SIGN_TYPE_GENERIC: 20,
  SUPP_SIGN_TYPE_RAPPEL: 21,
  SUPP_SIGN_TYPE_ZONE: 22,
  SUPP_SIGN_TYPE_INVALID: 255,
  FILTER_TYPE_NOT_FILTERED: 0,
  FILTER_TYPE_IRRELEVANT_TO_DRIVER: 1,
  FILTER_TYPE_ON_VEHICLE: 2,
  FILTER_TYPE_EMBEDDED: 3,
}

module.exports = Tsr;
