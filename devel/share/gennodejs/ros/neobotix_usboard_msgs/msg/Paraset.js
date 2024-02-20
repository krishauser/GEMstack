// Auto-generated. Do not edit!

// (in-package neobotix_usboard_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Paraset {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.command = null;
      this.set_num = null;
      this.paraset_byte54 = null;
      this.paraset_byte53 = null;
      this.paraset_byte52 = null;
      this.paraset_byte51 = null;
      this.paraset_byte50 = null;
      this.paraset_byte49 = null;
      this.paraset_byte48 = null;
      this.paraset_byte47 = null;
      this.paraset_byte46 = null;
      this.paraset_byte45 = null;
      this.paraset_byte44 = null;
      this.paraset_byte43 = null;
      this.paraset_byte42 = null;
      this.paraset_byte41 = null;
      this.paraset_byte40 = null;
      this.paraset_byte39 = null;
      this.paraset_byte38 = null;
      this.paraset_byte37 = null;
      this.paraset_byte36 = null;
      this.paraset_byte35 = null;
      this.paraset_byte34 = null;
      this.paraset_byte33 = null;
      this.paraset_byte32 = null;
      this.paraset_byte31 = null;
      this.paraset_byte30 = null;
      this.paraset_byte29 = null;
      this.paraset_byte28 = null;
      this.paraset_byte27 = null;
      this.paraset_byte26 = null;
      this.paraset_byte25 = null;
      this.paraset_byte24 = null;
      this.paraset_byte23 = null;
      this.paraset_byte22 = null;
      this.paraset_byte21 = null;
      this.paraset_byte20 = null;
      this.paraset_byte19 = null;
      this.paraset_byte18 = null;
      this.paraset_byte17 = null;
      this.paraset_byte16 = null;
      this.paraset_byte15 = null;
      this.paraset_byte14 = null;
      this.paraset_byte13 = null;
      this.paraset_byte12 = null;
      this.paraset_byte11 = null;
      this.paraset_byte10 = null;
      this.paraset_byte9 = null;
      this.paraset_byte8 = null;
      this.paraset_byte7 = null;
      this.paraset_byte6 = null;
      this.paraset_byte5 = null;
      this.paraset_byte4 = null;
      this.paraset_byte3 = null;
      this.paraset_byte2 = null;
      this.paraset_byte1 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('set_num')) {
        this.set_num = initObj.set_num
      }
      else {
        this.set_num = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte54')) {
        this.paraset_byte54 = initObj.paraset_byte54
      }
      else {
        this.paraset_byte54 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte53')) {
        this.paraset_byte53 = initObj.paraset_byte53
      }
      else {
        this.paraset_byte53 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte52')) {
        this.paraset_byte52 = initObj.paraset_byte52
      }
      else {
        this.paraset_byte52 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte51')) {
        this.paraset_byte51 = initObj.paraset_byte51
      }
      else {
        this.paraset_byte51 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte50')) {
        this.paraset_byte50 = initObj.paraset_byte50
      }
      else {
        this.paraset_byte50 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte49')) {
        this.paraset_byte49 = initObj.paraset_byte49
      }
      else {
        this.paraset_byte49 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte48')) {
        this.paraset_byte48 = initObj.paraset_byte48
      }
      else {
        this.paraset_byte48 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte47')) {
        this.paraset_byte47 = initObj.paraset_byte47
      }
      else {
        this.paraset_byte47 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte46')) {
        this.paraset_byte46 = initObj.paraset_byte46
      }
      else {
        this.paraset_byte46 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte45')) {
        this.paraset_byte45 = initObj.paraset_byte45
      }
      else {
        this.paraset_byte45 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte44')) {
        this.paraset_byte44 = initObj.paraset_byte44
      }
      else {
        this.paraset_byte44 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte43')) {
        this.paraset_byte43 = initObj.paraset_byte43
      }
      else {
        this.paraset_byte43 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte42')) {
        this.paraset_byte42 = initObj.paraset_byte42
      }
      else {
        this.paraset_byte42 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte41')) {
        this.paraset_byte41 = initObj.paraset_byte41
      }
      else {
        this.paraset_byte41 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte40')) {
        this.paraset_byte40 = initObj.paraset_byte40
      }
      else {
        this.paraset_byte40 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte39')) {
        this.paraset_byte39 = initObj.paraset_byte39
      }
      else {
        this.paraset_byte39 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte38')) {
        this.paraset_byte38 = initObj.paraset_byte38
      }
      else {
        this.paraset_byte38 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte37')) {
        this.paraset_byte37 = initObj.paraset_byte37
      }
      else {
        this.paraset_byte37 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte36')) {
        this.paraset_byte36 = initObj.paraset_byte36
      }
      else {
        this.paraset_byte36 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte35')) {
        this.paraset_byte35 = initObj.paraset_byte35
      }
      else {
        this.paraset_byte35 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte34')) {
        this.paraset_byte34 = initObj.paraset_byte34
      }
      else {
        this.paraset_byte34 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte33')) {
        this.paraset_byte33 = initObj.paraset_byte33
      }
      else {
        this.paraset_byte33 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte32')) {
        this.paraset_byte32 = initObj.paraset_byte32
      }
      else {
        this.paraset_byte32 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte31')) {
        this.paraset_byte31 = initObj.paraset_byte31
      }
      else {
        this.paraset_byte31 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte30')) {
        this.paraset_byte30 = initObj.paraset_byte30
      }
      else {
        this.paraset_byte30 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte29')) {
        this.paraset_byte29 = initObj.paraset_byte29
      }
      else {
        this.paraset_byte29 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte28')) {
        this.paraset_byte28 = initObj.paraset_byte28
      }
      else {
        this.paraset_byte28 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte27')) {
        this.paraset_byte27 = initObj.paraset_byte27
      }
      else {
        this.paraset_byte27 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte26')) {
        this.paraset_byte26 = initObj.paraset_byte26
      }
      else {
        this.paraset_byte26 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte25')) {
        this.paraset_byte25 = initObj.paraset_byte25
      }
      else {
        this.paraset_byte25 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte24')) {
        this.paraset_byte24 = initObj.paraset_byte24
      }
      else {
        this.paraset_byte24 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte23')) {
        this.paraset_byte23 = initObj.paraset_byte23
      }
      else {
        this.paraset_byte23 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte22')) {
        this.paraset_byte22 = initObj.paraset_byte22
      }
      else {
        this.paraset_byte22 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte21')) {
        this.paraset_byte21 = initObj.paraset_byte21
      }
      else {
        this.paraset_byte21 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte20')) {
        this.paraset_byte20 = initObj.paraset_byte20
      }
      else {
        this.paraset_byte20 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte19')) {
        this.paraset_byte19 = initObj.paraset_byte19
      }
      else {
        this.paraset_byte19 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte18')) {
        this.paraset_byte18 = initObj.paraset_byte18
      }
      else {
        this.paraset_byte18 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte17')) {
        this.paraset_byte17 = initObj.paraset_byte17
      }
      else {
        this.paraset_byte17 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte16')) {
        this.paraset_byte16 = initObj.paraset_byte16
      }
      else {
        this.paraset_byte16 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte15')) {
        this.paraset_byte15 = initObj.paraset_byte15
      }
      else {
        this.paraset_byte15 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte14')) {
        this.paraset_byte14 = initObj.paraset_byte14
      }
      else {
        this.paraset_byte14 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte13')) {
        this.paraset_byte13 = initObj.paraset_byte13
      }
      else {
        this.paraset_byte13 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte12')) {
        this.paraset_byte12 = initObj.paraset_byte12
      }
      else {
        this.paraset_byte12 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte11')) {
        this.paraset_byte11 = initObj.paraset_byte11
      }
      else {
        this.paraset_byte11 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte10')) {
        this.paraset_byte10 = initObj.paraset_byte10
      }
      else {
        this.paraset_byte10 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte9')) {
        this.paraset_byte9 = initObj.paraset_byte9
      }
      else {
        this.paraset_byte9 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte8')) {
        this.paraset_byte8 = initObj.paraset_byte8
      }
      else {
        this.paraset_byte8 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte7')) {
        this.paraset_byte7 = initObj.paraset_byte7
      }
      else {
        this.paraset_byte7 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte6')) {
        this.paraset_byte6 = initObj.paraset_byte6
      }
      else {
        this.paraset_byte6 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte5')) {
        this.paraset_byte5 = initObj.paraset_byte5
      }
      else {
        this.paraset_byte5 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte4')) {
        this.paraset_byte4 = initObj.paraset_byte4
      }
      else {
        this.paraset_byte4 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte3')) {
        this.paraset_byte3 = initObj.paraset_byte3
      }
      else {
        this.paraset_byte3 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte2')) {
        this.paraset_byte2 = initObj.paraset_byte2
      }
      else {
        this.paraset_byte2 = 0;
      }
      if (initObj.hasOwnProperty('paraset_byte1')) {
        this.paraset_byte1 = initObj.paraset_byte1
      }
      else {
        this.paraset_byte1 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Paraset
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [set_num]
    bufferOffset = _serializer.uint8(obj.set_num, buffer, bufferOffset);
    // Serialize message field [paraset_byte54]
    bufferOffset = _serializer.uint8(obj.paraset_byte54, buffer, bufferOffset);
    // Serialize message field [paraset_byte53]
    bufferOffset = _serializer.uint8(obj.paraset_byte53, buffer, bufferOffset);
    // Serialize message field [paraset_byte52]
    bufferOffset = _serializer.uint8(obj.paraset_byte52, buffer, bufferOffset);
    // Serialize message field [paraset_byte51]
    bufferOffset = _serializer.uint8(obj.paraset_byte51, buffer, bufferOffset);
    // Serialize message field [paraset_byte50]
    bufferOffset = _serializer.uint8(obj.paraset_byte50, buffer, bufferOffset);
    // Serialize message field [paraset_byte49]
    bufferOffset = _serializer.uint8(obj.paraset_byte49, buffer, bufferOffset);
    // Serialize message field [paraset_byte48]
    bufferOffset = _serializer.uint8(obj.paraset_byte48, buffer, bufferOffset);
    // Serialize message field [paraset_byte47]
    bufferOffset = _serializer.uint8(obj.paraset_byte47, buffer, bufferOffset);
    // Serialize message field [paraset_byte46]
    bufferOffset = _serializer.uint8(obj.paraset_byte46, buffer, bufferOffset);
    // Serialize message field [paraset_byte45]
    bufferOffset = _serializer.uint8(obj.paraset_byte45, buffer, bufferOffset);
    // Serialize message field [paraset_byte44]
    bufferOffset = _serializer.uint8(obj.paraset_byte44, buffer, bufferOffset);
    // Serialize message field [paraset_byte43]
    bufferOffset = _serializer.uint8(obj.paraset_byte43, buffer, bufferOffset);
    // Serialize message field [paraset_byte42]
    bufferOffset = _serializer.uint8(obj.paraset_byte42, buffer, bufferOffset);
    // Serialize message field [paraset_byte41]
    bufferOffset = _serializer.uint8(obj.paraset_byte41, buffer, bufferOffset);
    // Serialize message field [paraset_byte40]
    bufferOffset = _serializer.uint8(obj.paraset_byte40, buffer, bufferOffset);
    // Serialize message field [paraset_byte39]
    bufferOffset = _serializer.uint8(obj.paraset_byte39, buffer, bufferOffset);
    // Serialize message field [paraset_byte38]
    bufferOffset = _serializer.uint8(obj.paraset_byte38, buffer, bufferOffset);
    // Serialize message field [paraset_byte37]
    bufferOffset = _serializer.uint8(obj.paraset_byte37, buffer, bufferOffset);
    // Serialize message field [paraset_byte36]
    bufferOffset = _serializer.uint8(obj.paraset_byte36, buffer, bufferOffset);
    // Serialize message field [paraset_byte35]
    bufferOffset = _serializer.uint8(obj.paraset_byte35, buffer, bufferOffset);
    // Serialize message field [paraset_byte34]
    bufferOffset = _serializer.uint8(obj.paraset_byte34, buffer, bufferOffset);
    // Serialize message field [paraset_byte33]
    bufferOffset = _serializer.uint8(obj.paraset_byte33, buffer, bufferOffset);
    // Serialize message field [paraset_byte32]
    bufferOffset = _serializer.uint8(obj.paraset_byte32, buffer, bufferOffset);
    // Serialize message field [paraset_byte31]
    bufferOffset = _serializer.uint8(obj.paraset_byte31, buffer, bufferOffset);
    // Serialize message field [paraset_byte30]
    bufferOffset = _serializer.uint8(obj.paraset_byte30, buffer, bufferOffset);
    // Serialize message field [paraset_byte29]
    bufferOffset = _serializer.uint8(obj.paraset_byte29, buffer, bufferOffset);
    // Serialize message field [paraset_byte28]
    bufferOffset = _serializer.uint8(obj.paraset_byte28, buffer, bufferOffset);
    // Serialize message field [paraset_byte27]
    bufferOffset = _serializer.uint8(obj.paraset_byte27, buffer, bufferOffset);
    // Serialize message field [paraset_byte26]
    bufferOffset = _serializer.uint8(obj.paraset_byte26, buffer, bufferOffset);
    // Serialize message field [paraset_byte25]
    bufferOffset = _serializer.uint8(obj.paraset_byte25, buffer, bufferOffset);
    // Serialize message field [paraset_byte24]
    bufferOffset = _serializer.uint8(obj.paraset_byte24, buffer, bufferOffset);
    // Serialize message field [paraset_byte23]
    bufferOffset = _serializer.uint8(obj.paraset_byte23, buffer, bufferOffset);
    // Serialize message field [paraset_byte22]
    bufferOffset = _serializer.uint8(obj.paraset_byte22, buffer, bufferOffset);
    // Serialize message field [paraset_byte21]
    bufferOffset = _serializer.uint8(obj.paraset_byte21, buffer, bufferOffset);
    // Serialize message field [paraset_byte20]
    bufferOffset = _serializer.uint8(obj.paraset_byte20, buffer, bufferOffset);
    // Serialize message field [paraset_byte19]
    bufferOffset = _serializer.uint8(obj.paraset_byte19, buffer, bufferOffset);
    // Serialize message field [paraset_byte18]
    bufferOffset = _serializer.uint8(obj.paraset_byte18, buffer, bufferOffset);
    // Serialize message field [paraset_byte17]
    bufferOffset = _serializer.uint8(obj.paraset_byte17, buffer, bufferOffset);
    // Serialize message field [paraset_byte16]
    bufferOffset = _serializer.uint8(obj.paraset_byte16, buffer, bufferOffset);
    // Serialize message field [paraset_byte15]
    bufferOffset = _serializer.uint8(obj.paraset_byte15, buffer, bufferOffset);
    // Serialize message field [paraset_byte14]
    bufferOffset = _serializer.uint8(obj.paraset_byte14, buffer, bufferOffset);
    // Serialize message field [paraset_byte13]
    bufferOffset = _serializer.uint8(obj.paraset_byte13, buffer, bufferOffset);
    // Serialize message field [paraset_byte12]
    bufferOffset = _serializer.uint8(obj.paraset_byte12, buffer, bufferOffset);
    // Serialize message field [paraset_byte11]
    bufferOffset = _serializer.uint8(obj.paraset_byte11, buffer, bufferOffset);
    // Serialize message field [paraset_byte10]
    bufferOffset = _serializer.uint8(obj.paraset_byte10, buffer, bufferOffset);
    // Serialize message field [paraset_byte9]
    bufferOffset = _serializer.uint8(obj.paraset_byte9, buffer, bufferOffset);
    // Serialize message field [paraset_byte8]
    bufferOffset = _serializer.uint8(obj.paraset_byte8, buffer, bufferOffset);
    // Serialize message field [paraset_byte7]
    bufferOffset = _serializer.uint8(obj.paraset_byte7, buffer, bufferOffset);
    // Serialize message field [paraset_byte6]
    bufferOffset = _serializer.uint8(obj.paraset_byte6, buffer, bufferOffset);
    // Serialize message field [paraset_byte5]
    bufferOffset = _serializer.uint8(obj.paraset_byte5, buffer, bufferOffset);
    // Serialize message field [paraset_byte4]
    bufferOffset = _serializer.uint8(obj.paraset_byte4, buffer, bufferOffset);
    // Serialize message field [paraset_byte3]
    bufferOffset = _serializer.uint8(obj.paraset_byte3, buffer, bufferOffset);
    // Serialize message field [paraset_byte2]
    bufferOffset = _serializer.uint8(obj.paraset_byte2, buffer, bufferOffset);
    // Serialize message field [paraset_byte1]
    bufferOffset = _serializer.uint8(obj.paraset_byte1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Paraset
    let len;
    let data = new Paraset(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [set_num]
    data.set_num = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte54]
    data.paraset_byte54 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte53]
    data.paraset_byte53 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte52]
    data.paraset_byte52 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte51]
    data.paraset_byte51 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte50]
    data.paraset_byte50 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte49]
    data.paraset_byte49 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte48]
    data.paraset_byte48 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte47]
    data.paraset_byte47 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte46]
    data.paraset_byte46 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte45]
    data.paraset_byte45 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte44]
    data.paraset_byte44 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte43]
    data.paraset_byte43 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte42]
    data.paraset_byte42 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte41]
    data.paraset_byte41 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte40]
    data.paraset_byte40 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte39]
    data.paraset_byte39 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte38]
    data.paraset_byte38 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte37]
    data.paraset_byte37 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte36]
    data.paraset_byte36 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte35]
    data.paraset_byte35 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte34]
    data.paraset_byte34 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte33]
    data.paraset_byte33 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte32]
    data.paraset_byte32 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte31]
    data.paraset_byte31 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte30]
    data.paraset_byte30 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte29]
    data.paraset_byte29 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte28]
    data.paraset_byte28 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte27]
    data.paraset_byte27 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte26]
    data.paraset_byte26 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte25]
    data.paraset_byte25 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte24]
    data.paraset_byte24 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte23]
    data.paraset_byte23 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte22]
    data.paraset_byte22 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte21]
    data.paraset_byte21 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte20]
    data.paraset_byte20 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte19]
    data.paraset_byte19 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte18]
    data.paraset_byte18 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte17]
    data.paraset_byte17 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte16]
    data.paraset_byte16 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte15]
    data.paraset_byte15 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte14]
    data.paraset_byte14 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte13]
    data.paraset_byte13 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte12]
    data.paraset_byte12 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte11]
    data.paraset_byte11 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte10]
    data.paraset_byte10 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte9]
    data.paraset_byte9 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte8]
    data.paraset_byte8 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte7]
    data.paraset_byte7 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte6]
    data.paraset_byte6 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte5]
    data.paraset_byte5 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte4]
    data.paraset_byte4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte3]
    data.paraset_byte3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte2]
    data.paraset_byte2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [paraset_byte1]
    data.paraset_byte1 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neobotix_usboard_msgs/Paraset';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88e510d038a634b40698407f2e433138';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for Paramset
    
    std_msgs/Header header
    
    uint8      command                                 
    uint8      set_num                                 
    uint8      paraset_byte54                    
    uint8      paraset_byte53                    
    uint8      paraset_byte52                    
    uint8      paraset_byte51                    
    uint8      paraset_byte50                    
    uint8      paraset_byte49                    
    uint8      paraset_byte48                    
    uint8      paraset_byte47                    
    uint8      paraset_byte46                    
    uint8      paraset_byte45                    
    uint8      paraset_byte44                    
    uint8      paraset_byte43                    
    uint8      paraset_byte42                    
    uint8      paraset_byte41                    
    uint8      paraset_byte40                    
    uint8      paraset_byte39                    
    uint8      paraset_byte38                    
    uint8      paraset_byte37                    
    uint8      paraset_byte36                    
    uint8      paraset_byte35                    
    uint8      paraset_byte34                    
    uint8      paraset_byte33                    
    uint8      paraset_byte32                    
    uint8      paraset_byte31                    
    uint8      paraset_byte30                    
    uint8      paraset_byte29                    
    uint8      paraset_byte28                    
    uint8      paraset_byte27                    
    uint8      paraset_byte26                    
    uint8      paraset_byte25                    
    uint8      paraset_byte24                    
    uint8      paraset_byte23                    
    uint8      paraset_byte22                    
    uint8      paraset_byte21                    
    uint8      paraset_byte20                    
    uint8      paraset_byte19                    
    uint8      paraset_byte18                    
    uint8      paraset_byte17                    
    uint8      paraset_byte16                    
    uint8      paraset_byte15                    
    uint8      paraset_byte14                    
    uint8      paraset_byte13                    
    uint8      paraset_byte12                    
    uint8      paraset_byte11                    
    uint8      paraset_byte10                    
    uint8      paraset_byte9                     
    uint8      paraset_byte8                     
    uint8      paraset_byte7                     
    uint8      paraset_byte6                     
    uint8      paraset_byte5                     
    uint8      paraset_byte4                     
    uint8      paraset_byte3                     
    uint8      paraset_byte2                     
    uint8      paraset_byte1                     
    
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
    const resolved = new Paraset(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.set_num !== undefined) {
      resolved.set_num = msg.set_num;
    }
    else {
      resolved.set_num = 0
    }

    if (msg.paraset_byte54 !== undefined) {
      resolved.paraset_byte54 = msg.paraset_byte54;
    }
    else {
      resolved.paraset_byte54 = 0
    }

    if (msg.paraset_byte53 !== undefined) {
      resolved.paraset_byte53 = msg.paraset_byte53;
    }
    else {
      resolved.paraset_byte53 = 0
    }

    if (msg.paraset_byte52 !== undefined) {
      resolved.paraset_byte52 = msg.paraset_byte52;
    }
    else {
      resolved.paraset_byte52 = 0
    }

    if (msg.paraset_byte51 !== undefined) {
      resolved.paraset_byte51 = msg.paraset_byte51;
    }
    else {
      resolved.paraset_byte51 = 0
    }

    if (msg.paraset_byte50 !== undefined) {
      resolved.paraset_byte50 = msg.paraset_byte50;
    }
    else {
      resolved.paraset_byte50 = 0
    }

    if (msg.paraset_byte49 !== undefined) {
      resolved.paraset_byte49 = msg.paraset_byte49;
    }
    else {
      resolved.paraset_byte49 = 0
    }

    if (msg.paraset_byte48 !== undefined) {
      resolved.paraset_byte48 = msg.paraset_byte48;
    }
    else {
      resolved.paraset_byte48 = 0
    }

    if (msg.paraset_byte47 !== undefined) {
      resolved.paraset_byte47 = msg.paraset_byte47;
    }
    else {
      resolved.paraset_byte47 = 0
    }

    if (msg.paraset_byte46 !== undefined) {
      resolved.paraset_byte46 = msg.paraset_byte46;
    }
    else {
      resolved.paraset_byte46 = 0
    }

    if (msg.paraset_byte45 !== undefined) {
      resolved.paraset_byte45 = msg.paraset_byte45;
    }
    else {
      resolved.paraset_byte45 = 0
    }

    if (msg.paraset_byte44 !== undefined) {
      resolved.paraset_byte44 = msg.paraset_byte44;
    }
    else {
      resolved.paraset_byte44 = 0
    }

    if (msg.paraset_byte43 !== undefined) {
      resolved.paraset_byte43 = msg.paraset_byte43;
    }
    else {
      resolved.paraset_byte43 = 0
    }

    if (msg.paraset_byte42 !== undefined) {
      resolved.paraset_byte42 = msg.paraset_byte42;
    }
    else {
      resolved.paraset_byte42 = 0
    }

    if (msg.paraset_byte41 !== undefined) {
      resolved.paraset_byte41 = msg.paraset_byte41;
    }
    else {
      resolved.paraset_byte41 = 0
    }

    if (msg.paraset_byte40 !== undefined) {
      resolved.paraset_byte40 = msg.paraset_byte40;
    }
    else {
      resolved.paraset_byte40 = 0
    }

    if (msg.paraset_byte39 !== undefined) {
      resolved.paraset_byte39 = msg.paraset_byte39;
    }
    else {
      resolved.paraset_byte39 = 0
    }

    if (msg.paraset_byte38 !== undefined) {
      resolved.paraset_byte38 = msg.paraset_byte38;
    }
    else {
      resolved.paraset_byte38 = 0
    }

    if (msg.paraset_byte37 !== undefined) {
      resolved.paraset_byte37 = msg.paraset_byte37;
    }
    else {
      resolved.paraset_byte37 = 0
    }

    if (msg.paraset_byte36 !== undefined) {
      resolved.paraset_byte36 = msg.paraset_byte36;
    }
    else {
      resolved.paraset_byte36 = 0
    }

    if (msg.paraset_byte35 !== undefined) {
      resolved.paraset_byte35 = msg.paraset_byte35;
    }
    else {
      resolved.paraset_byte35 = 0
    }

    if (msg.paraset_byte34 !== undefined) {
      resolved.paraset_byte34 = msg.paraset_byte34;
    }
    else {
      resolved.paraset_byte34 = 0
    }

    if (msg.paraset_byte33 !== undefined) {
      resolved.paraset_byte33 = msg.paraset_byte33;
    }
    else {
      resolved.paraset_byte33 = 0
    }

    if (msg.paraset_byte32 !== undefined) {
      resolved.paraset_byte32 = msg.paraset_byte32;
    }
    else {
      resolved.paraset_byte32 = 0
    }

    if (msg.paraset_byte31 !== undefined) {
      resolved.paraset_byte31 = msg.paraset_byte31;
    }
    else {
      resolved.paraset_byte31 = 0
    }

    if (msg.paraset_byte30 !== undefined) {
      resolved.paraset_byte30 = msg.paraset_byte30;
    }
    else {
      resolved.paraset_byte30 = 0
    }

    if (msg.paraset_byte29 !== undefined) {
      resolved.paraset_byte29 = msg.paraset_byte29;
    }
    else {
      resolved.paraset_byte29 = 0
    }

    if (msg.paraset_byte28 !== undefined) {
      resolved.paraset_byte28 = msg.paraset_byte28;
    }
    else {
      resolved.paraset_byte28 = 0
    }

    if (msg.paraset_byte27 !== undefined) {
      resolved.paraset_byte27 = msg.paraset_byte27;
    }
    else {
      resolved.paraset_byte27 = 0
    }

    if (msg.paraset_byte26 !== undefined) {
      resolved.paraset_byte26 = msg.paraset_byte26;
    }
    else {
      resolved.paraset_byte26 = 0
    }

    if (msg.paraset_byte25 !== undefined) {
      resolved.paraset_byte25 = msg.paraset_byte25;
    }
    else {
      resolved.paraset_byte25 = 0
    }

    if (msg.paraset_byte24 !== undefined) {
      resolved.paraset_byte24 = msg.paraset_byte24;
    }
    else {
      resolved.paraset_byte24 = 0
    }

    if (msg.paraset_byte23 !== undefined) {
      resolved.paraset_byte23 = msg.paraset_byte23;
    }
    else {
      resolved.paraset_byte23 = 0
    }

    if (msg.paraset_byte22 !== undefined) {
      resolved.paraset_byte22 = msg.paraset_byte22;
    }
    else {
      resolved.paraset_byte22 = 0
    }

    if (msg.paraset_byte21 !== undefined) {
      resolved.paraset_byte21 = msg.paraset_byte21;
    }
    else {
      resolved.paraset_byte21 = 0
    }

    if (msg.paraset_byte20 !== undefined) {
      resolved.paraset_byte20 = msg.paraset_byte20;
    }
    else {
      resolved.paraset_byte20 = 0
    }

    if (msg.paraset_byte19 !== undefined) {
      resolved.paraset_byte19 = msg.paraset_byte19;
    }
    else {
      resolved.paraset_byte19 = 0
    }

    if (msg.paraset_byte18 !== undefined) {
      resolved.paraset_byte18 = msg.paraset_byte18;
    }
    else {
      resolved.paraset_byte18 = 0
    }

    if (msg.paraset_byte17 !== undefined) {
      resolved.paraset_byte17 = msg.paraset_byte17;
    }
    else {
      resolved.paraset_byte17 = 0
    }

    if (msg.paraset_byte16 !== undefined) {
      resolved.paraset_byte16 = msg.paraset_byte16;
    }
    else {
      resolved.paraset_byte16 = 0
    }

    if (msg.paraset_byte15 !== undefined) {
      resolved.paraset_byte15 = msg.paraset_byte15;
    }
    else {
      resolved.paraset_byte15 = 0
    }

    if (msg.paraset_byte14 !== undefined) {
      resolved.paraset_byte14 = msg.paraset_byte14;
    }
    else {
      resolved.paraset_byte14 = 0
    }

    if (msg.paraset_byte13 !== undefined) {
      resolved.paraset_byte13 = msg.paraset_byte13;
    }
    else {
      resolved.paraset_byte13 = 0
    }

    if (msg.paraset_byte12 !== undefined) {
      resolved.paraset_byte12 = msg.paraset_byte12;
    }
    else {
      resolved.paraset_byte12 = 0
    }

    if (msg.paraset_byte11 !== undefined) {
      resolved.paraset_byte11 = msg.paraset_byte11;
    }
    else {
      resolved.paraset_byte11 = 0
    }

    if (msg.paraset_byte10 !== undefined) {
      resolved.paraset_byte10 = msg.paraset_byte10;
    }
    else {
      resolved.paraset_byte10 = 0
    }

    if (msg.paraset_byte9 !== undefined) {
      resolved.paraset_byte9 = msg.paraset_byte9;
    }
    else {
      resolved.paraset_byte9 = 0
    }

    if (msg.paraset_byte8 !== undefined) {
      resolved.paraset_byte8 = msg.paraset_byte8;
    }
    else {
      resolved.paraset_byte8 = 0
    }

    if (msg.paraset_byte7 !== undefined) {
      resolved.paraset_byte7 = msg.paraset_byte7;
    }
    else {
      resolved.paraset_byte7 = 0
    }

    if (msg.paraset_byte6 !== undefined) {
      resolved.paraset_byte6 = msg.paraset_byte6;
    }
    else {
      resolved.paraset_byte6 = 0
    }

    if (msg.paraset_byte5 !== undefined) {
      resolved.paraset_byte5 = msg.paraset_byte5;
    }
    else {
      resolved.paraset_byte5 = 0
    }

    if (msg.paraset_byte4 !== undefined) {
      resolved.paraset_byte4 = msg.paraset_byte4;
    }
    else {
      resolved.paraset_byte4 = 0
    }

    if (msg.paraset_byte3 !== undefined) {
      resolved.paraset_byte3 = msg.paraset_byte3;
    }
    else {
      resolved.paraset_byte3 = 0
    }

    if (msg.paraset_byte2 !== undefined) {
      resolved.paraset_byte2 = msg.paraset_byte2;
    }
    else {
      resolved.paraset_byte2 = 0
    }

    if (msg.paraset_byte1 !== undefined) {
      resolved.paraset_byte1 = msg.paraset_byte1;
    }
    else {
      resolved.paraset_byte1 = 0
    }

    return resolved;
    }
};

module.exports = Paraset;
