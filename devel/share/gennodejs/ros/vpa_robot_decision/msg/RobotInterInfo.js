// Auto-generated. Do not edit!

// (in-package vpa_robot_decision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RobotInterInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inter_1 = null;
      this.inter_2 = null;
      this.inter_3 = null;
      this.inter_4 = null;
      this.inter_5 = null;
    }
    else {
      if (initObj.hasOwnProperty('inter_1')) {
        this.inter_1 = initObj.inter_1
      }
      else {
        this.inter_1 = [];
      }
      if (initObj.hasOwnProperty('inter_2')) {
        this.inter_2 = initObj.inter_2
      }
      else {
        this.inter_2 = [];
      }
      if (initObj.hasOwnProperty('inter_3')) {
        this.inter_3 = initObj.inter_3
      }
      else {
        this.inter_3 = [];
      }
      if (initObj.hasOwnProperty('inter_4')) {
        this.inter_4 = initObj.inter_4
      }
      else {
        this.inter_4 = [];
      }
      if (initObj.hasOwnProperty('inter_5')) {
        this.inter_5 = initObj.inter_5
      }
      else {
        this.inter_5 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotInterInfo
    // Serialize message field [inter_1]
    bufferOffset = _arraySerializer.int8(obj.inter_1, buffer, bufferOffset, null);
    // Serialize message field [inter_2]
    bufferOffset = _arraySerializer.int8(obj.inter_2, buffer, bufferOffset, null);
    // Serialize message field [inter_3]
    bufferOffset = _arraySerializer.int8(obj.inter_3, buffer, bufferOffset, null);
    // Serialize message field [inter_4]
    bufferOffset = _arraySerializer.int8(obj.inter_4, buffer, bufferOffset, null);
    // Serialize message field [inter_5]
    bufferOffset = _arraySerializer.int8(obj.inter_5, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotInterInfo
    let len;
    let data = new RobotInterInfo(null);
    // Deserialize message field [inter_1]
    data.inter_1 = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [inter_2]
    data.inter_2 = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [inter_3]
    data.inter_3 = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [inter_4]
    data.inter_4 = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [inter_5]
    data.inter_5 = _arrayDeserializer.int8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.inter_1.length;
    length += object.inter_2.length;
    length += object.inter_3.length;
    length += object.inter_4.length;
    length += object.inter_5.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vpa_robot_decision/RobotInterInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c7e2780d75dbdc968ce987e67cfdc2df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # RobotInterInfo.msg
    
    int8[] inter_1
    int8[] inter_2
    int8[] inter_3
    int8[] inter_4
    int8[] inter_5
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotInterInfo(null);
    if (msg.inter_1 !== undefined) {
      resolved.inter_1 = msg.inter_1;
    }
    else {
      resolved.inter_1 = []
    }

    if (msg.inter_2 !== undefined) {
      resolved.inter_2 = msg.inter_2;
    }
    else {
      resolved.inter_2 = []
    }

    if (msg.inter_3 !== undefined) {
      resolved.inter_3 = msg.inter_3;
    }
    else {
      resolved.inter_3 = []
    }

    if (msg.inter_4 !== undefined) {
      resolved.inter_4 = msg.inter_4;
    }
    else {
      resolved.inter_4 = []
    }

    if (msg.inter_5 !== undefined) {
      resolved.inter_5 = msg.inter_5;
    }
    else {
      resolved.inter_5 = []
    }

    return resolved;
    }
};

module.exports = RobotInterInfo;
