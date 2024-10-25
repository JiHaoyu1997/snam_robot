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

class RobotInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_name = null;
      this.robot_id = null;
      this.robot_a = null;
      this.robot_v = null;
      this.robot_p = null;
      this.robot_enter_time = null;
      this.robot_arrive_cp_time = null;
      this.robot_exit_time = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_name')) {
        this.robot_name = initObj.robot_name
      }
      else {
        this.robot_name = '';
      }
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = 0;
      }
      if (initObj.hasOwnProperty('robot_a')) {
        this.robot_a = initObj.robot_a
      }
      else {
        this.robot_a = 0.0;
      }
      if (initObj.hasOwnProperty('robot_v')) {
        this.robot_v = initObj.robot_v
      }
      else {
        this.robot_v = 0.0;
      }
      if (initObj.hasOwnProperty('robot_p')) {
        this.robot_p = initObj.robot_p
      }
      else {
        this.robot_p = 0.0;
      }
      if (initObj.hasOwnProperty('robot_enter_time')) {
        this.robot_enter_time = initObj.robot_enter_time
      }
      else {
        this.robot_enter_time = 0.0;
      }
      if (initObj.hasOwnProperty('robot_arrive_cp_time')) {
        this.robot_arrive_cp_time = initObj.robot_arrive_cp_time
      }
      else {
        this.robot_arrive_cp_time = 0.0;
      }
      if (initObj.hasOwnProperty('robot_exit_time')) {
        this.robot_exit_time = initObj.robot_exit_time
      }
      else {
        this.robot_exit_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotInfo
    // Serialize message field [robot_name]
    bufferOffset = _serializer.string(obj.robot_name, buffer, bufferOffset);
    // Serialize message field [robot_id]
    bufferOffset = _serializer.int8(obj.robot_id, buffer, bufferOffset);
    // Serialize message field [robot_a]
    bufferOffset = _serializer.float32(obj.robot_a, buffer, bufferOffset);
    // Serialize message field [robot_v]
    bufferOffset = _serializer.float32(obj.robot_v, buffer, bufferOffset);
    // Serialize message field [robot_p]
    bufferOffset = _serializer.float32(obj.robot_p, buffer, bufferOffset);
    // Serialize message field [robot_enter_time]
    bufferOffset = _serializer.float32(obj.robot_enter_time, buffer, bufferOffset);
    // Serialize message field [robot_arrive_cp_time]
    bufferOffset = _serializer.float32(obj.robot_arrive_cp_time, buffer, bufferOffset);
    // Serialize message field [robot_exit_time]
    bufferOffset = _serializer.float32(obj.robot_exit_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotInfo
    let len;
    let data = new RobotInfo(null);
    // Deserialize message field [robot_name]
    data.robot_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [robot_a]
    data.robot_a = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [robot_v]
    data.robot_v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [robot_p]
    data.robot_p = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [robot_enter_time]
    data.robot_enter_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [robot_arrive_cp_time]
    data.robot_arrive_cp_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [robot_exit_time]
    data.robot_exit_time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.robot_name);
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vpa_robot_decision/RobotInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '11867948fae731e82b48e6596c84aa31';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string  robot_name
    int8    robot_id
    float32 robot_a  # Acceleration
    float32 robot_v  # Velocity
    float32 robot_p  # Position
    float32 robot_enter_time
    float32 robot_arrive_cp_time
    float32 robot_exit_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotInfo(null);
    if (msg.robot_name !== undefined) {
      resolved.robot_name = msg.robot_name;
    }
    else {
      resolved.robot_name = ''
    }

    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = 0
    }

    if (msg.robot_a !== undefined) {
      resolved.robot_a = msg.robot_a;
    }
    else {
      resolved.robot_a = 0.0
    }

    if (msg.robot_v !== undefined) {
      resolved.robot_v = msg.robot_v;
    }
    else {
      resolved.robot_v = 0.0
    }

    if (msg.robot_p !== undefined) {
      resolved.robot_p = msg.robot_p;
    }
    else {
      resolved.robot_p = 0.0
    }

    if (msg.robot_enter_time !== undefined) {
      resolved.robot_enter_time = msg.robot_enter_time;
    }
    else {
      resolved.robot_enter_time = 0.0
    }

    if (msg.robot_arrive_cp_time !== undefined) {
      resolved.robot_arrive_cp_time = msg.robot_arrive_cp_time;
    }
    else {
      resolved.robot_arrive_cp_time = 0.0
    }

    if (msg.robot_exit_time !== undefined) {
      resolved.robot_exit_time = msg.robot_exit_time;
    }
    else {
      resolved.robot_exit_time = 0.0
    }

    return resolved;
    }
};

module.exports = RobotInfo;
