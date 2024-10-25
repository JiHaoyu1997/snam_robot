// Auto-generated. Do not edit!

// (in-package vpa_robot_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CrossInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_name = null;
      this.cross = null;
      this.last_inter_id = null;
      this.local_inter_id = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_name')) {
        this.robot_name = initObj.robot_name
      }
      else {
        this.robot_name = '';
      }
      if (initObj.hasOwnProperty('cross')) {
        this.cross = initObj.cross
      }
      else {
        this.cross = false;
      }
      if (initObj.hasOwnProperty('last_inter_id')) {
        this.last_inter_id = initObj.last_inter_id
      }
      else {
        this.last_inter_id = 0;
      }
      if (initObj.hasOwnProperty('local_inter_id')) {
        this.local_inter_id = initObj.local_inter_id
      }
      else {
        this.local_inter_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CrossInfo
    // Serialize message field [robot_name]
    bufferOffset = _serializer.string(obj.robot_name, buffer, bufferOffset);
    // Serialize message field [cross]
    bufferOffset = _serializer.bool(obj.cross, buffer, bufferOffset);
    // Serialize message field [last_inter_id]
    bufferOffset = _serializer.int8(obj.last_inter_id, buffer, bufferOffset);
    // Serialize message field [local_inter_id]
    bufferOffset = _serializer.int8(obj.local_inter_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CrossInfo
    let len;
    let data = new CrossInfo(null);
    // Deserialize message field [robot_name]
    data.robot_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cross]
    data.cross = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [last_inter_id]
    data.last_inter_id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [local_inter_id]
    data.local_inter_id = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.robot_name);
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vpa_robot_vision/CrossInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'abc28f0d82de76aab08a8f05250b1372';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string robot_name         # Name of the robot
    bool cross                # Indicates if the robot has crossed
    int8 last_inter_id       # ID of the last intersection
    int8 local_inter_id      # ID of the current (local) intersection
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CrossInfo(null);
    if (msg.robot_name !== undefined) {
      resolved.robot_name = msg.robot_name;
    }
    else {
      resolved.robot_name = ''
    }

    if (msg.cross !== undefined) {
      resolved.cross = msg.cross;
    }
    else {
      resolved.cross = false
    }

    if (msg.last_inter_id !== undefined) {
      resolved.last_inter_id = msg.last_inter_id;
    }
    else {
      resolved.last_inter_id = 0
    }

    if (msg.local_inter_id !== undefined) {
      resolved.local_inter_id = msg.local_inter_id;
    }
    else {
      resolved.local_inter_id = 0
    }

    return resolved;
    }
};

module.exports = CrossInfo;
