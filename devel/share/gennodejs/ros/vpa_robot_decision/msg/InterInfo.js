// Auto-generated. Do not edit!

// (in-package vpa_robot_decision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RobotInfo = require('./RobotInfo.js');

//-----------------------------------------------------------

class InterInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inter_id = null;
      this.robot_id_list = null;
      this.robot_info = null;
    }
    else {
      if (initObj.hasOwnProperty('inter_id')) {
        this.inter_id = initObj.inter_id
      }
      else {
        this.inter_id = 0;
      }
      if (initObj.hasOwnProperty('robot_id_list')) {
        this.robot_id_list = initObj.robot_id_list
      }
      else {
        this.robot_id_list = [];
      }
      if (initObj.hasOwnProperty('robot_info')) {
        this.robot_info = initObj.robot_info
      }
      else {
        this.robot_info = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterInfo
    // Serialize message field [inter_id]
    bufferOffset = _serializer.int8(obj.inter_id, buffer, bufferOffset);
    // Serialize message field [robot_id_list]
    bufferOffset = _arraySerializer.int8(obj.robot_id_list, buffer, bufferOffset, null);
    // Serialize message field [robot_info]
    // Serialize the length for message field [robot_info]
    bufferOffset = _serializer.uint32(obj.robot_info.length, buffer, bufferOffset);
    obj.robot_info.forEach((val) => {
      bufferOffset = RobotInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterInfo
    let len;
    let data = new InterInfo(null);
    // Deserialize message field [inter_id]
    data.inter_id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [robot_id_list]
    data.robot_id_list = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [robot_info]
    // Deserialize array length for message field [robot_info]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.robot_info = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.robot_info[i] = RobotInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.robot_id_list.length;
    object.robot_info.forEach((val) => {
      length += RobotInfo.getMessageSize(val);
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vpa_robot_decision/InterInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a651223f4498c1bc66c36b76f2257d3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8            inter_id        # Intersection ID
    int8[]          robot_id_list   # List of robot names or IDs
    RobotInfo[]     robot_info      # List of RobotInfo instances
    ================================================================================
    MSG: vpa_robot_decision/RobotInfo
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
    const resolved = new InterInfo(null);
    if (msg.inter_id !== undefined) {
      resolved.inter_id = msg.inter_id;
    }
    else {
      resolved.inter_id = 0
    }

    if (msg.robot_id_list !== undefined) {
      resolved.robot_id_list = msg.robot_id_list;
    }
    else {
      resolved.robot_id_list = []
    }

    if (msg.robot_info !== undefined) {
      resolved.robot_info = new Array(msg.robot_info.length);
      for (let i = 0; i < resolved.robot_info.length; ++i) {
        resolved.robot_info[i] = RobotInfo.Resolve(msg.robot_info[i]);
      }
    }
    else {
      resolved.robot_info = []
    }

    return resolved;
    }
};

module.exports = InterInfo;
