// Auto-generated. Do not edit!

// (in-package vpa_robot_task.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class AssignTaskRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_name = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_name')) {
        this.robot_name = initObj.robot_name
      }
      else {
        this.robot_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AssignTaskRequest
    // Serialize message field [robot_name]
    bufferOffset = _serializer.string(obj.robot_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AssignTaskRequest
    let len;
    let data = new AssignTaskRequest(null);
    // Deserialize message field [robot_name]
    data.robot_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.robot_name);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vpa_robot_task/AssignTaskRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '889a1d391e36604c7ce79bf95df72cb6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string robot_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AssignTaskRequest(null);
    if (msg.robot_name !== undefined) {
      resolved.robot_name = msg.robot_name;
    }
    else {
      resolved.robot_name = ''
    }

    return resolved;
    }
};

class AssignTaskResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_list = null;
    }
    else {
      if (initObj.hasOwnProperty('task_list')) {
        this.task_list = initObj.task_list
      }
      else {
        this.task_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AssignTaskResponse
    // Serialize message field [task_list]
    bufferOffset = _arraySerializer.int8(obj.task_list, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AssignTaskResponse
    let len;
    let data = new AssignTaskResponse(null);
    // Deserialize message field [task_list]
    data.task_list = _arrayDeserializer.int8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.task_list.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vpa_robot_task/AssignTaskResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34413843ec972b8746faccaca2ed6038';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8[] task_list
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AssignTaskResponse(null);
    if (msg.task_list !== undefined) {
      resolved.task_list = msg.task_list;
    }
    else {
      resolved.task_list = []
    }

    return resolved;
    }
};

module.exports = {
  Request: AssignTaskRequest,
  Response: AssignTaskResponse,
  md5sum() { return 'cdad0eb519e23dd2b5d3e4c2e2b7ecf6'; },
  datatype() { return 'vpa_robot_task/AssignTask'; }
};
