// Auto-generated. Do not edit!

// (in-package vpa_robot_interface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class WheelsOmegaInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.omega_left_ref = null;
      this.omega_left_sig = null;
      this.omega_right_ref = null;
      this.omega_right_sig = null;
    }
    else {
      if (initObj.hasOwnProperty('omega_left_ref')) {
        this.omega_left_ref = initObj.omega_left_ref
      }
      else {
        this.omega_left_ref = 0.0;
      }
      if (initObj.hasOwnProperty('omega_left_sig')) {
        this.omega_left_sig = initObj.omega_left_sig
      }
      else {
        this.omega_left_sig = 0.0;
      }
      if (initObj.hasOwnProperty('omega_right_ref')) {
        this.omega_right_ref = initObj.omega_right_ref
      }
      else {
        this.omega_right_ref = 0.0;
      }
      if (initObj.hasOwnProperty('omega_right_sig')) {
        this.omega_right_sig = initObj.omega_right_sig
      }
      else {
        this.omega_right_sig = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelsOmegaInfo
    // Serialize message field [omega_left_ref]
    bufferOffset = _serializer.float32(obj.omega_left_ref, buffer, bufferOffset);
    // Serialize message field [omega_left_sig]
    bufferOffset = _serializer.float32(obj.omega_left_sig, buffer, bufferOffset);
    // Serialize message field [omega_right_ref]
    bufferOffset = _serializer.float32(obj.omega_right_ref, buffer, bufferOffset);
    // Serialize message field [omega_right_sig]
    bufferOffset = _serializer.float32(obj.omega_right_sig, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelsOmegaInfo
    let len;
    let data = new WheelsOmegaInfo(null);
    // Deserialize message field [omega_left_ref]
    data.omega_left_ref = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [omega_left_sig]
    data.omega_left_sig = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [omega_right_ref]
    data.omega_right_ref = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [omega_right_sig]
    data.omega_right_sig = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vpa_robot_interface/WheelsOmegaInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba80e47329fa708db7e2fff0c002fd00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 omega_left_ref
    float32 omega_left_sig
    float32 omega_right_ref
    float32 omega_right_sig
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelsOmegaInfo(null);
    if (msg.omega_left_ref !== undefined) {
      resolved.omega_left_ref = msg.omega_left_ref;
    }
    else {
      resolved.omega_left_ref = 0.0
    }

    if (msg.omega_left_sig !== undefined) {
      resolved.omega_left_sig = msg.omega_left_sig;
    }
    else {
      resolved.omega_left_sig = 0.0
    }

    if (msg.omega_right_ref !== undefined) {
      resolved.omega_right_ref = msg.omega_right_ref;
    }
    else {
      resolved.omega_right_ref = 0.0
    }

    if (msg.omega_right_sig !== undefined) {
      resolved.omega_right_sig = msg.omega_right_sig;
    }
    else {
      resolved.omega_right_sig = 0.0
    }

    return resolved;
    }
};

module.exports = WheelsOmegaInfo;
