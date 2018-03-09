// Auto-generated. Do not edit!

// (in-package platooning.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class vehiclecontrol {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accelleration = null;
      this.steering = null;
    }
    else {
      if (initObj.hasOwnProperty('accelleration')) {
        this.accelleration = initObj.accelleration
      }
      else {
        this.accelleration = 0.0;
      }
      if (initObj.hasOwnProperty('steering')) {
        this.steering = initObj.steering
      }
      else {
        this.steering = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vehiclecontrol
    // Serialize message field [accelleration]
    bufferOffset = _serializer.float32(obj.accelleration, buffer, bufferOffset);
    // Serialize message field [steering]
    bufferOffset = _serializer.float32(obj.steering, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vehiclecontrol
    let len;
    let data = new vehiclecontrol(null);
    // Deserialize message field [accelleration]
    data.accelleration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering]
    data.steering = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/vehiclecontrol';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41ed81cddf0d4f2bc76ecf491da03a04';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 accelleration
    float32 steering
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vehiclecontrol(null);
    if (msg.accelleration !== undefined) {
      resolved.accelleration = msg.accelleration;
    }
    else {
      resolved.accelleration = 0.0
    }

    if (msg.steering !== undefined) {
      resolved.steering = msg.steering;
    }
    else {
      resolved.steering = 0.0
    }

    return resolved;
    }
};

module.exports = vehiclecontrol;
