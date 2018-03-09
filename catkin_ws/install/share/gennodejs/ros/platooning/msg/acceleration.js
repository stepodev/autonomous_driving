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

class acceleration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accelleration = null;
    }
    else {
      if (initObj.hasOwnProperty('accelleration')) {
        this.accelleration = initObj.accelleration
      }
      else {
        this.accelleration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type acceleration
    // Serialize message field [accelleration]
    bufferOffset = _serializer.float32(obj.accelleration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type acceleration
    let len;
    let data = new acceleration(null);
    // Deserialize message field [accelleration]
    data.accelleration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/acceleration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cb03383b65207f8a255b5fa95fedef3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 accelleration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new acceleration(null);
    if (msg.accelleration !== undefined) {
      resolved.accelleration = msg.accelleration;
    }
    else {
      resolved.accelleration = 0.0
    }

    return resolved;
    }
};

module.exports = acceleration;
