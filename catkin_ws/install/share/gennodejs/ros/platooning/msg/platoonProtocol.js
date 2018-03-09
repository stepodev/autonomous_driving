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

class platoonProtocol {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.payload = null;
      this.message_type = null;
    }
    else {
      if (initObj.hasOwnProperty('payload')) {
        this.payload = initObj.payload
      }
      else {
        this.payload = '';
      }
      if (initObj.hasOwnProperty('message_type')) {
        this.message_type = initObj.message_type
      }
      else {
        this.message_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type platoonProtocol
    // Serialize message field [payload]
    bufferOffset = _serializer.string(obj.payload, buffer, bufferOffset);
    // Serialize message field [message_type]
    bufferOffset = _serializer.uint32(obj.message_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type platoonProtocol
    let len;
    let data = new platoonProtocol(null);
    // Deserialize message field [payload]
    data.payload = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [message_type]
    data.message_type = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.payload.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/platoonProtocol';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd9bea63af715371a2b1507d7c3fd0da2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string payload
    uint32 message_type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new platoonProtocol(null);
    if (msg.payload !== undefined) {
      resolved.payload = msg.payload;
    }
    else {
      resolved.payload = ''
    }

    if (msg.message_type !== undefined) {
      resolved.message_type = msg.message_type;
    }
    else {
      resolved.message_type = 0
    }

    return resolved;
    }
};

module.exports = platoonProtocol;
