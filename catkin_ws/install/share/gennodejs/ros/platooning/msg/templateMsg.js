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

class templateMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.templatebool = null;
      this.templatefloat = null;
      this.templatestring = null;
    }
    else {
      if (initObj.hasOwnProperty('templatebool')) {
        this.templatebool = initObj.templatebool
      }
      else {
        this.templatebool = false;
      }
      if (initObj.hasOwnProperty('templatefloat')) {
        this.templatefloat = initObj.templatefloat
      }
      else {
        this.templatefloat = 0.0;
      }
      if (initObj.hasOwnProperty('templatestring')) {
        this.templatestring = initObj.templatestring
      }
      else {
        this.templatestring = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type templateMsg
    // Serialize message field [templatebool]
    bufferOffset = _serializer.bool(obj.templatebool, buffer, bufferOffset);
    // Serialize message field [templatefloat]
    bufferOffset = _serializer.float64(obj.templatefloat, buffer, bufferOffset);
    // Serialize message field [templatestring]
    bufferOffset = _serializer.string(obj.templatestring, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type templateMsg
    let len;
    let data = new templateMsg(null);
    // Deserialize message field [templatebool]
    data.templatebool = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [templatefloat]
    data.templatefloat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [templatestring]
    data.templatestring = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.templatestring.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/templateMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06f3640a725749b7cc7a9141d075f992';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool templatebool
    float64 templatefloat
    string templatestring
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new templateMsg(null);
    if (msg.templatebool !== undefined) {
      resolved.templatebool = msg.templatebool;
    }
    else {
      resolved.templatebool = false
    }

    if (msg.templatefloat !== undefined) {
      resolved.templatefloat = msg.templatefloat;
    }
    else {
      resolved.templatefloat = 0.0
    }

    if (msg.templatestring !== undefined) {
      resolved.templatestring = msg.templatestring;
    }
    else {
      resolved.templatestring = ''
    }

    return resolved;
    }
};

module.exports = templateMsg;
