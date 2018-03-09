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

class camera {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pic = null;
    }
    else {
      if (initObj.hasOwnProperty('pic')) {
        this.pic = initObj.pic
      }
      else {
        this.pic = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type camera
    // Serialize message field [pic]
    bufferOffset = _serializer.string(obj.pic, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type camera
    let len;
    let data = new camera(null);
    // Deserialize message field [pic]
    data.pic = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.pic.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/camera';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b029feab08f2c4877092228da5ee6702';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string pic
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new camera(null);
    if (msg.pic !== undefined) {
      resolved.pic = msg.pic;
    }
    else {
      resolved.pic = ''
    }

    return resolved;
    }
};

module.exports = camera;
