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

class registerTestcases {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.testcase = null;
    }
    else {
      if (initObj.hasOwnProperty('testcase')) {
        this.testcase = initObj.testcase
      }
      else {
        this.testcase = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type registerTestcases
    // Serialize message field [testcase]
    bufferOffset = _serializer.string(obj.testcase, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type registerTestcases
    let len;
    let data = new registerTestcases(null);
    // Deserialize message field [testcase]
    data.testcase = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.testcase.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/registerTestcases';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1ec9035577ebb432d9345a861e195f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string testcase
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new registerTestcases(null);
    if (msg.testcase !== undefined) {
      resolved.testcase = msg.testcase;
    }
    else {
      resolved.testcase = ''
    }

    return resolved;
    }
};

module.exports = registerTestcases;
