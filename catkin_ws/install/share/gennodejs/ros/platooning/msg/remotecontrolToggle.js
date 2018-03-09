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

class remotecontrolToggle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vehicle_id = null;
      this.enable_remotecontrol = null;
    }
    else {
      if (initObj.hasOwnProperty('vehicle_id')) {
        this.vehicle_id = initObj.vehicle_id
      }
      else {
        this.vehicle_id = 0;
      }
      if (initObj.hasOwnProperty('enable_remotecontrol')) {
        this.enable_remotecontrol = initObj.enable_remotecontrol
      }
      else {
        this.enable_remotecontrol = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type remotecontrolToggle
    // Serialize message field [vehicle_id]
    bufferOffset = _serializer.uint32(obj.vehicle_id, buffer, bufferOffset);
    // Serialize message field [enable_remotecontrol]
    bufferOffset = _serializer.bool(obj.enable_remotecontrol, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type remotecontrolToggle
    let len;
    let data = new remotecontrolToggle(null);
    // Deserialize message field [vehicle_id]
    data.vehicle_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [enable_remotecontrol]
    data.enable_remotecontrol = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/remotecontrolToggle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8f56e4c7d94d4f505d773a5b64c0ffed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 vehicle_id
    bool enable_remotecontrol
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new remotecontrolToggle(null);
    if (msg.vehicle_id !== undefined) {
      resolved.vehicle_id = msg.vehicle_id;
    }
    else {
      resolved.vehicle_id = 0
    }

    if (msg.enable_remotecontrol !== undefined) {
      resolved.enable_remotecontrol = msg.enable_remotecontrol;
    }
    else {
      resolved.enable_remotecontrol = false
    }

    return resolved;
    }
};

module.exports = remotecontrolToggle;
