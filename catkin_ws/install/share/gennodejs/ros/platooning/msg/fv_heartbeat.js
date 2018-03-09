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

class fv_heartbeat {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.src_vehicle = null;
      this.platoon_id = null;
    }
    else {
      if (initObj.hasOwnProperty('src_vehicle')) {
        this.src_vehicle = initObj.src_vehicle
      }
      else {
        this.src_vehicle = 0;
      }
      if (initObj.hasOwnProperty('platoon_id')) {
        this.platoon_id = initObj.platoon_id
      }
      else {
        this.platoon_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type fv_heartbeat
    // Serialize message field [src_vehicle]
    bufferOffset = _serializer.uint32(obj.src_vehicle, buffer, bufferOffset);
    // Serialize message field [platoon_id]
    bufferOffset = _serializer.uint32(obj.platoon_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type fv_heartbeat
    let len;
    let data = new fv_heartbeat(null);
    // Deserialize message field [src_vehicle]
    data.src_vehicle = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [platoon_id]
    data.platoon_id = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/fv_heartbeat';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4641cd1aacd2a8c9fb2185d9e7b6b3df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 src_vehicle
    uint32 platoon_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new fv_heartbeat(null);
    if (msg.src_vehicle !== undefined) {
      resolved.src_vehicle = msg.src_vehicle;
    }
    else {
      resolved.src_vehicle = 0
    }

    if (msg.platoon_id !== undefined) {
      resolved.platoon_id = msg.platoon_id;
    }
    else {
      resolved.platoon_id = 0
    }

    return resolved;
    }
};

module.exports = fv_heartbeat;
