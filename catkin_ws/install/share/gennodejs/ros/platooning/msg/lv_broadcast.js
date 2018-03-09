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

class lv_broadcast {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.src_vehicle = null;
      this.platoon_id = null;
      this.ipd = null;
      this.ps = null;
      this.followers = null;
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
      if (initObj.hasOwnProperty('ipd')) {
        this.ipd = initObj.ipd
      }
      else {
        this.ipd = 0.0;
      }
      if (initObj.hasOwnProperty('ps')) {
        this.ps = initObj.ps
      }
      else {
        this.ps = 0.0;
      }
      if (initObj.hasOwnProperty('followers')) {
        this.followers = initObj.followers
      }
      else {
        this.followers = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lv_broadcast
    // Serialize message field [src_vehicle]
    bufferOffset = _serializer.uint32(obj.src_vehicle, buffer, bufferOffset);
    // Serialize message field [platoon_id]
    bufferOffset = _serializer.uint32(obj.platoon_id, buffer, bufferOffset);
    // Serialize message field [ipd]
    bufferOffset = _serializer.float32(obj.ipd, buffer, bufferOffset);
    // Serialize message field [ps]
    bufferOffset = _serializer.float32(obj.ps, buffer, bufferOffset);
    // Serialize message field [followers]
    bufferOffset = _arraySerializer.uint32(obj.followers, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lv_broadcast
    let len;
    let data = new lv_broadcast(null);
    // Deserialize message field [src_vehicle]
    data.src_vehicle = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [platoon_id]
    data.platoon_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ipd]
    data.ipd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ps]
    data.ps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [followers]
    data.followers = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.followers.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/lv_broadcast';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a181b9158cc00e160e1fda722b669eac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 src_vehicle
    uint32 platoon_id
    float32 ipd
    float32 ps
    uint32[] followers
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lv_broadcast(null);
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

    if (msg.ipd !== undefined) {
      resolved.ipd = msg.ipd;
    }
    else {
      resolved.ipd = 0.0
    }

    if (msg.ps !== undefined) {
      resolved.ps = msg.ps;
    }
    else {
      resolved.ps = 0.0
    }

    if (msg.followers !== undefined) {
      resolved.followers = msg.followers;
    }
    else {
      resolved.followers = []
    }

    return resolved;
    }
};

module.exports = lv_broadcast;
