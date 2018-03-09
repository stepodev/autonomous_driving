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

class lv_reject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.src_vehicle = null;
      this.dst_vehicle = null;
      this.platoon_id = null;
    }
    else {
      if (initObj.hasOwnProperty('src_vehicle')) {
        this.src_vehicle = initObj.src_vehicle
      }
      else {
        this.src_vehicle = 0;
      }
      if (initObj.hasOwnProperty('dst_vehicle')) {
        this.dst_vehicle = initObj.dst_vehicle
      }
      else {
        this.dst_vehicle = 0;
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
    // Serializes a message object of type lv_reject
    // Serialize message field [src_vehicle]
    bufferOffset = _serializer.uint32(obj.src_vehicle, buffer, bufferOffset);
    // Serialize message field [dst_vehicle]
    bufferOffset = _serializer.uint32(obj.dst_vehicle, buffer, bufferOffset);
    // Serialize message field [platoon_id]
    bufferOffset = _serializer.uint32(obj.platoon_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lv_reject
    let len;
    let data = new lv_reject(null);
    // Deserialize message field [src_vehicle]
    data.src_vehicle = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dst_vehicle]
    data.dst_vehicle = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [platoon_id]
    data.platoon_id = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/lv_reject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '171c710bbe681f26bc25a8cb194a204b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 src_vehicle
    uint32 dst_vehicle
    uint32 platoon_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lv_reject(null);
    if (msg.src_vehicle !== undefined) {
      resolved.src_vehicle = msg.src_vehicle;
    }
    else {
      resolved.src_vehicle = 0
    }

    if (msg.dst_vehicle !== undefined) {
      resolved.dst_vehicle = msg.dst_vehicle;
    }
    else {
      resolved.dst_vehicle = 0
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

module.exports = lv_reject;
