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

class vehicleId {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vehicle_id = null;
    }
    else {
      if (initObj.hasOwnProperty('vehicle_id')) {
        this.vehicle_id = initObj.vehicle_id
      }
      else {
        this.vehicle_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vehicleId
    // Serialize message field [vehicle_id]
    bufferOffset = _serializer.uint32(obj.vehicle_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vehicleId
    let len;
    let data = new vehicleId(null);
    // Deserialize message field [vehicle_id]
    data.vehicle_id = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/vehicleId';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c96f4b8297034815b56c1d26627cd99';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 vehicle_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vehicleId(null);
    if (msg.vehicle_id !== undefined) {
      resolved.vehicle_id = msg.vehicle_id;
    }
    else {
      resolved.vehicle_id = 0
    }

    return resolved;
    }
};

module.exports = vehicleId;
