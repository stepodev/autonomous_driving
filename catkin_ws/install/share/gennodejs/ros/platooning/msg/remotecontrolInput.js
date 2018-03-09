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

class remotecontrolInput {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vehicle_id = null;
      this.remote_speed = null;
      this.remote_angle = null;
      this.emergency_stop = null;
    }
    else {
      if (initObj.hasOwnProperty('vehicle_id')) {
        this.vehicle_id = initObj.vehicle_id
      }
      else {
        this.vehicle_id = 0;
      }
      if (initObj.hasOwnProperty('remote_speed')) {
        this.remote_speed = initObj.remote_speed
      }
      else {
        this.remote_speed = 0.0;
      }
      if (initObj.hasOwnProperty('remote_angle')) {
        this.remote_angle = initObj.remote_angle
      }
      else {
        this.remote_angle = 0.0;
      }
      if (initObj.hasOwnProperty('emergency_stop')) {
        this.emergency_stop = initObj.emergency_stop
      }
      else {
        this.emergency_stop = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type remotecontrolInput
    // Serialize message field [vehicle_id]
    bufferOffset = _serializer.uint32(obj.vehicle_id, buffer, bufferOffset);
    // Serialize message field [remote_speed]
    bufferOffset = _serializer.float32(obj.remote_speed, buffer, bufferOffset);
    // Serialize message field [remote_angle]
    bufferOffset = _serializer.float32(obj.remote_angle, buffer, bufferOffset);
    // Serialize message field [emergency_stop]
    bufferOffset = _serializer.bool(obj.emergency_stop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type remotecontrolInput
    let len;
    let data = new remotecontrolInput(null);
    // Deserialize message field [vehicle_id]
    data.vehicle_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [remote_speed]
    data.remote_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [remote_angle]
    data.remote_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [emergency_stop]
    data.emergency_stop = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/remotecontrolInput';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '48eff1d5f037dfb934e32ce687673a3f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 vehicle_id
    float32 remote_speed
    float32 remote_angle
    bool emergency_stop
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new remotecontrolInput(null);
    if (msg.vehicle_id !== undefined) {
      resolved.vehicle_id = msg.vehicle_id;
    }
    else {
      resolved.vehicle_id = 0
    }

    if (msg.remote_speed !== undefined) {
      resolved.remote_speed = msg.remote_speed;
    }
    else {
      resolved.remote_speed = 0.0
    }

    if (msg.remote_angle !== undefined) {
      resolved.remote_angle = msg.remote_angle;
    }
    else {
      resolved.remote_angle = 0.0
    }

    if (msg.emergency_stop !== undefined) {
      resolved.emergency_stop = msg.emergency_stop;
    }
    else {
      resolved.emergency_stop = false
    }

    return resolved;
    }
};

module.exports = remotecontrolInput;
