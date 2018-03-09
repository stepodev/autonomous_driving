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

class userInterface {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.leading_vehicle = null;
      this.following_vehicle = null;
      this.potential_following_vehicle = null;
      this.inner_platoon_distance = null;
      this.actual_distance = null;
      this.platoon_speed = null;
      this.speed = null;
      this.platooning_state = null;
      this.src_vehicle = null;
      this.platoon_size = null;
      this.platoon_members = null;
      this.enable_remotecontrol = null;
    }
    else {
      if (initObj.hasOwnProperty('leading_vehicle')) {
        this.leading_vehicle = initObj.leading_vehicle
      }
      else {
        this.leading_vehicle = false;
      }
      if (initObj.hasOwnProperty('following_vehicle')) {
        this.following_vehicle = initObj.following_vehicle
      }
      else {
        this.following_vehicle = false;
      }
      if (initObj.hasOwnProperty('potential_following_vehicle')) {
        this.potential_following_vehicle = initObj.potential_following_vehicle
      }
      else {
        this.potential_following_vehicle = false;
      }
      if (initObj.hasOwnProperty('inner_platoon_distance')) {
        this.inner_platoon_distance = initObj.inner_platoon_distance
      }
      else {
        this.inner_platoon_distance = 0.0;
      }
      if (initObj.hasOwnProperty('actual_distance')) {
        this.actual_distance = initObj.actual_distance
      }
      else {
        this.actual_distance = 0.0;
      }
      if (initObj.hasOwnProperty('platoon_speed')) {
        this.platoon_speed = initObj.platoon_speed
      }
      else {
        this.platoon_speed = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('platooning_state')) {
        this.platooning_state = initObj.platooning_state
      }
      else {
        this.platooning_state = '';
      }
      if (initObj.hasOwnProperty('src_vehicle')) {
        this.src_vehicle = initObj.src_vehicle
      }
      else {
        this.src_vehicle = 0;
      }
      if (initObj.hasOwnProperty('platoon_size')) {
        this.platoon_size = initObj.platoon_size
      }
      else {
        this.platoon_size = 0;
      }
      if (initObj.hasOwnProperty('platoon_members')) {
        this.platoon_members = initObj.platoon_members
      }
      else {
        this.platoon_members = [];
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
    // Serializes a message object of type userInterface
    // Serialize message field [leading_vehicle]
    bufferOffset = _serializer.bool(obj.leading_vehicle, buffer, bufferOffset);
    // Serialize message field [following_vehicle]
    bufferOffset = _serializer.bool(obj.following_vehicle, buffer, bufferOffset);
    // Serialize message field [potential_following_vehicle]
    bufferOffset = _serializer.bool(obj.potential_following_vehicle, buffer, bufferOffset);
    // Serialize message field [inner_platoon_distance]
    bufferOffset = _serializer.float32(obj.inner_platoon_distance, buffer, bufferOffset);
    // Serialize message field [actual_distance]
    bufferOffset = _serializer.float32(obj.actual_distance, buffer, bufferOffset);
    // Serialize message field [platoon_speed]
    bufferOffset = _serializer.float32(obj.platoon_speed, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [platooning_state]
    bufferOffset = _serializer.string(obj.platooning_state, buffer, bufferOffset);
    // Serialize message field [src_vehicle]
    bufferOffset = _serializer.uint32(obj.src_vehicle, buffer, bufferOffset);
    // Serialize message field [platoon_size]
    bufferOffset = _serializer.uint32(obj.platoon_size, buffer, bufferOffset);
    // Serialize message field [platoon_members]
    bufferOffset = _arraySerializer.uint32(obj.platoon_members, buffer, bufferOffset, null);
    // Serialize message field [enable_remotecontrol]
    bufferOffset = _serializer.bool(obj.enable_remotecontrol, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type userInterface
    let len;
    let data = new userInterface(null);
    // Deserialize message field [leading_vehicle]
    data.leading_vehicle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [following_vehicle]
    data.following_vehicle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [potential_following_vehicle]
    data.potential_following_vehicle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [inner_platoon_distance]
    data.inner_platoon_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [actual_distance]
    data.actual_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [platoon_speed]
    data.platoon_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [platooning_state]
    data.platooning_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [src_vehicle]
    data.src_vehicle = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [platoon_size]
    data.platoon_size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [platoon_members]
    data.platoon_members = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [enable_remotecontrol]
    data.enable_remotecontrol = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.platooning_state.length;
    length += 4 * object.platoon_members.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/userInterface';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1d08fffe38d5dc5a1aad88d74db0b91';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool leading_vehicle
    bool following_vehicle
    bool potential_following_vehicle
    float32 inner_platoon_distance
    float32 actual_distance
    float32 platoon_speed
    float32 speed
    string platooning_state
    uint32 src_vehicle
    uint32 platoon_size
    uint32[] platoon_members
    bool enable_remotecontrol
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new userInterface(null);
    if (msg.leading_vehicle !== undefined) {
      resolved.leading_vehicle = msg.leading_vehicle;
    }
    else {
      resolved.leading_vehicle = false
    }

    if (msg.following_vehicle !== undefined) {
      resolved.following_vehicle = msg.following_vehicle;
    }
    else {
      resolved.following_vehicle = false
    }

    if (msg.potential_following_vehicle !== undefined) {
      resolved.potential_following_vehicle = msg.potential_following_vehicle;
    }
    else {
      resolved.potential_following_vehicle = false
    }

    if (msg.inner_platoon_distance !== undefined) {
      resolved.inner_platoon_distance = msg.inner_platoon_distance;
    }
    else {
      resolved.inner_platoon_distance = 0.0
    }

    if (msg.actual_distance !== undefined) {
      resolved.actual_distance = msg.actual_distance;
    }
    else {
      resolved.actual_distance = 0.0
    }

    if (msg.platoon_speed !== undefined) {
      resolved.platoon_speed = msg.platoon_speed;
    }
    else {
      resolved.platoon_speed = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.platooning_state !== undefined) {
      resolved.platooning_state = msg.platooning_state;
    }
    else {
      resolved.platooning_state = ''
    }

    if (msg.src_vehicle !== undefined) {
      resolved.src_vehicle = msg.src_vehicle;
    }
    else {
      resolved.src_vehicle = 0
    }

    if (msg.platoon_size !== undefined) {
      resolved.platoon_size = msg.platoon_size;
    }
    else {
      resolved.platoon_size = 0
    }

    if (msg.platoon_members !== undefined) {
      resolved.platoon_members = msg.platoon_members;
    }
    else {
      resolved.platoon_members = []
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

module.exports = userInterface;
