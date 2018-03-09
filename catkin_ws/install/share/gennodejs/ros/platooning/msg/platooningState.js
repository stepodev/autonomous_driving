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

class platooningState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.platooning_state = null;
      this.platoon_id = null;
      this.ipd = null;
      this.ps = null;
      this.vehicle_id = null;
      this.i_am_FV = null;
      this.i_am_LV = null;
      this.platoon_members = null;
    }
    else {
      if (initObj.hasOwnProperty('platooning_state')) {
        this.platooning_state = initObj.platooning_state
      }
      else {
        this.platooning_state = '';
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
      if (initObj.hasOwnProperty('vehicle_id')) {
        this.vehicle_id = initObj.vehicle_id
      }
      else {
        this.vehicle_id = 0;
      }
      if (initObj.hasOwnProperty('i_am_FV')) {
        this.i_am_FV = initObj.i_am_FV
      }
      else {
        this.i_am_FV = false;
      }
      if (initObj.hasOwnProperty('i_am_LV')) {
        this.i_am_LV = initObj.i_am_LV
      }
      else {
        this.i_am_LV = false;
      }
      if (initObj.hasOwnProperty('platoon_members')) {
        this.platoon_members = initObj.platoon_members
      }
      else {
        this.platoon_members = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type platooningState
    // Serialize message field [platooning_state]
    bufferOffset = _serializer.string(obj.platooning_state, buffer, bufferOffset);
    // Serialize message field [platoon_id]
    bufferOffset = _serializer.uint32(obj.platoon_id, buffer, bufferOffset);
    // Serialize message field [ipd]
    bufferOffset = _serializer.float32(obj.ipd, buffer, bufferOffset);
    // Serialize message field [ps]
    bufferOffset = _serializer.float32(obj.ps, buffer, bufferOffset);
    // Serialize message field [vehicle_id]
    bufferOffset = _serializer.uint32(obj.vehicle_id, buffer, bufferOffset);
    // Serialize message field [i_am_FV]
    bufferOffset = _serializer.bool(obj.i_am_FV, buffer, bufferOffset);
    // Serialize message field [i_am_LV]
    bufferOffset = _serializer.bool(obj.i_am_LV, buffer, bufferOffset);
    // Serialize message field [platoon_members]
    bufferOffset = _arraySerializer.uint32(obj.platoon_members, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type platooningState
    let len;
    let data = new platooningState(null);
    // Deserialize message field [platooning_state]
    data.platooning_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [platoon_id]
    data.platoon_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ipd]
    data.ipd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ps]
    data.ps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vehicle_id]
    data.vehicle_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [i_am_FV]
    data.i_am_FV = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [i_am_LV]
    data.i_am_LV = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [platoon_members]
    data.platoon_members = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.platooning_state.length;
    length += 4 * object.platoon_members.length;
    return length + 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'platooning/platooningState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '513c3389662a56c9d5dc40bcbdf23890';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string platooning_state
    uint32 platoon_id
    float32 ipd
    float32 ps
    uint32 vehicle_id
    bool i_am_FV
    bool i_am_LV
    uint32[] platoon_members
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new platooningState(null);
    if (msg.platooning_state !== undefined) {
      resolved.platooning_state = msg.platooning_state;
    }
    else {
      resolved.platooning_state = ''
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

    if (msg.vehicle_id !== undefined) {
      resolved.vehicle_id = msg.vehicle_id;
    }
    else {
      resolved.vehicle_id = 0
    }

    if (msg.i_am_FV !== undefined) {
      resolved.i_am_FV = msg.i_am_FV;
    }
    else {
      resolved.i_am_FV = false
    }

    if (msg.i_am_LV !== undefined) {
      resolved.i_am_LV = msg.i_am_LV;
    }
    else {
      resolved.i_am_LV = false
    }

    if (msg.platoon_members !== undefined) {
      resolved.platoon_members = msg.platoon_members;
    }
    else {
      resolved.platoon_members = []
    }

    return resolved;
    }
};

module.exports = platooningState;
