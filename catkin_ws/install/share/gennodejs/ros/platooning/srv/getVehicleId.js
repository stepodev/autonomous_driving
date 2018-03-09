// Auto-generated. Do not edit!

// (in-package platooning.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class getVehicleIdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getVehicleIdRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getVehicleIdRequest
    let len;
    let data = new getVehicleIdRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'platooning/getVehicleIdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getVehicleIdRequest(null);
    return resolved;
    }
};

class getVehicleIdResponse {
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
    // Serializes a message object of type getVehicleIdResponse
    // Serialize message field [vehicle_id]
    bufferOffset = _serializer.uint32(obj.vehicle_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getVehicleIdResponse
    let len;
    let data = new getVehicleIdResponse(null);
    // Deserialize message field [vehicle_id]
    data.vehicle_id = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'platooning/getVehicleIdResponse';
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
    const resolved = new getVehicleIdResponse(null);
    if (msg.vehicle_id !== undefined) {
      resolved.vehicle_id = msg.vehicle_id;
    }
    else {
      resolved.vehicle_id = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: getVehicleIdRequest,
  Response: getVehicleIdResponse,
  md5sum() { return '5c96f4b8297034815b56c1d26627cd99'; },
  datatype() { return 'platooning/getVehicleId'; }
};
