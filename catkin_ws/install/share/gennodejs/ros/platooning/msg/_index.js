
"use strict";

let steeringAngle = require('./steeringAngle.js');
let templateMsg = require('./templateMsg.js');
let targetAngle = require('./targetAngle.js');
let targetSpeed = require('./targetSpeed.js');
let targetDistance = require('./targetDistance.js');
let acceleration = require('./acceleration.js');
let userInterface = require('./userInterface.js');
let distanceToObj = require('./distanceToObj.js');
let platoonProtocol = require('./platoonProtocol.js');

module.exports = {
  steeringAngle: steeringAngle,
  templateMsg: templateMsg,
  targetAngle: targetAngle,
  targetSpeed: targetSpeed,
  targetDistance: targetDistance,
  acceleration: acceleration,
  userInterface: userInterface,
  distanceToObj: distanceToObj,
  platoonProtocol: platoonProtocol,
};
