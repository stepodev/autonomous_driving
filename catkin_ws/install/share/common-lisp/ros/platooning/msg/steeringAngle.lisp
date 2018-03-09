; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude steeringAngle.msg.html

(cl:defclass <steeringAngle> (roslisp-msg-protocol:ros-message)
  ((steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass steeringAngle (<steeringAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <steeringAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'steeringAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<steeringAngle> is deprecated: use platooning-msg:steeringAngle instead.")))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <steeringAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:steering_angle-val is deprecated.  Use platooning-msg:steering_angle instead.")
  (steering_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <steeringAngle>) ostream)
  "Serializes a message object of type '<steeringAngle>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <steeringAngle>) istream)
  "Deserializes a message object of type '<steeringAngle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<steeringAngle>)))
  "Returns string type for a message object of type '<steeringAngle>"
  "platooning/steeringAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'steeringAngle)))
  "Returns string type for a message object of type 'steeringAngle"
  "platooning/steeringAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<steeringAngle>)))
  "Returns md5sum for a message object of type '<steeringAngle>"
  "0a70504669a8dacda06aa18fb270817f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'steeringAngle)))
  "Returns md5sum for a message object of type 'steeringAngle"
  "0a70504669a8dacda06aa18fb270817f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<steeringAngle>)))
  "Returns full string definition for message of type '<steeringAngle>"
  (cl:format cl:nil "float64 steering_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'steeringAngle)))
  "Returns full string definition for message of type 'steeringAngle"
  (cl:format cl:nil "float64 steering_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <steeringAngle>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <steeringAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'steeringAngle
    (cl:cons ':steering_angle (steering_angle msg))
))
