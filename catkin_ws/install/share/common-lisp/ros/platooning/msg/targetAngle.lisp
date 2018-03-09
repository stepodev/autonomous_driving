; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude targetAngle.msg.html

(cl:defclass <targetAngle> (roslisp-msg-protocol:ros-message)
  ((steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass targetAngle (<targetAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <targetAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'targetAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<targetAngle> is deprecated: use platooning-msg:targetAngle instead.")))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <targetAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:steering_angle-val is deprecated.  Use platooning-msg:steering_angle instead.")
  (steering_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <targetAngle>) ostream)
  "Serializes a message object of type '<targetAngle>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <targetAngle>) istream)
  "Deserializes a message object of type '<targetAngle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<targetAngle>)))
  "Returns string type for a message object of type '<targetAngle>"
  "platooning/targetAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'targetAngle)))
  "Returns string type for a message object of type 'targetAngle"
  "platooning/targetAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<targetAngle>)))
  "Returns md5sum for a message object of type '<targetAngle>"
  "3de175f327000a99f382f084344450d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'targetAngle)))
  "Returns md5sum for a message object of type 'targetAngle"
  "3de175f327000a99f382f084344450d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<targetAngle>)))
  "Returns full string definition for message of type '<targetAngle>"
  (cl:format cl:nil "float32 steering_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'targetAngle)))
  "Returns full string definition for message of type 'targetAngle"
  (cl:format cl:nil "float32 steering_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <targetAngle>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <targetAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'targetAngle
    (cl:cons ':steering_angle (steering_angle msg))
))
