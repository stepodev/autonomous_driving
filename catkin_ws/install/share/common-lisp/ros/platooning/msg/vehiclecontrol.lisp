; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude vehiclecontrol.msg.html

(cl:defclass <vehiclecontrol> (roslisp-msg-protocol:ros-message)
  ((accelleration
    :reader accelleration
    :initarg :accelleration
    :type cl:float
    :initform 0.0)
   (steering
    :reader steering
    :initarg :steering
    :type cl:float
    :initform 0.0))
)

(cl:defclass vehiclecontrol (<vehiclecontrol>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vehiclecontrol>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vehiclecontrol)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<vehiclecontrol> is deprecated: use platooning-msg:vehiclecontrol instead.")))

(cl:ensure-generic-function 'accelleration-val :lambda-list '(m))
(cl:defmethod accelleration-val ((m <vehiclecontrol>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:accelleration-val is deprecated.  Use platooning-msg:accelleration instead.")
  (accelleration m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <vehiclecontrol>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:steering-val is deprecated.  Use platooning-msg:steering instead.")
  (steering m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vehiclecontrol>) ostream)
  "Serializes a message object of type '<vehiclecontrol>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'accelleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vehiclecontrol>) istream)
  "Deserializes a message object of type '<vehiclecontrol>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accelleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vehiclecontrol>)))
  "Returns string type for a message object of type '<vehiclecontrol>"
  "platooning/vehiclecontrol")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vehiclecontrol)))
  "Returns string type for a message object of type 'vehiclecontrol"
  "platooning/vehiclecontrol")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vehiclecontrol>)))
  "Returns md5sum for a message object of type '<vehiclecontrol>"
  "41ed81cddf0d4f2bc76ecf491da03a04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vehiclecontrol)))
  "Returns md5sum for a message object of type 'vehiclecontrol"
  "41ed81cddf0d4f2bc76ecf491da03a04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vehiclecontrol>)))
  "Returns full string definition for message of type '<vehiclecontrol>"
  (cl:format cl:nil "float32 accelleration~%float32 steering~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vehiclecontrol)))
  "Returns full string definition for message of type 'vehiclecontrol"
  (cl:format cl:nil "float32 accelleration~%float32 steering~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vehiclecontrol>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vehiclecontrol>))
  "Converts a ROS message object to a list"
  (cl:list 'vehiclecontrol
    (cl:cons ':accelleration (accelleration msg))
    (cl:cons ':steering (steering msg))
))
