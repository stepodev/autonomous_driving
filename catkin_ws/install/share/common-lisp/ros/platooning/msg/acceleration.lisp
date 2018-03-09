; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude acceleration.msg.html

(cl:defclass <acceleration> (roslisp-msg-protocol:ros-message)
  ((accelleration
    :reader accelleration
    :initarg :accelleration
    :type cl:float
    :initform 0.0))
)

(cl:defclass acceleration (<acceleration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <acceleration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'acceleration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<acceleration> is deprecated: use platooning-msg:acceleration instead.")))

(cl:ensure-generic-function 'accelleration-val :lambda-list '(m))
(cl:defmethod accelleration-val ((m <acceleration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:accelleration-val is deprecated.  Use platooning-msg:accelleration instead.")
  (accelleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <acceleration>) ostream)
  "Serializes a message object of type '<acceleration>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'accelleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <acceleration>) istream)
  "Deserializes a message object of type '<acceleration>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accelleration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<acceleration>)))
  "Returns string type for a message object of type '<acceleration>"
  "platooning/acceleration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'acceleration)))
  "Returns string type for a message object of type 'acceleration"
  "platooning/acceleration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<acceleration>)))
  "Returns md5sum for a message object of type '<acceleration>"
  "5cb03383b65207f8a255b5fa95fedef3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'acceleration)))
  "Returns md5sum for a message object of type 'acceleration"
  "5cb03383b65207f8a255b5fa95fedef3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<acceleration>)))
  "Returns full string definition for message of type '<acceleration>"
  (cl:format cl:nil "float32 accelleration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'acceleration)))
  "Returns full string definition for message of type 'acceleration"
  (cl:format cl:nil "float32 accelleration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <acceleration>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <acceleration>))
  "Converts a ROS message object to a list"
  (cl:list 'acceleration
    (cl:cons ':accelleration (accelleration msg))
))
