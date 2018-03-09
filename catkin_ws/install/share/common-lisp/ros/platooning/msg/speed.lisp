; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude speed.msg.html

(cl:defclass <speed> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass speed (<speed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<speed> is deprecated: use platooning-msg:speed instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <speed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:speed-val is deprecated.  Use platooning-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speed>) ostream)
  "Serializes a message object of type '<speed>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speed>) istream)
  "Deserializes a message object of type '<speed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speed>)))
  "Returns string type for a message object of type '<speed>"
  "platooning/speed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speed)))
  "Returns string type for a message object of type 'speed"
  "platooning/speed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speed>)))
  "Returns md5sum for a message object of type '<speed>"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speed)))
  "Returns md5sum for a message object of type 'speed"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speed>)))
  "Returns full string definition for message of type '<speed>"
  (cl:format cl:nil "float32 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speed)))
  "Returns full string definition for message of type 'speed"
  (cl:format cl:nil "float32 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speed>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speed>))
  "Converts a ROS message object to a list"
  (cl:list 'speed
    (cl:cons ':speed (speed msg))
))
