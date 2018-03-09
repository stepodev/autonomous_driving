; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude targetSpeed.msg.html

(cl:defclass <targetSpeed> (roslisp-msg-protocol:ros-message)
  ((target_speed
    :reader target_speed
    :initarg :target_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass targetSpeed (<targetSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <targetSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'targetSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<targetSpeed> is deprecated: use platooning-msg:targetSpeed instead.")))

(cl:ensure-generic-function 'target_speed-val :lambda-list '(m))
(cl:defmethod target_speed-val ((m <targetSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:target_speed-val is deprecated.  Use platooning-msg:target_speed instead.")
  (target_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <targetSpeed>) ostream)
  "Serializes a message object of type '<targetSpeed>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <targetSpeed>) istream)
  "Deserializes a message object of type '<targetSpeed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<targetSpeed>)))
  "Returns string type for a message object of type '<targetSpeed>"
  "platooning/targetSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'targetSpeed)))
  "Returns string type for a message object of type 'targetSpeed"
  "platooning/targetSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<targetSpeed>)))
  "Returns md5sum for a message object of type '<targetSpeed>"
  "a0e65cab2c5b59829b64f16ffc712119")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'targetSpeed)))
  "Returns md5sum for a message object of type 'targetSpeed"
  "a0e65cab2c5b59829b64f16ffc712119")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<targetSpeed>)))
  "Returns full string definition for message of type '<targetSpeed>"
  (cl:format cl:nil "float32 target_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'targetSpeed)))
  "Returns full string definition for message of type 'targetSpeed"
  (cl:format cl:nil "float32 target_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <targetSpeed>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <targetSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'targetSpeed
    (cl:cons ':target_speed (target_speed msg))
))
