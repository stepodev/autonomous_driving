; Auto-generated. Do not edit!


(cl:in-package prius_msgs-msg)


;//! \htmlinclude Speed.msg.html

(cl:defclass <Speed> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass Speed (<Speed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name prius_msgs-msg:<Speed> is deprecated: use prius_msgs-msg:Speed instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Speed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prius_msgs-msg:speed-val is deprecated.  Use prius_msgs-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speed>) ostream)
  "Serializes a message object of type '<Speed>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speed>) istream)
  "Deserializes a message object of type '<Speed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speed>)))
  "Returns string type for a message object of type '<Speed>"
  "prius_msgs/Speed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speed)))
  "Returns string type for a message object of type 'Speed"
  "prius_msgs/Speed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speed>)))
  "Returns md5sum for a message object of type '<Speed>"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speed)))
  "Returns md5sum for a message object of type 'Speed"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speed>)))
  "Returns full string definition for message of type '<Speed>"
  (cl:format cl:nil "float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speed)))
  "Returns full string definition for message of type 'Speed"
  (cl:format cl:nil "float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speed>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speed>))
  "Converts a ROS message object to a list"
  (cl:list 'Speed
    (cl:cons ':speed (speed msg))
))
