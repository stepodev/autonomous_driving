; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude targetDistance.msg.html

(cl:defclass <targetDistance> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass targetDistance (<targetDistance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <targetDistance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'targetDistance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<targetDistance> is deprecated: use platooning-msg:targetDistance instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <targetDistance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:distance-val is deprecated.  Use platooning-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <targetDistance>) ostream)
  "Serializes a message object of type '<targetDistance>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <targetDistance>) istream)
  "Deserializes a message object of type '<targetDistance>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<targetDistance>)))
  "Returns string type for a message object of type '<targetDistance>"
  "platooning/targetDistance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'targetDistance)))
  "Returns string type for a message object of type 'targetDistance"
  "platooning/targetDistance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<targetDistance>)))
  "Returns md5sum for a message object of type '<targetDistance>"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'targetDistance)))
  "Returns md5sum for a message object of type 'targetDistance"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<targetDistance>)))
  "Returns full string definition for message of type '<targetDistance>"
  (cl:format cl:nil "float32 distance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'targetDistance)))
  "Returns full string definition for message of type 'targetDistance"
  (cl:format cl:nil "float32 distance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <targetDistance>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <targetDistance>))
  "Converts a ROS message object to a list"
  (cl:list 'targetDistance
    (cl:cons ':distance (distance msg))
))
