; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude platooningToggle.msg.html

(cl:defclass <platooningToggle> (roslisp-msg-protocol:ros-message)
  ((enable_platooning
    :reader enable_platooning
    :initarg :enable_platooning
    :type cl:boolean
    :initform cl:nil)
   (inner_platoon_distance
    :reader inner_platoon_distance
    :initarg :inner_platoon_distance
    :type cl:float
    :initform 0.0)
   (platoon_speed
    :reader platoon_speed
    :initarg :platoon_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass platooningToggle (<platooningToggle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <platooningToggle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'platooningToggle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<platooningToggle> is deprecated: use platooning-msg:platooningToggle instead.")))

(cl:ensure-generic-function 'enable_platooning-val :lambda-list '(m))
(cl:defmethod enable_platooning-val ((m <platooningToggle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:enable_platooning-val is deprecated.  Use platooning-msg:enable_platooning instead.")
  (enable_platooning m))

(cl:ensure-generic-function 'inner_platoon_distance-val :lambda-list '(m))
(cl:defmethod inner_platoon_distance-val ((m <platooningToggle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:inner_platoon_distance-val is deprecated.  Use platooning-msg:inner_platoon_distance instead.")
  (inner_platoon_distance m))

(cl:ensure-generic-function 'platoon_speed-val :lambda-list '(m))
(cl:defmethod platoon_speed-val ((m <platooningToggle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_speed-val is deprecated.  Use platooning-msg:platoon_speed instead.")
  (platoon_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <platooningToggle>) ostream)
  "Serializes a message object of type '<platooningToggle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable_platooning) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'inner_platoon_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'platoon_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <platooningToggle>) istream)
  "Deserializes a message object of type '<platooningToggle>"
    (cl:setf (cl:slot-value msg 'enable_platooning) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inner_platoon_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'platoon_speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<platooningToggle>)))
  "Returns string type for a message object of type '<platooningToggle>"
  "platooning/platooningToggle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'platooningToggle)))
  "Returns string type for a message object of type 'platooningToggle"
  "platooning/platooningToggle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<platooningToggle>)))
  "Returns md5sum for a message object of type '<platooningToggle>"
  "d873b59209a1794389a9e491f37fe75f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'platooningToggle)))
  "Returns md5sum for a message object of type 'platooningToggle"
  "d873b59209a1794389a9e491f37fe75f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<platooningToggle>)))
  "Returns full string definition for message of type '<platooningToggle>"
  (cl:format cl:nil "bool enable_platooning~%float32 inner_platoon_distance~%float32 platoon_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'platooningToggle)))
  "Returns full string definition for message of type 'platooningToggle"
  (cl:format cl:nil "bool enable_platooning~%float32 inner_platoon_distance~%float32 platoon_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <platooningToggle>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <platooningToggle>))
  "Converts a ROS message object to a list"
  (cl:list 'platooningToggle
    (cl:cons ':enable_platooning (enable_platooning msg))
    (cl:cons ':inner_platoon_distance (inner_platoon_distance msg))
    (cl:cons ':platoon_speed (platoon_speed msg))
))
