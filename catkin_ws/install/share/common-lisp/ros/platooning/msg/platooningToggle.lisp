; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude platooningToggle.msg.html

(cl:defclass <platooningToggle> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:integer
    :initform 0)
   (enable_platooning
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
    :initform 0.0)
   (lvfv
    :reader lvfv
    :initarg :lvfv
    :type cl:string
    :initform ""))
)

(cl:defclass platooningToggle (<platooningToggle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <platooningToggle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'platooningToggle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<platooningToggle> is deprecated: use platooning-msg:platooningToggle instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <platooningToggle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:vehicle_id-val is deprecated.  Use platooning-msg:vehicle_id instead.")
  (vehicle_id m))

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

(cl:ensure-generic-function 'lvfv-val :lambda-list '(m))
(cl:defmethod lvfv-val ((m <platooningToggle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:lvfv-val is deprecated.  Use platooning-msg:lvfv instead.")
  (lvfv m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <platooningToggle>) ostream)
  "Serializes a message object of type '<platooningToggle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) ostream)
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lvfv))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lvfv))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <platooningToggle>) istream)
  "Deserializes a message object of type '<platooningToggle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lvfv) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lvfv) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
  "0b95d94536aba6d55c50abe7d1619f57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'platooningToggle)))
  "Returns md5sum for a message object of type 'platooningToggle"
  "0b95d94536aba6d55c50abe7d1619f57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<platooningToggle>)))
  "Returns full string definition for message of type '<platooningToggle>"
  (cl:format cl:nil "uint32 vehicle_id~%bool enable_platooning~%float32 inner_platoon_distance~%float32 platoon_speed~%string lvfv~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'platooningToggle)))
  "Returns full string definition for message of type 'platooningToggle"
  (cl:format cl:nil "uint32 vehicle_id~%bool enable_platooning~%float32 inner_platoon_distance~%float32 platoon_speed~%string lvfv~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <platooningToggle>))
  (cl:+ 0
     4
     1
     4
     4
     4 (cl:length (cl:slot-value msg 'lvfv))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <platooningToggle>))
  "Converts a ROS message object to a list"
  (cl:list 'platooningToggle
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':enable_platooning (enable_platooning msg))
    (cl:cons ':inner_platoon_distance (inner_platoon_distance msg))
    (cl:cons ':platoon_speed (platoon_speed msg))
    (cl:cons ':lvfv (lvfv msg))
))
