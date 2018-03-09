; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude remotecontrolInput.msg.html

(cl:defclass <remotecontrolInput> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:integer
    :initform 0)
   (remote_speed
    :reader remote_speed
    :initarg :remote_speed
    :type cl:float
    :initform 0.0)
   (remote_angle
    :reader remote_angle
    :initarg :remote_angle
    :type cl:float
    :initform 0.0)
   (emergency_stop
    :reader emergency_stop
    :initarg :emergency_stop
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass remotecontrolInput (<remotecontrolInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <remotecontrolInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'remotecontrolInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<remotecontrolInput> is deprecated: use platooning-msg:remotecontrolInput instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <remotecontrolInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:vehicle_id-val is deprecated.  Use platooning-msg:vehicle_id instead.")
  (vehicle_id m))

(cl:ensure-generic-function 'remote_speed-val :lambda-list '(m))
(cl:defmethod remote_speed-val ((m <remotecontrolInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:remote_speed-val is deprecated.  Use platooning-msg:remote_speed instead.")
  (remote_speed m))

(cl:ensure-generic-function 'remote_angle-val :lambda-list '(m))
(cl:defmethod remote_angle-val ((m <remotecontrolInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:remote_angle-val is deprecated.  Use platooning-msg:remote_angle instead.")
  (remote_angle m))

(cl:ensure-generic-function 'emergency_stop-val :lambda-list '(m))
(cl:defmethod emergency_stop-val ((m <remotecontrolInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:emergency_stop-val is deprecated.  Use platooning-msg:emergency_stop instead.")
  (emergency_stop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <remotecontrolInput>) ostream)
  "Serializes a message object of type '<remotecontrolInput>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'remote_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'remote_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency_stop) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <remotecontrolInput>) istream)
  "Deserializes a message object of type '<remotecontrolInput>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'remote_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'remote_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'emergency_stop) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<remotecontrolInput>)))
  "Returns string type for a message object of type '<remotecontrolInput>"
  "platooning/remotecontrolInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'remotecontrolInput)))
  "Returns string type for a message object of type 'remotecontrolInput"
  "platooning/remotecontrolInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<remotecontrolInput>)))
  "Returns md5sum for a message object of type '<remotecontrolInput>"
  "48eff1d5f037dfb934e32ce687673a3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'remotecontrolInput)))
  "Returns md5sum for a message object of type 'remotecontrolInput"
  "48eff1d5f037dfb934e32ce687673a3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<remotecontrolInput>)))
  "Returns full string definition for message of type '<remotecontrolInput>"
  (cl:format cl:nil "uint32 vehicle_id~%float32 remote_speed~%float32 remote_angle~%bool emergency_stop~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'remotecontrolInput)))
  "Returns full string definition for message of type 'remotecontrolInput"
  (cl:format cl:nil "uint32 vehicle_id~%float32 remote_speed~%float32 remote_angle~%bool emergency_stop~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <remotecontrolInput>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <remotecontrolInput>))
  "Converts a ROS message object to a list"
  (cl:list 'remotecontrolInput
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':remote_speed (remote_speed msg))
    (cl:cons ':remote_angle (remote_angle msg))
    (cl:cons ':emergency_stop (emergency_stop msg))
))
