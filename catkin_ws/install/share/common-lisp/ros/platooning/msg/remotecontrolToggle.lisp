; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude remotecontrolToggle.msg.html

(cl:defclass <remotecontrolToggle> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:integer
    :initform 0)
   (enable_remotecontrol
    :reader enable_remotecontrol
    :initarg :enable_remotecontrol
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass remotecontrolToggle (<remotecontrolToggle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <remotecontrolToggle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'remotecontrolToggle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<remotecontrolToggle> is deprecated: use platooning-msg:remotecontrolToggle instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <remotecontrolToggle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:vehicle_id-val is deprecated.  Use platooning-msg:vehicle_id instead.")
  (vehicle_id m))

(cl:ensure-generic-function 'enable_remotecontrol-val :lambda-list '(m))
(cl:defmethod enable_remotecontrol-val ((m <remotecontrolToggle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:enable_remotecontrol-val is deprecated.  Use platooning-msg:enable_remotecontrol instead.")
  (enable_remotecontrol m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <remotecontrolToggle>) ostream)
  "Serializes a message object of type '<remotecontrolToggle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable_remotecontrol) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <remotecontrolToggle>) istream)
  "Deserializes a message object of type '<remotecontrolToggle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enable_remotecontrol) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<remotecontrolToggle>)))
  "Returns string type for a message object of type '<remotecontrolToggle>"
  "platooning/remotecontrolToggle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'remotecontrolToggle)))
  "Returns string type for a message object of type 'remotecontrolToggle"
  "platooning/remotecontrolToggle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<remotecontrolToggle>)))
  "Returns md5sum for a message object of type '<remotecontrolToggle>"
  "8f56e4c7d94d4f505d773a5b64c0ffed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'remotecontrolToggle)))
  "Returns md5sum for a message object of type 'remotecontrolToggle"
  "8f56e4c7d94d4f505d773a5b64c0ffed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<remotecontrolToggle>)))
  "Returns full string definition for message of type '<remotecontrolToggle>"
  (cl:format cl:nil "uint32 vehicle_id~%bool enable_remotecontrol~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'remotecontrolToggle)))
  "Returns full string definition for message of type 'remotecontrolToggle"
  (cl:format cl:nil "uint32 vehicle_id~%bool enable_remotecontrol~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <remotecontrolToggle>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <remotecontrolToggle>))
  "Converts a ROS message object to a list"
  (cl:list 'remotecontrolToggle
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':enable_remotecontrol (enable_remotecontrol msg))
))
