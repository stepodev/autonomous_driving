; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude vehicleId.msg.html

(cl:defclass <vehicleId> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:integer
    :initform 0))
)

(cl:defclass vehicleId (<vehicleId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vehicleId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vehicleId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<vehicleId> is deprecated: use platooning-msg:vehicleId instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <vehicleId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:vehicle_id-val is deprecated.  Use platooning-msg:vehicle_id instead.")
  (vehicle_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vehicleId>) ostream)
  "Serializes a message object of type '<vehicleId>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vehicleId>) istream)
  "Deserializes a message object of type '<vehicleId>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vehicleId>)))
  "Returns string type for a message object of type '<vehicleId>"
  "platooning/vehicleId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vehicleId)))
  "Returns string type for a message object of type 'vehicleId"
  "platooning/vehicleId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vehicleId>)))
  "Returns md5sum for a message object of type '<vehicleId>"
  "5c96f4b8297034815b56c1d26627cd99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vehicleId)))
  "Returns md5sum for a message object of type 'vehicleId"
  "5c96f4b8297034815b56c1d26627cd99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vehicleId>)))
  "Returns full string definition for message of type '<vehicleId>"
  (cl:format cl:nil "uint32 vehicle_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vehicleId)))
  "Returns full string definition for message of type 'vehicleId"
  (cl:format cl:nil "uint32 vehicle_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vehicleId>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vehicleId>))
  "Converts a ROS message object to a list"
  (cl:list 'vehicleId
    (cl:cons ':vehicle_id (vehicle_id msg))
))
