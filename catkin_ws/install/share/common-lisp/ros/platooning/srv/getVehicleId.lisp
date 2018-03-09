; Auto-generated. Do not edit!


(cl:in-package platooning-srv)


;//! \htmlinclude getVehicleId-request.msg.html

(cl:defclass <getVehicleId-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getVehicleId-request (<getVehicleId-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getVehicleId-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getVehicleId-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-srv:<getVehicleId-request> is deprecated: use platooning-srv:getVehicleId-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getVehicleId-request>) ostream)
  "Serializes a message object of type '<getVehicleId-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getVehicleId-request>) istream)
  "Deserializes a message object of type '<getVehicleId-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getVehicleId-request>)))
  "Returns string type for a service object of type '<getVehicleId-request>"
  "platooning/getVehicleIdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getVehicleId-request)))
  "Returns string type for a service object of type 'getVehicleId-request"
  "platooning/getVehicleIdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getVehicleId-request>)))
  "Returns md5sum for a message object of type '<getVehicleId-request>"
  "5c96f4b8297034815b56c1d26627cd99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getVehicleId-request)))
  "Returns md5sum for a message object of type 'getVehicleId-request"
  "5c96f4b8297034815b56c1d26627cd99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getVehicleId-request>)))
  "Returns full string definition for message of type '<getVehicleId-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getVehicleId-request)))
  "Returns full string definition for message of type 'getVehicleId-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getVehicleId-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getVehicleId-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getVehicleId-request
))
;//! \htmlinclude getVehicleId-response.msg.html

(cl:defclass <getVehicleId-response> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:integer
    :initform 0))
)

(cl:defclass getVehicleId-response (<getVehicleId-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getVehicleId-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getVehicleId-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-srv:<getVehicleId-response> is deprecated: use platooning-srv:getVehicleId-response instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <getVehicleId-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-srv:vehicle_id-val is deprecated.  Use platooning-srv:vehicle_id instead.")
  (vehicle_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getVehicleId-response>) ostream)
  "Serializes a message object of type '<getVehicleId-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getVehicleId-response>) istream)
  "Deserializes a message object of type '<getVehicleId-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getVehicleId-response>)))
  "Returns string type for a service object of type '<getVehicleId-response>"
  "platooning/getVehicleIdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getVehicleId-response)))
  "Returns string type for a service object of type 'getVehicleId-response"
  "platooning/getVehicleIdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getVehicleId-response>)))
  "Returns md5sum for a message object of type '<getVehicleId-response>"
  "5c96f4b8297034815b56c1d26627cd99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getVehicleId-response)))
  "Returns md5sum for a message object of type 'getVehicleId-response"
  "5c96f4b8297034815b56c1d26627cd99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getVehicleId-response>)))
  "Returns full string definition for message of type '<getVehicleId-response>"
  (cl:format cl:nil "uint32 vehicle_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getVehicleId-response)))
  "Returns full string definition for message of type 'getVehicleId-response"
  (cl:format cl:nil "uint32 vehicle_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getVehicleId-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getVehicleId-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getVehicleId-response
    (cl:cons ':vehicle_id (vehicle_id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getVehicleId)))
  'getVehicleId-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getVehicleId)))
  'getVehicleId-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getVehicleId)))
  "Returns string type for a service object of type '<getVehicleId>"
  "platooning/getVehicleId")