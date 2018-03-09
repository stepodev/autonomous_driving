; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude fv_request.msg.html

(cl:defclass <fv_request> (roslisp-msg-protocol:ros-message)
  ((src_vehicle
    :reader src_vehicle
    :initarg :src_vehicle
    :type cl:integer
    :initform 0))
)

(cl:defclass fv_request (<fv_request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <fv_request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'fv_request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<fv_request> is deprecated: use platooning-msg:fv_request instead.")))

(cl:ensure-generic-function 'src_vehicle-val :lambda-list '(m))
(cl:defmethod src_vehicle-val ((m <fv_request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:src_vehicle-val is deprecated.  Use platooning-msg:src_vehicle instead.")
  (src_vehicle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <fv_request>) ostream)
  "Serializes a message object of type '<fv_request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <fv_request>) istream)
  "Deserializes a message object of type '<fv_request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<fv_request>)))
  "Returns string type for a message object of type '<fv_request>"
  "platooning/fv_request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fv_request)))
  "Returns string type for a message object of type 'fv_request"
  "platooning/fv_request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<fv_request>)))
  "Returns md5sum for a message object of type '<fv_request>"
  "07b1b45a74ecd2446be6ced3738fb3a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'fv_request)))
  "Returns md5sum for a message object of type 'fv_request"
  "07b1b45a74ecd2446be6ced3738fb3a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<fv_request>)))
  "Returns full string definition for message of type '<fv_request>"
  (cl:format cl:nil "uint32 src_vehicle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'fv_request)))
  "Returns full string definition for message of type 'fv_request"
  (cl:format cl:nil "uint32 src_vehicle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <fv_request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <fv_request>))
  "Converts a ROS message object to a list"
  (cl:list 'fv_request
    (cl:cons ':src_vehicle (src_vehicle msg))
))
