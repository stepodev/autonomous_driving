; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude fv_leave.msg.html

(cl:defclass <fv_leave> (roslisp-msg-protocol:ros-message)
  ((src_vehicle
    :reader src_vehicle
    :initarg :src_vehicle
    :type cl:integer
    :initform 0)
   (platoon_id
    :reader platoon_id
    :initarg :platoon_id
    :type cl:integer
    :initform 0))
)

(cl:defclass fv_leave (<fv_leave>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <fv_leave>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'fv_leave)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<fv_leave> is deprecated: use platooning-msg:fv_leave instead.")))

(cl:ensure-generic-function 'src_vehicle-val :lambda-list '(m))
(cl:defmethod src_vehicle-val ((m <fv_leave>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:src_vehicle-val is deprecated.  Use platooning-msg:src_vehicle instead.")
  (src_vehicle m))

(cl:ensure-generic-function 'platoon_id-val :lambda-list '(m))
(cl:defmethod platoon_id-val ((m <fv_leave>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_id-val is deprecated.  Use platooning-msg:platoon_id instead.")
  (platoon_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <fv_leave>) ostream)
  "Serializes a message object of type '<fv_leave>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <fv_leave>) istream)
  "Deserializes a message object of type '<fv_leave>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<fv_leave>)))
  "Returns string type for a message object of type '<fv_leave>"
  "platooning/fv_leave")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fv_leave)))
  "Returns string type for a message object of type 'fv_leave"
  "platooning/fv_leave")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<fv_leave>)))
  "Returns md5sum for a message object of type '<fv_leave>"
  "4641cd1aacd2a8c9fb2185d9e7b6b3df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'fv_leave)))
  "Returns md5sum for a message object of type 'fv_leave"
  "4641cd1aacd2a8c9fb2185d9e7b6b3df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<fv_leave>)))
  "Returns full string definition for message of type '<fv_leave>"
  (cl:format cl:nil "uint32 src_vehicle~%uint32 platoon_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'fv_leave)))
  "Returns full string definition for message of type 'fv_leave"
  (cl:format cl:nil "uint32 src_vehicle~%uint32 platoon_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <fv_leave>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <fv_leave>))
  "Converts a ROS message object to a list"
  (cl:list 'fv_leave
    (cl:cons ':src_vehicle (src_vehicle msg))
    (cl:cons ':platoon_id (platoon_id msg))
))
