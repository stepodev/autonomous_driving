; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude lv_accept.msg.html

(cl:defclass <lv_accept> (roslisp-msg-protocol:ros-message)
  ((src_vehicle
    :reader src_vehicle
    :initarg :src_vehicle
    :type cl:integer
    :initform 0)
   (dst_vehicle
    :reader dst_vehicle
    :initarg :dst_vehicle
    :type cl:integer
    :initform 0)
   (platoon_id
    :reader platoon_id
    :initarg :platoon_id
    :type cl:integer
    :initform 0))
)

(cl:defclass lv_accept (<lv_accept>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lv_accept>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lv_accept)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<lv_accept> is deprecated: use platooning-msg:lv_accept instead.")))

(cl:ensure-generic-function 'src_vehicle-val :lambda-list '(m))
(cl:defmethod src_vehicle-val ((m <lv_accept>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:src_vehicle-val is deprecated.  Use platooning-msg:src_vehicle instead.")
  (src_vehicle m))

(cl:ensure-generic-function 'dst_vehicle-val :lambda-list '(m))
(cl:defmethod dst_vehicle-val ((m <lv_accept>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:dst_vehicle-val is deprecated.  Use platooning-msg:dst_vehicle instead.")
  (dst_vehicle m))

(cl:ensure-generic-function 'platoon_id-val :lambda-list '(m))
(cl:defmethod platoon_id-val ((m <lv_accept>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_id-val is deprecated.  Use platooning-msg:platoon_id instead.")
  (platoon_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lv_accept>) ostream)
  "Serializes a message object of type '<lv_accept>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dst_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dst_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dst_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dst_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lv_accept>) istream)
  "Deserializes a message object of type '<lv_accept>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dst_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dst_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dst_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dst_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lv_accept>)))
  "Returns string type for a message object of type '<lv_accept>"
  "platooning/lv_accept")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lv_accept)))
  "Returns string type for a message object of type 'lv_accept"
  "platooning/lv_accept")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lv_accept>)))
  "Returns md5sum for a message object of type '<lv_accept>"
  "171c710bbe681f26bc25a8cb194a204b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lv_accept)))
  "Returns md5sum for a message object of type 'lv_accept"
  "171c710bbe681f26bc25a8cb194a204b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lv_accept>)))
  "Returns full string definition for message of type '<lv_accept>"
  (cl:format cl:nil "uint32 src_vehicle~%uint32 dst_vehicle~%uint32 platoon_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lv_accept)))
  "Returns full string definition for message of type 'lv_accept"
  (cl:format cl:nil "uint32 src_vehicle~%uint32 dst_vehicle~%uint32 platoon_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lv_accept>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lv_accept>))
  "Converts a ROS message object to a list"
  (cl:list 'lv_accept
    (cl:cons ':src_vehicle (src_vehicle msg))
    (cl:cons ':dst_vehicle (dst_vehicle msg))
    (cl:cons ':platoon_id (platoon_id msg))
))