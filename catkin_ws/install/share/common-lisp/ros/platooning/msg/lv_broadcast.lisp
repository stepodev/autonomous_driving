; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude lv_broadcast.msg.html

(cl:defclass <lv_broadcast> (roslisp-msg-protocol:ros-message)
  ((src_vehicle
    :reader src_vehicle
    :initarg :src_vehicle
    :type cl:integer
    :initform 0)
   (platoon_id
    :reader platoon_id
    :initarg :platoon_id
    :type cl:integer
    :initform 0)
   (ipd
    :reader ipd
    :initarg :ipd
    :type cl:float
    :initform 0.0)
   (ps
    :reader ps
    :initarg :ps
    :type cl:float
    :initform 0.0)
   (followers
    :reader followers
    :initarg :followers
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass lv_broadcast (<lv_broadcast>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lv_broadcast>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lv_broadcast)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<lv_broadcast> is deprecated: use platooning-msg:lv_broadcast instead.")))

(cl:ensure-generic-function 'src_vehicle-val :lambda-list '(m))
(cl:defmethod src_vehicle-val ((m <lv_broadcast>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:src_vehicle-val is deprecated.  Use platooning-msg:src_vehicle instead.")
  (src_vehicle m))

(cl:ensure-generic-function 'platoon_id-val :lambda-list '(m))
(cl:defmethod platoon_id-val ((m <lv_broadcast>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_id-val is deprecated.  Use platooning-msg:platoon_id instead.")
  (platoon_id m))

(cl:ensure-generic-function 'ipd-val :lambda-list '(m))
(cl:defmethod ipd-val ((m <lv_broadcast>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:ipd-val is deprecated.  Use platooning-msg:ipd instead.")
  (ipd m))

(cl:ensure-generic-function 'ps-val :lambda-list '(m))
(cl:defmethod ps-val ((m <lv_broadcast>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:ps-val is deprecated.  Use platooning-msg:ps instead.")
  (ps m))

(cl:ensure-generic-function 'followers-val :lambda-list '(m))
(cl:defmethod followers-val ((m <lv_broadcast>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:followers-val is deprecated.  Use platooning-msg:followers instead.")
  (followers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lv_broadcast>) ostream)
  "Serializes a message object of type '<lv_broadcast>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ipd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'followers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'followers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lv_broadcast>) istream)
  "Deserializes a message object of type '<lv_broadcast>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ipd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ps) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'followers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'followers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lv_broadcast>)))
  "Returns string type for a message object of type '<lv_broadcast>"
  "platooning/lv_broadcast")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lv_broadcast)))
  "Returns string type for a message object of type 'lv_broadcast"
  "platooning/lv_broadcast")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lv_broadcast>)))
  "Returns md5sum for a message object of type '<lv_broadcast>"
  "a181b9158cc00e160e1fda722b669eac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lv_broadcast)))
  "Returns md5sum for a message object of type 'lv_broadcast"
  "a181b9158cc00e160e1fda722b669eac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lv_broadcast>)))
  "Returns full string definition for message of type '<lv_broadcast>"
  (cl:format cl:nil "uint32 src_vehicle~%uint32 platoon_id~%float32 ipd~%float32 ps~%uint32[] followers~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lv_broadcast)))
  "Returns full string definition for message of type 'lv_broadcast"
  (cl:format cl:nil "uint32 src_vehicle~%uint32 platoon_id~%float32 ipd~%float32 ps~%uint32[] followers~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lv_broadcast>))
  (cl:+ 0
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'followers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lv_broadcast>))
  "Converts a ROS message object to a list"
  (cl:list 'lv_broadcast
    (cl:cons ':src_vehicle (src_vehicle msg))
    (cl:cons ':platoon_id (platoon_id msg))
    (cl:cons ':ipd (ipd msg))
    (cl:cons ':ps (ps msg))
    (cl:cons ':followers (followers msg))
))
