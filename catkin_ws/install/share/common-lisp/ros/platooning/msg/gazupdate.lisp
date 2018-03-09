; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude gazupdate.msg.html

(cl:defclass <gazupdate> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass gazupdate (<gazupdate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gazupdate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gazupdate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<gazupdate> is deprecated: use platooning-msg:gazupdate instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <gazupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:id-val is deprecated.  Use platooning-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <gazupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:distance-val is deprecated.  Use platooning-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <gazupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:speed-val is deprecated.  Use platooning-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gazupdate>) ostream)
  "Serializes a message object of type '<gazupdate>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gazupdate>) istream)
  "Deserializes a message object of type '<gazupdate>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gazupdate>)))
  "Returns string type for a message object of type '<gazupdate>"
  "platooning/gazupdate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazupdate)))
  "Returns string type for a message object of type 'gazupdate"
  "platooning/gazupdate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gazupdate>)))
  "Returns md5sum for a message object of type '<gazupdate>"
  "e013a91e1d04c1d0c4b05e286456c85e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gazupdate)))
  "Returns md5sum for a message object of type 'gazupdate"
  "e013a91e1d04c1d0c4b05e286456c85e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gazupdate>)))
  "Returns full string definition for message of type '<gazupdate>"
  (cl:format cl:nil "uint32 id~%float32 distance~%float32 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gazupdate)))
  "Returns full string definition for message of type 'gazupdate"
  (cl:format cl:nil "uint32 id~%float32 distance~%float32 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gazupdate>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gazupdate>))
  "Converts a ROS message object to a list"
  (cl:list 'gazupdate
    (cl:cons ':id (id msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':speed (speed msg))
))
