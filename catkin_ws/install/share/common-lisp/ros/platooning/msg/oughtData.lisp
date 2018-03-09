; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude oughtData.msg.html

(cl:defclass <oughtData> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (steeringAngle
    :reader steeringAngle
    :initarg :steeringAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass oughtData (<oughtData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <oughtData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'oughtData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<oughtData> is deprecated: use platooning-msg:oughtData instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <oughtData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:distance-val is deprecated.  Use platooning-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <oughtData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:speed-val is deprecated.  Use platooning-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'steeringAngle-val :lambda-list '(m))
(cl:defmethod steeringAngle-val ((m <oughtData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:steeringAngle-val is deprecated.  Use platooning-msg:steeringAngle instead.")
  (steeringAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <oughtData>) ostream)
  "Serializes a message object of type '<oughtData>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steeringAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <oughtData>) istream)
  "Deserializes a message object of type '<oughtData>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steeringAngle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<oughtData>)))
  "Returns string type for a message object of type '<oughtData>"
  "platooning/oughtData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'oughtData)))
  "Returns string type for a message object of type 'oughtData"
  "platooning/oughtData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<oughtData>)))
  "Returns md5sum for a message object of type '<oughtData>"
  "5d11ac8cfd9baa9abe3a207005a675b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'oughtData)))
  "Returns md5sum for a message object of type 'oughtData"
  "5d11ac8cfd9baa9abe3a207005a675b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<oughtData>)))
  "Returns full string definition for message of type '<oughtData>"
  (cl:format cl:nil "float32 distance~%float32 speed~%float32 steeringAngle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'oughtData)))
  "Returns full string definition for message of type 'oughtData"
  (cl:format cl:nil "float32 distance~%float32 speed~%float32 steeringAngle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <oughtData>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <oughtData>))
  "Converts a ROS message object to a list"
  (cl:list 'oughtData
    (cl:cons ':distance (distance msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':steeringAngle (steeringAngle msg))
))
