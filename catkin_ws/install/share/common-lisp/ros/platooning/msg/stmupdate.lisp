; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude stmupdate.msg.html

(cl:defclass <stmupdate> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0)
   (steeringAngle
    :reader steeringAngle
    :initarg :steeringAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass stmupdate (<stmupdate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stmupdate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stmupdate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<stmupdate> is deprecated: use platooning-msg:stmupdate instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <stmupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:id-val is deprecated.  Use platooning-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <stmupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:acceleration-val is deprecated.  Use platooning-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'steeringAngle-val :lambda-list '(m))
(cl:defmethod steeringAngle-val ((m <stmupdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:steeringAngle-val is deprecated.  Use platooning-msg:steeringAngle instead.")
  (steeringAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stmupdate>) ostream)
  "Serializes a message object of type '<stmupdate>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acceleration))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stmupdate>) istream)
  "Deserializes a message object of type '<stmupdate>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steeringAngle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stmupdate>)))
  "Returns string type for a message object of type '<stmupdate>"
  "platooning/stmupdate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stmupdate)))
  "Returns string type for a message object of type 'stmupdate"
  "platooning/stmupdate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stmupdate>)))
  "Returns md5sum for a message object of type '<stmupdate>"
  "85aa0214068a35b3ce74e798a0c59f18")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stmupdate)))
  "Returns md5sum for a message object of type 'stmupdate"
  "85aa0214068a35b3ce74e798a0c59f18")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stmupdate>)))
  "Returns full string definition for message of type '<stmupdate>"
  (cl:format cl:nil "uint32 id~%float32 acceleration~%float32 steeringAngle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stmupdate)))
  "Returns full string definition for message of type 'stmupdate"
  (cl:format cl:nil "uint32 id~%float32 acceleration~%float32 steeringAngle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stmupdate>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stmupdate>))
  "Converts a ROS message object to a list"
  (cl:list 'stmupdate
    (cl:cons ':id (id msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':steeringAngle (steeringAngle msg))
))
