; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude platoonProtocol.msg.html

(cl:defclass <platoonProtocol> (roslisp-msg-protocol:ros-message)
  ((payload
    :reader payload
    :initarg :payload
    :type cl:string
    :initform "")
   (message_type
    :reader message_type
    :initarg :message_type
    :type cl:integer
    :initform 0))
)

(cl:defclass platoonProtocol (<platoonProtocol>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <platoonProtocol>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'platoonProtocol)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<platoonProtocol> is deprecated: use platooning-msg:platoonProtocol instead.")))

(cl:ensure-generic-function 'payload-val :lambda-list '(m))
(cl:defmethod payload-val ((m <platoonProtocol>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:payload-val is deprecated.  Use platooning-msg:payload instead.")
  (payload m))

(cl:ensure-generic-function 'message_type-val :lambda-list '(m))
(cl:defmethod message_type-val ((m <platoonProtocol>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:message_type-val is deprecated.  Use platooning-msg:message_type instead.")
  (message_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <platoonProtocol>) ostream)
  "Serializes a message object of type '<platoonProtocol>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'payload))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'payload))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <platoonProtocol>) istream)
  "Deserializes a message object of type '<platoonProtocol>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'payload) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'payload) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<platoonProtocol>)))
  "Returns string type for a message object of type '<platoonProtocol>"
  "platooning/platoonProtocol")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'platoonProtocol)))
  "Returns string type for a message object of type 'platoonProtocol"
  "platooning/platoonProtocol")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<platoonProtocol>)))
  "Returns md5sum for a message object of type '<platoonProtocol>"
  "d9bea63af715371a2b1507d7c3fd0da2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'platoonProtocol)))
  "Returns md5sum for a message object of type 'platoonProtocol"
  "d9bea63af715371a2b1507d7c3fd0da2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<platoonProtocol>)))
  "Returns full string definition for message of type '<platoonProtocol>"
  (cl:format cl:nil "string payload~%uint32 message_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'platoonProtocol)))
  "Returns full string definition for message of type 'platoonProtocol"
  (cl:format cl:nil "string payload~%uint32 message_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <platoonProtocol>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'payload))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <platoonProtocol>))
  "Converts a ROS message object to a list"
  (cl:list 'platoonProtocol
    (cl:cons ':payload (payload msg))
    (cl:cons ':message_type (message_type msg))
))
