; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude camera.msg.html

(cl:defclass <camera> (roslisp-msg-protocol:ros-message)
  ((pic
    :reader pic
    :initarg :pic
    :type cl:string
    :initform ""))
)

(cl:defclass camera (<camera>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <camera>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'camera)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<camera> is deprecated: use platooning-msg:camera instead.")))

(cl:ensure-generic-function 'pic-val :lambda-list '(m))
(cl:defmethod pic-val ((m <camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:pic-val is deprecated.  Use platooning-msg:pic instead.")
  (pic m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <camera>) ostream)
  "Serializes a message object of type '<camera>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pic))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <camera>) istream)
  "Deserializes a message object of type '<camera>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<camera>)))
  "Returns string type for a message object of type '<camera>"
  "platooning/camera")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'camera)))
  "Returns string type for a message object of type 'camera"
  "platooning/camera")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<camera>)))
  "Returns md5sum for a message object of type '<camera>"
  "b029feab08f2c4877092228da5ee6702")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'camera)))
  "Returns md5sum for a message object of type 'camera"
  "b029feab08f2c4877092228da5ee6702")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<camera>)))
  "Returns full string definition for message of type '<camera>"
  (cl:format cl:nil "string pic~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'camera)))
  "Returns full string definition for message of type 'camera"
  (cl:format cl:nil "string pic~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <camera>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'pic))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <camera>))
  "Converts a ROS message object to a list"
  (cl:list 'camera
    (cl:cons ':pic (pic msg))
))
