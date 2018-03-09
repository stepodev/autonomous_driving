; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude registerTestcases.msg.html

(cl:defclass <registerTestcases> (roslisp-msg-protocol:ros-message)
  ((testcase
    :reader testcase
    :initarg :testcase
    :type cl:string
    :initform ""))
)

(cl:defclass registerTestcases (<registerTestcases>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <registerTestcases>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'registerTestcases)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<registerTestcases> is deprecated: use platooning-msg:registerTestcases instead.")))

(cl:ensure-generic-function 'testcase-val :lambda-list '(m))
(cl:defmethod testcase-val ((m <registerTestcases>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:testcase-val is deprecated.  Use platooning-msg:testcase instead.")
  (testcase m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <registerTestcases>) ostream)
  "Serializes a message object of type '<registerTestcases>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'testcase))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'testcase))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <registerTestcases>) istream)
  "Deserializes a message object of type '<registerTestcases>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'testcase) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'testcase) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<registerTestcases>)))
  "Returns string type for a message object of type '<registerTestcases>"
  "platooning/registerTestcases")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'registerTestcases)))
  "Returns string type for a message object of type 'registerTestcases"
  "platooning/registerTestcases")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<registerTestcases>)))
  "Returns md5sum for a message object of type '<registerTestcases>"
  "f1ec9035577ebb432d9345a861e195f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'registerTestcases)))
  "Returns md5sum for a message object of type 'registerTestcases"
  "f1ec9035577ebb432d9345a861e195f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<registerTestcases>)))
  "Returns full string definition for message of type '<registerTestcases>"
  (cl:format cl:nil "string testcase~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'registerTestcases)))
  "Returns full string definition for message of type 'registerTestcases"
  (cl:format cl:nil "string testcase~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <registerTestcases>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'testcase))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <registerTestcases>))
  "Converts a ROS message object to a list"
  (cl:list 'registerTestcases
    (cl:cons ':testcase (testcase msg))
))
