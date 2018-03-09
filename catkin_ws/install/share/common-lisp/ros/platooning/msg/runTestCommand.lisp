; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude runTestCommand.msg.html

(cl:defclass <runTestCommand> (roslisp-msg-protocol:ros-message)
  ((testToRun
    :reader testToRun
    :initarg :testToRun
    :type cl:string
    :initform ""))
)

(cl:defclass runTestCommand (<runTestCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <runTestCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'runTestCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<runTestCommand> is deprecated: use platooning-msg:runTestCommand instead.")))

(cl:ensure-generic-function 'testToRun-val :lambda-list '(m))
(cl:defmethod testToRun-val ((m <runTestCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:testToRun-val is deprecated.  Use platooning-msg:testToRun instead.")
  (testToRun m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <runTestCommand>) ostream)
  "Serializes a message object of type '<runTestCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'testToRun))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'testToRun))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <runTestCommand>) istream)
  "Deserializes a message object of type '<runTestCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'testToRun) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'testToRun) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<runTestCommand>)))
  "Returns string type for a message object of type '<runTestCommand>"
  "platooning/runTestCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'runTestCommand)))
  "Returns string type for a message object of type 'runTestCommand"
  "platooning/runTestCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<runTestCommand>)))
  "Returns md5sum for a message object of type '<runTestCommand>"
  "01c8c3a6d2ae4ece1652d4c711eb8664")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'runTestCommand)))
  "Returns md5sum for a message object of type 'runTestCommand"
  "01c8c3a6d2ae4ece1652d4c711eb8664")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<runTestCommand>)))
  "Returns full string definition for message of type '<runTestCommand>"
  (cl:format cl:nil "string testToRun~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'runTestCommand)))
  "Returns full string definition for message of type 'runTestCommand"
  (cl:format cl:nil "string testToRun~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <runTestCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'testToRun))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <runTestCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'runTestCommand
    (cl:cons ':testToRun (testToRun msg))
))
