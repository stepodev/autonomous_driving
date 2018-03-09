; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude testResult.msg.html

(cl:defclass <testResult> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (comment
    :reader comment
    :initarg :comment
    :type cl:string
    :initform ""))
)

(cl:defclass testResult (<testResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <testResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'testResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<testResult> is deprecated: use platooning-msg:testResult instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <testResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:success-val is deprecated.  Use platooning-msg:success instead.")
  (success m))

(cl:ensure-generic-function 'comment-val :lambda-list '(m))
(cl:defmethod comment-val ((m <testResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:comment-val is deprecated.  Use platooning-msg:comment instead.")
  (comment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <testResult>) ostream)
  "Serializes a message object of type '<testResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'comment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'comment))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <testResult>) istream)
  "Deserializes a message object of type '<testResult>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'comment) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'comment) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<testResult>)))
  "Returns string type for a message object of type '<testResult>"
  "platooning/testResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'testResult)))
  "Returns string type for a message object of type 'testResult"
  "platooning/testResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<testResult>)))
  "Returns md5sum for a message object of type '<testResult>"
  "1e664c0b6b9801e28898f96d696eb445")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'testResult)))
  "Returns md5sum for a message object of type 'testResult"
  "1e664c0b6b9801e28898f96d696eb445")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<testResult>)))
  "Returns full string definition for message of type '<testResult>"
  (cl:format cl:nil "bool success~%string comment~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'testResult)))
  "Returns full string definition for message of type 'testResult"
  (cl:format cl:nil "bool success~%string comment~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <testResult>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'comment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <testResult>))
  "Converts a ROS message object to a list"
  (cl:list 'testResult
    (cl:cons ':success (success msg))
    (cl:cons ':comment (comment msg))
))
