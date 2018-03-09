; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude templateMsg.msg.html

(cl:defclass <templateMsg> (roslisp-msg-protocol:ros-message)
  ((templatebool
    :reader templatebool
    :initarg :templatebool
    :type cl:boolean
    :initform cl:nil)
   (templatefloat
    :reader templatefloat
    :initarg :templatefloat
    :type cl:float
    :initform 0.0)
   (templatestring
    :reader templatestring
    :initarg :templatestring
    :type cl:string
    :initform ""))
)

(cl:defclass templateMsg (<templateMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <templateMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'templateMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<templateMsg> is deprecated: use platooning-msg:templateMsg instead.")))

(cl:ensure-generic-function 'templatebool-val :lambda-list '(m))
(cl:defmethod templatebool-val ((m <templateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:templatebool-val is deprecated.  Use platooning-msg:templatebool instead.")
  (templatebool m))

(cl:ensure-generic-function 'templatefloat-val :lambda-list '(m))
(cl:defmethod templatefloat-val ((m <templateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:templatefloat-val is deprecated.  Use platooning-msg:templatefloat instead.")
  (templatefloat m))

(cl:ensure-generic-function 'templatestring-val :lambda-list '(m))
(cl:defmethod templatestring-val ((m <templateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:templatestring-val is deprecated.  Use platooning-msg:templatestring instead.")
  (templatestring m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <templateMsg>) ostream)
  "Serializes a message object of type '<templateMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'templatebool) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'templatefloat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'templatestring))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'templatestring))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <templateMsg>) istream)
  "Deserializes a message object of type '<templateMsg>"
    (cl:setf (cl:slot-value msg 'templatebool) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'templatefloat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'templatestring) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'templatestring) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<templateMsg>)))
  "Returns string type for a message object of type '<templateMsg>"
  "platooning/templateMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'templateMsg)))
  "Returns string type for a message object of type 'templateMsg"
  "platooning/templateMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<templateMsg>)))
  "Returns md5sum for a message object of type '<templateMsg>"
  "06f3640a725749b7cc7a9141d075f992")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'templateMsg)))
  "Returns md5sum for a message object of type 'templateMsg"
  "06f3640a725749b7cc7a9141d075f992")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<templateMsg>)))
  "Returns full string definition for message of type '<templateMsg>"
  (cl:format cl:nil "bool templatebool~%float64 templatefloat~%string templatestring~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'templateMsg)))
  "Returns full string definition for message of type 'templateMsg"
  (cl:format cl:nil "bool templatebool~%float64 templatefloat~%string templatestring~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <templateMsg>))
  (cl:+ 0
     1
     8
     4 (cl:length (cl:slot-value msg 'templatestring))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <templateMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'templateMsg
    (cl:cons ':templatebool (templatebool msg))
    (cl:cons ':templatefloat (templatefloat msg))
    (cl:cons ':templatestring (templatestring msg))
))
