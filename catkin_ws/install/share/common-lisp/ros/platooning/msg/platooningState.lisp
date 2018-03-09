; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude platooningState.msg.html

(cl:defclass <platooningState> (roslisp-msg-protocol:ros-message)
  ((platooning_state
    :reader platooning_state
    :initarg :platooning_state
    :type cl:string
    :initform "")
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
   (vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:integer
    :initform 0)
   (i_am_FV
    :reader i_am_FV
    :initarg :i_am_FV
    :type cl:boolean
    :initform cl:nil)
   (i_am_LV
    :reader i_am_LV
    :initarg :i_am_LV
    :type cl:boolean
    :initform cl:nil)
   (platoon_members
    :reader platoon_members
    :initarg :platoon_members
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass platooningState (<platooningState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <platooningState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'platooningState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<platooningState> is deprecated: use platooning-msg:platooningState instead.")))

(cl:ensure-generic-function 'platooning_state-val :lambda-list '(m))
(cl:defmethod platooning_state-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platooning_state-val is deprecated.  Use platooning-msg:platooning_state instead.")
  (platooning_state m))

(cl:ensure-generic-function 'platoon_id-val :lambda-list '(m))
(cl:defmethod platoon_id-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_id-val is deprecated.  Use platooning-msg:platoon_id instead.")
  (platoon_id m))

(cl:ensure-generic-function 'ipd-val :lambda-list '(m))
(cl:defmethod ipd-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:ipd-val is deprecated.  Use platooning-msg:ipd instead.")
  (ipd m))

(cl:ensure-generic-function 'ps-val :lambda-list '(m))
(cl:defmethod ps-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:ps-val is deprecated.  Use platooning-msg:ps instead.")
  (ps m))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:vehicle_id-val is deprecated.  Use platooning-msg:vehicle_id instead.")
  (vehicle_id m))

(cl:ensure-generic-function 'i_am_FV-val :lambda-list '(m))
(cl:defmethod i_am_FV-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:i_am_FV-val is deprecated.  Use platooning-msg:i_am_FV instead.")
  (i_am_FV m))

(cl:ensure-generic-function 'i_am_LV-val :lambda-list '(m))
(cl:defmethod i_am_LV-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:i_am_LV-val is deprecated.  Use platooning-msg:i_am_LV instead.")
  (i_am_LV m))

(cl:ensure-generic-function 'platoon_members-val :lambda-list '(m))
(cl:defmethod platoon_members-val ((m <platooningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_members-val is deprecated.  Use platooning-msg:platoon_members instead.")
  (platoon_members m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <platooningState>) ostream)
  "Serializes a message object of type '<platooningState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'platooning_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'platooning_state))
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'i_am_FV) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'i_am_LV) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'platoon_members))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'platoon_members))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <platooningState>) istream)
  "Deserializes a message object of type '<platooningState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'platooning_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'platooning_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'i_am_FV) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'i_am_LV) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'platoon_members) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'platoon_members)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<platooningState>)))
  "Returns string type for a message object of type '<platooningState>"
  "platooning/platooningState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'platooningState)))
  "Returns string type for a message object of type 'platooningState"
  "platooning/platooningState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<platooningState>)))
  "Returns md5sum for a message object of type '<platooningState>"
  "513c3389662a56c9d5dc40bcbdf23890")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'platooningState)))
  "Returns md5sum for a message object of type 'platooningState"
  "513c3389662a56c9d5dc40bcbdf23890")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<platooningState>)))
  "Returns full string definition for message of type '<platooningState>"
  (cl:format cl:nil "string platooning_state~%uint32 platoon_id~%float32 ipd~%float32 ps~%uint32 vehicle_id~%bool i_am_FV~%bool i_am_LV~%uint32[] platoon_members~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'platooningState)))
  "Returns full string definition for message of type 'platooningState"
  (cl:format cl:nil "string platooning_state~%uint32 platoon_id~%float32 ipd~%float32 ps~%uint32 vehicle_id~%bool i_am_FV~%bool i_am_LV~%uint32[] platoon_members~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <platooningState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'platooning_state))
     4
     4
     4
     4
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'platoon_members) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <platooningState>))
  "Converts a ROS message object to a list"
  (cl:list 'platooningState
    (cl:cons ':platooning_state (platooning_state msg))
    (cl:cons ':platoon_id (platoon_id msg))
    (cl:cons ':ipd (ipd msg))
    (cl:cons ':ps (ps msg))
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':i_am_FV (i_am_FV msg))
    (cl:cons ':i_am_LV (i_am_LV msg))
    (cl:cons ':platoon_members (platoon_members msg))
))
