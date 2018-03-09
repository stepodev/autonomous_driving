; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude userInterface.msg.html

(cl:defclass <userInterface> (roslisp-msg-protocol:ros-message)
  ((leading_vehicle
    :reader leading_vehicle
    :initarg :leading_vehicle
    :type cl:boolean
    :initform cl:nil)
   (following_vehicle
    :reader following_vehicle
    :initarg :following_vehicle
    :type cl:boolean
    :initform cl:nil)
   (potential_following_vehicle
    :reader potential_following_vehicle
    :initarg :potential_following_vehicle
    :type cl:boolean
    :initform cl:nil)
   (inner_platoon_distance
    :reader inner_platoon_distance
    :initarg :inner_platoon_distance
    :type cl:float
    :initform 0.0)
   (actual_distance
    :reader actual_distance
    :initarg :actual_distance
    :type cl:float
    :initform 0.0)
   (platoon_speed
    :reader platoon_speed
    :initarg :platoon_speed
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (platooning_state
    :reader platooning_state
    :initarg :platooning_state
    :type cl:string
    :initform "")
   (src_vehicle
    :reader src_vehicle
    :initarg :src_vehicle
    :type cl:integer
    :initform 0)
   (platoon_size
    :reader platoon_size
    :initarg :platoon_size
    :type cl:integer
    :initform 0)
   (platoon_members
    :reader platoon_members
    :initarg :platoon_members
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (enable_remotecontrol
    :reader enable_remotecontrol
    :initarg :enable_remotecontrol
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass userInterface (<userInterface>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <userInterface>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'userInterface)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<userInterface> is deprecated: use platooning-msg:userInterface instead.")))

(cl:ensure-generic-function 'leading_vehicle-val :lambda-list '(m))
(cl:defmethod leading_vehicle-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:leading_vehicle-val is deprecated.  Use platooning-msg:leading_vehicle instead.")
  (leading_vehicle m))

(cl:ensure-generic-function 'following_vehicle-val :lambda-list '(m))
(cl:defmethod following_vehicle-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:following_vehicle-val is deprecated.  Use platooning-msg:following_vehicle instead.")
  (following_vehicle m))

(cl:ensure-generic-function 'potential_following_vehicle-val :lambda-list '(m))
(cl:defmethod potential_following_vehicle-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:potential_following_vehicle-val is deprecated.  Use platooning-msg:potential_following_vehicle instead.")
  (potential_following_vehicle m))

(cl:ensure-generic-function 'inner_platoon_distance-val :lambda-list '(m))
(cl:defmethod inner_platoon_distance-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:inner_platoon_distance-val is deprecated.  Use platooning-msg:inner_platoon_distance instead.")
  (inner_platoon_distance m))

(cl:ensure-generic-function 'actual_distance-val :lambda-list '(m))
(cl:defmethod actual_distance-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:actual_distance-val is deprecated.  Use platooning-msg:actual_distance instead.")
  (actual_distance m))

(cl:ensure-generic-function 'platoon_speed-val :lambda-list '(m))
(cl:defmethod platoon_speed-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_speed-val is deprecated.  Use platooning-msg:platoon_speed instead.")
  (platoon_speed m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:speed-val is deprecated.  Use platooning-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'platooning_state-val :lambda-list '(m))
(cl:defmethod platooning_state-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platooning_state-val is deprecated.  Use platooning-msg:platooning_state instead.")
  (platooning_state m))

(cl:ensure-generic-function 'src_vehicle-val :lambda-list '(m))
(cl:defmethod src_vehicle-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:src_vehicle-val is deprecated.  Use platooning-msg:src_vehicle instead.")
  (src_vehicle m))

(cl:ensure-generic-function 'platoon_size-val :lambda-list '(m))
(cl:defmethod platoon_size-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_size-val is deprecated.  Use platooning-msg:platoon_size instead.")
  (platoon_size m))

(cl:ensure-generic-function 'platoon_members-val :lambda-list '(m))
(cl:defmethod platoon_members-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:platoon_members-val is deprecated.  Use platooning-msg:platoon_members instead.")
  (platoon_members m))

(cl:ensure-generic-function 'enable_remotecontrol-val :lambda-list '(m))
(cl:defmethod enable_remotecontrol-val ((m <userInterface>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:enable_remotecontrol-val is deprecated.  Use platooning-msg:enable_remotecontrol instead.")
  (enable_remotecontrol m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <userInterface>) ostream)
  "Serializes a message object of type '<userInterface>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'leading_vehicle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'following_vehicle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'potential_following_vehicle) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'inner_platoon_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'actual_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'platoon_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'platooning_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'platooning_state))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_size)) ostream)
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable_remotecontrol) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <userInterface>) istream)
  "Deserializes a message object of type '<userInterface>"
    (cl:setf (cl:slot-value msg 'leading_vehicle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'following_vehicle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'potential_following_vehicle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inner_platoon_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'actual_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'platoon_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'platooning_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'platooning_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'src_vehicle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'platoon_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'platoon_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'platoon_size)) (cl:read-byte istream))
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
    (cl:setf (cl:slot-value msg 'enable_remotecontrol) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<userInterface>)))
  "Returns string type for a message object of type '<userInterface>"
  "platooning/userInterface")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'userInterface)))
  "Returns string type for a message object of type 'userInterface"
  "platooning/userInterface")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<userInterface>)))
  "Returns md5sum for a message object of type '<userInterface>"
  "d1d08fffe38d5dc5a1aad88d74db0b91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'userInterface)))
  "Returns md5sum for a message object of type 'userInterface"
  "d1d08fffe38d5dc5a1aad88d74db0b91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<userInterface>)))
  "Returns full string definition for message of type '<userInterface>"
  (cl:format cl:nil "bool leading_vehicle~%bool following_vehicle~%bool potential_following_vehicle~%float32 inner_platoon_distance~%float32 actual_distance~%float32 platoon_speed~%float32 speed~%string platooning_state~%uint32 src_vehicle~%uint32 platoon_size~%uint32[] platoon_members~%bool enable_remotecontrol~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'userInterface)))
  "Returns full string definition for message of type 'userInterface"
  (cl:format cl:nil "bool leading_vehicle~%bool following_vehicle~%bool potential_following_vehicle~%float32 inner_platoon_distance~%float32 actual_distance~%float32 platoon_speed~%float32 speed~%string platooning_state~%uint32 src_vehicle~%uint32 platoon_size~%uint32[] platoon_members~%bool enable_remotecontrol~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <userInterface>))
  (cl:+ 0
     1
     1
     1
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'platooning_state))
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'platoon_members) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <userInterface>))
  "Converts a ROS message object to a list"
  (cl:list 'userInterface
    (cl:cons ':leading_vehicle (leading_vehicle msg))
    (cl:cons ':following_vehicle (following_vehicle msg))
    (cl:cons ':potential_following_vehicle (potential_following_vehicle msg))
    (cl:cons ':inner_platoon_distance (inner_platoon_distance msg))
    (cl:cons ':actual_distance (actual_distance msg))
    (cl:cons ':platoon_speed (platoon_speed msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':platooning_state (platooning_state msg))
    (cl:cons ':src_vehicle (src_vehicle msg))
    (cl:cons ':platoon_size (platoon_size msg))
    (cl:cons ':platoon_members (platoon_members msg))
    (cl:cons ':enable_remotecontrol (enable_remotecontrol msg))
))
