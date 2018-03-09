; Auto-generated. Do not edit!


(cl:in-package platooning-msg)


;//! \htmlinclude distanceToObj.msg.html

(cl:defclass <distanceToObj> (roslisp-msg-protocol:ros-message)
  ((distance_to_obj
    :reader distance_to_obj
    :initarg :distance_to_obj
    :type cl:float
    :initform 0.0))
)

(cl:defclass distanceToObj (<distanceToObj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <distanceToObj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'distanceToObj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platooning-msg:<distanceToObj> is deprecated: use platooning-msg:distanceToObj instead.")))

(cl:ensure-generic-function 'distance_to_obj-val :lambda-list '(m))
(cl:defmethod distance_to_obj-val ((m <distanceToObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platooning-msg:distance_to_obj-val is deprecated.  Use platooning-msg:distance_to_obj instead.")
  (distance_to_obj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <distanceToObj>) ostream)
  "Serializes a message object of type '<distanceToObj>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_to_obj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <distanceToObj>) istream)
  "Deserializes a message object of type '<distanceToObj>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_obj) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<distanceToObj>)))
  "Returns string type for a message object of type '<distanceToObj>"
  "platooning/distanceToObj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'distanceToObj)))
  "Returns string type for a message object of type 'distanceToObj"
  "platooning/distanceToObj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<distanceToObj>)))
  "Returns md5sum for a message object of type '<distanceToObj>"
  "b94f75528a81496ec1f35e38db49c1c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'distanceToObj)))
  "Returns md5sum for a message object of type 'distanceToObj"
  "b94f75528a81496ec1f35e38db49c1c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<distanceToObj>)))
  "Returns full string definition for message of type '<distanceToObj>"
  (cl:format cl:nil "float32 distance_to_obj~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'distanceToObj)))
  "Returns full string definition for message of type 'distanceToObj"
  (cl:format cl:nil "float32 distance_to_obj~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <distanceToObj>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <distanceToObj>))
  "Converts a ROS message object to a list"
  (cl:list 'distanceToObj
    (cl:cons ':distance_to_obj (distance_to_obj msg))
))
