;; Auto-generated. Do not edit!


(when (boundp 'platooning::platoonProtocol)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'platoonProtocol (find-package "PLATOONING")))
(unless (find-package "PLATOONING::PLATOONPROTOCOL")
  (make-package "PLATOONING::PLATOONPROTOCOL"))

(in-package "ROS")
;;//! \htmlinclude platoonProtocol.msg.html


(defclass platooning::platoonProtocol
  :super ros::object
  :slots (_payload _message_type ))

(defmethod platooning::platoonProtocol
  (:init
   (&key
    ((:payload __payload) "")
    ((:message_type __message_type) 0)
    )
   (send-super :init)
   (setq _payload (string __payload))
   (setq _message_type (round __message_type))
   self)
  (:payload
   (&optional __payload)
   (if __payload (setq _payload __payload)) _payload)
  (:message_type
   (&optional __message_type)
   (if __message_type (setq _message_type __message_type)) _message_type)
  (:serialization-length
   ()
   (+
    ;; string _payload
    4 (length _payload)
    ;; uint32 _message_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _payload
       (write-long (length _payload) s) (princ _payload s)
     ;; uint32 _message_type
       (write-long _message_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _payload
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _payload (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _message_type
     (setq _message_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::platoonProtocol :md5sum-) "d9bea63af715371a2b1507d7c3fd0da2")
(setf (get platooning::platoonProtocol :datatype-) "platooning/platoonProtocol")
(setf (get platooning::platoonProtocol :definition-)
      "string payload
uint32 message_type
")



(provide :platooning/platoonProtocol "d9bea63af715371a2b1507d7c3fd0da2")


