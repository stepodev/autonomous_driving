;; Auto-generated. Do not edit!


(when (boundp 'platooning::templateMsg)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'templateMsg (find-package "PLATOONING")))
(unless (find-package "PLATOONING::TEMPLATEMSG")
  (make-package "PLATOONING::TEMPLATEMSG"))

(in-package "ROS")
;;//! \htmlinclude templateMsg.msg.html


(defclass platooning::templateMsg
  :super ros::object
  :slots (_templatebool _templatefloat _templatestring ))

(defmethod platooning::templateMsg
  (:init
   (&key
    ((:templatebool __templatebool) nil)
    ((:templatefloat __templatefloat) 0.0)
    ((:templatestring __templatestring) "")
    )
   (send-super :init)
   (setq _templatebool __templatebool)
   (setq _templatefloat (float __templatefloat))
   (setq _templatestring (string __templatestring))
   self)
  (:templatebool
   (&optional __templatebool)
   (if __templatebool (setq _templatebool __templatebool)) _templatebool)
  (:templatefloat
   (&optional __templatefloat)
   (if __templatefloat (setq _templatefloat __templatefloat)) _templatefloat)
  (:templatestring
   (&optional __templatestring)
   (if __templatestring (setq _templatestring __templatestring)) _templatestring)
  (:serialization-length
   ()
   (+
    ;; bool _templatebool
    1
    ;; float64 _templatefloat
    8
    ;; string _templatestring
    4 (length _templatestring)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _templatebool
       (if _templatebool (write-byte -1 s) (write-byte 0 s))
     ;; float64 _templatefloat
       (sys::poke _templatefloat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _templatestring
       (write-long (length _templatestring) s) (princ _templatestring s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _templatebool
     (setq _templatebool (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _templatefloat
     (setq _templatefloat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _templatestring
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _templatestring (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get platooning::templateMsg :md5sum-) "06f3640a725749b7cc7a9141d075f992")
(setf (get platooning::templateMsg :datatype-) "platooning/templateMsg")
(setf (get platooning::templateMsg :definition-)
      "bool templatebool
float64 templatefloat
string templatestring
")



(provide :platooning/templateMsg "06f3640a725749b7cc7a9141d075f992")


