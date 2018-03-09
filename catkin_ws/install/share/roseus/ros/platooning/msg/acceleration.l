;; Auto-generated. Do not edit!


(when (boundp 'platooning::acceleration)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'acceleration (find-package "PLATOONING")))
(unless (find-package "PLATOONING::ACCELERATION")
  (make-package "PLATOONING::ACCELERATION"))

(in-package "ROS")
;;//! \htmlinclude acceleration.msg.html


(defclass platooning::acceleration
  :super ros::object
  :slots (_accelleration ))

(defmethod platooning::acceleration
  (:init
   (&key
    ((:accelleration __accelleration) 0.0)
    )
   (send-super :init)
   (setq _accelleration (float __accelleration))
   self)
  (:accelleration
   (&optional __accelleration)
   (if __accelleration (setq _accelleration __accelleration)) _accelleration)
  (:serialization-length
   ()
   (+
    ;; float32 _accelleration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _accelleration
       (sys::poke _accelleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _accelleration
     (setq _accelleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::acceleration :md5sum-) "5cb03383b65207f8a255b5fa95fedef3")
(setf (get platooning::acceleration :datatype-) "platooning/acceleration")
(setf (get platooning::acceleration :definition-)
      "float32 accelleration
")



(provide :platooning/acceleration "5cb03383b65207f8a255b5fa95fedef3")


