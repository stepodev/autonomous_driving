;; Auto-generated. Do not edit!


(when (boundp 'platooning::speed)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'speed (find-package "PLATOONING")))
(unless (find-package "PLATOONING::SPEED")
  (make-package "PLATOONING::SPEED"))

(in-package "ROS")
;;//! \htmlinclude speed.msg.html


(defclass platooning::speed
  :super ros::object
  :slots (_speed ))

(defmethod platooning::speed
  (:init
   (&key
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::speed :md5sum-) "ca65bba734a79b4a6707341d829f4d5c")
(setf (get platooning::speed :datatype-) "platooning/speed")
(setf (get platooning::speed :definition-)
      "float32 speed
")



(provide :platooning/speed "ca65bba734a79b4a6707341d829f4d5c")


