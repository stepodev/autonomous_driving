;; Auto-generated. Do not edit!


(when (boundp 'platooning::targetSpeed)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'targetSpeed (find-package "PLATOONING")))
(unless (find-package "PLATOONING::TARGETSPEED")
  (make-package "PLATOONING::TARGETSPEED"))

(in-package "ROS")
;;//! \htmlinclude targetSpeed.msg.html


(defclass platooning::targetSpeed
  :super ros::object
  :slots (_target_speed ))

(defmethod platooning::targetSpeed
  (:init
   (&key
    ((:target_speed __target_speed) 0.0)
    )
   (send-super :init)
   (setq _target_speed (float __target_speed))
   self)
  (:target_speed
   (&optional __target_speed)
   (if __target_speed (setq _target_speed __target_speed)) _target_speed)
  (:serialization-length
   ()
   (+
    ;; float32 _target_speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _target_speed
       (sys::poke _target_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _target_speed
     (setq _target_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::targetSpeed :md5sum-) "a0e65cab2c5b59829b64f16ffc712119")
(setf (get platooning::targetSpeed :datatype-) "platooning/targetSpeed")
(setf (get platooning::targetSpeed :definition-)
      "float32 target_speed
")



(provide :platooning/targetSpeed "a0e65cab2c5b59829b64f16ffc712119")


