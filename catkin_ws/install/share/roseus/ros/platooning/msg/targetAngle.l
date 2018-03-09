;; Auto-generated. Do not edit!


(when (boundp 'platooning::targetAngle)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'targetAngle (find-package "PLATOONING")))
(unless (find-package "PLATOONING::TARGETANGLE")
  (make-package "PLATOONING::TARGETANGLE"))

(in-package "ROS")
;;//! \htmlinclude targetAngle.msg.html


(defclass platooning::targetAngle
  :super ros::object
  :slots (_steering_angle ))

(defmethod platooning::targetAngle
  (:init
   (&key
    ((:steering_angle __steering_angle) 0.0)
    )
   (send-super :init)
   (setq _steering_angle (float __steering_angle))
   self)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:serialization-length
   ()
   (+
    ;; float32 _steering_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::targetAngle :md5sum-) "3de175f327000a99f382f084344450d4")
(setf (get platooning::targetAngle :datatype-) "platooning/targetAngle")
(setf (get platooning::targetAngle :definition-)
      "float32 steering_angle
")



(provide :platooning/targetAngle "3de175f327000a99f382f084344450d4")


