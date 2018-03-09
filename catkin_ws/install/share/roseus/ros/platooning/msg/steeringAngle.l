;; Auto-generated. Do not edit!


(when (boundp 'platooning::steeringAngle)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'steeringAngle (find-package "PLATOONING")))
(unless (find-package "PLATOONING::STEERINGANGLE")
  (make-package "PLATOONING::STEERINGANGLE"))

(in-package "ROS")
;;//! \htmlinclude steeringAngle.msg.html


(defclass platooning::steeringAngle
  :super ros::object
  :slots (_steering_angle ))

(defmethod platooning::steeringAngle
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
    ;; float64 _steering_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get platooning::steeringAngle :md5sum-) "0a70504669a8dacda06aa18fb270817f")
(setf (get platooning::steeringAngle :datatype-) "platooning/steeringAngle")
(setf (get platooning::steeringAngle :definition-)
      "float64 steering_angle

")



(provide :platooning/steeringAngle "0a70504669a8dacda06aa18fb270817f")


