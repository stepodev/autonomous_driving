;; Auto-generated. Do not edit!


(when (boundp 'platooning::oughtData)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'oughtData (find-package "PLATOONING")))
(unless (find-package "PLATOONING::OUGHTDATA")
  (make-package "PLATOONING::OUGHTDATA"))

(in-package "ROS")
;;//! \htmlinclude oughtData.msg.html


(defclass platooning::oughtData
  :super ros::object
  :slots (_distance _speed _steeringAngle ))

(defmethod platooning::oughtData
  (:init
   (&key
    ((:distance __distance) 0.0)
    ((:speed __speed) 0.0)
    ((:steeringAngle __steeringAngle) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   (setq _speed (float __speed))
   (setq _steeringAngle (float __steeringAngle))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:steeringAngle
   (&optional __steeringAngle)
   (if __steeringAngle (setq _steeringAngle __steeringAngle)) _steeringAngle)
  (:serialization-length
   ()
   (+
    ;; float32 _distance
    4
    ;; float32 _speed
    4
    ;; float32 _steeringAngle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steeringAngle
       (sys::poke _steeringAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steeringAngle
     (setq _steeringAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::oughtData :md5sum-) "5d11ac8cfd9baa9abe3a207005a675b7")
(setf (get platooning::oughtData :datatype-) "platooning/oughtData")
(setf (get platooning::oughtData :definition-)
      "float32 distance
float32 speed
float32 steeringAngle
")



(provide :platooning/oughtData "5d11ac8cfd9baa9abe3a207005a675b7")


