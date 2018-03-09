;; Auto-generated. Do not edit!


(when (boundp 'platooning::platooningToggle)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'platooningToggle (find-package "PLATOONING")))
(unless (find-package "PLATOONING::PLATOONINGTOGGLE")
  (make-package "PLATOONING::PLATOONINGTOGGLE"))

(in-package "ROS")
;;//! \htmlinclude platooningToggle.msg.html


(defclass platooning::platooningToggle
  :super ros::object
  :slots (_enable_platooning _inner_platoon_distance _platoon_speed ))

(defmethod platooning::platooningToggle
  (:init
   (&key
    ((:enable_platooning __enable_platooning) nil)
    ((:inner_platoon_distance __inner_platoon_distance) 0.0)
    ((:platoon_speed __platoon_speed) 0.0)
    )
   (send-super :init)
   (setq _enable_platooning __enable_platooning)
   (setq _inner_platoon_distance (float __inner_platoon_distance))
   (setq _platoon_speed (float __platoon_speed))
   self)
  (:enable_platooning
   (&optional __enable_platooning)
   (if __enable_platooning (setq _enable_platooning __enable_platooning)) _enable_platooning)
  (:inner_platoon_distance
   (&optional __inner_platoon_distance)
   (if __inner_platoon_distance (setq _inner_platoon_distance __inner_platoon_distance)) _inner_platoon_distance)
  (:platoon_speed
   (&optional __platoon_speed)
   (if __platoon_speed (setq _platoon_speed __platoon_speed)) _platoon_speed)
  (:serialization-length
   ()
   (+
    ;; bool _enable_platooning
    1
    ;; float32 _inner_platoon_distance
    4
    ;; float32 _platoon_speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable_platooning
       (if _enable_platooning (write-byte -1 s) (write-byte 0 s))
     ;; float32 _inner_platoon_distance
       (sys::poke _inner_platoon_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _platoon_speed
       (sys::poke _platoon_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable_platooning
     (setq _enable_platooning (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _inner_platoon_distance
     (setq _inner_platoon_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _platoon_speed
     (setq _platoon_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::platooningToggle :md5sum-) "d873b59209a1794389a9e491f37fe75f")
(setf (get platooning::platooningToggle :datatype-) "platooning/platooningToggle")
(setf (get platooning::platooningToggle :definition-)
      "bool enable_platooning
float32 inner_platoon_distance
float32 platoon_speed
")



(provide :platooning/platooningToggle "d873b59209a1794389a9e491f37fe75f")


