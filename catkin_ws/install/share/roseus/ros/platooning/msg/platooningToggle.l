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
  :slots (_vehicle_id _enable_platooning _inner_platoon_distance _platoon_speed _lvfv ))

(defmethod platooning::platooningToggle
  (:init
   (&key
    ((:vehicle_id __vehicle_id) 0)
    ((:enable_platooning __enable_platooning) nil)
    ((:inner_platoon_distance __inner_platoon_distance) 0.0)
    ((:platoon_speed __platoon_speed) 0.0)
    ((:lvfv __lvfv) "")
    )
   (send-super :init)
   (setq _vehicle_id (round __vehicle_id))
   (setq _enable_platooning __enable_platooning)
   (setq _inner_platoon_distance (float __inner_platoon_distance))
   (setq _platoon_speed (float __platoon_speed))
   (setq _lvfv (string __lvfv))
   self)
  (:vehicle_id
   (&optional __vehicle_id)
   (if __vehicle_id (setq _vehicle_id __vehicle_id)) _vehicle_id)
  (:enable_platooning
   (&optional __enable_platooning)
   (if __enable_platooning (setq _enable_platooning __enable_platooning)) _enable_platooning)
  (:inner_platoon_distance
   (&optional __inner_platoon_distance)
   (if __inner_platoon_distance (setq _inner_platoon_distance __inner_platoon_distance)) _inner_platoon_distance)
  (:platoon_speed
   (&optional __platoon_speed)
   (if __platoon_speed (setq _platoon_speed __platoon_speed)) _platoon_speed)
  (:lvfv
   (&optional __lvfv)
   (if __lvfv (setq _lvfv __lvfv)) _lvfv)
  (:serialization-length
   ()
   (+
    ;; uint32 _vehicle_id
    4
    ;; bool _enable_platooning
    1
    ;; float32 _inner_platoon_distance
    4
    ;; float32 _platoon_speed
    4
    ;; string _lvfv
    4 (length _lvfv)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _vehicle_id
       (write-long _vehicle_id s)
     ;; bool _enable_platooning
       (if _enable_platooning (write-byte -1 s) (write-byte 0 s))
     ;; float32 _inner_platoon_distance
       (sys::poke _inner_platoon_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _platoon_speed
       (sys::poke _platoon_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _lvfv
       (write-long (length _lvfv) s) (princ _lvfv s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _vehicle_id
     (setq _vehicle_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _enable_platooning
     (setq _enable_platooning (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _inner_platoon_distance
     (setq _inner_platoon_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _platoon_speed
     (setq _platoon_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _lvfv
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lvfv (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get platooning::platooningToggle :md5sum-) "0b95d94536aba6d55c50abe7d1619f57")
(setf (get platooning::platooningToggle :datatype-) "platooning/platooningToggle")
(setf (get platooning::platooningToggle :definition-)
      "uint32 vehicle_id
bool enable_platooning
float32 inner_platoon_distance
float32 platoon_speed
string lvfv
")



(provide :platooning/platooningToggle "0b95d94536aba6d55c50abe7d1619f57")


