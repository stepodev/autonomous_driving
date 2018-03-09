;; Auto-generated. Do not edit!


(when (boundp 'platooning::lv_broadcast)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'lv_broadcast (find-package "PLATOONING")))
(unless (find-package "PLATOONING::LV_BROADCAST")
  (make-package "PLATOONING::LV_BROADCAST"))

(in-package "ROS")
;;//! \htmlinclude lv_broadcast.msg.html


(defclass platooning::lv_broadcast
  :super ros::object
  :slots (_src_vehicle _platoon_id _ipd _ps _followers ))

(defmethod platooning::lv_broadcast
  (:init
   (&key
    ((:src_vehicle __src_vehicle) 0)
    ((:platoon_id __platoon_id) 0)
    ((:ipd __ipd) 0.0)
    ((:ps __ps) 0.0)
    ((:followers __followers) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _src_vehicle (round __src_vehicle))
   (setq _platoon_id (round __platoon_id))
   (setq _ipd (float __ipd))
   (setq _ps (float __ps))
   (setq _followers __followers)
   self)
  (:src_vehicle
   (&optional __src_vehicle)
   (if __src_vehicle (setq _src_vehicle __src_vehicle)) _src_vehicle)
  (:platoon_id
   (&optional __platoon_id)
   (if __platoon_id (setq _platoon_id __platoon_id)) _platoon_id)
  (:ipd
   (&optional __ipd)
   (if __ipd (setq _ipd __ipd)) _ipd)
  (:ps
   (&optional __ps)
   (if __ps (setq _ps __ps)) _ps)
  (:followers
   (&optional __followers)
   (if __followers (setq _followers __followers)) _followers)
  (:serialization-length
   ()
   (+
    ;; uint32 _src_vehicle
    4
    ;; uint32 _platoon_id
    4
    ;; float32 _ipd
    4
    ;; float32 _ps
    4
    ;; uint32[] _followers
    (* 4    (length _followers)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _src_vehicle
       (write-long _src_vehicle s)
     ;; uint32 _platoon_id
       (write-long _platoon_id s)
     ;; float32 _ipd
       (sys::poke _ipd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ps
       (sys::poke _ps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32[] _followers
     (write-long (length _followers) s)
     (dotimes (i (length _followers))
       (write-long (elt _followers i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _src_vehicle
     (setq _src_vehicle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _platoon_id
     (setq _platoon_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _ipd
     (setq _ipd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ps
     (setq _ps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32[] _followers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _followers (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _followers i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get platooning::lv_broadcast :md5sum-) "a181b9158cc00e160e1fda722b669eac")
(setf (get platooning::lv_broadcast :datatype-) "platooning/lv_broadcast")
(setf (get platooning::lv_broadcast :definition-)
      "uint32 src_vehicle
uint32 platoon_id
float32 ipd
float32 ps
uint32[] followers
")



(provide :platooning/lv_broadcast "a181b9158cc00e160e1fda722b669eac")


