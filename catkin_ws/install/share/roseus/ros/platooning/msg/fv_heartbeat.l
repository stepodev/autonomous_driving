;; Auto-generated. Do not edit!


(when (boundp 'platooning::fv_heartbeat)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'fv_heartbeat (find-package "PLATOONING")))
(unless (find-package "PLATOONING::FV_HEARTBEAT")
  (make-package "PLATOONING::FV_HEARTBEAT"))

(in-package "ROS")
;;//! \htmlinclude fv_heartbeat.msg.html


(defclass platooning::fv_heartbeat
  :super ros::object
  :slots (_src_vehicle _platoon_id ))

(defmethod platooning::fv_heartbeat
  (:init
   (&key
    ((:src_vehicle __src_vehicle) 0)
    ((:platoon_id __platoon_id) 0)
    )
   (send-super :init)
   (setq _src_vehicle (round __src_vehicle))
   (setq _platoon_id (round __platoon_id))
   self)
  (:src_vehicle
   (&optional __src_vehicle)
   (if __src_vehicle (setq _src_vehicle __src_vehicle)) _src_vehicle)
  (:platoon_id
   (&optional __platoon_id)
   (if __platoon_id (setq _platoon_id __platoon_id)) _platoon_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _src_vehicle
    4
    ;; uint32 _platoon_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _src_vehicle
       (write-long _src_vehicle s)
     ;; uint32 _platoon_id
       (write-long _platoon_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _src_vehicle
     (setq _src_vehicle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _platoon_id
     (setq _platoon_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::fv_heartbeat :md5sum-) "4641cd1aacd2a8c9fb2185d9e7b6b3df")
(setf (get platooning::fv_heartbeat :datatype-) "platooning/fv_heartbeat")
(setf (get platooning::fv_heartbeat :definition-)
      "uint32 src_vehicle
uint32 platoon_id
")



(provide :platooning/fv_heartbeat "4641cd1aacd2a8c9fb2185d9e7b6b3df")


