;; Auto-generated. Do not edit!


(when (boundp 'platooning::lv_reject)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'lv_reject (find-package "PLATOONING")))
(unless (find-package "PLATOONING::LV_REJECT")
  (make-package "PLATOONING::LV_REJECT"))

(in-package "ROS")
;;//! \htmlinclude lv_reject.msg.html


(defclass platooning::lv_reject
  :super ros::object
  :slots (_src_vehicle _dst_vehicle _platoon_id ))

(defmethod platooning::lv_reject
  (:init
   (&key
    ((:src_vehicle __src_vehicle) 0)
    ((:dst_vehicle __dst_vehicle) 0)
    ((:platoon_id __platoon_id) 0)
    )
   (send-super :init)
   (setq _src_vehicle (round __src_vehicle))
   (setq _dst_vehicle (round __dst_vehicle))
   (setq _platoon_id (round __platoon_id))
   self)
  (:src_vehicle
   (&optional __src_vehicle)
   (if __src_vehicle (setq _src_vehicle __src_vehicle)) _src_vehicle)
  (:dst_vehicle
   (&optional __dst_vehicle)
   (if __dst_vehicle (setq _dst_vehicle __dst_vehicle)) _dst_vehicle)
  (:platoon_id
   (&optional __platoon_id)
   (if __platoon_id (setq _platoon_id __platoon_id)) _platoon_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _src_vehicle
    4
    ;; uint32 _dst_vehicle
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
     ;; uint32 _dst_vehicle
       (write-long _dst_vehicle s)
     ;; uint32 _platoon_id
       (write-long _platoon_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _src_vehicle
     (setq _src_vehicle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _dst_vehicle
     (setq _dst_vehicle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _platoon_id
     (setq _platoon_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::lv_reject :md5sum-) "171c710bbe681f26bc25a8cb194a204b")
(setf (get platooning::lv_reject :datatype-) "platooning/lv_reject")
(setf (get platooning::lv_reject :definition-)
      "uint32 src_vehicle
uint32 dst_vehicle
uint32 platoon_id
")



(provide :platooning/lv_reject "171c710bbe681f26bc25a8cb194a204b")


