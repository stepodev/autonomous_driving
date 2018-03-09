;; Auto-generated. Do not edit!


(when (boundp 'platooning::vehicleId)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'vehicleId (find-package "PLATOONING")))
(unless (find-package "PLATOONING::VEHICLEID")
  (make-package "PLATOONING::VEHICLEID"))

(in-package "ROS")
;;//! \htmlinclude vehicleId.msg.html


(defclass platooning::vehicleId
  :super ros::object
  :slots (_vehicle_id ))

(defmethod platooning::vehicleId
  (:init
   (&key
    ((:vehicle_id __vehicle_id) 0)
    )
   (send-super :init)
   (setq _vehicle_id (round __vehicle_id))
   self)
  (:vehicle_id
   (&optional __vehicle_id)
   (if __vehicle_id (setq _vehicle_id __vehicle_id)) _vehicle_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _vehicle_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _vehicle_id
       (write-long _vehicle_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _vehicle_id
     (setq _vehicle_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::vehicleId :md5sum-) "5c96f4b8297034815b56c1d26627cd99")
(setf (get platooning::vehicleId :datatype-) "platooning/vehicleId")
(setf (get platooning::vehicleId :definition-)
      "uint32 vehicle_id
")



(provide :platooning/vehicleId "5c96f4b8297034815b56c1d26627cd99")


