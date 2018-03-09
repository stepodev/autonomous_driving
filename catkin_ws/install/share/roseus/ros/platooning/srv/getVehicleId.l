;; Auto-generated. Do not edit!


(when (boundp 'platooning::getVehicleId)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'getVehicleId (find-package "PLATOONING")))
(unless (find-package "PLATOONING::GETVEHICLEID")
  (make-package "PLATOONING::GETVEHICLEID"))
(unless (find-package "PLATOONING::GETVEHICLEIDREQUEST")
  (make-package "PLATOONING::GETVEHICLEIDREQUEST"))
(unless (find-package "PLATOONING::GETVEHICLEIDRESPONSE")
  (make-package "PLATOONING::GETVEHICLEIDRESPONSE"))

(in-package "ROS")





(defclass platooning::getVehicleIdRequest
  :super ros::object
  :slots ())

(defmethod platooning::getVehicleIdRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass platooning::getVehicleIdResponse
  :super ros::object
  :slots (_vehicle_id ))

(defmethod platooning::getVehicleIdResponse
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

(defclass platooning::getVehicleId
  :super ros::object
  :slots ())

(setf (get platooning::getVehicleId :md5sum-) "5c96f4b8297034815b56c1d26627cd99")
(setf (get platooning::getVehicleId :datatype-) "platooning/getVehicleId")
(setf (get platooning::getVehicleId :request) platooning::getVehicleIdRequest)
(setf (get platooning::getVehicleId :response) platooning::getVehicleIdResponse)

(defmethod platooning::getVehicleIdRequest
  (:response () (instance platooning::getVehicleIdResponse :init)))

(setf (get platooning::getVehicleIdRequest :md5sum-) "5c96f4b8297034815b56c1d26627cd99")
(setf (get platooning::getVehicleIdRequest :datatype-) "platooning/getVehicleIdRequest")
(setf (get platooning::getVehicleIdRequest :definition-)
      "
---
uint32 vehicle_id
")

(setf (get platooning::getVehicleIdResponse :md5sum-) "5c96f4b8297034815b56c1d26627cd99")
(setf (get platooning::getVehicleIdResponse :datatype-) "platooning/getVehicleIdResponse")
(setf (get platooning::getVehicleIdResponse :definition-)
      "
---
uint32 vehicle_id
")



(provide :platooning/getVehicleId "5c96f4b8297034815b56c1d26627cd99")


