;; Auto-generated. Do not edit!


(when (boundp 'platooning::fv_request)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'fv_request (find-package "PLATOONING")))
(unless (find-package "PLATOONING::FV_REQUEST")
  (make-package "PLATOONING::FV_REQUEST"))

(in-package "ROS")
;;//! \htmlinclude fv_request.msg.html


(defclass platooning::fv_request
  :super ros::object
  :slots (_src_vehicle ))

(defmethod platooning::fv_request
  (:init
   (&key
    ((:src_vehicle __src_vehicle) 0)
    )
   (send-super :init)
   (setq _src_vehicle (round __src_vehicle))
   self)
  (:src_vehicle
   (&optional __src_vehicle)
   (if __src_vehicle (setq _src_vehicle __src_vehicle)) _src_vehicle)
  (:serialization-length
   ()
   (+
    ;; uint32 _src_vehicle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _src_vehicle
       (write-long _src_vehicle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _src_vehicle
     (setq _src_vehicle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::fv_request :md5sum-) "07b1b45a74ecd2446be6ced3738fb3a9")
(setf (get platooning::fv_request :datatype-) "platooning/fv_request")
(setf (get platooning::fv_request :definition-)
      "uint32 src_vehicle
")



(provide :platooning/fv_request "07b1b45a74ecd2446be6ced3738fb3a9")


