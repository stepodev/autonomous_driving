;; Auto-generated. Do not edit!


(when (boundp 'platooning::stmupdate)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'stmupdate (find-package "PLATOONING")))
(unless (find-package "PLATOONING::STMUPDATE")
  (make-package "PLATOONING::STMUPDATE"))

(in-package "ROS")
;;//! \htmlinclude stmupdate.msg.html


(defclass platooning::stmupdate
  :super ros::object
  :slots (_id _acceleration _steeringAngle ))

(defmethod platooning::stmupdate
  (:init
   (&key
    ((:id __id) 0)
    ((:acceleration __acceleration) 0.0)
    ((:steeringAngle __steeringAngle) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _acceleration (float __acceleration))
   (setq _steeringAngle (float __steeringAngle))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:steeringAngle
   (&optional __steeringAngle)
   (if __steeringAngle (setq _steeringAngle __steeringAngle)) _steeringAngle)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; float32 _acceleration
    4
    ;; float32 _steeringAngle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steeringAngle
       (sys::poke _steeringAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steeringAngle
     (setq _steeringAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::stmupdate :md5sum-) "85aa0214068a35b3ce74e798a0c59f18")
(setf (get platooning::stmupdate :datatype-) "platooning/stmupdate")
(setf (get platooning::stmupdate :definition-)
      "uint32 id
float32 acceleration
float32 steeringAngle
")



(provide :platooning/stmupdate "85aa0214068a35b3ce74e798a0c59f18")


