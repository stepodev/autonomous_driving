;; Auto-generated. Do not edit!


(when (boundp 'platooning::vehiclecontrol)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'vehiclecontrol (find-package "PLATOONING")))
(unless (find-package "PLATOONING::VEHICLECONTROL")
  (make-package "PLATOONING::VEHICLECONTROL"))

(in-package "ROS")
;;//! \htmlinclude vehiclecontrol.msg.html


(defclass platooning::vehiclecontrol
  :super ros::object
  :slots (_accelleration _steering ))

(defmethod platooning::vehiclecontrol
  (:init
   (&key
    ((:accelleration __accelleration) 0.0)
    ((:steering __steering) 0.0)
    )
   (send-super :init)
   (setq _accelleration (float __accelleration))
   (setq _steering (float __steering))
   self)
  (:accelleration
   (&optional __accelleration)
   (if __accelleration (setq _accelleration __accelleration)) _accelleration)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:serialization-length
   ()
   (+
    ;; float32 _accelleration
    4
    ;; float32 _steering
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _accelleration
       (sys::poke _accelleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _accelleration
     (setq _accelleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering
     (setq _steering (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::vehiclecontrol :md5sum-) "41ed81cddf0d4f2bc76ecf491da03a04")
(setf (get platooning::vehiclecontrol :datatype-) "platooning/vehiclecontrol")
(setf (get platooning::vehiclecontrol :definition-)
      "float32 accelleration
float32 steering
")



(provide :platooning/vehiclecontrol "41ed81cddf0d4f2bc76ecf491da03a04")


