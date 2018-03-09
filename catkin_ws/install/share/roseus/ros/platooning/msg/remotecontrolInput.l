;; Auto-generated. Do not edit!


(when (boundp 'platooning::remotecontrolInput)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'remotecontrolInput (find-package "PLATOONING")))
(unless (find-package "PLATOONING::REMOTECONTROLINPUT")
  (make-package "PLATOONING::REMOTECONTROLINPUT"))

(in-package "ROS")
;;//! \htmlinclude remotecontrolInput.msg.html


(defclass platooning::remotecontrolInput
  :super ros::object
  :slots (_remote_speed _remote_angle _emergency_stop ))

(defmethod platooning::remotecontrolInput
  (:init
   (&key
    ((:remote_speed __remote_speed) 0.0)
    ((:remote_angle __remote_angle) 0.0)
    ((:emergency_stop __emergency_stop) nil)
    )
   (send-super :init)
   (setq _remote_speed (float __remote_speed))
   (setq _remote_angle (float __remote_angle))
   (setq _emergency_stop __emergency_stop)
   self)
  (:remote_speed
   (&optional __remote_speed)
   (if __remote_speed (setq _remote_speed __remote_speed)) _remote_speed)
  (:remote_angle
   (&optional __remote_angle)
   (if __remote_angle (setq _remote_angle __remote_angle)) _remote_angle)
  (:emergency_stop
   (&optional __emergency_stop)
   (if __emergency_stop (setq _emergency_stop __emergency_stop)) _emergency_stop)
  (:serialization-length
   ()
   (+
    ;; float32 _remote_speed
    4
    ;; float32 _remote_angle
    4
    ;; bool _emergency_stop
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _remote_speed
       (sys::poke _remote_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _remote_angle
       (sys::poke _remote_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _emergency_stop
       (if _emergency_stop (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _remote_speed
     (setq _remote_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _remote_angle
     (setq _remote_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _emergency_stop
     (setq _emergency_stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get platooning::remotecontrolInput :md5sum-) "893b90aabdb4526f8be41f8f7b297e01")
(setf (get platooning::remotecontrolInput :datatype-) "platooning/remotecontrolInput")
(setf (get platooning::remotecontrolInput :definition-)
      "float32 remote_speed
float32 remote_angle
bool emergency_stop
")



(provide :platooning/remotecontrolInput "893b90aabdb4526f8be41f8f7b297e01")


