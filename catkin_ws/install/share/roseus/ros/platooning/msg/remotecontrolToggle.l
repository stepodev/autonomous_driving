;; Auto-generated. Do not edit!


(when (boundp 'platooning::remotecontrolToggle)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'remotecontrolToggle (find-package "PLATOONING")))
(unless (find-package "PLATOONING::REMOTECONTROLTOGGLE")
  (make-package "PLATOONING::REMOTECONTROLTOGGLE"))

(in-package "ROS")
;;//! \htmlinclude remotecontrolToggle.msg.html


(defclass platooning::remotecontrolToggle
  :super ros::object
  :slots (_enable_remotecontrol ))

(defmethod platooning::remotecontrolToggle
  (:init
   (&key
    ((:enable_remotecontrol __enable_remotecontrol) nil)
    )
   (send-super :init)
   (setq _enable_remotecontrol __enable_remotecontrol)
   self)
  (:enable_remotecontrol
   (&optional __enable_remotecontrol)
   (if __enable_remotecontrol (setq _enable_remotecontrol __enable_remotecontrol)) _enable_remotecontrol)
  (:serialization-length
   ()
   (+
    ;; bool _enable_remotecontrol
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable_remotecontrol
       (if _enable_remotecontrol (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable_remotecontrol
     (setq _enable_remotecontrol (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get platooning::remotecontrolToggle :md5sum-) "dba02a7ea23346dfdb6c9a3c2e957b7d")
(setf (get platooning::remotecontrolToggle :datatype-) "platooning/remotecontrolToggle")
(setf (get platooning::remotecontrolToggle :definition-)
      "bool enable_remotecontrol
")



(provide :platooning/remotecontrolToggle "dba02a7ea23346dfdb6c9a3c2e957b7d")


