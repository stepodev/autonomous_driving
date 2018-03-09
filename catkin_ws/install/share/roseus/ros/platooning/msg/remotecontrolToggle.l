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
  :slots (_vehicle_id _enable_remotecontrol ))

(defmethod platooning::remotecontrolToggle
  (:init
   (&key
    ((:vehicle_id __vehicle_id) 0)
    ((:enable_remotecontrol __enable_remotecontrol) nil)
    )
   (send-super :init)
   (setq _vehicle_id (round __vehicle_id))
   (setq _enable_remotecontrol __enable_remotecontrol)
   self)
  (:vehicle_id
   (&optional __vehicle_id)
   (if __vehicle_id (setq _vehicle_id __vehicle_id)) _vehicle_id)
  (:enable_remotecontrol
   (&optional __enable_remotecontrol)
   (if __enable_remotecontrol (setq _enable_remotecontrol __enable_remotecontrol)) _enable_remotecontrol)
  (:serialization-length
   ()
   (+
    ;; uint32 _vehicle_id
    4
    ;; bool _enable_remotecontrol
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _vehicle_id
       (write-long _vehicle_id s)
     ;; bool _enable_remotecontrol
       (if _enable_remotecontrol (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _vehicle_id
     (setq _vehicle_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _enable_remotecontrol
     (setq _enable_remotecontrol (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get platooning::remotecontrolToggle :md5sum-) "8f56e4c7d94d4f505d773a5b64c0ffed")
(setf (get platooning::remotecontrolToggle :datatype-) "platooning/remotecontrolToggle")
(setf (get platooning::remotecontrolToggle :definition-)
      "uint32 vehicle_id
bool enable_remotecontrol
")



(provide :platooning/remotecontrolToggle "8f56e4c7d94d4f505d773a5b64c0ffed")


