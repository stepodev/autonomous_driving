;; Auto-generated. Do not edit!


(when (boundp 'prius_msgs::Speed)
  (if (not (find-package "PRIUS_MSGS"))
    (make-package "PRIUS_MSGS"))
  (shadow 'Speed (find-package "PRIUS_MSGS")))
(unless (find-package "PRIUS_MSGS::SPEED")
  (make-package "PRIUS_MSGS::SPEED"))

(in-package "ROS")
;;//! \htmlinclude Speed.msg.html


(defclass prius_msgs::Speed
  :super ros::object
  :slots (_speed ))

(defmethod prius_msgs::Speed
  (:init
   (&key
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get prius_msgs::Speed :md5sum-) "ca65bba734a79b4a6707341d829f4d5c")
(setf (get prius_msgs::Speed :datatype-) "prius_msgs/Speed")
(setf (get prius_msgs::Speed :definition-)
      "float32 speed

")



(provide :prius_msgs/Speed "ca65bba734a79b4a6707341d829f4d5c")


