;; Auto-generated. Do not edit!


(when (boundp 'platooning::gazupdate)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'gazupdate (find-package "PLATOONING")))
(unless (find-package "PLATOONING::GAZUPDATE")
  (make-package "PLATOONING::GAZUPDATE"))

(in-package "ROS")
;;//! \htmlinclude gazupdate.msg.html


(defclass platooning::gazupdate
  :super ros::object
  :slots (_id _distance _speed ))

(defmethod platooning::gazupdate
  (:init
   (&key
    ((:id __id) 0)
    ((:distance __distance) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _distance (float __distance))
   (setq _speed (float __speed))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; float32 _distance
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::gazupdate :md5sum-) "e013a91e1d04c1d0c4b05e286456c85e")
(setf (get platooning::gazupdate :datatype-) "platooning/gazupdate")
(setf (get platooning::gazupdate :definition-)
      "uint32 id
float32 distance
float32 speed
")



(provide :platooning/gazupdate "e013a91e1d04c1d0c4b05e286456c85e")


