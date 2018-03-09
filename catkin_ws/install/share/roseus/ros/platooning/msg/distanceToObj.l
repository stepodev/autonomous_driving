;; Auto-generated. Do not edit!


(when (boundp 'platooning::distanceToObj)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'distanceToObj (find-package "PLATOONING")))
(unless (find-package "PLATOONING::DISTANCETOOBJ")
  (make-package "PLATOONING::DISTANCETOOBJ"))

(in-package "ROS")
;;//! \htmlinclude distanceToObj.msg.html


(defclass platooning::distanceToObj
  :super ros::object
  :slots (_distance_to_obj ))

(defmethod platooning::distanceToObj
  (:init
   (&key
    ((:distance_to_obj __distance_to_obj) 0.0)
    )
   (send-super :init)
   (setq _distance_to_obj (float __distance_to_obj))
   self)
  (:distance_to_obj
   (&optional __distance_to_obj)
   (if __distance_to_obj (setq _distance_to_obj __distance_to_obj)) _distance_to_obj)
  (:serialization-length
   ()
   (+
    ;; float32 _distance_to_obj
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _distance_to_obj
       (sys::poke _distance_to_obj (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _distance_to_obj
     (setq _distance_to_obj (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get platooning::distanceToObj :md5sum-) "b94f75528a81496ec1f35e38db49c1c0")
(setf (get platooning::distanceToObj :datatype-) "platooning/distanceToObj")
(setf (get platooning::distanceToObj :definition-)
      "float32 distance_to_obj

")



(provide :platooning/distanceToObj "b94f75528a81496ec1f35e38db49c1c0")


