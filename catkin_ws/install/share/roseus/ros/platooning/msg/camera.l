;; Auto-generated. Do not edit!


(when (boundp 'platooning::camera)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'camera (find-package "PLATOONING")))
(unless (find-package "PLATOONING::CAMERA")
  (make-package "PLATOONING::CAMERA"))

(in-package "ROS")
;;//! \htmlinclude camera.msg.html


(defclass platooning::camera
  :super ros::object
  :slots (_pic ))

(defmethod platooning::camera
  (:init
   (&key
    ((:pic __pic) "")
    )
   (send-super :init)
   (setq _pic (string __pic))
   self)
  (:pic
   (&optional __pic)
   (if __pic (setq _pic __pic)) _pic)
  (:serialization-length
   ()
   (+
    ;; string _pic
    4 (length _pic)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _pic
       (write-long (length _pic) s) (princ _pic s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _pic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get platooning::camera :md5sum-) "b029feab08f2c4877092228da5ee6702")
(setf (get platooning::camera :datatype-) "platooning/camera")
(setf (get platooning::camera :definition-)
      "string pic
")



(provide :platooning/camera "b029feab08f2c4877092228da5ee6702")


