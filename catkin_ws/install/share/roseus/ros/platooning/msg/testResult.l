;; Auto-generated. Do not edit!


(when (boundp 'platooning::testResult)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'testResult (find-package "PLATOONING")))
(unless (find-package "PLATOONING::TESTRESULT")
  (make-package "PLATOONING::TESTRESULT"))

(in-package "ROS")
;;//! \htmlinclude testResult.msg.html


(defclass platooning::testResult
  :super ros::object
  :slots (_success _comment ))

(defmethod platooning::testResult
  (:init
   (&key
    ((:success __success) nil)
    ((:comment __comment) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _comment (string __comment))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:comment
   (&optional __comment)
   (if __comment (setq _comment __comment)) _comment)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _comment
    4 (length _comment)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _comment
       (write-long (length _comment) s) (princ _comment s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _comment
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _comment (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get platooning::testResult :md5sum-) "1e664c0b6b9801e28898f96d696eb445")
(setf (get platooning::testResult :datatype-) "platooning/testResult")
(setf (get platooning::testResult :definition-)
      "bool success
string comment
")



(provide :platooning/testResult "1e664c0b6b9801e28898f96d696eb445")


