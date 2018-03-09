;; Auto-generated. Do not edit!


(when (boundp 'platooning::runTestCommand)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'runTestCommand (find-package "PLATOONING")))
(unless (find-package "PLATOONING::RUNTESTCOMMAND")
  (make-package "PLATOONING::RUNTESTCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude runTestCommand.msg.html


(defclass platooning::runTestCommand
  :super ros::object
  :slots (_testToRun ))

(defmethod platooning::runTestCommand
  (:init
   (&key
    ((:testToRun __testToRun) "")
    )
   (send-super :init)
   (setq _testToRun (string __testToRun))
   self)
  (:testToRun
   (&optional __testToRun)
   (if __testToRun (setq _testToRun __testToRun)) _testToRun)
  (:serialization-length
   ()
   (+
    ;; string _testToRun
    4 (length _testToRun)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _testToRun
       (write-long (length _testToRun) s) (princ _testToRun s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _testToRun
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _testToRun (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get platooning::runTestCommand :md5sum-) "01c8c3a6d2ae4ece1652d4c711eb8664")
(setf (get platooning::runTestCommand :datatype-) "platooning/runTestCommand")
(setf (get platooning::runTestCommand :definition-)
      "string testToRun
")



(provide :platooning/runTestCommand "01c8c3a6d2ae4ece1652d4c711eb8664")


