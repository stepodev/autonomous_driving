;; Auto-generated. Do not edit!


(when (boundp 'platooning::registerTestcases)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'registerTestcases (find-package "PLATOONING")))
(unless (find-package "PLATOONING::REGISTERTESTCASES")
  (make-package "PLATOONING::REGISTERTESTCASES"))

(in-package "ROS")
;;//! \htmlinclude registerTestcases.msg.html


(defclass platooning::registerTestcases
  :super ros::object
  :slots (_testcase ))

(defmethod platooning::registerTestcases
  (:init
   (&key
    ((:testcase __testcase) "")
    )
   (send-super :init)
   (setq _testcase (string __testcase))
   self)
  (:testcase
   (&optional __testcase)
   (if __testcase (setq _testcase __testcase)) _testcase)
  (:serialization-length
   ()
   (+
    ;; string _testcase
    4 (length _testcase)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _testcase
       (write-long (length _testcase) s) (princ _testcase s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _testcase
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _testcase (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get platooning::registerTestcases :md5sum-) "f1ec9035577ebb432d9345a861e195f6")
(setf (get platooning::registerTestcases :datatype-) "platooning/registerTestcases")
(setf (get platooning::registerTestcases :definition-)
      "string testcase
")



(provide :platooning/registerTestcases "f1ec9035577ebb432d9345a861e195f6")


