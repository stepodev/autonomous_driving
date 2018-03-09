;; Auto-generated. Do not edit!


(when (boundp 'platooning::platooningState)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'platooningState (find-package "PLATOONING")))
(unless (find-package "PLATOONING::PLATOONINGSTATE")
  (make-package "PLATOONING::PLATOONINGSTATE"))

(in-package "ROS")
;;//! \htmlinclude platooningState.msg.html


(defclass platooning::platooningState
  :super ros::object
  :slots (_platooning_state _platoon_id _ipd _ps _vehicle_id _i_am_FV _i_am_LV _platoon_members ))

(defmethod platooning::platooningState
  (:init
   (&key
    ((:platooning_state __platooning_state) "")
    ((:platoon_id __platoon_id) 0)
    ((:ipd __ipd) 0.0)
    ((:ps __ps) 0.0)
    ((:vehicle_id __vehicle_id) 0)
    ((:i_am_FV __i_am_FV) nil)
    ((:i_am_LV __i_am_LV) nil)
    ((:platoon_members __platoon_members) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _platooning_state (string __platooning_state))
   (setq _platoon_id (round __platoon_id))
   (setq _ipd (float __ipd))
   (setq _ps (float __ps))
   (setq _vehicle_id (round __vehicle_id))
   (setq _i_am_FV __i_am_FV)
   (setq _i_am_LV __i_am_LV)
   (setq _platoon_members __platoon_members)
   self)
  (:platooning_state
   (&optional __platooning_state)
   (if __platooning_state (setq _platooning_state __platooning_state)) _platooning_state)
  (:platoon_id
   (&optional __platoon_id)
   (if __platoon_id (setq _platoon_id __platoon_id)) _platoon_id)
  (:ipd
   (&optional __ipd)
   (if __ipd (setq _ipd __ipd)) _ipd)
  (:ps
   (&optional __ps)
   (if __ps (setq _ps __ps)) _ps)
  (:vehicle_id
   (&optional __vehicle_id)
   (if __vehicle_id (setq _vehicle_id __vehicle_id)) _vehicle_id)
  (:i_am_FV
   (&optional __i_am_FV)
   (if __i_am_FV (setq _i_am_FV __i_am_FV)) _i_am_FV)
  (:i_am_LV
   (&optional __i_am_LV)
   (if __i_am_LV (setq _i_am_LV __i_am_LV)) _i_am_LV)
  (:platoon_members
   (&optional __platoon_members)
   (if __platoon_members (setq _platoon_members __platoon_members)) _platoon_members)
  (:serialization-length
   ()
   (+
    ;; string _platooning_state
    4 (length _platooning_state)
    ;; uint32 _platoon_id
    4
    ;; float32 _ipd
    4
    ;; float32 _ps
    4
    ;; uint32 _vehicle_id
    4
    ;; bool _i_am_FV
    1
    ;; bool _i_am_LV
    1
    ;; uint32[] _platoon_members
    (* 4    (length _platoon_members)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _platooning_state
       (write-long (length _platooning_state) s) (princ _platooning_state s)
     ;; uint32 _platoon_id
       (write-long _platoon_id s)
     ;; float32 _ipd
       (sys::poke _ipd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ps
       (sys::poke _ps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _vehicle_id
       (write-long _vehicle_id s)
     ;; bool _i_am_FV
       (if _i_am_FV (write-byte -1 s) (write-byte 0 s))
     ;; bool _i_am_LV
       (if _i_am_LV (write-byte -1 s) (write-byte 0 s))
     ;; uint32[] _platoon_members
     (write-long (length _platoon_members) s)
     (dotimes (i (length _platoon_members))
       (write-long (elt _platoon_members i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _platooning_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _platooning_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _platoon_id
     (setq _platoon_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _ipd
     (setq _ipd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ps
     (setq _ps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _vehicle_id
     (setq _vehicle_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _i_am_FV
     (setq _i_am_FV (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _i_am_LV
     (setq _i_am_LV (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32[] _platoon_members
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _platoon_members (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _platoon_members i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get platooning::platooningState :md5sum-) "513c3389662a56c9d5dc40bcbdf23890")
(setf (get platooning::platooningState :datatype-) "platooning/platooningState")
(setf (get platooning::platooningState :definition-)
      "string platooning_state
uint32 platoon_id
float32 ipd
float32 ps
uint32 vehicle_id
bool i_am_FV
bool i_am_LV
uint32[] platoon_members
")



(provide :platooning/platooningState "513c3389662a56c9d5dc40bcbdf23890")


