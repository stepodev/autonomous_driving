;; Auto-generated. Do not edit!


(when (boundp 'platooning::userInterface)
  (if (not (find-package "PLATOONING"))
    (make-package "PLATOONING"))
  (shadow 'userInterface (find-package "PLATOONING")))
(unless (find-package "PLATOONING::USERINTERFACE")
  (make-package "PLATOONING::USERINTERFACE"))

(in-package "ROS")
;;//! \htmlinclude userInterface.msg.html


(defclass platooning::userInterface
  :super ros::object
  :slots (_leading_vehicle _following_vehicle _potential_following_vehicle _inner_platoon_distance _actual_distance _platoon_speed _speed _platooning_state _src_vehicle _platoon_size _platoon_members _enable_remotecontrol ))

(defmethod platooning::userInterface
  (:init
   (&key
    ((:leading_vehicle __leading_vehicle) nil)
    ((:following_vehicle __following_vehicle) nil)
    ((:potential_following_vehicle __potential_following_vehicle) nil)
    ((:inner_platoon_distance __inner_platoon_distance) 0.0)
    ((:actual_distance __actual_distance) 0.0)
    ((:platoon_speed __platoon_speed) 0.0)
    ((:speed __speed) 0.0)
    ((:platooning_state __platooning_state) "")
    ((:src_vehicle __src_vehicle) 0)
    ((:platoon_size __platoon_size) 0)
    ((:platoon_members __platoon_members) (make-array 0 :initial-element 0 :element-type :integer))
    ((:enable_remotecontrol __enable_remotecontrol) nil)
    )
   (send-super :init)
   (setq _leading_vehicle __leading_vehicle)
   (setq _following_vehicle __following_vehicle)
   (setq _potential_following_vehicle __potential_following_vehicle)
   (setq _inner_platoon_distance (float __inner_platoon_distance))
   (setq _actual_distance (float __actual_distance))
   (setq _platoon_speed (float __platoon_speed))
   (setq _speed (float __speed))
   (setq _platooning_state (string __platooning_state))
   (setq _src_vehicle (round __src_vehicle))
   (setq _platoon_size (round __platoon_size))
   (setq _platoon_members __platoon_members)
   (setq _enable_remotecontrol __enable_remotecontrol)
   self)
  (:leading_vehicle
   (&optional __leading_vehicle)
   (if __leading_vehicle (setq _leading_vehicle __leading_vehicle)) _leading_vehicle)
  (:following_vehicle
   (&optional __following_vehicle)
   (if __following_vehicle (setq _following_vehicle __following_vehicle)) _following_vehicle)
  (:potential_following_vehicle
   (&optional __potential_following_vehicle)
   (if __potential_following_vehicle (setq _potential_following_vehicle __potential_following_vehicle)) _potential_following_vehicle)
  (:inner_platoon_distance
   (&optional __inner_platoon_distance)
   (if __inner_platoon_distance (setq _inner_platoon_distance __inner_platoon_distance)) _inner_platoon_distance)
  (:actual_distance
   (&optional __actual_distance)
   (if __actual_distance (setq _actual_distance __actual_distance)) _actual_distance)
  (:platoon_speed
   (&optional __platoon_speed)
   (if __platoon_speed (setq _platoon_speed __platoon_speed)) _platoon_speed)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:platooning_state
   (&optional __platooning_state)
   (if __platooning_state (setq _platooning_state __platooning_state)) _platooning_state)
  (:src_vehicle
   (&optional __src_vehicle)
   (if __src_vehicle (setq _src_vehicle __src_vehicle)) _src_vehicle)
  (:platoon_size
   (&optional __platoon_size)
   (if __platoon_size (setq _platoon_size __platoon_size)) _platoon_size)
  (:platoon_members
   (&optional __platoon_members)
   (if __platoon_members (setq _platoon_members __platoon_members)) _platoon_members)
  (:enable_remotecontrol
   (&optional __enable_remotecontrol)
   (if __enable_remotecontrol (setq _enable_remotecontrol __enable_remotecontrol)) _enable_remotecontrol)
  (:serialization-length
   ()
   (+
    ;; bool _leading_vehicle
    1
    ;; bool _following_vehicle
    1
    ;; bool _potential_following_vehicle
    1
    ;; float32 _inner_platoon_distance
    4
    ;; float32 _actual_distance
    4
    ;; float32 _platoon_speed
    4
    ;; float32 _speed
    4
    ;; string _platooning_state
    4 (length _platooning_state)
    ;; uint32 _src_vehicle
    4
    ;; uint32 _platoon_size
    4
    ;; uint32[] _platoon_members
    (* 4    (length _platoon_members)) 4
    ;; bool _enable_remotecontrol
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _leading_vehicle
       (if _leading_vehicle (write-byte -1 s) (write-byte 0 s))
     ;; bool _following_vehicle
       (if _following_vehicle (write-byte -1 s) (write-byte 0 s))
     ;; bool _potential_following_vehicle
       (if _potential_following_vehicle (write-byte -1 s) (write-byte 0 s))
     ;; float32 _inner_platoon_distance
       (sys::poke _inner_platoon_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _actual_distance
       (sys::poke _actual_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _platoon_speed
       (sys::poke _platoon_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _platooning_state
       (write-long (length _platooning_state) s) (princ _platooning_state s)
     ;; uint32 _src_vehicle
       (write-long _src_vehicle s)
     ;; uint32 _platoon_size
       (write-long _platoon_size s)
     ;; uint32[] _platoon_members
     (write-long (length _platoon_members) s)
     (dotimes (i (length _platoon_members))
       (write-long (elt _platoon_members i) s)
       )
     ;; bool _enable_remotecontrol
       (if _enable_remotecontrol (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _leading_vehicle
     (setq _leading_vehicle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _following_vehicle
     (setq _following_vehicle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _potential_following_vehicle
     (setq _potential_following_vehicle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _inner_platoon_distance
     (setq _inner_platoon_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _actual_distance
     (setq _actual_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _platoon_speed
     (setq _platoon_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _platooning_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _platooning_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _src_vehicle
     (setq _src_vehicle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _platoon_size
     (setq _platoon_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32[] _platoon_members
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _platoon_members (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _platoon_members i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; bool _enable_remotecontrol
     (setq _enable_remotecontrol (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get platooning::userInterface :md5sum-) "d1d08fffe38d5dc5a1aad88d74db0b91")
(setf (get platooning::userInterface :datatype-) "platooning/userInterface")
(setf (get platooning::userInterface :definition-)
      "bool leading_vehicle
bool following_vehicle
bool potential_following_vehicle
float32 inner_platoon_distance
float32 actual_distance
float32 platoon_speed
float32 speed
string platooning_state
uint32 src_vehicle
uint32 platoon_size
uint32[] platoon_members
bool enable_remotecontrol
")



(provide :platooning/userInterface "d1d08fffe38d5dc5a1aad88d74db0b91")


