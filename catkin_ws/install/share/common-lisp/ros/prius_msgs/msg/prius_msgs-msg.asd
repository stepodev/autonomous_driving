
(cl:in-package :asdf)

(defsystem "prius_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Control" :depends-on ("_package_Control"))
    (:file "_package_Control" :depends-on ("_package"))
    (:file "Speed" :depends-on ("_package_Speed"))
    (:file "_package_Speed" :depends-on ("_package"))
  ))