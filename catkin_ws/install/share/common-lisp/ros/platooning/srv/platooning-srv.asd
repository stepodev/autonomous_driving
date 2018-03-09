
(cl:in-package :asdf)

(defsystem "platooning-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "getVehicleId" :depends-on ("_package_getVehicleId"))
    (:file "_package_getVehicleId" :depends-on ("_package"))
  ))