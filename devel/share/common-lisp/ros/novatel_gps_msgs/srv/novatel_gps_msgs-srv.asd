
(cl:in-package :asdf)

(defsystem "novatel_gps_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NovatelFRESET" :depends-on ("_package_NovatelFRESET"))
    (:file "_package_NovatelFRESET" :depends-on ("_package"))
  ))