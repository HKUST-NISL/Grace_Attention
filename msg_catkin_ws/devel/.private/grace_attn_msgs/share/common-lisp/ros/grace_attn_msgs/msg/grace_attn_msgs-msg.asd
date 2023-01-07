
(cl:in-package :asdf)

(defsystem "grace_attn_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :hr_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "TrackingReIDResult" :depends-on ("_package_TrackingReIDResult"))
    (:file "_package_TrackingReIDResult" :depends-on ("_package"))
  ))