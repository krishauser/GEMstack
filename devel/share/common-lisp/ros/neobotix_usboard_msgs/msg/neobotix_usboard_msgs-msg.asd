
(cl:in-package :asdf)

(defsystem "neobotix_usboard_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AnalogIn" :depends-on ("_package_AnalogIn"))
    (:file "_package_AnalogIn" :depends-on ("_package"))
    (:file "AnsParasetToEEPROM" :depends-on ("_package_AnsParasetToEEPROM"))
    (:file "_package_AnsParasetToEEPROM" :depends-on ("_package"))
    (:file "AnsToCmdConnect" :depends-on ("_package_AnsToCmdConnect"))
    (:file "_package_AnsToCmdConnect" :depends-on ("_package"))
    (:file "AnsWriteParaset" :depends-on ("_package_AnsWriteParaset"))
    (:file "_package_AnsWriteParaset" :depends-on ("_package"))
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "Paraset" :depends-on ("_package_Paraset"))
    (:file "_package_Paraset" :depends-on ("_package"))
    (:file "SensorData" :depends-on ("_package_SensorData"))
    (:file "_package_SensorData" :depends-on ("_package"))
    (:file "Sensors" :depends-on ("_package_Sensors"))
    (:file "_package_Sensors" :depends-on ("_package"))
  ))