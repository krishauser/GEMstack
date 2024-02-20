;; Auto-generated. Do not edit!


(when (boundp 'neobotix_usboard_msgs::Paraset)
  (if (not (find-package "NEOBOTIX_USBOARD_MSGS"))
    (make-package "NEOBOTIX_USBOARD_MSGS"))
  (shadow 'Paraset (find-package "NEOBOTIX_USBOARD_MSGS")))
(unless (find-package "NEOBOTIX_USBOARD_MSGS::PARASET")
  (make-package "NEOBOTIX_USBOARD_MSGS::PARASET"))

(in-package "ROS")
;;//! \htmlinclude Paraset.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neobotix_usboard_msgs::Paraset
  :super ros::object
  :slots (_header _command _set_num _paraset_byte54 _paraset_byte53 _paraset_byte52 _paraset_byte51 _paraset_byte50 _paraset_byte49 _paraset_byte48 _paraset_byte47 _paraset_byte46 _paraset_byte45 _paraset_byte44 _paraset_byte43 _paraset_byte42 _paraset_byte41 _paraset_byte40 _paraset_byte39 _paraset_byte38 _paraset_byte37 _paraset_byte36 _paraset_byte35 _paraset_byte34 _paraset_byte33 _paraset_byte32 _paraset_byte31 _paraset_byte30 _paraset_byte29 _paraset_byte28 _paraset_byte27 _paraset_byte26 _paraset_byte25 _paraset_byte24 _paraset_byte23 _paraset_byte22 _paraset_byte21 _paraset_byte20 _paraset_byte19 _paraset_byte18 _paraset_byte17 _paraset_byte16 _paraset_byte15 _paraset_byte14 _paraset_byte13 _paraset_byte12 _paraset_byte11 _paraset_byte10 _paraset_byte9 _paraset_byte8 _paraset_byte7 _paraset_byte6 _paraset_byte5 _paraset_byte4 _paraset_byte3 _paraset_byte2 _paraset_byte1 ))

(defmethod neobotix_usboard_msgs::Paraset
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command __command) 0)
    ((:set_num __set_num) 0)
    ((:paraset_byte54 __paraset_byte54) 0)
    ((:paraset_byte53 __paraset_byte53) 0)
    ((:paraset_byte52 __paraset_byte52) 0)
    ((:paraset_byte51 __paraset_byte51) 0)
    ((:paraset_byte50 __paraset_byte50) 0)
    ((:paraset_byte49 __paraset_byte49) 0)
    ((:paraset_byte48 __paraset_byte48) 0)
    ((:paraset_byte47 __paraset_byte47) 0)
    ((:paraset_byte46 __paraset_byte46) 0)
    ((:paraset_byte45 __paraset_byte45) 0)
    ((:paraset_byte44 __paraset_byte44) 0)
    ((:paraset_byte43 __paraset_byte43) 0)
    ((:paraset_byte42 __paraset_byte42) 0)
    ((:paraset_byte41 __paraset_byte41) 0)
    ((:paraset_byte40 __paraset_byte40) 0)
    ((:paraset_byte39 __paraset_byte39) 0)
    ((:paraset_byte38 __paraset_byte38) 0)
    ((:paraset_byte37 __paraset_byte37) 0)
    ((:paraset_byte36 __paraset_byte36) 0)
    ((:paraset_byte35 __paraset_byte35) 0)
    ((:paraset_byte34 __paraset_byte34) 0)
    ((:paraset_byte33 __paraset_byte33) 0)
    ((:paraset_byte32 __paraset_byte32) 0)
    ((:paraset_byte31 __paraset_byte31) 0)
    ((:paraset_byte30 __paraset_byte30) 0)
    ((:paraset_byte29 __paraset_byte29) 0)
    ((:paraset_byte28 __paraset_byte28) 0)
    ((:paraset_byte27 __paraset_byte27) 0)
    ((:paraset_byte26 __paraset_byte26) 0)
    ((:paraset_byte25 __paraset_byte25) 0)
    ((:paraset_byte24 __paraset_byte24) 0)
    ((:paraset_byte23 __paraset_byte23) 0)
    ((:paraset_byte22 __paraset_byte22) 0)
    ((:paraset_byte21 __paraset_byte21) 0)
    ((:paraset_byte20 __paraset_byte20) 0)
    ((:paraset_byte19 __paraset_byte19) 0)
    ((:paraset_byte18 __paraset_byte18) 0)
    ((:paraset_byte17 __paraset_byte17) 0)
    ((:paraset_byte16 __paraset_byte16) 0)
    ((:paraset_byte15 __paraset_byte15) 0)
    ((:paraset_byte14 __paraset_byte14) 0)
    ((:paraset_byte13 __paraset_byte13) 0)
    ((:paraset_byte12 __paraset_byte12) 0)
    ((:paraset_byte11 __paraset_byte11) 0)
    ((:paraset_byte10 __paraset_byte10) 0)
    ((:paraset_byte9 __paraset_byte9) 0)
    ((:paraset_byte8 __paraset_byte8) 0)
    ((:paraset_byte7 __paraset_byte7) 0)
    ((:paraset_byte6 __paraset_byte6) 0)
    ((:paraset_byte5 __paraset_byte5) 0)
    ((:paraset_byte4 __paraset_byte4) 0)
    ((:paraset_byte3 __paraset_byte3) 0)
    ((:paraset_byte2 __paraset_byte2) 0)
    ((:paraset_byte1 __paraset_byte1) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command (round __command))
   (setq _set_num (round __set_num))
   (setq _paraset_byte54 (round __paraset_byte54))
   (setq _paraset_byte53 (round __paraset_byte53))
   (setq _paraset_byte52 (round __paraset_byte52))
   (setq _paraset_byte51 (round __paraset_byte51))
   (setq _paraset_byte50 (round __paraset_byte50))
   (setq _paraset_byte49 (round __paraset_byte49))
   (setq _paraset_byte48 (round __paraset_byte48))
   (setq _paraset_byte47 (round __paraset_byte47))
   (setq _paraset_byte46 (round __paraset_byte46))
   (setq _paraset_byte45 (round __paraset_byte45))
   (setq _paraset_byte44 (round __paraset_byte44))
   (setq _paraset_byte43 (round __paraset_byte43))
   (setq _paraset_byte42 (round __paraset_byte42))
   (setq _paraset_byte41 (round __paraset_byte41))
   (setq _paraset_byte40 (round __paraset_byte40))
   (setq _paraset_byte39 (round __paraset_byte39))
   (setq _paraset_byte38 (round __paraset_byte38))
   (setq _paraset_byte37 (round __paraset_byte37))
   (setq _paraset_byte36 (round __paraset_byte36))
   (setq _paraset_byte35 (round __paraset_byte35))
   (setq _paraset_byte34 (round __paraset_byte34))
   (setq _paraset_byte33 (round __paraset_byte33))
   (setq _paraset_byte32 (round __paraset_byte32))
   (setq _paraset_byte31 (round __paraset_byte31))
   (setq _paraset_byte30 (round __paraset_byte30))
   (setq _paraset_byte29 (round __paraset_byte29))
   (setq _paraset_byte28 (round __paraset_byte28))
   (setq _paraset_byte27 (round __paraset_byte27))
   (setq _paraset_byte26 (round __paraset_byte26))
   (setq _paraset_byte25 (round __paraset_byte25))
   (setq _paraset_byte24 (round __paraset_byte24))
   (setq _paraset_byte23 (round __paraset_byte23))
   (setq _paraset_byte22 (round __paraset_byte22))
   (setq _paraset_byte21 (round __paraset_byte21))
   (setq _paraset_byte20 (round __paraset_byte20))
   (setq _paraset_byte19 (round __paraset_byte19))
   (setq _paraset_byte18 (round __paraset_byte18))
   (setq _paraset_byte17 (round __paraset_byte17))
   (setq _paraset_byte16 (round __paraset_byte16))
   (setq _paraset_byte15 (round __paraset_byte15))
   (setq _paraset_byte14 (round __paraset_byte14))
   (setq _paraset_byte13 (round __paraset_byte13))
   (setq _paraset_byte12 (round __paraset_byte12))
   (setq _paraset_byte11 (round __paraset_byte11))
   (setq _paraset_byte10 (round __paraset_byte10))
   (setq _paraset_byte9 (round __paraset_byte9))
   (setq _paraset_byte8 (round __paraset_byte8))
   (setq _paraset_byte7 (round __paraset_byte7))
   (setq _paraset_byte6 (round __paraset_byte6))
   (setq _paraset_byte5 (round __paraset_byte5))
   (setq _paraset_byte4 (round __paraset_byte4))
   (setq _paraset_byte3 (round __paraset_byte3))
   (setq _paraset_byte2 (round __paraset_byte2))
   (setq _paraset_byte1 (round __paraset_byte1))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:set_num
   (&optional __set_num)
   (if __set_num (setq _set_num __set_num)) _set_num)
  (:paraset_byte54
   (&optional __paraset_byte54)
   (if __paraset_byte54 (setq _paraset_byte54 __paraset_byte54)) _paraset_byte54)
  (:paraset_byte53
   (&optional __paraset_byte53)
   (if __paraset_byte53 (setq _paraset_byte53 __paraset_byte53)) _paraset_byte53)
  (:paraset_byte52
   (&optional __paraset_byte52)
   (if __paraset_byte52 (setq _paraset_byte52 __paraset_byte52)) _paraset_byte52)
  (:paraset_byte51
   (&optional __paraset_byte51)
   (if __paraset_byte51 (setq _paraset_byte51 __paraset_byte51)) _paraset_byte51)
  (:paraset_byte50
   (&optional __paraset_byte50)
   (if __paraset_byte50 (setq _paraset_byte50 __paraset_byte50)) _paraset_byte50)
  (:paraset_byte49
   (&optional __paraset_byte49)
   (if __paraset_byte49 (setq _paraset_byte49 __paraset_byte49)) _paraset_byte49)
  (:paraset_byte48
   (&optional __paraset_byte48)
   (if __paraset_byte48 (setq _paraset_byte48 __paraset_byte48)) _paraset_byte48)
  (:paraset_byte47
   (&optional __paraset_byte47)
   (if __paraset_byte47 (setq _paraset_byte47 __paraset_byte47)) _paraset_byte47)
  (:paraset_byte46
   (&optional __paraset_byte46)
   (if __paraset_byte46 (setq _paraset_byte46 __paraset_byte46)) _paraset_byte46)
  (:paraset_byte45
   (&optional __paraset_byte45)
   (if __paraset_byte45 (setq _paraset_byte45 __paraset_byte45)) _paraset_byte45)
  (:paraset_byte44
   (&optional __paraset_byte44)
   (if __paraset_byte44 (setq _paraset_byte44 __paraset_byte44)) _paraset_byte44)
  (:paraset_byte43
   (&optional __paraset_byte43)
   (if __paraset_byte43 (setq _paraset_byte43 __paraset_byte43)) _paraset_byte43)
  (:paraset_byte42
   (&optional __paraset_byte42)
   (if __paraset_byte42 (setq _paraset_byte42 __paraset_byte42)) _paraset_byte42)
  (:paraset_byte41
   (&optional __paraset_byte41)
   (if __paraset_byte41 (setq _paraset_byte41 __paraset_byte41)) _paraset_byte41)
  (:paraset_byte40
   (&optional __paraset_byte40)
   (if __paraset_byte40 (setq _paraset_byte40 __paraset_byte40)) _paraset_byte40)
  (:paraset_byte39
   (&optional __paraset_byte39)
   (if __paraset_byte39 (setq _paraset_byte39 __paraset_byte39)) _paraset_byte39)
  (:paraset_byte38
   (&optional __paraset_byte38)
   (if __paraset_byte38 (setq _paraset_byte38 __paraset_byte38)) _paraset_byte38)
  (:paraset_byte37
   (&optional __paraset_byte37)
   (if __paraset_byte37 (setq _paraset_byte37 __paraset_byte37)) _paraset_byte37)
  (:paraset_byte36
   (&optional __paraset_byte36)
   (if __paraset_byte36 (setq _paraset_byte36 __paraset_byte36)) _paraset_byte36)
  (:paraset_byte35
   (&optional __paraset_byte35)
   (if __paraset_byte35 (setq _paraset_byte35 __paraset_byte35)) _paraset_byte35)
  (:paraset_byte34
   (&optional __paraset_byte34)
   (if __paraset_byte34 (setq _paraset_byte34 __paraset_byte34)) _paraset_byte34)
  (:paraset_byte33
   (&optional __paraset_byte33)
   (if __paraset_byte33 (setq _paraset_byte33 __paraset_byte33)) _paraset_byte33)
  (:paraset_byte32
   (&optional __paraset_byte32)
   (if __paraset_byte32 (setq _paraset_byte32 __paraset_byte32)) _paraset_byte32)
  (:paraset_byte31
   (&optional __paraset_byte31)
   (if __paraset_byte31 (setq _paraset_byte31 __paraset_byte31)) _paraset_byte31)
  (:paraset_byte30
   (&optional __paraset_byte30)
   (if __paraset_byte30 (setq _paraset_byte30 __paraset_byte30)) _paraset_byte30)
  (:paraset_byte29
   (&optional __paraset_byte29)
   (if __paraset_byte29 (setq _paraset_byte29 __paraset_byte29)) _paraset_byte29)
  (:paraset_byte28
   (&optional __paraset_byte28)
   (if __paraset_byte28 (setq _paraset_byte28 __paraset_byte28)) _paraset_byte28)
  (:paraset_byte27
   (&optional __paraset_byte27)
   (if __paraset_byte27 (setq _paraset_byte27 __paraset_byte27)) _paraset_byte27)
  (:paraset_byte26
   (&optional __paraset_byte26)
   (if __paraset_byte26 (setq _paraset_byte26 __paraset_byte26)) _paraset_byte26)
  (:paraset_byte25
   (&optional __paraset_byte25)
   (if __paraset_byte25 (setq _paraset_byte25 __paraset_byte25)) _paraset_byte25)
  (:paraset_byte24
   (&optional __paraset_byte24)
   (if __paraset_byte24 (setq _paraset_byte24 __paraset_byte24)) _paraset_byte24)
  (:paraset_byte23
   (&optional __paraset_byte23)
   (if __paraset_byte23 (setq _paraset_byte23 __paraset_byte23)) _paraset_byte23)
  (:paraset_byte22
   (&optional __paraset_byte22)
   (if __paraset_byte22 (setq _paraset_byte22 __paraset_byte22)) _paraset_byte22)
  (:paraset_byte21
   (&optional __paraset_byte21)
   (if __paraset_byte21 (setq _paraset_byte21 __paraset_byte21)) _paraset_byte21)
  (:paraset_byte20
   (&optional __paraset_byte20)
   (if __paraset_byte20 (setq _paraset_byte20 __paraset_byte20)) _paraset_byte20)
  (:paraset_byte19
   (&optional __paraset_byte19)
   (if __paraset_byte19 (setq _paraset_byte19 __paraset_byte19)) _paraset_byte19)
  (:paraset_byte18
   (&optional __paraset_byte18)
   (if __paraset_byte18 (setq _paraset_byte18 __paraset_byte18)) _paraset_byte18)
  (:paraset_byte17
   (&optional __paraset_byte17)
   (if __paraset_byte17 (setq _paraset_byte17 __paraset_byte17)) _paraset_byte17)
  (:paraset_byte16
   (&optional __paraset_byte16)
   (if __paraset_byte16 (setq _paraset_byte16 __paraset_byte16)) _paraset_byte16)
  (:paraset_byte15
   (&optional __paraset_byte15)
   (if __paraset_byte15 (setq _paraset_byte15 __paraset_byte15)) _paraset_byte15)
  (:paraset_byte14
   (&optional __paraset_byte14)
   (if __paraset_byte14 (setq _paraset_byte14 __paraset_byte14)) _paraset_byte14)
  (:paraset_byte13
   (&optional __paraset_byte13)
   (if __paraset_byte13 (setq _paraset_byte13 __paraset_byte13)) _paraset_byte13)
  (:paraset_byte12
   (&optional __paraset_byte12)
   (if __paraset_byte12 (setq _paraset_byte12 __paraset_byte12)) _paraset_byte12)
  (:paraset_byte11
   (&optional __paraset_byte11)
   (if __paraset_byte11 (setq _paraset_byte11 __paraset_byte11)) _paraset_byte11)
  (:paraset_byte10
   (&optional __paraset_byte10)
   (if __paraset_byte10 (setq _paraset_byte10 __paraset_byte10)) _paraset_byte10)
  (:paraset_byte9
   (&optional __paraset_byte9)
   (if __paraset_byte9 (setq _paraset_byte9 __paraset_byte9)) _paraset_byte9)
  (:paraset_byte8
   (&optional __paraset_byte8)
   (if __paraset_byte8 (setq _paraset_byte8 __paraset_byte8)) _paraset_byte8)
  (:paraset_byte7
   (&optional __paraset_byte7)
   (if __paraset_byte7 (setq _paraset_byte7 __paraset_byte7)) _paraset_byte7)
  (:paraset_byte6
   (&optional __paraset_byte6)
   (if __paraset_byte6 (setq _paraset_byte6 __paraset_byte6)) _paraset_byte6)
  (:paraset_byte5
   (&optional __paraset_byte5)
   (if __paraset_byte5 (setq _paraset_byte5 __paraset_byte5)) _paraset_byte5)
  (:paraset_byte4
   (&optional __paraset_byte4)
   (if __paraset_byte4 (setq _paraset_byte4 __paraset_byte4)) _paraset_byte4)
  (:paraset_byte3
   (&optional __paraset_byte3)
   (if __paraset_byte3 (setq _paraset_byte3 __paraset_byte3)) _paraset_byte3)
  (:paraset_byte2
   (&optional __paraset_byte2)
   (if __paraset_byte2 (setq _paraset_byte2 __paraset_byte2)) _paraset_byte2)
  (:paraset_byte1
   (&optional __paraset_byte1)
   (if __paraset_byte1 (setq _paraset_byte1 __paraset_byte1)) _paraset_byte1)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _command
    1
    ;; uint8 _set_num
    1
    ;; uint8 _paraset_byte54
    1
    ;; uint8 _paraset_byte53
    1
    ;; uint8 _paraset_byte52
    1
    ;; uint8 _paraset_byte51
    1
    ;; uint8 _paraset_byte50
    1
    ;; uint8 _paraset_byte49
    1
    ;; uint8 _paraset_byte48
    1
    ;; uint8 _paraset_byte47
    1
    ;; uint8 _paraset_byte46
    1
    ;; uint8 _paraset_byte45
    1
    ;; uint8 _paraset_byte44
    1
    ;; uint8 _paraset_byte43
    1
    ;; uint8 _paraset_byte42
    1
    ;; uint8 _paraset_byte41
    1
    ;; uint8 _paraset_byte40
    1
    ;; uint8 _paraset_byte39
    1
    ;; uint8 _paraset_byte38
    1
    ;; uint8 _paraset_byte37
    1
    ;; uint8 _paraset_byte36
    1
    ;; uint8 _paraset_byte35
    1
    ;; uint8 _paraset_byte34
    1
    ;; uint8 _paraset_byte33
    1
    ;; uint8 _paraset_byte32
    1
    ;; uint8 _paraset_byte31
    1
    ;; uint8 _paraset_byte30
    1
    ;; uint8 _paraset_byte29
    1
    ;; uint8 _paraset_byte28
    1
    ;; uint8 _paraset_byte27
    1
    ;; uint8 _paraset_byte26
    1
    ;; uint8 _paraset_byte25
    1
    ;; uint8 _paraset_byte24
    1
    ;; uint8 _paraset_byte23
    1
    ;; uint8 _paraset_byte22
    1
    ;; uint8 _paraset_byte21
    1
    ;; uint8 _paraset_byte20
    1
    ;; uint8 _paraset_byte19
    1
    ;; uint8 _paraset_byte18
    1
    ;; uint8 _paraset_byte17
    1
    ;; uint8 _paraset_byte16
    1
    ;; uint8 _paraset_byte15
    1
    ;; uint8 _paraset_byte14
    1
    ;; uint8 _paraset_byte13
    1
    ;; uint8 _paraset_byte12
    1
    ;; uint8 _paraset_byte11
    1
    ;; uint8 _paraset_byte10
    1
    ;; uint8 _paraset_byte9
    1
    ;; uint8 _paraset_byte8
    1
    ;; uint8 _paraset_byte7
    1
    ;; uint8 _paraset_byte6
    1
    ;; uint8 _paraset_byte5
    1
    ;; uint8 _paraset_byte4
    1
    ;; uint8 _paraset_byte3
    1
    ;; uint8 _paraset_byte2
    1
    ;; uint8 _paraset_byte1
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _command
       (write-byte _command s)
     ;; uint8 _set_num
       (write-byte _set_num s)
     ;; uint8 _paraset_byte54
       (write-byte _paraset_byte54 s)
     ;; uint8 _paraset_byte53
       (write-byte _paraset_byte53 s)
     ;; uint8 _paraset_byte52
       (write-byte _paraset_byte52 s)
     ;; uint8 _paraset_byte51
       (write-byte _paraset_byte51 s)
     ;; uint8 _paraset_byte50
       (write-byte _paraset_byte50 s)
     ;; uint8 _paraset_byte49
       (write-byte _paraset_byte49 s)
     ;; uint8 _paraset_byte48
       (write-byte _paraset_byte48 s)
     ;; uint8 _paraset_byte47
       (write-byte _paraset_byte47 s)
     ;; uint8 _paraset_byte46
       (write-byte _paraset_byte46 s)
     ;; uint8 _paraset_byte45
       (write-byte _paraset_byte45 s)
     ;; uint8 _paraset_byte44
       (write-byte _paraset_byte44 s)
     ;; uint8 _paraset_byte43
       (write-byte _paraset_byte43 s)
     ;; uint8 _paraset_byte42
       (write-byte _paraset_byte42 s)
     ;; uint8 _paraset_byte41
       (write-byte _paraset_byte41 s)
     ;; uint8 _paraset_byte40
       (write-byte _paraset_byte40 s)
     ;; uint8 _paraset_byte39
       (write-byte _paraset_byte39 s)
     ;; uint8 _paraset_byte38
       (write-byte _paraset_byte38 s)
     ;; uint8 _paraset_byte37
       (write-byte _paraset_byte37 s)
     ;; uint8 _paraset_byte36
       (write-byte _paraset_byte36 s)
     ;; uint8 _paraset_byte35
       (write-byte _paraset_byte35 s)
     ;; uint8 _paraset_byte34
       (write-byte _paraset_byte34 s)
     ;; uint8 _paraset_byte33
       (write-byte _paraset_byte33 s)
     ;; uint8 _paraset_byte32
       (write-byte _paraset_byte32 s)
     ;; uint8 _paraset_byte31
       (write-byte _paraset_byte31 s)
     ;; uint8 _paraset_byte30
       (write-byte _paraset_byte30 s)
     ;; uint8 _paraset_byte29
       (write-byte _paraset_byte29 s)
     ;; uint8 _paraset_byte28
       (write-byte _paraset_byte28 s)
     ;; uint8 _paraset_byte27
       (write-byte _paraset_byte27 s)
     ;; uint8 _paraset_byte26
       (write-byte _paraset_byte26 s)
     ;; uint8 _paraset_byte25
       (write-byte _paraset_byte25 s)
     ;; uint8 _paraset_byte24
       (write-byte _paraset_byte24 s)
     ;; uint8 _paraset_byte23
       (write-byte _paraset_byte23 s)
     ;; uint8 _paraset_byte22
       (write-byte _paraset_byte22 s)
     ;; uint8 _paraset_byte21
       (write-byte _paraset_byte21 s)
     ;; uint8 _paraset_byte20
       (write-byte _paraset_byte20 s)
     ;; uint8 _paraset_byte19
       (write-byte _paraset_byte19 s)
     ;; uint8 _paraset_byte18
       (write-byte _paraset_byte18 s)
     ;; uint8 _paraset_byte17
       (write-byte _paraset_byte17 s)
     ;; uint8 _paraset_byte16
       (write-byte _paraset_byte16 s)
     ;; uint8 _paraset_byte15
       (write-byte _paraset_byte15 s)
     ;; uint8 _paraset_byte14
       (write-byte _paraset_byte14 s)
     ;; uint8 _paraset_byte13
       (write-byte _paraset_byte13 s)
     ;; uint8 _paraset_byte12
       (write-byte _paraset_byte12 s)
     ;; uint8 _paraset_byte11
       (write-byte _paraset_byte11 s)
     ;; uint8 _paraset_byte10
       (write-byte _paraset_byte10 s)
     ;; uint8 _paraset_byte9
       (write-byte _paraset_byte9 s)
     ;; uint8 _paraset_byte8
       (write-byte _paraset_byte8 s)
     ;; uint8 _paraset_byte7
       (write-byte _paraset_byte7 s)
     ;; uint8 _paraset_byte6
       (write-byte _paraset_byte6 s)
     ;; uint8 _paraset_byte5
       (write-byte _paraset_byte5 s)
     ;; uint8 _paraset_byte4
       (write-byte _paraset_byte4 s)
     ;; uint8 _paraset_byte3
       (write-byte _paraset_byte3 s)
     ;; uint8 _paraset_byte2
       (write-byte _paraset_byte2 s)
     ;; uint8 _paraset_byte1
       (write-byte _paraset_byte1 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _set_num
     (setq _set_num (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte54
     (setq _paraset_byte54 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte53
     (setq _paraset_byte53 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte52
     (setq _paraset_byte52 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte51
     (setq _paraset_byte51 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte50
     (setq _paraset_byte50 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte49
     (setq _paraset_byte49 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte48
     (setq _paraset_byte48 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte47
     (setq _paraset_byte47 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte46
     (setq _paraset_byte46 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte45
     (setq _paraset_byte45 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte44
     (setq _paraset_byte44 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte43
     (setq _paraset_byte43 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte42
     (setq _paraset_byte42 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte41
     (setq _paraset_byte41 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte40
     (setq _paraset_byte40 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte39
     (setq _paraset_byte39 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte38
     (setq _paraset_byte38 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte37
     (setq _paraset_byte37 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte36
     (setq _paraset_byte36 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte35
     (setq _paraset_byte35 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte34
     (setq _paraset_byte34 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte33
     (setq _paraset_byte33 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte32
     (setq _paraset_byte32 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte31
     (setq _paraset_byte31 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte30
     (setq _paraset_byte30 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte29
     (setq _paraset_byte29 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte28
     (setq _paraset_byte28 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte27
     (setq _paraset_byte27 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte26
     (setq _paraset_byte26 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte25
     (setq _paraset_byte25 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte24
     (setq _paraset_byte24 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte23
     (setq _paraset_byte23 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte22
     (setq _paraset_byte22 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte21
     (setq _paraset_byte21 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte20
     (setq _paraset_byte20 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte19
     (setq _paraset_byte19 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte18
     (setq _paraset_byte18 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte17
     (setq _paraset_byte17 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte16
     (setq _paraset_byte16 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte15
     (setq _paraset_byte15 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte14
     (setq _paraset_byte14 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte13
     (setq _paraset_byte13 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte12
     (setq _paraset_byte12 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte11
     (setq _paraset_byte11 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte10
     (setq _paraset_byte10 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte9
     (setq _paraset_byte9 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte8
     (setq _paraset_byte8 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte7
     (setq _paraset_byte7 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte6
     (setq _paraset_byte6 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte5
     (setq _paraset_byte5 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte4
     (setq _paraset_byte4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte3
     (setq _paraset_byte3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte2
     (setq _paraset_byte2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte1
     (setq _paraset_byte1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get neobotix_usboard_msgs::Paraset :md5sum-) "88e510d038a634b40698407f2e433138")
(setf (get neobotix_usboard_msgs::Paraset :datatype-) "neobotix_usboard_msgs/Paraset")
(setf (get neobotix_usboard_msgs::Paraset :definition-)
      "# Message file for Paramset

std_msgs/Header header

uint8      command                                 
uint8      set_num                                 
uint8      paraset_byte54                    
uint8      paraset_byte53                    
uint8      paraset_byte52                    
uint8      paraset_byte51                    
uint8      paraset_byte50                    
uint8      paraset_byte49                    
uint8      paraset_byte48                    
uint8      paraset_byte47                    
uint8      paraset_byte46                    
uint8      paraset_byte45                    
uint8      paraset_byte44                    
uint8      paraset_byte43                    
uint8      paraset_byte42                    
uint8      paraset_byte41                    
uint8      paraset_byte40                    
uint8      paraset_byte39                    
uint8      paraset_byte38                    
uint8      paraset_byte37                    
uint8      paraset_byte36                    
uint8      paraset_byte35                    
uint8      paraset_byte34                    
uint8      paraset_byte33                    
uint8      paraset_byte32                    
uint8      paraset_byte31                    
uint8      paraset_byte30                    
uint8      paraset_byte29                    
uint8      paraset_byte28                    
uint8      paraset_byte27                    
uint8      paraset_byte26                    
uint8      paraset_byte25                    
uint8      paraset_byte24                    
uint8      paraset_byte23                    
uint8      paraset_byte22                    
uint8      paraset_byte21                    
uint8      paraset_byte20                    
uint8      paraset_byte19                    
uint8      paraset_byte18                    
uint8      paraset_byte17                    
uint8      paraset_byte16                    
uint8      paraset_byte15                    
uint8      paraset_byte14                    
uint8      paraset_byte13                    
uint8      paraset_byte12                    
uint8      paraset_byte11                    
uint8      paraset_byte10                    
uint8      paraset_byte9                     
uint8      paraset_byte8                     
uint8      paraset_byte7                     
uint8      paraset_byte6                     
uint8      paraset_byte5                     
uint8      paraset_byte4                     
uint8      paraset_byte3                     
uint8      paraset_byte2                     
uint8      paraset_byte1                     

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :neobotix_usboard_msgs/Paraset "88e510d038a634b40698407f2e433138")


