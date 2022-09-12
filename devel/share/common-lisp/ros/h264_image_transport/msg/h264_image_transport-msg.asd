
(cl:in-package :asdf)

(defsystem "h264_image_transport-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "H264Packet" :depends-on ("_package_H264Packet"))
    (:file "_package_H264Packet" :depends-on ("_package"))
  ))