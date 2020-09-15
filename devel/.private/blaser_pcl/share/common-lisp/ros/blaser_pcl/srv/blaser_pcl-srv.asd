
(cl:in-package :asdf)

(defsystem "blaser_pcl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "VoxelGridStitch" :depends-on ("_package_VoxelGridStitch"))
    (:file "_package_VoxelGridStitch" :depends-on ("_package"))
  ))