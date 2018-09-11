;; Auto-generated. Do not edit!


(when (boundp 'simulator::Teleop)
  (if (not (find-package "SIMULATOR"))
    (make-package "SIMULATOR"))
  (shadow 'Teleop (find-package "SIMULATOR")))
(unless (find-package "SIMULATOR::TELEOP")
  (make-package "SIMULATOR::TELEOP"))

(in-package "ROS")
;;//! \htmlinclude Teleop.msg.html


(defclass simulator::Teleop
  :super ros::object
  :slots (_dir_2D ))

(defmethod simulator::Teleop
  (:init
   (&key
    ((:dir_2D __dir_2D) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _dir_2D __dir_2D)
   self)
  (:dir_2D
   (&optional __dir_2D)
   (if __dir_2D (setq _dir_2D __dir_2D)) _dir_2D)
  (:serialization-length
   ()
   (+
    ;; float64[] _dir_2D
    (* 8    (length _dir_2D)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _dir_2D
     (write-long (length _dir_2D) s)
     (dotimes (i (length _dir_2D))
       (sys::poke (elt _dir_2D i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _dir_2D
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dir_2D (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dir_2D i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get simulator::Teleop :md5sum-) "1e5c4f3d30b301acd82ac238f760cdb9")
(setf (get simulator::Teleop :datatype-) "simulator/Teleop")
(setf (get simulator::Teleop :definition-)
      "float64[] dir_2D

")



(provide :simulator/Teleop "1e5c4f3d30b301acd82ac238f760cdb9")


