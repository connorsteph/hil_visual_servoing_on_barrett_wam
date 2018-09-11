; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude Teleop.msg.html

(cl:defclass <Teleop> (roslisp-msg-protocol:ros-message)
  ((dir_2D
    :reader dir_2D
    :initarg :dir_2D
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Teleop (<Teleop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Teleop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Teleop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<Teleop> is deprecated: use simulator-msg:Teleop instead.")))

(cl:ensure-generic-function 'dir_2D-val :lambda-list '(m))
(cl:defmethod dir_2D-val ((m <Teleop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:dir_2D-val is deprecated.  Use simulator-msg:dir_2D instead.")
  (dir_2D m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Teleop>) ostream)
  "Serializes a message object of type '<Teleop>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dir_2D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'dir_2D))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Teleop>) istream)
  "Deserializes a message object of type '<Teleop>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dir_2D) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dir_2D)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Teleop>)))
  "Returns string type for a message object of type '<Teleop>"
  "simulator/Teleop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Teleop)))
  "Returns string type for a message object of type 'Teleop"
  "simulator/Teleop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Teleop>)))
  "Returns md5sum for a message object of type '<Teleop>"
  "1e5c4f3d30b301acd82ac238f760cdb9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Teleop)))
  "Returns md5sum for a message object of type 'Teleop"
  "1e5c4f3d30b301acd82ac238f760cdb9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Teleop>)))
  "Returns full string definition for message of type '<Teleop>"
  (cl:format cl:nil "float64[] dir_2D~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Teleop)))
  "Returns full string definition for message of type 'Teleop"
  (cl:format cl:nil "float64[] dir_2D~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Teleop>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dir_2D) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Teleop>))
  "Converts a ROS message object to a list"
  (cl:list 'Teleop
    (cl:cons ':dir_2D (dir_2D msg))
))
