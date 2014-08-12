; Auto-generated. Do not edit!


(cl:in-package se306_p1_pkg-msg)


;//! \htmlinclude Loneliness.msg.html

(cl:defclass <Loneliness> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Loneliness (<Loneliness>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Loneliness>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Loneliness)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se306_p1_pkg-msg:<Loneliness> is deprecated: use se306_p1_pkg-msg:Loneliness instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <Loneliness>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se306_p1_pkg-msg:level-val is deprecated.  Use se306_p1_pkg-msg:level instead.")
  (level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Loneliness>) ostream)
  "Serializes a message object of type '<Loneliness>"
  (cl:let* ((signed (cl:slot-value msg 'level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Loneliness>) istream)
  "Deserializes a message object of type '<Loneliness>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'level) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Loneliness>)))
  "Returns string type for a message object of type '<Loneliness>"
  "se306_p1_pkg/Loneliness")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Loneliness)))
  "Returns string type for a message object of type 'Loneliness"
  "se306_p1_pkg/Loneliness")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Loneliness>)))
  "Returns md5sum for a message object of type '<Loneliness>"
  "ecb6b6da01cd8c542143b24978f9b80f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Loneliness)))
  "Returns md5sum for a message object of type 'Loneliness"
  "ecb6b6da01cd8c542143b24978f9b80f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Loneliness>)))
  "Returns full string definition for message of type '<Loneliness>"
  (cl:format cl:nil "int8 level~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Loneliness)))
  "Returns full string definition for message of type 'Loneliness"
  (cl:format cl:nil "int8 level~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Loneliness>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Loneliness>))
  "Converts a ROS message object to a list"
  (cl:list 'Loneliness
    (cl:cons ':level (level msg))
))
