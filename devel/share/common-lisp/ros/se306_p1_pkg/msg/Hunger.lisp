; Auto-generated. Do not edit!


(cl:in-package se306_p1_pkg-msg)


;//! \htmlinclude Hunger.msg.html

(cl:defclass <Hunger> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Hunger (<Hunger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hunger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hunger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se306_p1_pkg-msg:<Hunger> is deprecated: use se306_p1_pkg-msg:Hunger instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <Hunger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se306_p1_pkg-msg:level-val is deprecated.  Use se306_p1_pkg-msg:level instead.")
  (level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hunger>) ostream)
  "Serializes a message object of type '<Hunger>"
  (cl:let* ((signed (cl:slot-value msg 'level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hunger>) istream)
  "Deserializes a message object of type '<Hunger>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'level) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hunger>)))
  "Returns string type for a message object of type '<Hunger>"
  "se306_p1_pkg/Hunger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hunger)))
  "Returns string type for a message object of type 'Hunger"
  "se306_p1_pkg/Hunger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hunger>)))
  "Returns md5sum for a message object of type '<Hunger>"
  "ecb6b6da01cd8c542143b24978f9b80f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hunger)))
  "Returns md5sum for a message object of type 'Hunger"
  "ecb6b6da01cd8c542143b24978f9b80f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hunger>)))
  "Returns full string definition for message of type '<Hunger>"
  (cl:format cl:nil "int8 level~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hunger)))
  "Returns full string definition for message of type 'Hunger"
  (cl:format cl:nil "int8 level~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hunger>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hunger>))
  "Converts a ROS message object to a list"
  (cl:list 'Hunger
    (cl:cons ':level (level msg))
))
