; Auto-generated. Do not edit!


(cl:in-package se306_p1_pkg-msg)


;//! \htmlinclude Entertainedness.msg.html

(cl:defclass <Entertainedness> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Entertainedness (<Entertainedness>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Entertainedness>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Entertainedness)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se306_p1_pkg-msg:<Entertainedness> is deprecated: use se306_p1_pkg-msg:Entertainedness instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <Entertainedness>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se306_p1_pkg-msg:level-val is deprecated.  Use se306_p1_pkg-msg:level instead.")
  (level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Entertainedness>) ostream)
  "Serializes a message object of type '<Entertainedness>"
  (cl:let* ((signed (cl:slot-value msg 'level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Entertainedness>) istream)
  "Deserializes a message object of type '<Entertainedness>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'level) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Entertainedness>)))
  "Returns string type for a message object of type '<Entertainedness>"
  "se306_p1_pkg/Entertainedness")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Entertainedness)))
  "Returns string type for a message object of type 'Entertainedness"
  "se306_p1_pkg/Entertainedness")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Entertainedness>)))
  "Returns md5sum for a message object of type '<Entertainedness>"
  "ecb6b6da01cd8c542143b24978f9b80f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Entertainedness)))
  "Returns md5sum for a message object of type 'Entertainedness"
  "ecb6b6da01cd8c542143b24978f9b80f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Entertainedness>)))
  "Returns full string definition for message of type '<Entertainedness>"
  (cl:format cl:nil "int8 level~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Entertainedness)))
  "Returns full string definition for message of type 'Entertainedness"
  (cl:format cl:nil "int8 level~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Entertainedness>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Entertainedness>))
  "Converts a ROS message object to a list"
  (cl:list 'Entertainedness
    (cl:cons ':level (level msg))
))
