; Auto-generated. Do not edit!


(cl:in-package se306_p1_pkg-msg)


;//! \htmlinclude Leave.msg.html

(cl:defclass <Leave> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Leave (<Leave>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Leave>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Leave)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se306_p1_pkg-msg:<Leave> is deprecated: use se306_p1_pkg-msg:Leave instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Leave>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se306_p1_pkg-msg:id-val is deprecated.  Use se306_p1_pkg-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Leave>) ostream)
  "Serializes a message object of type '<Leave>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Leave>) istream)
  "Deserializes a message object of type '<Leave>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Leave>)))
  "Returns string type for a message object of type '<Leave>"
  "se306_p1_pkg/Leave")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Leave)))
  "Returns string type for a message object of type 'Leave"
  "se306_p1_pkg/Leave")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Leave>)))
  "Returns md5sum for a message object of type '<Leave>"
  "7d504299943ad968aabe3de24053d208")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Leave)))
  "Returns md5sum for a message object of type 'Leave"
  "7d504299943ad968aabe3de24053d208")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Leave>)))
  "Returns full string definition for message of type '<Leave>"
  (cl:format cl:nil "int8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Leave)))
  "Returns full string definition for message of type 'Leave"
  (cl:format cl:nil "int8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Leave>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Leave>))
  "Converts a ROS message object to a list"
  (cl:list 'Leave
    (cl:cons ':id (id msg))
))
