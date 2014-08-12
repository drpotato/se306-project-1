; Auto-generated. Do not edit!


(cl:in-package se306_p1_pkg-msg)


;//! \htmlinclude Interaction.msg.html

(cl:defclass <Interaction> (roslisp-msg-protocol:ros-message)
  ((attribute
    :reader attribute
    :initarg :attribute
    :type cl:string
    :initform ""))
)

(cl:defclass Interaction (<Interaction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Interaction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Interaction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se306_p1_pkg-msg:<Interaction> is deprecated: use se306_p1_pkg-msg:Interaction instead.")))

(cl:ensure-generic-function 'attribute-val :lambda-list '(m))
(cl:defmethod attribute-val ((m <Interaction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se306_p1_pkg-msg:attribute-val is deprecated.  Use se306_p1_pkg-msg:attribute instead.")
  (attribute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Interaction>) ostream)
  "Serializes a message object of type '<Interaction>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'attribute))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'attribute))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Interaction>) istream)
  "Deserializes a message object of type '<Interaction>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'attribute) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'attribute) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Interaction>)))
  "Returns string type for a message object of type '<Interaction>"
  "se306_p1_pkg/Interaction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Interaction)))
  "Returns string type for a message object of type 'Interaction"
  "se306_p1_pkg/Interaction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Interaction>)))
  "Returns md5sum for a message object of type '<Interaction>"
  "b718a1f64372c4ec6cb016022002bad8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Interaction)))
  "Returns md5sum for a message object of type 'Interaction"
  "b718a1f64372c4ec6cb016022002bad8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Interaction>)))
  "Returns full string definition for message of type '<Interaction>"
  (cl:format cl:nil "string attribute~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Interaction)))
  "Returns full string definition for message of type 'Interaction"
  (cl:format cl:nil "string attribute~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Interaction>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'attribute))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Interaction>))
  "Converts a ROS message object to a list"
  (cl:list 'Interaction
    (cl:cons ':attribute (attribute msg))
))
