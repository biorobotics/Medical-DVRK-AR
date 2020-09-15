; Auto-generated. Do not edit!


(cl:in-package blaser_pcl-srv)


;//! \htmlinclude VoxelGridStitch-request.msg.html

(cl:defclass <VoxelGridStitch-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:fixnum
    :initform 0)
   (pause
    :reader pause
    :initarg :pause
    :type cl:fixnum
    :initform 0)
   (clear
    :reader clear
    :initarg :clear
    :type cl:fixnum
    :initform 0)
   (leaf_size
    :reader leaf_size
    :initarg :leaf_size
    :type cl:float
    :initform 0.0))
)

(cl:defclass VoxelGridStitch-request (<VoxelGridStitch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VoxelGridStitch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VoxelGridStitch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blaser_pcl-srv:<VoxelGridStitch-request> is deprecated: use blaser_pcl-srv:VoxelGridStitch-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <VoxelGridStitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blaser_pcl-srv:start-val is deprecated.  Use blaser_pcl-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'pause-val :lambda-list '(m))
(cl:defmethod pause-val ((m <VoxelGridStitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blaser_pcl-srv:pause-val is deprecated.  Use blaser_pcl-srv:pause instead.")
  (pause m))

(cl:ensure-generic-function 'clear-val :lambda-list '(m))
(cl:defmethod clear-val ((m <VoxelGridStitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blaser_pcl-srv:clear-val is deprecated.  Use blaser_pcl-srv:clear instead.")
  (clear m))

(cl:ensure-generic-function 'leaf_size-val :lambda-list '(m))
(cl:defmethod leaf_size-val ((m <VoxelGridStitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blaser_pcl-srv:leaf_size-val is deprecated.  Use blaser_pcl-srv:leaf_size instead.")
  (leaf_size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VoxelGridStitch-request>) ostream)
  "Serializes a message object of type '<VoxelGridStitch-request>"
  (cl:let* ((signed (cl:slot-value msg 'start)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pause)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'clear)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'leaf_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VoxelGridStitch-request>) istream)
  "Deserializes a message object of type '<VoxelGridStitch-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pause) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'clear) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leaf_size) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VoxelGridStitch-request>)))
  "Returns string type for a service object of type '<VoxelGridStitch-request>"
  "blaser_pcl/VoxelGridStitchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoxelGridStitch-request)))
  "Returns string type for a service object of type 'VoxelGridStitch-request"
  "blaser_pcl/VoxelGridStitchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VoxelGridStitch-request>)))
  "Returns md5sum for a message object of type '<VoxelGridStitch-request>"
  "60c18bc0467086605bc6bd7bde5154dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VoxelGridStitch-request)))
  "Returns md5sum for a message object of type 'VoxelGridStitch-request"
  "60c18bc0467086605bc6bd7bde5154dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VoxelGridStitch-request>)))
  "Returns full string definition for message of type '<VoxelGridStitch-request>"
  (cl:format cl:nil "int8 start~%int8 pause~%int8 clear~%float64 leaf_size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VoxelGridStitch-request)))
  "Returns full string definition for message of type 'VoxelGridStitch-request"
  (cl:format cl:nil "int8 start~%int8 pause~%int8 clear~%float64 leaf_size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VoxelGridStitch-request>))
  (cl:+ 0
     1
     1
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VoxelGridStitch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VoxelGridStitch-request
    (cl:cons ':start (start msg))
    (cl:cons ':pause (pause msg))
    (cl:cons ':clear (clear msg))
    (cl:cons ':leaf_size (leaf_size msg))
))
;//! \htmlinclude VoxelGridStitch-response.msg.html

(cl:defclass <VoxelGridStitch-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass VoxelGridStitch-response (<VoxelGridStitch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VoxelGridStitch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VoxelGridStitch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blaser_pcl-srv:<VoxelGridStitch-response> is deprecated: use blaser_pcl-srv:VoxelGridStitch-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <VoxelGridStitch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blaser_pcl-srv:status-val is deprecated.  Use blaser_pcl-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VoxelGridStitch-response>) ostream)
  "Serializes a message object of type '<VoxelGridStitch-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VoxelGridStitch-response>) istream)
  "Deserializes a message object of type '<VoxelGridStitch-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VoxelGridStitch-response>)))
  "Returns string type for a service object of type '<VoxelGridStitch-response>"
  "blaser_pcl/VoxelGridStitchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoxelGridStitch-response)))
  "Returns string type for a service object of type 'VoxelGridStitch-response"
  "blaser_pcl/VoxelGridStitchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VoxelGridStitch-response>)))
  "Returns md5sum for a message object of type '<VoxelGridStitch-response>"
  "60c18bc0467086605bc6bd7bde5154dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VoxelGridStitch-response)))
  "Returns md5sum for a message object of type 'VoxelGridStitch-response"
  "60c18bc0467086605bc6bd7bde5154dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VoxelGridStitch-response>)))
  "Returns full string definition for message of type '<VoxelGridStitch-response>"
  (cl:format cl:nil "int64 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VoxelGridStitch-response)))
  "Returns full string definition for message of type 'VoxelGridStitch-response"
  (cl:format cl:nil "int64 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VoxelGridStitch-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VoxelGridStitch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VoxelGridStitch-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VoxelGridStitch)))
  'VoxelGridStitch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VoxelGridStitch)))
  'VoxelGridStitch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoxelGridStitch)))
  "Returns string type for a service object of type '<VoxelGridStitch>"
  "blaser_pcl/VoxelGridStitch")