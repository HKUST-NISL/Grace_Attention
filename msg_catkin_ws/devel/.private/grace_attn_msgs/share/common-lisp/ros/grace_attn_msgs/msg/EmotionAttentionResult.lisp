; Auto-generated. Do not edit!


(cl:in-package grace_attn_msgs-msg)


;//! \htmlinclude EmotionAttentionResult.msg.html

(cl:defclass <EmotionAttentionResult> (roslisp-msg-protocol:ros-message)
  ((attention
    :reader attention
    :initarg :attention
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (emotion
    :reader emotion
    :initarg :emotion
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (visualization_frame
    :reader visualization_frame
    :initarg :visualization_frame
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass EmotionAttentionResult (<EmotionAttentionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmotionAttentionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmotionAttentionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name grace_attn_msgs-msg:<EmotionAttentionResult> is deprecated: use grace_attn_msgs-msg:EmotionAttentionResult instead.")))

(cl:ensure-generic-function 'attention-val :lambda-list '(m))
(cl:defmethod attention-val ((m <EmotionAttentionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grace_attn_msgs-msg:attention-val is deprecated.  Use grace_attn_msgs-msg:attention instead.")
  (attention m))

(cl:ensure-generic-function 'emotion-val :lambda-list '(m))
(cl:defmethod emotion-val ((m <EmotionAttentionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grace_attn_msgs-msg:emotion-val is deprecated.  Use grace_attn_msgs-msg:emotion instead.")
  (emotion m))

(cl:ensure-generic-function 'visualization_frame-val :lambda-list '(m))
(cl:defmethod visualization_frame-val ((m <EmotionAttentionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grace_attn_msgs-msg:visualization_frame-val is deprecated.  Use grace_attn_msgs-msg:visualization_frame instead.")
  (visualization_frame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmotionAttentionResult>) ostream)
  "Serializes a message object of type '<EmotionAttentionResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attention) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emotion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'visualization_frame) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmotionAttentionResult>) istream)
  "Deserializes a message object of type '<EmotionAttentionResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attention) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emotion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'visualization_frame) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmotionAttentionResult>)))
  "Returns string type for a message object of type '<EmotionAttentionResult>"
  "grace_attn_msgs/EmotionAttentionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionAttentionResult)))
  "Returns string type for a message object of type 'EmotionAttentionResult"
  "grace_attn_msgs/EmotionAttentionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmotionAttentionResult>)))
  "Returns md5sum for a message object of type '<EmotionAttentionResult>"
  "fb0f00a6f3b056b562935c8def10f302")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmotionAttentionResult)))
  "Returns md5sum for a message object of type 'EmotionAttentionResult"
  "fb0f00a6f3b056b562935c8def10f302")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmotionAttentionResult>)))
  "Returns full string definition for message of type '<EmotionAttentionResult>"
  (cl:format cl:nil "std_msgs/Int64 attention~%std_msgs/Int64 emotion~%sensor_msgs/Image visualization_frame~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmotionAttentionResult)))
  "Returns full string definition for message of type 'EmotionAttentionResult"
  (cl:format cl:nil "std_msgs/Int64 attention~%std_msgs/Int64 emotion~%sensor_msgs/Image visualization_frame~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmotionAttentionResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attention))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emotion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'visualization_frame))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmotionAttentionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'EmotionAttentionResult
    (cl:cons ':attention (attention msg))
    (cl:cons ':emotion (emotion msg))
    (cl:cons ':visualization_frame (visualization_frame msg))
))
