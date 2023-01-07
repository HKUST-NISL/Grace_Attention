; Auto-generated. Do not edit!


(cl:in-package grace_attn_msgs-msg)


;//! \htmlinclude tracking_reid_result.msg.html

(cl:defclass <tracking_reid_result> (roslisp-msg-protocol:ros-message)
  ((accompanying_frame
    :reader accompanying_frame
    :initarg :accompanying_frame
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (target_person
    :reader target_person
    :initarg :target_person
    :type hr_msgs-msg:Person
    :initform (cl:make-instance 'hr_msgs-msg:Person)))
)

(cl:defclass tracking_reid_result (<tracking_reid_result>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tracking_reid_result>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tracking_reid_result)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name grace_attn_msgs-msg:<tracking_reid_result> is deprecated: use grace_attn_msgs-msg:tracking_reid_result instead.")))

(cl:ensure-generic-function 'accompanying_frame-val :lambda-list '(m))
(cl:defmethod accompanying_frame-val ((m <tracking_reid_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grace_attn_msgs-msg:accompanying_frame-val is deprecated.  Use grace_attn_msgs-msg:accompanying_frame instead.")
  (accompanying_frame m))

(cl:ensure-generic-function 'target_person-val :lambda-list '(m))
(cl:defmethod target_person-val ((m <tracking_reid_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grace_attn_msgs-msg:target_person-val is deprecated.  Use grace_attn_msgs-msg:target_person instead.")
  (target_person m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tracking_reid_result>) ostream)
  "Serializes a message object of type '<tracking_reid_result>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accompanying_frame) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_person) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tracking_reid_result>) istream)
  "Deserializes a message object of type '<tracking_reid_result>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accompanying_frame) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_person) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tracking_reid_result>)))
  "Returns string type for a message object of type '<tracking_reid_result>"
  "grace_attn_msgs/tracking_reid_result")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tracking_reid_result)))
  "Returns string type for a message object of type 'tracking_reid_result"
  "grace_attn_msgs/tracking_reid_result")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tracking_reid_result>)))
  "Returns md5sum for a message object of type '<tracking_reid_result>"
  "23bd691bb2b76cf204aca0c66b104aa1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tracking_reid_result)))
  "Returns md5sum for a message object of type 'tracking_reid_result"
  "23bd691bb2b76cf204aca0c66b104aa1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tracking_reid_result>)))
  "Returns full string definition for message of type '<tracking_reid_result>"
  (cl:format cl:nil "sensor_msgs/Image accompanying_frame~%hr_msgs/Person target_person~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hr_msgs/Person~%std_msgs/Header header~%string id~%hr_msgs/Face face~%hr_msgs/Body body~%================================================================================~%MSG: hr_msgs/Face~%std_msgs/Header header~%string id~%geometry_msgs/Pose left_gaze~%geometry_msgs/Pose right_gaze~%geometry_msgs/Vector3 gaze_angle~%geometry_msgs/Point location~%geometry_msgs/Pose head_pose~%geometry_msgs/Point[] landmarks~%hr_msgs/FacialActionUnit[] action_units~%sensor_msgs/RegionOfInterest bounding_box~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: hr_msgs/FacialActionUnit~%string name~%float64 presence~%float64 intensity~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: hr_msgs/Body~%std_msgs/Header header~%string id~%geometry_msgs/Point location~%geometry_msgs/Point[] landmarks~%string[] landmarks_names~%sensor_msgs/RegionOfInterest bounding_box~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tracking_reid_result)))
  "Returns full string definition for message of type 'tracking_reid_result"
  (cl:format cl:nil "sensor_msgs/Image accompanying_frame~%hr_msgs/Person target_person~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hr_msgs/Person~%std_msgs/Header header~%string id~%hr_msgs/Face face~%hr_msgs/Body body~%================================================================================~%MSG: hr_msgs/Face~%std_msgs/Header header~%string id~%geometry_msgs/Pose left_gaze~%geometry_msgs/Pose right_gaze~%geometry_msgs/Vector3 gaze_angle~%geometry_msgs/Point location~%geometry_msgs/Pose head_pose~%geometry_msgs/Point[] landmarks~%hr_msgs/FacialActionUnit[] action_units~%sensor_msgs/RegionOfInterest bounding_box~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: hr_msgs/FacialActionUnit~%string name~%float64 presence~%float64 intensity~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: hr_msgs/Body~%std_msgs/Header header~%string id~%geometry_msgs/Point location~%geometry_msgs/Point[] landmarks~%string[] landmarks_names~%sensor_msgs/RegionOfInterest bounding_box~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tracking_reid_result>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accompanying_frame))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_person))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tracking_reid_result>))
  "Converts a ROS message object to a list"
  (cl:list 'tracking_reid_result
    (cl:cons ':accompanying_frame (accompanying_frame msg))
    (cl:cons ':target_person (target_person msg))
))
