;; Auto-generated. Do not edit!


(when (boundp 'grace_attn_msgs::TrackingReIDResult)
  (if (not (find-package "GRACE_ATTN_MSGS"))
    (make-package "GRACE_ATTN_MSGS"))
  (shadow 'TrackingReIDResult (find-package "GRACE_ATTN_MSGS")))
(unless (find-package "GRACE_ATTN_MSGS::TRACKINGREIDRESULT")
  (make-package "GRACE_ATTN_MSGS::TRACKINGREIDRESULT"))

(in-package "ROS")
;;//! \htmlinclude TrackingReIDResult.msg.html
(if (not (find-package "HR_MSGS"))
  (ros::roseus-add-msgs "hr_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass grace_attn_msgs::TrackingReIDResult
  :super ros::object
  :slots (_accompanying_frame _bounding_box _target_person ))

(defmethod grace_attn_msgs::TrackingReIDResult
  (:init
   (&key
    ((:accompanying_frame __accompanying_frame) (instance sensor_msgs::Image :init))
    ((:bounding_box __bounding_box) ())
    ((:target_person __target_person) (instance hr_msgs::Person :init))
    )
   (send-super :init)
   (setq _accompanying_frame __accompanying_frame)
   (setq _bounding_box __bounding_box)
   (setq _target_person __target_person)
   self)
  (:accompanying_frame
   (&rest __accompanying_frame)
   (if (keywordp (car __accompanying_frame))
       (send* _accompanying_frame __accompanying_frame)
     (progn
       (if __accompanying_frame (setq _accompanying_frame (car __accompanying_frame)))
       _accompanying_frame)))
  (:bounding_box
   (&rest __bounding_box)
   (if (keywordp (car __bounding_box))
       (send* _bounding_box __bounding_box)
     (progn
       (if __bounding_box (setq _bounding_box (car __bounding_box)))
       _bounding_box)))
  (:target_person
   (&rest __target_person)
   (if (keywordp (car __target_person))
       (send* _target_person __target_person)
     (progn
       (if __target_person (setq _target_person (car __target_person)))
       _target_person)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _accompanying_frame
    (send _accompanying_frame :serialization-length)
    ;; std_msgs/Int64[] _bounding_box
    (apply #'+ (send-all _bounding_box :serialization-length)) 4
    ;; hr_msgs/Person _target_person
    (send _target_person :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _accompanying_frame
       (send _accompanying_frame :serialize s)
     ;; std_msgs/Int64[] _bounding_box
     (write-long (length _bounding_box) s)
     (dolist (elem _bounding_box)
       (send elem :serialize s)
       )
     ;; hr_msgs/Person _target_person
       (send _target_person :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _accompanying_frame
     (send _accompanying_frame :deserialize buf ptr-) (incf ptr- (send _accompanying_frame :serialization-length))
   ;; std_msgs/Int64[] _bounding_box
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_box (let (r) (dotimes (i n) (push (instance std_msgs::Int64 :init) r)) r))
     (dolist (elem- _bounding_box)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; hr_msgs/Person _target_person
     (send _target_person :deserialize buf ptr-) (incf ptr- (send _target_person :serialization-length))
   ;;
   self)
  )

(setf (get grace_attn_msgs::TrackingReIDResult :md5sum-) "26b9b313443164cd67c804c21e7d603c")
(setf (get grace_attn_msgs::TrackingReIDResult :datatype-) "grace_attn_msgs/TrackingReIDResult")
(setf (get grace_attn_msgs::TrackingReIDResult :definition-)
      "sensor_msgs/Image accompanying_frame
std_msgs/Int64[] bounding_box #[width_1,height_1,width_2,height_2]
hr_msgs/Person target_person
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: hr_msgs/Person
std_msgs/Header header
string id
hr_msgs/Face face
hr_msgs/Body body
================================================================================
MSG: hr_msgs/Face
std_msgs/Header header
string id
geometry_msgs/Pose left_gaze
geometry_msgs/Pose right_gaze
geometry_msgs/Vector3 gaze_angle
geometry_msgs/Point location
geometry_msgs/Pose head_pose
geometry_msgs/Point[] landmarks
hr_msgs/FacialActionUnit[] action_units
sensor_msgs/RegionOfInterest bounding_box

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: hr_msgs/FacialActionUnit
string name
float64 presence
float64 intensity
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify

================================================================================
MSG: hr_msgs/Body
std_msgs/Header header
string id
geometry_msgs/Point location
geometry_msgs/Point[] landmarks
string[] landmarks_names
sensor_msgs/RegionOfInterest bounding_box

")



(provide :grace_attn_msgs/TrackingReIDResult "26b9b313443164cd67c804c21e7d603c")


