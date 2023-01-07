;; Auto-generated. Do not edit!


(when (boundp 'grace_attn_msgs::EmotionAttentionResult)
  (if (not (find-package "GRACE_ATTN_MSGS"))
    (make-package "GRACE_ATTN_MSGS"))
  (shadow 'EmotionAttentionResult (find-package "GRACE_ATTN_MSGS")))
(unless (find-package "GRACE_ATTN_MSGS::EMOTIONATTENTIONRESULT")
  (make-package "GRACE_ATTN_MSGS::EMOTIONATTENTIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude EmotionAttentionResult.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass grace_attn_msgs::EmotionAttentionResult
  :super ros::object
  :slots (_attention _emotion _visualization_frame ))

(defmethod grace_attn_msgs::EmotionAttentionResult
  (:init
   (&key
    ((:attention __attention) (instance std_msgs::Int64 :init))
    ((:emotion __emotion) (instance std_msgs::Int64 :init))
    ((:visualization_frame __visualization_frame) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _attention __attention)
   (setq _emotion __emotion)
   (setq _visualization_frame __visualization_frame)
   self)
  (:attention
   (&rest __attention)
   (if (keywordp (car __attention))
       (send* _attention __attention)
     (progn
       (if __attention (setq _attention (car __attention)))
       _attention)))
  (:emotion
   (&rest __emotion)
   (if (keywordp (car __emotion))
       (send* _emotion __emotion)
     (progn
       (if __emotion (setq _emotion (car __emotion)))
       _emotion)))
  (:visualization_frame
   (&rest __visualization_frame)
   (if (keywordp (car __visualization_frame))
       (send* _visualization_frame __visualization_frame)
     (progn
       (if __visualization_frame (setq _visualization_frame (car __visualization_frame)))
       _visualization_frame)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int64 _attention
    (send _attention :serialization-length)
    ;; std_msgs/Int64 _emotion
    (send _emotion :serialization-length)
    ;; sensor_msgs/Image _visualization_frame
    (send _visualization_frame :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int64 _attention
       (send _attention :serialize s)
     ;; std_msgs/Int64 _emotion
       (send _emotion :serialize s)
     ;; sensor_msgs/Image _visualization_frame
       (send _visualization_frame :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int64 _attention
     (send _attention :deserialize buf ptr-) (incf ptr- (send _attention :serialization-length))
   ;; std_msgs/Int64 _emotion
     (send _emotion :deserialize buf ptr-) (incf ptr- (send _emotion :serialization-length))
   ;; sensor_msgs/Image _visualization_frame
     (send _visualization_frame :deserialize buf ptr-) (incf ptr- (send _visualization_frame :serialization-length))
   ;;
   self)
  )

(setf (get grace_attn_msgs::EmotionAttentionResult :md5sum-) "fb0f00a6f3b056b562935c8def10f302")
(setf (get grace_attn_msgs::EmotionAttentionResult :datatype-) "grace_attn_msgs/EmotionAttentionResult")
(setf (get grace_attn_msgs::EmotionAttentionResult :definition-)
      "std_msgs/Int64 attention
std_msgs/Int64 emotion
sensor_msgs/Image visualization_frame
================================================================================
MSG: std_msgs/Int64
int64 data
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

")



(provide :grace_attn_msgs/EmotionAttentionResult "fb0f00a6f3b056b562935c8def10f302")


