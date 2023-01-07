// Auto-generated. Do not edit!

// (in-package grace_attn_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let hr_msgs = _finder('hr_msgs');

//-----------------------------------------------------------

class TrackingReIDResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accompanying_frame = null;
      this.target_person = null;
    }
    else {
      if (initObj.hasOwnProperty('accompanying_frame')) {
        this.accompanying_frame = initObj.accompanying_frame
      }
      else {
        this.accompanying_frame = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('target_person')) {
        this.target_person = initObj.target_person
      }
      else {
        this.target_person = new hr_msgs.msg.Person();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackingReIDResult
    // Serialize message field [accompanying_frame]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.accompanying_frame, buffer, bufferOffset);
    // Serialize message field [target_person]
    bufferOffset = hr_msgs.msg.Person.serialize(obj.target_person, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackingReIDResult
    let len;
    let data = new TrackingReIDResult(null);
    // Deserialize message field [accompanying_frame]
    data.accompanying_frame = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_person]
    data.target_person = hr_msgs.msg.Person.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.accompanying_frame);
    length += hr_msgs.msg.Person.getMessageSize(object.target_person);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'grace_attn_msgs/TrackingReIDResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '23bd691bb2b76cf204aca0c66b104aa1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image accompanying_frame
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
    
    # True if a distinct rectified ROI should be calculated from the "raw"
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackingReIDResult(null);
    if (msg.accompanying_frame !== undefined) {
      resolved.accompanying_frame = sensor_msgs.msg.Image.Resolve(msg.accompanying_frame)
    }
    else {
      resolved.accompanying_frame = new sensor_msgs.msg.Image()
    }

    if (msg.target_person !== undefined) {
      resolved.target_person = hr_msgs.msg.Person.Resolve(msg.target_person)
    }
    else {
      resolved.target_person = new hr_msgs.msg.Person()
    }

    return resolved;
    }
};

module.exports = TrackingReIDResult;
