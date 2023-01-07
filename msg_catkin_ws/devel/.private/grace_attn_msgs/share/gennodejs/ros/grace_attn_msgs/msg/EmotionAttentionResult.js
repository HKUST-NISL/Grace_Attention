// Auto-generated. Do not edit!

// (in-package grace_attn_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class EmotionAttentionResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.attention = null;
      this.emotion = null;
      this.visualization_frame = null;
    }
    else {
      if (initObj.hasOwnProperty('attention')) {
        this.attention = initObj.attention
      }
      else {
        this.attention = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('emotion')) {
        this.emotion = initObj.emotion
      }
      else {
        this.emotion = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('visualization_frame')) {
        this.visualization_frame = initObj.visualization_frame
      }
      else {
        this.visualization_frame = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EmotionAttentionResult
    // Serialize message field [attention]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.attention, buffer, bufferOffset);
    // Serialize message field [emotion]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.emotion, buffer, bufferOffset);
    // Serialize message field [visualization_frame]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.visualization_frame, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EmotionAttentionResult
    let len;
    let data = new EmotionAttentionResult(null);
    // Deserialize message field [attention]
    data.attention = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [emotion]
    data.emotion = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [visualization_frame]
    data.visualization_frame = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.visualization_frame);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'grace_attn_msgs/EmotionAttentionResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fb0f00a6f3b056b562935c8def10f302';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int64 attention
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EmotionAttentionResult(null);
    if (msg.attention !== undefined) {
      resolved.attention = std_msgs.msg.Int64.Resolve(msg.attention)
    }
    else {
      resolved.attention = new std_msgs.msg.Int64()
    }

    if (msg.emotion !== undefined) {
      resolved.emotion = std_msgs.msg.Int64.Resolve(msg.emotion)
    }
    else {
      resolved.emotion = new std_msgs.msg.Int64()
    }

    if (msg.visualization_frame !== undefined) {
      resolved.visualization_frame = sensor_msgs.msg.Image.Resolve(msg.visualization_frame)
    }
    else {
      resolved.visualization_frame = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

module.exports = EmotionAttentionResult;
