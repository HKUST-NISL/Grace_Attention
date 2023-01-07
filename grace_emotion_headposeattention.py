import rospy

import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg

from cv_bridge import CvBridge

from MOS_HSEmotion import grace_emotion_attention

class GraceEmotionHeadPoseAttention:
    node_name = "grace_emotion_headposeattention"
	cv_bridge = CvBridge()
    topic_queue_size = 100

    emotion_attention_target_person_input_topic = "/grace_proj/emotion_attention_target_person_input_topic"
	# emotion_attention_accompanying_frame_input_topic = "/grace_proj/target_img"
	# emotion_attention_annotated_frame_output_topic = "/grace_proj/emotion_attention_annotated_frame_output_topic"
	emotion_attention_target_person_output_topic = "/grace_proj/emotion_attention_target_person_output_topic"

    def __init__(self):
        rospy.init_node(self.node_name)

        self.grace_emotion_attention_modules_pipepline = grace_emotion_attention.Pipeline()
		self.emotion_attention_target_person_input_sub = rospy.Subscriber(self.emotion_attention_target_person_input_topic, 
																	hr_msgs.msg.Person, 
																	self.__emotionAttentionTargetPersonMsgCallback, 
																	queue_size=self.topic_queue_size)
		# self.emotion_attention_accompanying_frame_sub = rospy.Subscriber(self.emotion_attention_accompanying_frame_input_topic, 
		# 																 sensor_msgs.msg.Image,
		# 																 self.__emotionAttentionAccompanyingFrameMsgCallback, 
		# 																 queue_size=self.topic_queue_size)
		# self.emotion_attention_annotated_frame_pub = rospy.Publisher(self.emotion_attention_annotated_frame_output_topic, 
		# 															 sensor_msgs.msg.Image, 
		# 															 queue_size=self.topic_queue_size)
		self.emotion_attention_target_person_output_pub = rospy.Publisher(self.emotion_attention_target_person_output_topic, 
																   hr_msgs.msg.Person, 
																   queue_size=self.topic_queue_size)
    
    def __emotionAttentionTargetPersonMsgCallback(self, msg):
        data = msg.data
        input_frame = self.cv_bridge.imgmsg_to_cv2(
			data.img,
			"bgr8")
        target_bbox = data.bbox
        res = grace_emotion_attention_modules_pipepline.infer(input_frame, target_box)

        # Use similar custom msg 
        self.emotion_attention_target_person_output_pub.publish(out_msg)