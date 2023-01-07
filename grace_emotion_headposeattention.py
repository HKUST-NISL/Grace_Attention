import rospy

import grace_attn_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import cv2
from cv_bridge import CvBridge
from MOS_HSEmotion import grace_emotion_attention

class GraceEmotionHeadPoseAttention:
	node_name = "grace_emotion_headposeattention"
	cv_bridge = CvBridge()
	topic_queue_size = 100

	emotion_attention_target_person_input_topic = "/grace_proj/tracking_reid_output"
	emotion_attention_target_person_output_topic = "/grace_proj/emotion_attention_target_person_output_topic"

	def __init__(self):
		rospy.init_node(self.node_name)

		
		self.grace_emotion_attention_modules_pipepline = grace_emotion_attention.Pipeline()

		self.emotion_attention_target_person_input_sub = rospy.Subscriber(self.emotion_attention_target_person_input_topic, 
																			grace_attn_msgs.msg.TrackingReIDResult, 
																			self.__emotionAttentionTargetPersonMsgCallback, 
																			queue_size=self.topic_queue_size)
		self.emotion_attention_target_person_output_pub = rospy.Publisher(self.emotion_attention_target_person_output_topic, 
																   grace_attn_msgs.msg.EmotionAttentionResult, 
																   queue_size=self.topic_queue_size)

	def __cv2ImgAsRosMsg(self,cv2_img_bgr8):
		#Convert the image of interest into ros color format
		cv2_img_rgb8 = cv2.cvtColor(cv2_img_bgr8,cv2.COLOR_BGR2RGB)
		#Convert the image of interest into a ros img msg
		ros_img_msg = self.cv_bridge.cv2_to_imgmsg(
				cv2_img_rgb8,#img of the target person in standard ros-rgb8 format
				'rgb8')
		return ros_img_msg

	def __emotionAttentionTargetPersonMsgCallback(self, msg):
		#Change ros frame to cv2 frame
		input_frame = self.cv_bridge.imgmsg_to_cv2(msg.accompanying_frame)
		
		#Array of ros integer type
		target_bbox_ros_msg = msg.bounding_box
		if(len(target_bbox_ros_msg) == 4):
			target_bbox = [
				target_bbox_ros_msg[0].data,
				target_bbox_ros_msg[1].data,
				target_bbox_ros_msg[2].data,
				target_bbox_ros_msg[3].data
				]
			# print(target_bbox)

			#Do inference
			res = grace_emotion_attention_modules_pipepline.infer(input_frame, target_bbox)

			# # Fake output for testing
			# res = {'attention': 1, 'emotion': 2, 'vis': input_frame}


			# Compose output message
			emotion_attention_msg_to_pub = grace_attn_msgs.msg.EmotionAttentionResult()
			emotion_attention_msg_to_pub.attention = std_msgs.msg.Int64(res["attention"])
			emotion_attention_msg_to_pub.emotion = std_msgs.msg.Int64(res["emotion"])
			emotion_attention_msg_to_pub.visualization_frame = self.__cv2ImgAsRosMsg(res["vis"])
			self.emotion_attention_target_person_output_pub.publish(emotion_attention_msg_to_pub)
		else:
			print("No target bounding box.")



if __name__ == '__main__':
	test_instance = GraceEmotionHeadPoseAttention()
	rospy.spin()