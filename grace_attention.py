import argparse
import sys
import threading

import rospy
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import geometry_msgs.msg
import dynamic_reconfigure.client
from cv_bridge import CvBridge
import cv2
from copy import deepcopy

class GraceAttention:
	node_name = "grace_attention_node"

	bridge = CvBridge()

	hr_perception_topic = "/hr/perception/people"
	hr_img_topic = "/hr/sensors/vision/realsense/camera/color/image_raw"
	topic_queue_size = 100

	dynamic_reconfig_request_timeout = 0.1
	hr_ATTN_reconfig_interval = 0.25

	hr_CAM_cfg_server = "/hr/perception/camera_angle"
	hr_ATTN_cfg_server = "/hr/behavior/attention"
	grace_chest_cam_motor_angle = 0.15
	hr_ATTN_patient_id = "patient"
	hr_ATTN_timeout = 1.0

	latest_people_msg = None
	latest_img_msg = None
	fallback_patient_msg = None


	def __init__(self):
		rospy.init_node(self.node_name)
		rospy.Subscriber(self.hr_perception_topic,hr_msgs.msg.People,self.peoplePerceptionCallback, queue_size=self.topic_queue_size)
		rospy.Subscriber(self.hr_img_topic, sensor_msgs.msg.Image,self.chestCamRGBImgCallback, queue_size=self.topic_queue_size)
		self.hr_people_pub = rospy.Publisher(self.hr_perception_topic, hr_msgs.msg.People, queue_size=self.topic_queue_size)
		self.dynamic_CAM_cfg_client = dynamic_reconfigure.client.Client(self.hr_CAM_cfg_server, timeout=self.dynamic_reconfig_request_timeout, config_callback=self.configureGraceCAMCallback)
		self.dynamic_ATTN_cfg_client = dynamic_reconfigure.client.Client(self.hr_ATTN_cfg_server, timeout=self.dynamic_reconfig_request_timeout, config_callback=self.configureGraceATTNCallback)

	def initChestCam(self):
		#tilt chest cam to a pre-defined angle
		self.dynamic_CAM_cfg_client.update_configuration({"motor_angle":self.grace_chest_cam_motor_angle})
	
	def configureGraceCAMCallback(self,config):
		# # hr sdk seems to be repeatedly throwing back this response
		# rospy.loginfo("Config set to {motor_angle}".format(**config))
		pass
	
	def configureGraceATTNCallback(self, config):
		# rospy.loginfo("Config set to {look_at_face}, {look_at_time}, {look_at_start}".format(**config))
		pass

	def peoplePerceptionCallback(self,people_msg):
		self.latest_people_msg = people_msg

	def chestCamRGBImgCallback(self, image_msg):
		self.latest_img_msg = image_msg
	
	def configureGraceATTN(self):
		#A separate asynchronous thread for configuring Grace to attend to the target
		while(True):
			#Sleep
			rospy.sleep(self.hr_ATTN_reconfig_interval)

			#Check input availability
			if((not (self.latest_people_msg is None)) and (not (self.latest_img_msg is None)) ):
				# print("Configuring attention using latest perception input.")
				
				#Make a deep copy of the person and the image message	
				people_copy = deepcopy(self.latest_people_msg)
				img_copy = deepcopy(self.latest_img_msg)
				
				if(len(people_copy.people) > 0):#if people detected
					#Convert the image
					try:
						img_cv = self.bridge.imgmsg_to_cv2(img_copy, desired_encoding='bgr8')				
					except CvBridgeError as e:
						print(e)
						continue

					#Retrieve the bounding boxes and crop the image
					cropped_imgs = []
					for p in people_copy.people:

						x_offset = p.body.bounding_box.x_offset
						width = p.body.bounding_box.width
						y_offset = p.body.bounding_box.y_offset
						height = p.body.bounding_box.height

						print(p.body.bounding_box)

						cropped_img = img_cv[y_offset : y_offset + height, x_offset : x_offset + width]
						cropped_imgs.append(cropped_img)
						#For debugging
						cv2.imshow("cropped body img", cropped_img)
						cv2.waitKey(10)

			else:
				# print("No person and / or rgb msgs right now.")
				pass


if __name__ == '__main__':

	# parser=argparse.ArgumentParser()
	# parser.add_argument("--language_code", help="English: en-US Cantonese: yue-Hant-HK")
	# parser.add_argument("--topic", help="/grace_chat")
	# args=parser.parse_args()

	grace_attention = GraceAttention()
	grace_attention.initChestCam()
	threading._start_new_thread(grace_attention.configureGraceATTN())

	rospy.spin()










#Deprecated
'''
def decideCurrentTarget(self, persons_detected):
	#Find potential targets
	potential_targets = []#person messages
	potential_targets_id = []#corresponding raw id
	for i in range(0,len(persons_detected)):
		#Filter out targets by performing height thresholding on the patient's body
		#note that we have to skip our customized patient message
		if(persons_detected[i].id != self.hr_ATTN_patient_id):
			if(persons_detected[i].body.location.z <= self.height_threshold):
				potential_targets.append(persons_detected[i])
				potential_targets_id.append(persons_detected[i].id)
	
	#Choose target to use as the patient message
	if(len(potential_targets)>0):
		#If there are some potential targets, 
		new_target_index = -1
		try:
			#If the target used for constructing the previous patient message
			#is still preset, no need to swith target, just update message conten with this target
			new_target_index = potential_targets_id.index(self.current_raw_id)
			self.raw_id_changed = False
		except:
			#if the target used for constructing the previous patient message
			#is no-longer visible, switch to some other target and update patient message content
			new_target_index = 0
			self.raw_id_changed = True
		self.current_raw_id = potential_targets_id[new_target_index]
		self.current_patient_msg = potential_targets[new_target_index]
		self.current_patient_msg.id = self.hr_ATTN_patient_id
	else:
		#If no potential targets found, stick with the old patient message and raw id
		self.raw_id_changed = False
		pass
	
	#Logging
	if(self.raw_id_changed):
		print("New target patient's raw id is %s" % (self.current_raw_id[-3:]))
		print("Accompanying face id is %s" % (self.current_patient_msg.face.id[-3:]))
'''


'''
def configureGraceATTN(self):
	#A separate asynchronous thread for configuring Grace to attend to the target
	while(True):
		#Publish the latest patient message 
		hr_people_msg = hr_msgs.msg.People()
		hr_people_msg.people.append(self.current_patient_msg)
		self.hr_people_pub.publish(hr_people_msg)
		#Dynamic reconfigure to look at the patient
		self.dynamic_ATTN_cfg_client.update_configuration({"look_at_face":self.hr_ATTN_patient_id, "look_at_start":True, "look_at_time":self.hr_ATTN_timeout})
		rospy.sleep(self.hr_ATTN_reconfig_interval)
'''
