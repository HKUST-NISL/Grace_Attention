import argparse
import sys
import threading
import time
import numpy
from signal import signal
from signal import SIGINT
import rospy
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import geometry_msgs.msg
import dynamic_reconfigure.client
from cv_bridge import CvBridge
import cv2
from copy import deepcopy
from .Yolov5_StrongSORT_OSNet import grace_track
import os

POSE_CHAIN = [
	("nose", "leftEye"), ("leftEye", "leftEar"), ("nose", "rightEye"),
	("rightEye", "rightEar"), ("nose", "leftShoulder"),
	("leftShoulder", "leftElbow"), ("leftElbow", "leftWrist"),
	("leftShoulder", "leftHip"), ("leftHip", "leftKnee"),
	("leftKnee", "leftAnkle"), ("nose", "rightShoulder"),
	("rightShoulder", "rightElbow"), ("rightElbow", "rightWrist"),
	("rightShoulder", "rightHip"), ("rightHip", "rightKnee"),
	("rightKnee", "rightAnkle")
]

def drawPose(img, person,  color=(0,255,0),diameter=4, line_width = 2):
	#Draw landmark points
	for i in range(len(person.body.landmarks)):
		lm_pos = person.body.landmarks[i]
		cv2.circle(img, (int(lm_pos.x), int(lm_pos.y)), diameter , color, -1)
	#Draw landmark links
	for line in POSE_CHAIN:
		try:
			idx_first = person.body.landmarks_names.index(line[0])
			pos_first = person.body.landmarks[idx_first]

			idx_second = person.body.landmarks_names.index(line[1])
			pos_second = person.body.landmarks[idx_second]

			cv2.line(img, (int(pos_first.x), int(pos_first.y)),\
				(int(pos_second.x), int(pos_second.y)), color, line_width)
		except Exception as e:
			pass

class GraceAttention:
	node_name = "grace_attention"

	bridge = CvBridge()

	hr_perception_topic = "/hr/perception/people"
	hr_img_topic = "/hr/sensors/vision/realsense/camera/color/image_raw"
	topic_queue_size = 100

	hr_ATTN_reconfig_rate = 1 #Hz
	dynamic_reconfig_request_timeout = 0.5

	hr_CAM_cfg_server = "/hr/perception/camera_angle"
	hr_ATTN_cfg_server = "/hr/behavior/attention"
	grace_chest_cam_motor_angle = 0.15
	hr_ATTN_patient_id = "patient"
	hr_ATTN_timeout = 2.0

	tracker_polling_rate = 30#Hz

	inerested_source_idx = 0	#Assume we only use source 0
	latest_people_msg = None
	
	attn_id_now = None


	def __init__(self):
		#ROS IO
		rospy.init_node(self.node_name)
		rospy.Subscriber(self.hr_perception_topic,hr_msgs.msg.People,self.__peoplePerceptionCallback, queue_size=self.topic_queue_size)
		rospy.Subscriber(self.hr_img_topic, sensor_msgs.msg.Image,self.__chestCamRGBImgCallback, queue_size=self.topic_queue_size)
		self.hr_people_pub = rospy.Publisher(self.hr_perception_topic, hr_msgs.msg.People, queue_size=self.topic_queue_size)
		self.dynamic_CAM_cfg_client = dynamic_reconfigure.client.Client(self.hr_CAM_cfg_server, timeout=self.dynamic_reconfig_request_timeout, config_callback=self.__configureGraceCAMCallback)
		self.dynamic_ATTN_cfg_client = dynamic_reconfigure.client.Client(self.hr_ATTN_cfg_server, timeout=self.dynamic_reconfig_request_timeout, config_callback=self.__configureGraceATTNCallback)
		
		#Tracking module initialization
		self.grace_tracker = grace_track.GraceTracker(
			self.__trackingCallBack,
			[self.hr_img_topic],
			"0")

	def mainThread(self):
		rate = rospy.Rate(self.hr_ATTN_reconfig_rate)
		while(True):
			self.__configureGraceATTN()
			rate.sleep()

	def runAttention(self, visualize, annotate):
		self.attention_vis = visualize
		self.annotate_img = annotate
		self.__initChestCam()

		#Debug: For source 0, use a test image
		test_img_path = os.path.join(grace_track.ROOT,'1669796149240.jpg')
		self.grace_tracker.registerQueryImage(0, cv2.imread(test_img_path))

		self.grace_tracker.startTracking(self.tracker_polling_rate, False, annotate)#Visualize at attention module level

	def __initChestCam(self):
		#tilt chest cam to a pre-defined angle
		self.dynamic_CAM_cfg_client.update_configuration({"motor_angle":self.grace_chest_cam_motor_angle})
	
	def __configureGraceCAMCallback(self,config):
		# # hr sdk seems to be repeatedly throwing back this response
		# rospy.loginfo("Config set to {motor_angle}".format(**config))
		pass
	
	def __configureGraceATTNCallback(self, config):
		# rospy.loginfo("Config set to {look_at_face}, {look_at_time}, {look_at_start}".format(**config))
		pass

	def __peoplePerceptionCallback(self,people_msg):
		self.latest_people_msg = people_msg

	def __chestCamRGBImgCallback(self, image_msg):
		pass

	def __configureGraceATTN(self):
		if(self.attn_id_now is not None):
			self.dynamic_ATTN_cfg_client.update_configuration({"look_at_face":self.attn_id_now, "look_at_start":True, "look_at_time":self.hr_ATTN_timeout})

	def __trackingCallBack(self, tracking_results, target_idx_among_tracked_obj, annotated_frames):
		#For now we assume only one of the sources are of interest
		if(
			tracking_results[self.inerested_source_idx] is not None 
			and 
			target_idx_among_tracked_obj[self.inerested_source_idx] is not None
			and
			len(self.latest_people_msg.people) > 0
		):
			t0 = time.time()
			#Check which person detected best-matches the target found by the tracker
			#we do this by counting landmarks within a bounding box
			x0,y0,x1,y1 = tracking_results[self.inerested_source_idx][target_idx_among_tracked_obj[self.inerested_source_idx]].getBBox()
			n_people = len(self.latest_people_msg.people)
			landmark_matching_cnt = [0] * n_people
			for i in range(n_people):
				person = self.latest_people_msg.people[i]
				for lm in person.body.landmarks:
					if(lm.x >= x0 and lm.x <= x1 and lm.y >= y0 and lm.y <= y1):
						landmark_matching_cnt[i] = landmark_matching_cnt[i] + 1
			sort_idx = numpy.argsort(numpy.array(landmark_matching_cnt))
			best_match_idx = sort_idx[-1]
			best_match_person = self.latest_people_msg.people[best_match_idx]
			t1 = time.time()

			print("[Attention Module]: pose-binding took %.3f seconds." % (t1 - t0))

			#Further annotate the image by drawing pose tree of the matching target
			if(self.annotate_img):
				drawPose(annotated_frames[self.inerested_source_idx],best_match_person)

			#Configure grace to look at this person
			self.attn_id_now = best_match_person.id
		else:
			self.attn_id_now = None

		#Show the annotated img of the source of interest
		if(self.attention_vis):
			cv2.imshow("Attention View",annotated_frames[self.inerested_source_idx])
			cv2.waitKey(1)

def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()

if __name__ == '__main__':

	# parser=argparse.ArgumentParser()
	# parser.add_argument("--language_code", help="English: en-US Cantonese: yue-Hant-HK")
	# parser.add_argument("--topic", help="/grace_chat")
	# args=parser.parse_args()

	grace_attention = GraceAttention()
	grace_attention.runAttention(True,True)

	signal(SIGINT, handle_sigint)
	grace_attention.mainThread()
	









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
def __configureGraceATTN(self):
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
