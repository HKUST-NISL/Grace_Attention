import argparse
import sys
import threading

import rospy
import std_msgs.msg
import hr_msgs.msg
import geometry_msgs.msg
import dynamic_reconfigure.client


class GraceAttention:
	node_name = "grace_attention_node"

	hr_perception_topic = "/hr/perception/people"
	topic_queue_size = 100

	dynamic_reconfig_request_timeout = 0.1

	hr_CAM_cfg_server = "/hr/perception/camera_angle"
	grace_chest_cam_motor_angle = 0.7

	hr_ATTN_cfg_server = "/hr/behavior/attention"
	hr_ATTN_patient_id = "patient"
	hr_ATTN_reconfig_interval = 0.25
	hr_ATTN_timeout = 1.0

	def __init__(self):
		rospy.init_node(self.node_name)
		rospy.Subscriber(self.hr_perception_topic,hr_msgs.msg.People,self.peoplePerceptionCallback, queue_size=self.topic_queue_size)
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

	def peoplePerceptionCallback(self,hr_people_msg):
		persons_detected = hr_people_msg.people
		#Decide attention target based on perception input
		self.decideCurrentTarget(persons_detected)

	#Height threshold 
	height_threshold = 100
	#A customized hr_msgs/person msg for Grace to attend to the patient 
	current_patient_msg = hr_msgs.msg.Person()
	current_raw_id = ""#the raw id from which the current patient message is constructed 
	raw_id_changed = False#for logging
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

	def configureGraceATTNCallback(self, config):
		# rospy.loginfo("Config set to {look_at_face}, {look_at_time}, {look_at_start}".format(**config))
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

