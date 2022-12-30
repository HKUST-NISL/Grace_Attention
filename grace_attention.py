import logging
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
import rosbag
import geometry_msgs.msg
import dynamic_reconfigure.client
from cv_bridge import CvBridge
import cv2
from copy import deepcopy
from .Yolov5_StrongSORT_OSNet import grace_track
from .Yolov5_StrongSORT_OSNet.yolov5.utils.loggers import LOGGER
import os
from tkinter import *
import PIL.Image, PIL.ImageTk

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
	cv_bridge = CvBridge()

	#Emergency stop
	stop_topic = "/grace_proj/stop"

	#Camera Angling
	dynamic_reconfig_request_timeout = 0.5
	hr_CAM_cfg_server = "/hr/perception/camera_angle"
	grace_chest_cam_motor_angle = 0.15

	#Tracking & Gaze Attention
	hr_face_target_topic = "/hr/animation/set_face_target"
	hr_people_perception_topic = "/hr/perception/people"
	hr_img_topic = "/hr/sensors/vision/realsense/camera/color/image_raw"
	topic_queue_size = 100
	latest_people_msg = None
	tracking_start_people_msg = None
	debug_img = None
	hr_ATTN_cfg_server = "/hr/behavior/attention"
	hr_ATTN_timeout = 2.0
	attn_id_now = None
	target_person_msg = None
	# people_debug_bag = None
	tracker_polling_rate = 10#Hz
	inerested_source_idx = 0#Assume we only use source 0
	target_person_topic = "/grace_proj/target_person"
	target_img_topic = "/grace_proj/target_img"
	no_target_string = "None"
	manual_face_target_rate = 5#Hz

	#Aversion
	toggle_aversion_topic = "/grace_proj/toggle_aversion"
	aversion_enabled = False #Whether the random aversion is enabled
	is_gaze_averting = False #Whether it's averting at the moment
	gaze_aversion_start_time = None
	gaze_aversion_stop_time = None
	gaze_aversion_target_msg = None
	aversion_target_id = "gaze_aversion_target"
	aversion_loc_range = [0.1,0.3]
	aversion_mean_interval = 10#seconds
	aversion_duration_range = [1,2]#seconds
	aversion_minimal_interval = 3.0#Seconds
	aversion_thread_rate = 5#Hz
	tracking_text = "Tracking. "
	no_aversion_text = "Aversion disabled."
	averting_text = "Averting for"

	#Nodding
	toggle_nodding_topic = "/grace_proj/toggle_nodding"
	nodding_enabled = False
	hr_head_gesture_topic = "/hr/animation/set_animation"
	hr_nod_type_string = "nod_Short"
	hr_nod_variants_range = [1,7] #8 and 9 use lower case "s" in SHORT
	hr_nod_magnitude_range = [0.25,1.0]
	nodding_mean_interval = 12#Seconds
	nodding_minimal_interval = 3.0#Seconds
	noding_thread_rate = 5#Hz
	pend_nod_text = "Next nod in "
	no_nod_text = "Nodding disabled."

	#Miscellaneous	
	main_thread  = None
	main_thread_rate = 2 #Hz

	def __init__(self):
		# self.__targetATTNGui()


		#Thread safety
		self.people_msg_lock = threading.Lock()

		#ROS IO
		rospy.init_node(self.node_name)

		rospy.Subscriber(self.hr_people_perception_topic,hr_msgs.msg.People,self.__peoplePerceptionCallback, queue_size=self.topic_queue_size)
		self.people_pub = rospy.Publisher(self.hr_people_perception_topic, hr_msgs.msg.People, queue_size=self.topic_queue_size)
		# rospy.Subscriber(self.hr_img_topic, sensor_msgs.msg.Image,self.__chestCamRGBImgCallback, queue_size=self.topic_queue_size)
		self.hr_people_pub = rospy.Publisher(self.hr_people_perception_topic, hr_msgs.msg.People, queue_size=self.topic_queue_size)
		self.hr_face_target_pub = rospy.Publisher(self.hr_face_target_topic, hr_msgs.msg.Target, queue_size=self.topic_queue_size)
		self.target_person_pub = rospy.Publisher(self.target_person_topic, hr_msgs.msg.Person, queue_size=self.topic_queue_size)
		self.accompanying_frame_pub = rospy.Publisher(self.target_img_topic, sensor_msgs.msg.Image, queue_size=self.topic_queue_size)
		self.hr_head_gesture_pub = rospy.Publisher(self.hr_head_gesture_topic, hr_msgs.msg.SetAnimation, queue_size=self.topic_queue_size)
		self.stop_pub = rospy.Publisher(self.stop_topic, std_msgs.msg.Bool, queue_size=self.topic_queue_size)
		self.stop_sub = rospy.Subscriber(self.stop_topic, std_msgs.msg.Bool, self.__stopMsgCallback, queue_size=self.topic_queue_size)
		self.toggle_aversion_sub = rospy.Subscriber(self.toggle_aversion_topic, std_msgs.msg.Bool, self.__toggleAversionMsgCallback, queue_size=self.topic_queue_size)
		self.toggle_nodding_sub = rospy.Subscriber(self.toggle_nodding_topic, std_msgs.msg.Bool, self.__toggleNoddingMsgCallback, queue_size=self.topic_queue_size)
		self.dynamic_CAM_cfg_client = dynamic_reconfigure.client.Client(self.hr_CAM_cfg_server, timeout=self.dynamic_reconfig_request_timeout, config_callback=self.__configureGraceCAMCallback)
		self.dynamic_ATTN_cfg_client = dynamic_reconfigure.client.Client(self.hr_ATTN_cfg_server, timeout=self.dynamic_reconfig_request_timeout, config_callback=self.__configureGraceATTNCallback)

		#Tracking module initialization
		self.grace_tracker = grace_track.GraceTracker(
			self.__trackingIterationStartCallBack,
			self.__trackingIterationFinishCallBack,
			[self.hr_img_topic],
			"0")

	def __mainThread(self):
		rate = rospy.Rate(self.main_thread_rate)
		while(True):
			self.__configureGraceATTN()
			rate.sleep()

	def runAttention(self, visualize, annotate):
		self.attention_vis = visualize
		self.annotate_img = annotate
		self.__initChestCam()

		# #For debugging the people topic
		# self.people_debug_bag = rosbag.Bag('people_topic_debug.bag', 'w')

		#Start tracking thread (note that without registering a query image no actual re-indentification would be performed)
		self.grace_tracker.startTracking(self.tracker_polling_rate, False, annotate)#Visualize at attention module level


		#Start the gaze aversion thread
		self.grace_aversion_thread = threading.Thread(target = self.__aversionThread,daemon = False)
		self.grace_aversion_thread.start()

		#Start the nodding thread
		self.grace_nodding_thread = threading.Thread(target= self.__noddingThread, daemon= False)
		self.grace_nodding_thread.start()

		#Create the target reg text input gui
		self.target_reg_thread = threading.Thread(target = self.__targetATTNGui,daemon = False)
		self.target_reg_thread.start()

		#External face target
		self.face_target_thread = threading.Thread(target = self.__manualFaceTargetThread,daemon = False)
		self.face_target_thread.start()

		#Start main thread
		self.__mainThread()

	def __TargetATTNGuiClose(self):
		self.attention_ui_frame.destroy()

	def __targetATTNGui(self):
		#UI Frame
		self.attention_ui_frame = Tk()
		self.attention_ui_frame.title("Grace Attention GUI")
		self.attention_ui_frame.geometry('1920x1080')

		#STOP
		stop_btn = Button(self.attention_ui_frame,
							text = "STOP", 
							command = self.__stopBtnCallback)
		# stop_btn.pack()
		stop_btn.place(y=50, x=50)

		#Attention enabled state
		self.attention_enabled_tk = BooleanVar()
		enable_attention = Checkbutton(
			self.attention_ui_frame,
			text = "Enable Attention", 
			variable= self.attention_enabled_tk,
			onvalue = True, offvalue = False,
			command = self.__toggleGraceAttentionBtnCallback)
		enable_attention.place(y=50, x= 200)

		#Target Selection
		self.target_reg_input = Text(
			self.attention_ui_frame,
			height = 2,
			width = 10)
		self.target_reg_input.pack()
		self.target_reg_input.place(y=50, x= 650)

		confirm_target_reg = Button(self.attention_ui_frame,
								text = "REG", 
								command = self.__targetRegCallback)
		# confirm_target_reg.pack()
		confirm_target_reg.place(y=50, x= 550)

		#Aversion: enable, disable, state
		self.aversion_enabled_tk = BooleanVar()
		enableAversion = Checkbutton(
			self.attention_ui_frame,
			text = "Enable Aversion", 
			variable= self.aversion_enabled_tk,
			onvalue = True, offvalue = False,
			command = self.__toggleAversionBtnCallback)
		# enableAversion.pack()
		enableAversion.place(y=150, x=50)

		self.aversionStateText = Label(self.attention_ui_frame, text = self.tracking_text)
		# self.aversionStateText.pack()
		self.aversionStateText.place(y=150, x=400)

		#Nodding: enable, disable, state
		self.nodding_enabled_tk = BooleanVar()
		enableNodding = Checkbutton(
			self.attention_ui_frame,
			text = "ENABLE Nodding", 
			variable= self.nodding_enabled_tk,
			onvalue = True, offvalue = False,
			command = self.__toggleNoddingBtnCallback)
		# enableNodding.pack()
		enableNodding.place(y=250, x=50)
		
		self.noddingStateText = Label(self.attention_ui_frame, text = self.pend_nod_text)
		# self.noddingStateText.pack()
		self.noddingStateText.place(y=250, x=400)


		#Visualize the image of the target person
		self.source_0_target_img_canvas = Canvas(self.attention_ui_frame, width = 640, height = 480)
		blank_img = PIL.ImageTk.PhotoImage(image=PIL.Image.new("RGB", (640, 480)))
		self.source_0_target_img_container = self.source_0_target_img_canvas.create_image(0,0, anchor=NW, image=blank_img)
		# self.source_0_target_img_canvas.pack()
		self.source_0_target_img_canvas.place(y=350, x=750)


		#Visualize the image of the camera view
		self.source_0_img_canvas = Canvas(self.attention_ui_frame, width = 640, height = 480)
		blank_img = PIL.ImageTk.PhotoImage(image=PIL.Image.new("RGB", (640, 480)))
		self.source_0_img_container = self.source_0_img_canvas.create_image(0,0, anchor=NW, image=blank_img)
		# self.source_0_img_canvas.pack()
		self.source_0_img_canvas.place(y=350, x=50)
			
		#Stop all local functions in the beginning
		self.__stopAllLocal()

		#Blocks
		self.attention_ui_frame.mainloop()

	def __stopBtnCallback(self):
		#Broadcast a stop signal, which acts on the program itself as well
		self.stop_pub.publish(std_msgs.msg.Bool(True))
		
	def __stopAllLocal(self):
		try:
			self.__regQueryImg(None,self.inerested_source_idx)
			#Stop Grace attention
			self.attention_enabled_tk.set(False)
			self.__toggleGraceAttentionBtnCallback()
			#Stop aversion
			self.aversion_enabled_tk.set(False)
			self.__toggleAversionBtnCallback()
			#Stop nodding
			self.nodding_enabled_tk.set(False)
			self.__toggleNoddingBtnCallback()
			LOGGER.info("Everything stopped.")
		except Exception as e:
			LOGGER.error(e)

	def __stopMsgCallback(self, msg):
		if(msg.data):
			self.__stopAllLocal()

	def __toggleNoddingMsgCallback(self, msg):
		self.nodding_enabled_tk.set(msg.data)
		self.__toggleNoddingBtnCallback()

	def __toggleAversionMsgCallback(self, msg):
		self.aversion_enabled_tk.set(msg.data)
		self.__toggleAversionBtnCallback()

	def __targetRegCallback(self):
		try:
			target_raw_idx = int(self.target_reg_input.get(1.0, "end-1c"))
			source_0_tracked_obj = self.grace_tracker.getTrackedObjByRawIdx(target_raw_idx,0)
			if(source_0_tracked_obj is not None):
				self.source_0_target_img = source_0_tracked_obj.copyCroppedImg()
				self.__regQueryImg(self.source_0_target_img,self.inerested_source_idx)
				#Visualization
				self.source_0_target_img_corrected = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(cv2.cvtColor(self.source_0_target_img, cv2.COLOR_BGR2RGB)))
				self.source_0_target_img_canvas.itemconfig(self.source_0_target_img_container,image=self.source_0_target_img_corrected)
			else:
				LOGGER.warning("Failed to retrieve the tracked object corresponding to the raw idx %d." % (target_raw_idx))
				self.__regQueryImg(None,self.inerested_source_idx)#Clear target reg
		except Exception as e:
			print(e)
			self.__regQueryImg(None,self.inerested_source_idx)#Clear target reg

	def __regQueryImg(self, query_img , souorce_idx = 0):
		#Register new query target
		self.grace_tracker.registerQueryImage(souorce_idx, query_img)

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
		self.people_msg_lock.acquire()
		self.latest_people_msg = people_msg
		# #For debugging
		# print('people @ %f'%(time.time()))
		# if(self.debug_img is not None):
		# 	for person in self.latest_people_msg.people:
		# 		drawPose(self.debug_img,person)
		# 	cv2.imshow('debug',self.debug_img)
		# 	cv2.waitKey(1)
		
		# #For people message debugging
		# if(self.people_debug_bag is not None):
		# 	self.people_debug_bag.write(self.hr_people_perception_topic, people_msg)
		
		self.people_msg_lock.release()

	def __chestCamRGBImgCallback(self, image_msg):
		# #For debugging
		# print('img@ %f'%(time.time()))
		# self.debug_img =deepcopy(self.cv_bridge.imgmsg_to_cv2(image_msg,desired_encoding='bgr8'))
		pass

	def __configureGraceATTN(self):
		if(self.is_gaze_averting == False):
			#Attend to the target person normally
			#choose target
			if( self.attn_id_now is not None ):
				try:
					self.dynamic_ATTN_cfg_client.update_configuration({"look_at_face":self.attn_id_now, "look_at_start":True, "look_at_time":self.hr_ATTN_timeout})
					LOGGER.debug("Configuring grace attention on target %s." % (self.attn_id_now))
				except Exception as e:
					LOGGER.error(e)
		else:
			#Look at aversion target
			self.aversionStateText.config(text = self.averting_text)
			LOGGER.debug("Gaze Averting.")
			self.__publishGazeAversionTarget()
			try:
				self.dynamic_ATTN_cfg_client.update_configuration({"look_at_face":self.gaze_aversion_target_msg.id, "look_at_start":True, "look_at_time":self.hr_ATTN_timeout})
			except Exception as e:
				LOGGER.error(e)

	def __manualFaceTargetThread(self):
		rate = rospy.Rate(self.manual_face_target_rate)

		fake_target_in_space = hr_msgs.msg.Target()
		fake_target_in_space.x = 1.5
		fake_target_in_space.y = 0
		fake_target_in_space.x = 1.5
		fake_target_in_space.speed = 1.0

		while(True):
			try:
				if(not self.attention_enabled_tk.get()):
					#When automatic grace attention is disabled, we manually specify a fake face target
					#to make grace look disengaged
					self.hr_face_target_pub.publish(fake_target_in_space)
			except Exception as e:
				LOGGER.debug(e)

			rate.sleep()

	def __toggleGraceAttentionBtnCallback(self):
		#Toggle the attention functionality of Grace via dynamic reconfig
		try:
			self.dynamic_ATTN_cfg_client.update_configuration({"enable_flag":self.attention_enabled_tk.get()})
			LOGGER.info("Grace attention flag set to %s." % (self.attention_enabled_tk.get()))
		except Exception as e:
			LOGGER.error(e)

	def __toggleAversionBtnCallback(self):
		if(self.aversion_enabled_tk.get()):
			#Setup the first time stamp for aversion
			self.aversion_enabled = True
			self.__setupAversionTime(time.time())
			LOGGER.info("Aversion Enabled.")
		else:
			#Disable the aversion mechanism
			self.aversion_enabled = False
			LOGGER.info("Aversion Disabled.")

	def __aversionThread(self):
		rate = rospy.Rate(self.aversion_thread_rate)
		while(True):
			if(self.aversion_enabled):
				t = time.time()

				if(self.is_gaze_averting == False):
					if(t >= self.gaze_aversion_start_time):
						#Create a new aversion target
						self.__createGazeAversionTarget()
						#Indicate the averting state
						self.is_gaze_averting = True
					else:
						#text state display
						try:
							self.aversionStateText.config(
								text = self.tracking_text + "Next aversion in " + f"{max(0,self.gaze_aversion_start_time - t):.2f}")
						except Exception as e:
							LOGGER.debug(e)
				else:
					if(t >= self.gaze_aversion_end_time):
						#Stop the current averting act
						self.is_gaze_averting = False
						#Decide the timing for the next aversion
						self.__setupAversionTime(t)


				if(self.is_gaze_averting):
					#text state display
					try:
						self.aversionStateText.config(
							text = self.averting_text + f"{max(0,self.gaze_aversion_end_time - t):.2f}")
					except Exception as e:
						LOGGER.debug(e)
			else:
				self.is_gaze_averting = False
				#text state display
				try:
					self.aversionStateText.config(
						text = self.tracking_text + self.no_aversion_text)
				except Exception as e:
					LOGGER.debug(e)


			rate.sleep()

	def __setupAversionTime(self, ref_time):
		#Start time of aversion
		self.gaze_aversion_start_time = ref_time + max(self.aversion_minimal_interval,numpy.random.exponential(self.aversion_mean_interval))
		#Duration of aversion
		dur = numpy.random.uniform(self.aversion_duration_range[0],self.aversion_duration_range[1])
		#End time of aversion
		self.gaze_aversion_end_time = self.gaze_aversion_start_time + dur
		LOGGER.info("New aversion in %f seconds." % (self.gaze_aversion_start_time - ref_time))

	def __publishGazeAversionTarget(self):
		hr_people = hr_msgs.msg.People()
		hr_people.people.append(self.gaze_aversion_target_msg)
		self.hr_people_pub.publish(hr_people)

	def __createGazeAversionTarget(self):
		if(self.attn_id_now is not None):
			try:
				# # Offset around a person message
				self.gaze_aversion_target_msg = deepcopy(self.target_person_msg)
				self.gaze_aversion_target_msg.id = self.aversion_target_id

				#Perturb the location of the person to create a fake person for aversion
				old_point = self.target_person_msg.body.location
				new_point = geometry_msgs.msg.Point()
				new_point.x = old_point.x + 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
				new_point.y = old_point.y + 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
				new_point.z = old_point.z + 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
				# new_point.y = 0.5
				# new_point.x = 0.5
				# new_point.z = 0.5

				self.gaze_aversion_target_msg.body.location = new_point
			except Exception as e:
				LOGGER.error(e)
		else:
			self.gaze_aversion_target_msg = hr_msgs.msg.Person()
			self.gaze_aversion_target_msg.id = self.aversion_target_id
			
			new_point = geometry_msgs.msg.Point()
			new_point.x = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
			new_point.y = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
			new_point.z = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
			
			self.gaze_aversion_target_msg.body.location = new_point

	def __noddingThread(self):
		rate = rospy.Rate(self.noding_thread_rate)
		while(True):
			if(self.nodding_enabled):
				t = time.time()
				#Do nodding
				if(t>= self.nodding_time):
					#Compose and publish a nodding command
					nodding_gesture_cmd = hr_msgs.msg.SetAnimation()
					nodding_gesture_cmd.name = self.hr_nod_type_string + str(numpy.random.randint(self.hr_nod_variants_range[0],self.hr_nod_variants_range[1]+1))
					nodding_gesture_cmd.repeat = 1
					nodding_gesture_cmd.speed = 1.0
					nodding_gesture_cmd.magnitude = numpy.random.uniform(self.hr_nod_magnitude_range[0],self.hr_nod_magnitude_range[1])
					self.hr_head_gesture_pub.publish(nodding_gesture_cmd)

					#Setup the next nodding time
					self.__setupNoddingTime(t)
				else:
					try:
						self.noddingStateText.config(text=self.pend_nod_text + f"{max(0,self.nodding_time - t):.2f}")
					except Exception as e:
						print(e)
			else:
				try:
					self.noddingStateText.config(text=self.no_nod_text)
				except Exception as e:
					print(e)
			
			rate.sleep()

	def __toggleNoddingBtnCallback(self):
		if(self.nodding_enabled_tk.get()):
			self.nodding_enabled = True
			self.__setupNoddingTime(time.time())
			LOGGER.info("Nodding Enabled.")
		else:
			self.nodding_enabled = False
			LOGGER.info("Nodding Disabled.")

	def __setupNoddingTime(self, ref_time):
		#A minimal interval between two nodding
		self.nodding_time = ref_time + max(self.nodding_minimal_interval, numpy.random.exponential(self.nodding_mean_interval))

	def __writeBag(self):
		self.people_debug_bag.close()
		self.people_debug_bag = None

	def __trackingIterationStartCallBack(self):
		self.people_msg_lock.acquire()
		#keep a deep copy of the people msg at this instant
		self.tracking_start_people_msg = deepcopy(self.latest_people_msg)
		self.people_msg_lock.release()

	def __trackingIterationFinishCallBack(
		self, 
		tracking_results, 
		target_idx_among_tracked_obj, 
		raw_frames, 
		annotated_frames):		
		#For now we assume only one of the sources are of interest
		if(
			tracking_results[self.inerested_source_idx] is not None 
			and 
			target_idx_among_tracked_obj[self.inerested_source_idx] is not None
			and
			self.tracking_start_people_msg is not None
			and
			len(self.tracking_start_people_msg.people) > 0
		):
			t0 = time.time()
		
			#Check which person detected best-matches the target found by the tracker
			#we do this by counting landmarks within a bounding box
			x0,y0,x1,y1 = tracking_results[self.inerested_source_idx][target_idx_among_tracked_obj[self.inerested_source_idx]].getBBox()
			n_people = len(self.tracking_start_people_msg.people)
			
			landmark_matching_score = [0] * n_people
			for i in range(n_people):
				person = self.tracking_start_people_msg.people[i]

				if(person.id == self.aversion_target_id):
					return #Skip the people message with fake aversion target

				#Compute a score to relate tracking bounding box with pose from hrsdk
				#Use bounding box inclusion counting
				for lm in person.body.landmarks:
					if(lm.x >= x0 and lm.x <= x1 and lm.y >= y0 and lm.y <= y1):
							landmark_matching_score[i] = landmark_matching_score[i] + 1
				# #For debugging
				# drawPose(annotated_frames[self.inerested_source_idx],person,(0,255,0))

			t1 = time.time()
			LOGGER.debug("[Attention Module]: pose-binding took %.3f seconds." % (t1 - t0))

			sort_idx = numpy.argsort(numpy.array(landmark_matching_score))
			best_match_idx = sort_idx[-1]
			if(landmark_matching_score[best_match_idx] > 0):#At least one skeleton points need to fall inside the bounding box
				best_match_person = self.tracking_start_people_msg.people[best_match_idx]

				#Further annotate the image by drawing pose tree of the matching target
				if(self.annotate_img):
					drawPose(annotated_frames[self.inerested_source_idx],best_match_person,(0,0,255))

				#Configure grace to look at this person
				self.attn_id_now = best_match_person.id
				self.target_person_msg = deepcopy(best_match_person)
			else:
				self.attn_id_now = None
				self.target_person_msg = None
		else:
			self.attn_id_now = None
			self.target_person_msg = None

		#Drop the people message
		self.tracking_start_people_msg = None

		#Show the annotated img from the source of interest
		if(self.attention_vis):
			self.annotated_frame_of_interested_source = annotated_frames[self.inerested_source_idx]
			self.source_0_annotated_img_vis = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(cv2.cvtColor(self.annotated_frame_of_interested_source, cv2.COLOR_BGR2RGB)))
			self.source_0_img_canvas.itemconfig(self.source_0_img_container,image=self.source_0_annotated_img_vis)


		#Publish relevant information for the upcoming expression and gaze detection
		#Image frame used
		img_used = self.cv_bridge.cv2_to_imgmsg(raw_frames[self.inerested_source_idx])
		self.accompanying_frame_pub.publish(img_used)
		#People msg
		target_person_msg = hr_msgs.msg.Person()
		if(self.target_person_msg is None):
			target_person_msg.id = self.no_target_string
		else:
			target_person_msg = deepcopy(self.target_person_msg)
		self.target_person_pub.publish(target_person_msg)


def handle_sigint(signalnum, frame):
    # terminate
    LOGGER.warning('Main interrupted! Exiting.')
    sys.exit()

if __name__ == '__main__':

	# parser=argparse.ArgumentParser()
	# parser.add_argument("--language_code", help="English: en-US Cantonese: yue-Hant-HK")
	# parser.add_argument("--topic", help="/grace_chat")
	# args=parser.parse_args()

	signal(SIGINT, handle_sigint)

	grace_attention = GraceAttention()

	grace_attention.runAttention(True,True)
	
	# #For dbeugging
	# r = rospy.Rate(1)
	# while(True):
	# 	print('1 sec @ %f' %(time.time()))
	# 	r.sleep()

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
