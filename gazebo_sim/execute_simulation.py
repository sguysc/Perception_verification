#!/usr/bin/env python

"""
Created on Thu Oct 22 15:13:53 2020

@author: Guy

Run a sinlge simulation with the noisy lidar scans

"""


import numpy as np
import copy
import logging
from time import localtime, strftime, sleep

# ROS stuff
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from nav_msgs.msg import Path

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.msg import ModelState, ModelStates

logger = None
Fs = 10 #hz

def set_obj_pose(x, y, yaw = None):
	if yaw is None:
		yaw_angle = 0.0
	else:
		yaw_angle = yaw * np.pi / 180.

	# Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
	Q = Quaternion(*(quaternion_from_euler(0., 0., yaw_angle, axes='sxyz')))

	my_pose = Pose(Point(x, y, 0.0), Q)

	return my_pose


class SingleSimulation():
	def __init__(self):
		# initialize stuff
		rospy.init_node('run_sim', anonymous=False)

		self.jackalpose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
		self.jackalpose_sub  = rospy.Subscriber('/gazebo/model_states', ModelStates, self.measurement_cb)
		self.globalplanner_sub  = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', \
												   Path, self.globalplanner_cb)

		# robot's initial position (x, y = 4, -8.5)
		self.pose = np.array([0.0, 0.0, 0.0])
		self.global_path = np.array([0.0, 0.0, 0.0])
		self.percieved_pose = np.array([0.0, 0.0, 0.0])
		self.goal = np.array([6.5, 1.5, 0.0])
		
		# Create and start action client
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.start_action_client()
		
		self.goal_completed = False
		self.r = rospy.Rate(Fs)

	def run(self):
		# send the command 
		self.movebase_client(self.goal)
		
		while not rospy.is_shutdown() and not self.goal_completed:
			# output the telemetry to file
			self.telemetry()
			self.r.sleep()
			
	# location of obstacle, ground truth
	def measurement_cb(self, msg):
		i    = msg.name.index('jackal')
		pose = msg.pose[i].position
		Q    = msg.pose[i].orientation
		angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
		self.pose = np.array([pose.x, pose.y, angles[2]])

	# get the commands the global planner is using
	def globalplanner_cb(self, glob_path):
		pose_cmd = glob_path.poses[0].pose.position
		Q        = glob_path.poses[0].pose.orientation
		angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
		
		self.global_path = np.array([pose_cmd.x, pose_cmd.y, angles[2]])

			
	# client stuff
	def start_action_client(self):
		print("Waiting for move_base action server...")
		wait = self.client.wait_for_server(rospy.Duration(60))
		if not wait:
			print("Action server not available! Try running again.")
			rospy.signal_shutdown("Action server not available!")
			return
		print("Connected to move base server")
		
	def movebase_client(self, goal_pose):
		goal_msg = MoveBaseGoal()
		goal_msg.target_pose.header.frame_id = "map"
		goal_msg.target_pose.header.stamp = rospy.Time.now()

		goal_msg.target_pose.pose = set_obj_pose(*goal_pose)
		print("Sending goal pose (%f, %f) to Action Server" % (goal_pose[0], goal_pose[1]))

		self.client.send_goal(goal_msg, self.done_cb, self.active_cb, self.feedback_cb)

	def telemetry(self):
		# telemetry
		now = rospy.get_rostime()
		t = now.secs + now.nsecs/1.0E9
		try:
			logger.debug('%.3f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;' \
					   %(t, self.goal[0], self.goal[1], \
						 self.global_path[0], self.global_path[1], self.global_path[2],
						 self.pose[0], self.pose[1], self.pose[2], \
						 self.percieved_pose[0], self.percieved_pose[1], self.percieved_pose[2] ) )
		except:
			pass
		
	def done_cb(self, status, result):
		print('finished simulation')
		self.goal_completed = True
		#rospy.signal_shutdown("succesfull run")
		
	def active_cb(self):
		# print("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")
		pass
	
	def feedback_cb(self, feedback):
		# To print current pose at each feedback:
		# print("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
		#print("Feedback for goal pose "+ str(self.goal_cnt+1)+" received")
		#import pdb; pdb.set_trace()
		percieved_pose = feedback.base_position.pose.position
		Q = feedback.base_position.pose.orientation
		angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
		
		self.percieved_pose = np.array([percieved_pose.x, percieved_pose.y, angles[2]])


if __name__ == '__main__':
	timenow     = localtime()
	log_file    = strftime('ex1_%Y_%m_%d_%H_%M_%S', timenow)
	
	level       = logging.DEBUG
	logger      = logging.getLogger(__name__)
	formatter   = logging.Formatter('%(message)s')
	fileHandler = logging.FileHandler('./telemetry/%s.log' %(log_file), mode='w')
	fileHandler.setFormatter(formatter)
	logger.setLevel(level)
	logger.addHandler(fileHandler)
	# set header of telemetry file
	logger.debug('t;xgoal;ygoal;x_waypoint;y_waypoint;yaw_waypoint;x_t;y_t;yaw_t;x_p;y_p;yaw_p;')

	sim = SingleSimulation()

	try:
		sim.run()
	except KeyboardInterrupt:
		pass
	except rospy.ROSInterruptException:
		pass

