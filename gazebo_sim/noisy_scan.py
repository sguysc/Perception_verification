#!/usr/bin/env python

"""
Created on Tue Sep 30 10:37:53 2020

@author: Guy

Implements a defective lidar scans for the robot

"""

import numpy as np
import copy
#import warnings

# ROS stuff
import rospy
from sensor_msgs.msg import LaserScan

# class to handle a single robot comm.
class NoisyScan:
	def __init__(self, mu=0.2, sigma=0.5):
		self.initialized = False
		#r = rospy.Rate(self.Fs)
		self.mu = mu # 80% chance of selecting the normal distribution, 20% to select max
		self.normal_sigma = sigma #[m]
		
		self.scan_in  = rospy.Subscriber('/front/scan', LaserScan, self.process_scan)
		
		self.scan_out = rospy.Publisher('/front/noisy_scan', LaserScan, queue_size=1)
		
		
	
	def process_scan(self, scan):
		if(not self.initialized):
			self.N = len(scan.ranges) # number of points in a scan
			self.range_max = 6.0 #scan.range_max
			self.angle_min = scan.angle_min
			self.angle_max = scan.angle_max
			self.angle_increment = scan.angle_increment
		
		self.initialized = True
		#
		new_scan = copy.deepcopy(scan)
		# there's some trouble with inf's so we convert them to the BIG maximum range
		meas = np.array(new_scan.ranges)
		meas[meas == np.inf] = 30.
		new_scan.ranges = tuple(meas)
		
		# draw which distribution to pick for each "laser"
		sel_dist = (np.random.rand(self.N) <= self.mu)*1.
		normal_noise = np.random.randn(self.N)*self.normal_sigma
		maxrange_noise = np.random.rand(self.N) + self.range_max - 1.0 #+-0.5m
		passthrough_obs = (np.array(new_scan.ranges) <= self.range_max)*1.
		#import pdb; pdb.set_trace()
		#try:
		# where mu=1, output the max range +-0.5m, when mu=0, add a normal noise
		new_scan.ranges = (new_scan.ranges + normal_noise)*(1.-sel_dist) + \
						  maxrange_noise*sel_dist*passthrough_obs + \
						  (new_scan.ranges + normal_noise)*sel_dist*(1.-passthrough_obs)
		#except:
		#	print(new_scan.ranges)
			
		self.scan_out.publish(new_scan)

			
if __name__ == '__main__':
	#warnings.simplefilter('error', RuntimeWarning)

	rospy.init_node('noisy_scan_node')
	
	NS = NoisyScan(mu=0.3, sigma=0.1)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		pass
	except rospy.ROSInterruptException:
		pass
	
