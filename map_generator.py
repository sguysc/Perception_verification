#!/usr/bin/env python

"""
Created on Tue Sep 15 10:37:53 2020

@author: Guy

Implements a map for the robot

"""

import numpy as np
import scipy.interpolate as interpolate


class MapGenerator():
	def __init__(self, north_type='const', south_type='const', center_type='const',
				 north_wall_filename = 'north_wall.csv', south_wall_filename = 'south_wall.csv', 
				 center_line_filename = 'center_line.csv'):

		self.north_wall_function_src = north_type
		self.south_wall_function_src = south_type
		self.center_line_function_src = center_type

		try:
			self.north_wall_data = np.loadtxt(north_wall_filename, delimiter=",", skiprows=1)
		except:
			# didn't find files, use some constant default
			self.north_wall_data = np.array([[0., 3.], [100., 3.]])
		try:
			self.south_wall_data = np.loadtxt(south_wall_filename, delimiter=",", skiprows=1)
		except:
			# didn't find files, use some constant default
			self.south_wall_data = np.array([[0., -3.], [100., -3.]])
		try:
			self.center_line_data = np.loadtxt(center_line_filename, delimiter=",", skiprows=1)
		except:
			# didn't find files, use some constant default
			self.center_line_data = np.array([[0., 0.], [100., 0.]])
		

	# returns the y location of the north wall as a function of x
	def north_wall(self, x):
		if(self.north_wall_function_src == 'const'):
			# constant location
			return 3.0
		elif(self.north_wall_function_src == 'linear'):
			# linear 
			m  = 0.5 # slope
			y0 = 3.0 # bias
			return y0 + m*x
		elif(self.north_wall_function_src == 'saw'):
			# sawtooth
			y0 = 3.0 # bias
			m  = 0.5 # slope
			w  = 2.0 # sawtooth width
			dx = (x%w)
			if ( dx >= w/2.0 ):
				return y0 + m*w - m*dx
			else:
				return y0 + m*dx
		elif(self.north_wall_function_src == 'square'):
			# sawtooth
			y0 = 3.0 # bias
			y1 = 4.0 # bias 2
			w  = 2.0 # square width
			dx = (x%w)
			if ( dx >= w/2.0 ):
				return y1
			else:
				return y0
		elif(self.north_wall_function_src == 'sine'):
			# sine
			y0 = 3.0 # bias
			A = 1.0 # sine amplitude
			f = 1.0 # spatial frequency
			return y0 + A * np.sin(2.*np.pi*f*x)
		else:
			# from file
			f = interpolate.interp1d(self.north_wall_data[:,0], self.north_wall_data[:,1])
			return f(x)

	# returns the y location of the south wall as a function of x
	def south_wall(self, x):
		if(self.south_wall_function_src == 'const'):
			# constant location
			return -3.0
		elif(self.south_wall_function_src == 'linear'):
			# linear 
			m  = -0.5 # slope
			y0 = -3.0 # bias
			return y0 + m*x
		elif(self.south_wall_function_src == 'saw'):
			# sawtooth
			y0 = -3.0 # bias
			m  = -0.5 # slope
			w  = 2.0 # sawtooth width
			dx = (x%w)
			if ( dx >= w/2.0 ):
				return y0 + m*w - m*dx
			else:
				return y0 + m*dx
		elif(self.south_wall_function_src == 'square'):
			# sawtooth
			y0 = 3.0 # bias
			y1 = 4.0 # bias 2
			w  = 2.0 # square width
			dx = (x%w)
			if ( dx >= w/2.0 ):
				return y1
			else:
				return y0
		elif(self.south_wall_function_src == 'sine'):
			# sine
			y0 = -3.0 # bias
			A = 1.0 # sine amplitude
			f = 1.0 # spatial frequency
			return y0 + A * np.sin(2.*np.pi*f*x)
		else:
			# from file
			f = interpolate.interp1d(self.south_wall_data[:,0], self.south_wall_data[:,1])
			return f(x)

	# returns the y location of the center line as a function of x
	def center_line(self, x):
		if(self.center_line_function_src == 'const'):
			# constant location
			return 0.0
		elif(self.center_line_function_src == 'linear'):
			# linear 
			m  = 0.5 # slope
			y0 = 0.0 # bias
			return y0 + m*x
		elif(self.center_line_function_src == 'saw'):
			# sawtooth
			y0 = 0.0 # bias
			m  = 0.5 # slope
			w  = 2.0 # sawtooth width
			dx = (x%w)
			if ( dx >= w/2.0 ):
				return y0 + m*w - m*dx
			else:
				return y0 + m*dx
		elif(self.center_line_function_src == 'square'):
			# sawtooth
			y0 = 3.0 # bias
			y1 = 4.0 # bias 2
			w  = 2.0 # square width
			dx = (x%w)
			if ( dx >= w/2.0 ):
				return y1
			else:
				return y0
		elif(self.center_line_function_src == 'sine'):
			# sine
			y0 = 0.0 # bias
			A = 1.0 # sine amplitude
			f = 1.0 # spatial frequency
			return y0 + A * np.sin(2.*np.pi*f*x)
		else:
			# from file
			f = interpolate.interp1d(self.center_line_data[:,0], self.center_line_data[:,1])
			return f(x)
