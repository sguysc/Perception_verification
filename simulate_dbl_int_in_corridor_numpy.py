#!/usr/bin/env python

"""
Created on Tue Sep 15 10:37:53 2020

@author: Guy

Implements a dynamical system of a robot (double integrator) with a range sensor measuring
its position relative to a wall. The measurements are not Normally distributed, but characterized
with the "beam model" (see "Probablistic Robotics" chapter 6). The aim is to see the reachable 
sets of the system under given controls and this noise.
"""

# general stuff
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as spi
import scipy.interpolate as interpolate
from scipy import stats

#try:
#	from pydrake.all import LinearQuadraticRegulator
#except:
#	# newer version of drake
from pydrake.systems.controllers import LinearQuadraticRegulator

# my helper classes
from map_generator import MapGenerator

# some behavioral control switches
continuous = False
#'const', 'linear', 'saw', 'sine', 'square', 'file'
Map = MapGenerator(north_type='square', south_type='saw', center_type='const')

# parameters of simulation
const_velocity = 0.5 # in x direction [m/sec]
m    = 1.0  # robot mass [Kg]
dt   = 0.01 # integration step time [sec]
Tmax = 10   # simulation time [sec]
meas_max = 6.0 # max measurement of the range sensor [m]

# vector of measurements
#sensor_array = np.array([90.]) # array of angles with respect to the x axis
sensor_array = np.array([90., -90.]) # array of angles with respect to the x axis
#sensor_array = np.array([90., 45., -45., -90.]) # array of angles with respect to the x axis

# estimator memory variables
z_est = np.array([[0.], [0.]]) # y, y'
u_est = 0.
# check via matlab or place some algorithm here to calculate the gains for each (A,C)
# A = [0 1; 0 0]
# C = [-1/sin(a1)  0; -1/sin(a2)  0 ....]
# p = -2*pi*1*[cosd(45)+i*sind(45), -cosd(45)-i*sind(45)]
# L = place(A',C',p).'
#L = np.array([[-8.89], [-39.48]]) # [+90]; w=1Hz, zeta=0.7
L = np.array([[-4.44,   4.44], [-19.74,   19.74]])  # [+90,-90]; w=1Hz, zeta=0.7
#L = np.array([[-2.96,-2.10,2.10,2.96], [-13.16,-9.31,9.31,13.16]]) # [+90,+45,-45,-90]; w=1Hz, zeta=0.7

# wall y location with respect to x. y_dir =1 if north wall, y_dir=-1 if south
def wall_y_location(x, y_dir=1.):
	if(y_dir > 0.):
		return Map.north_wall(x)
	else:
		return Map.south_wall(x)

# state z has three components: z=[x, y, y'].
def plant(z, t):
	N_sensors = sensor_array.shape[0]
	meas = np.zeros( [N_sensors, 1] )
	for i, sensor in enumerate(sensor_array):
		north_wall = 1.0 
		if sensor < 0.:
			north_wall = -1.0
		# state feedback control (on y-position) is the distance to the wall
		ideal_y   = wall_y_location(z[0], y_dir=north_wall) - z[1]
		if(np.abs(sensor) <= 0.001):
			# case where there is a sensor heading in x direction
			# assume infinite wall so no wall is infront
			meas[i][0] = meas_max
		else:
			# get the ideal ray to the wall
			ideal_meas= ideal_y / np.sin(np.deg2rad(sensor))
			# get the new noise distribution (because it varies with the nominal meas.)
			pdf, bins = noise_dist(ideal_meas, a1=.3, a2=0.1, a3=.1, a4=.04, norm_sig=0.1)
			# only Gaussian noise (for debug)
			# pdf, bins = noise_dist(ideal_meas, a1=.3, a2=0., a3=0., a4=0., norm_sig=0.1)
			# sample a new measurement from the inverse cdf using a uniform random number in [0,1]
			meas[i][0] = inverse_transform_sampling(pdf, bins) 

	# output feedback control
	u = control(Map.center_line(z[0]), meas, z[0])
	
	'''
	# discretized
    zdot[0] = z[0] + 0    + 0         + dt * const_velocity
    zdot[1] = 0    + z[1] + dt * z[2] + dt*dt/(2.*m) * u
	zdot[2] = 0    + 0    + z[2]      + dt/m * u
	'''
	zdot    = z.copy()
	zdot[0] = const_velocity   # x'    = v_const
	zdot[1] = z[2]             # y'    = y'
	zdot[2] = 1./m * u         # m*y'' = u
	
    # We return z'=[x', y', y''] and measured output y
	return zdot, meas

# state estimation (pole placement) + state feedback (LQR). 
#ref- y to be in. meas- the measurements vector. x- just for the wall function
def control(ref, meas, x):
	global z_est
	global u_est
	global L
	
	# observer gain (only for 1 measurement, if you have more, compute in matlab).
	#wn = 2.*np.pi*1. # 1Hz observer
	#xi = 0.7 # damping factor
	#LL = np.array([[-2.*xi*wn], [-wn*wn]]) # pole-placement
	
	# create the range estimation for each sensor
	N_sensors = meas.shape[0]
	meas_est = np.zeros( [N_sensors, 1] )
	for i, sensor in enumerate(sensor_array):
		north_wall = 1.0 
		if sensor < 0.:
			north_wall = -1.0
		y_est = wall_y_location(x, y_dir=north_wall) - z_est[0][0]
		if(np.abs(sensor) < 0.001):
			meas_est[i][0] = np.inf
		else:
			meas_est[i][0] = y_est / np.sin(np.deg2rad(sensor))
		
	#  Luenberger Observer
	f = np.array([[z_est[1][0]], \
				  [1./m * u_est] ])
	# Estimator equations
	zdot_est = f + L.dot(meas - meas_est)
	# Euler integration
	z_est    = z_est + dt * zdot_est
	
	# now we can do state feedback control (x-axis doesn't really play here)
	Af = np.array([[0.,1.], [0.,0.]])
	Bf = np.array([[0.], [1./m]])
	Q  = np.eye(2)
	R  = np.eye(1)
	Kf, Qf = LinearQuadraticRegulator(Af, Bf, Q, R)
	
	u_est = ref - Kf.dot(z_est)[0][0]
	# do nothing (for debug)
	#u_est = 0.
	
	return u_est

# implements the noise pdf of the beam model and allows some parameters to be set
def noise_dist(x_true, a1=1., a2=1., a3=1., a4=.1, norm_sig=1., exp_lambda=1., uni_delta=0.5, plot=False):
	N = 100
	
	# the discretization of the space (bins)
	x = np.linspace(0, meas_max, N)
	# -x_true because we shift it and for some reason it looks at the truncated dist before shift :(
	rv_norm = stats.truncnorm(-x_true, meas_max-x_true, loc=x_true, scale=norm_sig) 
	rv_exp  = stats.expon()
	rv_uni  = stats.uniform()
	
	# the beam model pdf (see prob. robotics book ch. 6)
	pdf = a1 * rv_norm.pdf(x) + \
		  a2 * rv_exp.pdf(exp_lambda*x)*exp_lambda + \
		  a3 * rv_uni.pdf((x-0.)/meas_max)/meas_max + \
		  a4 * rv_uni.pdf((x-(meas_max-uni_delta))/uni_delta)/uni_delta

	if(plot):
		# plot the dist for debugging
		fig, ax = plt.subplots(1, 1)
		ax.plot(x, pdf)
		plt.title('p(x) for the beam model')
		plt.show(block=False)
		
	return pdf, x

# both creates the inverse cdf and samples and returns numbers from this distribution
def inverse_transform_sampling(pdf, bin_edges, n_samples=1):
	#import pdb; pdb.set_trace()
	# this sort of creates the histogram by taking to adjacent pdf values and averaging them for every bin
	pdf = 0.5 * ( pdf[:-1] + pdf[1:] )
	# construct the CDF
	cum_values = np.zeros(bin_edges.shape)
	cum_values[1:] = np.cumsum(pdf*np.diff(bin_edges))
	# normalize to a standard distribution because it wasn't done before
	cum_values = cum_values/cum_values[-1] 
	
	inv_cdf = interpolate.interp1d(cum_values, bin_edges)
	# u in [0,1]
	u = np.random.rand(n_samples)
	# return the function for later use
	return inv_cdf(u)

def simulate_c():
	# We want to evaluate the system on N linearly spaced times between t=0 and t=Tmax.
	t = np.linspace(0., 10., Tmax)
	# The initial position is (0, 0).
	z0 = np.zeros(3)
	
	# We simulate the system and evaluate z
	z = spi.odeint(plant, z0, t, args=(k,))
	
	return t, z

# implements a standard Euler integration steps in a loop
def simulate_d():
	global z_est
	global u_est
	z_est = np.array([[0.], [0.]])
	u_est = 0.
	
	# We want to evaluate the system on N linearly spaced times between t=0 and t=10.
	t_vec = np.arange(0., Tmax, dt)
	# The initial position is (0, 0).
	#z = np.array([0., 0., 0.])
	z = np.array([0., 1., 0.])
	z_save, z_est_save, u_save, meas_save = [], [], [], []
	
	N_sensors = sensor_array.shape[0]

	# We simulate the system and evaluate z
	for t in t_vec:
		# get the derivatives
		zdot, meas = plant(z, t)
		# Euler integration
		z = z + dt * zdot 
		# save for telemetry
		if(len(z_save) == 0):
			z_save = np.array([z])
			z_est_save = np.array(z_est.reshape((1,2))) # only save y,y' at the moment
			meas_save = np.array(meas.reshape((1,N_sensors)))
			u_save = np.array([u_est])
		else:
			z_save = np.vstack([z_save, z])
			meas_save = np.vstack([meas_save, meas.reshape((1,N_sensors))])
			z_est_save = np.vstack([z_est_save, z_est.reshape((1,2))])
			u_save = np.vstack([u_save, u_est])
	
	return t_vec, z_save, meas_save, z_est_save, u_save

def single_run():
	if(continuous):
		t, state = simulate_c()
	else:
		t, state, meas, state_est, controls = simulate_d()

	# plot the dist for debugging
	#pdf, bins = noise_dist(3.0, a1=.5, a2=0.1, a3=.3, a4=.04)
	pdf, bins = noise_dist(3.0, a1=.3, a2=0.1, a3=.1, a4=.04, norm_sig=0.1, plot=True)

	# visualizations
	fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

	ax1.plot(t, state[:, 1], label='$y(t)$')
	ax1.plot(t, state_est[:, 0], label='$\^y(t)$')
	ax2.plot(t, controls[:], label='$u_y(t)$', alpha=0.2)

	# walls
	n_wall, s_wall = [], []
	for i in range(len(t)):
		n_wall.append( wall_y_location(state[i, 0], y_dir=1.0) )
		s_wall.append( wall_y_location(state[i, 0], y_dir=-1.0 ) )
	
	for i, sensor in enumerate(sensor_array):
		if sensor > 0.:
			ax1.plot(t, n_wall - meas[:,i]*np.sin(np.deg2rad(sensor)), \
					label='$\^y(sensor_%d)$'%i, marker='o', linestyle='', alpha=0.2)
		else:
			ax1.plot(t, s_wall - meas[:,i]*np.sin(np.deg2rad(sensor)), \
					label='$\^y(sensor_%d)$'%i, marker='o', linestyle='', alpha=0.2)

	#ax.plot([t[0], t[-1]], [3., 3.], label='north wall', color='k', linewidth=6)
	ax1.plot(t, n_wall, label='north wall', color='k', linewidth=6)
	#ax.plot([t[0], t[-1]], [-3., -3.], label='south wall', color='k', linewidth=6)
	ax1.plot(t, s_wall, label='south wall', color='k', linewidth=6)
	ax1.legend()
	ax1.set_title('state vs. time (%d sensors)'%(len(sensor_array)))
	ax2.legend()
	ax2.set_title('controls vs. time')
	ax1.set_xlim(t[0], t[-1])
	ax1.set_ylim(-np.max(np.abs(n_wall)), np.max(np.abs(n_wall)))
	
	fig, ax3 = plt.subplots(1, 1)
	ax3.plot(state[:, 0], state[:, 1], label='$robot$')
	ax3.plot(state[:, 0], n_wall, label='north wall', color='k', linewidth=6)
	ax3.plot(state[:, 0], s_wall, label='south wall', color='k', linewidth=6)
	ax3.legend()
	ax3.set_title('Robot in workspace')
	
	plt.show() #block=True)
	
def multi_run(n=10):
	# visualizations
	fig, ax3 = plt.subplots(1, 1)

	for idx in range(n):
		t, state, meas, state_est, controls = simulate_d()
		print('finished run #%d' %(idx))
		ax3.plot(state[:, 0], state[:, 1], label='$%d$'%(idx), alpha=0.2)
	
	# walls
	n_wall, s_wall = [], []
	for i in range(len(t)):
		n_wall.append( wall_y_location(state[i, 0]) )
		s_wall.append( -3. )
	ax3.plot(state[:, 0], n_wall, label='north wall', color='k', linewidth=6)
	ax3.plot(state[:, 0], s_wall, label='south wall', color='k', linewidth=6)
	ax3.legend()
	ax3.set_title('Robot in workspace')

	plt.show()


if __name__ == "__main__":
	
	single_run()
	#multi_run()
