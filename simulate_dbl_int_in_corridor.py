#!/usr/bin/env python

# general stuff
import numpy as np
import matplotlib.pyplot as plt

# pydrake stuff
from pydrake.systems.framework import BasicVector, LeafSystem, VectorSystem
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogOutput, Adder, Gain, RandomSource, ConstantVectorSource, Demultiplexer
from pydrake.common import RandomDistribution
from pydrake.systems.drawing import plot_system_graphviz

# some global constants
X_d = 10.0
Y_d = 0.0
X_0 = [0. , 0.]
Y_0 = [1.0, 0.]
Y_wall = 3.
Z_d = Y_wall-Y_d
z_max  = 5.0
# the coefficients of the distributions
normal_coeff, exp_coeff, rand_coeff = .2, .2, .5

# simple double integrator plant
# m*xdd(t) = Fx(t)
# m*ydd(t) = Fy(t)
class DoubleIntegrator(LeafSystem):
	def __init__(self):
		LeafSystem.__init__(self)
		# parameters
		self.m = 1.0

		# definitions
		self.DeclareContinuousState(2) # z=[x, xdot]
		self.DeclareVectorInputPort("u", BasicVector(1)) # force
		self.DeclareVectorOutputPort("x", BasicVector(1), self.CopyStateOut, \
									 prerequisites_of_calc=set([self.all_state_ticket()]))    # outputs - x

	def DoCalcTimeDerivatives(self, context, derivatives):
		z = context.get_continuous_state_vector().CopyToVector()
		u = self.EvalVectorInput(context, 0).CopyToVector()

		# [xdot=xdot, xddot=U/m]
		zdot = np.array([z[1],           \
						 u[0]/self.m])   

		derivatives.get_mutable_vector().SetFromVector(zdot)

	# y = [x]
	def CopyStateOut(self, context, output):
		z = context.get_continuous_state_vector().CopyToVector()
		output.SetFromVector([z[0]])

class Controller(LeafSystem):
	def __init__(self, tau_num=0., tau_den=0., k=0.):
		# implements lead or lag controllers with a controller form
		LeafSystem.__init__(self)
		# parameters
		self.tau_num = tau_num
		self.tau_den = tau_den
		self.k = k

		# definitions
		self.DeclareContinuousState(1) 
		self.DeclareVectorInputPort("e", BasicVector(1))                      # inputs (ref, meas)
		self.DeclareVectorOutputPort("u", BasicVector(1), self.CopyStateOut)  # outputs

	def DoCalcTimeDerivatives(self, context, derivatives):
		x = context.get_continuous_state_vector().CopyToVector()
		e = self.EvalVectorInput(context, 0).CopyToVector()

		# xdot = Ax+Bu = -an*x + 1.0*u
		xdot = np.array([-1./self.tau_den * x[0] + 1.0 * e[0]])

		derivatives.get_mutable_vector().SetFromVector(xdot)

	# x
	def CopyStateOut(self, context, output):
		#import pdb; pdb.set_trace()
		x = context.get_continuous_state_vector().CopyToVector()
		e = self.EvalVectorInput(context, 0).CopyToVector()
		
		#y = Cx+Du = k*((bn-an*b0)*X + b0*u)
		y = np.array([self.k/self.tau_den * ( (1.-self.tau_num/self.tau_den) * x[0] + self.tau_num * e[0] )])
		output.SetFromVector(y)

class ClosedLoopModel():
	def __init__(self):
		#import pdb; pdb.set_trace()
		# Create a block diagram containing our system. 
		builder = DiagramBuilder()

		# add the two decoupled plants (x(s)=u/s^2;  y(s)=u/s^2)
		plant_x = builder.AddSystem(DoubleIntegrator())
		plant_y = builder.AddSystem(DoubleIntegrator())
		plant_x.set_name("double_integrator_x")
		plant_y.set_name("double_integrator_y")

		# add the controller, lead compensator for now just to stablize it
		controller_x = builder.AddSystem(Controller(tau_num=2., tau_den=.2, k=1.))
		controller_y = builder.AddSystem(Controller(tau_num=2., tau_den=.2, k=1.))
		controller_x.set_name("controller_x")
		controller_y.set_name("controller_y")

		# the perception's "Beam" model with the four sources of noise
		x_meas_fb = builder.AddSystem(Adder(num_inputs=4, size=1))
		x_meas_fb.set_name("x_fb")
		y_meas_fb = builder.AddSystem(Adder(num_inputs=4, size=1))
		y_meas_fb.set_name("y_fb")
		y_perception = builder.AddSystem(Adder(num_inputs=2, size=1))
		y_perception.set_name("range_measurment_y")
		neg_y_meas = builder.AddSystem(Gain(k=-1., size=1))
		neg_y_meas.set_name("neg_y_meas")
		wall_position = builder.AddSystem(ConstantVectorSource([Y_wall]))
		
		p_hit   = builder.AddSystem(RandomSource(distribution=RandomDistribution.kGaussian, num_outputs=2,\
												 sampling_interval_sec=0.01))
		p_hit.set_name("GaussionNoise(0,1)")
		p_short = builder.AddSystem(RandomSource(distribution=RandomDistribution.kExponential, num_outputs=2,\
												 sampling_interval_sec=0.01))
		p_short.set_name("ExponentialNoise(1)")
		#p_max   = builder.AddSystem(RandomSource(distribution=RandomDistribution.kUniform, num_outputs=1,\
		#										 sampling_interval_sec=0.01))
		p_rand  = builder.AddSystem(RandomSource(distribution=RandomDistribution.kUniform, num_outputs=2,\
												 sampling_interval_sec=0.01))
		p_rand.set_name("UniformNoise(0,1)")
		#import pdb; pdb.set_trace()
		p_hit_Dx = builder.AddSystem(Demultiplexer(size=2))
		p_hit_Dx.set_name('Dmux1')
		p_short_Dx = builder.AddSystem(Demultiplexer(size=2))
		p_short_Dx.set_name('Dmux2')
		p_rand_Dx = builder.AddSystem(Demultiplexer(size=2))
		p_rand_Dx.set_name('Dmux3')
		normgain_x = builder.AddSystem(Gain(k=normal_coeff, size=1))
		normgain_x.set_name("Sigma_x")
		normgain_y = builder.AddSystem(Gain(k=normal_coeff, size=1))
		normgain_y.set_name("Sigma_y")
		expgain_x = builder.AddSystem(Gain(k=exp_coeff, size=1))
		expgain_x.set_name("Exp_x")
		expgain_y = builder.AddSystem(Gain(k=exp_coeff, size=1))
		expgain_y.set_name("Exp_y")
		randgain_x = builder.AddSystem(Gain(k=rand_coeff, size=1))
		randgain_x.set_name("Rand_x")
		randgain_y = builder.AddSystem(Gain(k=rand_coeff, size=1))
		randgain_y.set_name("Rand_y")
		#maxgain_x = builder.AddSystem(Adder(num_inputs=2, size=1))
		#maxgain_x.set_name("Max_x")
		#maxgain_y = builder.AddSystem(Adder(num_inputs=2, size=1))
		#maxgain_y.set_name("Max_y")

		# the summation to get the error (closing the loop)
		summ_x = builder.AddSystem(Adder(num_inputs=2, size=1))
		summ_y = builder.AddSystem(Adder(num_inputs=2, size=1))
		summ_x.set_name("summation_x")
		summ_y.set_name("summation_y")
		neg_x = builder.AddSystem(Gain(k=-1., size=1))
		neg_y = builder.AddSystem(Gain(k=-1., size=1))
		neg_uy = builder.AddSystem(Gain(k=-1., size=1))
		neg_x.set_name("neg_x")
		neg_y.set_name("neg_y")
		neg_uy.set_name("neg_uy")

		# wire up all the blocks (summation to the controller to the plant ...)
		builder.Connect(summ_x.get_output_port(0), controller_x.get_input_port(0)) # e_x
		builder.Connect(summ_y.get_output_port(0), controller_y.get_input_port(0)) # e_y
		
		builder.Connect(controller_x.get_output_port(0), plant_x.get_input_port(0)) # u_x
		builder.Connect(controller_y.get_output_port(0), neg_uy.get_input_port(0)) # u_y (to deal with directions)
		builder.Connect(neg_uy.get_output_port(0), plant_y.get_input_port(0)) # u_y
		
		builder.Connect(plant_x.get_output_port(0), x_meas_fb.get_input_port(0)) # perception, nominal state meas
		builder.Connect(wall_position.get_output_port(0), y_perception.get_input_port(0)) # perception
		builder.Connect(plant_y.get_output_port(0), neg_y_meas.get_input_port(0)) # perception
		builder.Connect(neg_y_meas.get_output_port(0), y_perception.get_input_port(1)) # perception, z meas
		builder.Connect(y_perception.get_output_port(0), y_meas_fb.get_input_port(0)) # perception, nominal state meas
		
		builder.Connect(p_hit.get_output_port(0), p_hit_Dx.get_input_port(0)) # demux the noise
		builder.Connect(p_hit_Dx.get_output_port(0), normgain_x.get_input_port(0)) # normalize Normal dist
		builder.Connect(normgain_x.get_output_port(0), x_meas_fb.get_input_port(1)) # Normal dist
		builder.Connect(p_hit_Dx.get_output_port(1), normgain_y.get_input_port(0)) # normalize Normal dist
		builder.Connect(normgain_y.get_output_port(0), y_meas_fb.get_input_port(1)) # Normal dist
		
		builder.Connect(p_short.get_output_port(0), p_short_Dx.get_input_port(0)) # demux the noise
		builder.Connect(p_short_Dx.get_output_port(0), expgain_x.get_input_port(0)) # normalize Exp dist
		builder.Connect(expgain_x.get_output_port(0), x_meas_fb.get_input_port(2)) # Exp dist
		builder.Connect(p_short_Dx.get_output_port(1), expgain_y.get_input_port(0)) # normalize Exp dist
		builder.Connect(expgain_y.get_output_port(0), y_meas_fb.get_input_port(2)) # Exp dist
		
		#builder.Connect(p_max.get_output_port(0), x_meas_fb.get_input_port(3)) # Uniform dist
		#builder.Connect(p_max.get_output_port(0), y_meas_fb.get_input_port(3)) # Uniform dist
		
		builder.Connect(p_rand.get_output_port(0), p_rand_Dx.get_input_port(0)) # normalize Uniform dist
		builder.Connect(p_rand_Dx.get_output_port(0), randgain_x.get_input_port(0)) # normalize Uniform dist
		builder.Connect(randgain_x.get_output_port(0), x_meas_fb.get_input_port(3)) # Uniform dist
		builder.Connect(p_rand_Dx.get_output_port(1), randgain_y.get_input_port(0)) # normalize Uniform dist
		builder.Connect(randgain_y.get_output_port(0), y_meas_fb.get_input_port(3)) # Uniform dist

		builder.Connect(x_meas_fb.get_output_port(0), neg_x.get_input_port(0)) # -x
		builder.Connect(y_meas_fb.get_output_port(0), neg_y.get_input_port(0)) # -y	
		builder.Connect(neg_x.get_output_port(0), summ_x.get_input_port(1)) # r-x
		builder.Connect(neg_y.get_output_port(0), summ_y.get_input_port(1)) # r-y

		# Make the desired_state input of the controller, an input to the diagram.
		builder.ExportInput(summ_x.get_input_port(0))
		builder.ExportInput(summ_y.get_input_port(0))

		# get telemetry
		logger_x  = LogOutput(plant_x.get_output_port(0), builder)
		logger_noise_x  = LogOutput(x_meas_fb.get_output_port(0), builder)
		logger_y  = LogOutput(plant_y.get_output_port(0), builder)
		logger_noise_y  = LogOutput(y_meas_fb.get_output_port(0), builder)
		logger_x.set_name("logger_x_state")
		logger_y.set_name("logger_y_state")
		logger_noise_x.set_name("x_state_meas")
		logger_noise_y.set_name("y_meas")

		# compile the system
		diagram = builder.Build()
		diagram.set_name("diagram")

		# Create the simulator, and simulate for 10 seconds.
		simulator = Simulator(diagram)
		context   = simulator.get_mutable_context()

		# First we extract the subsystem context for the plant
		plant_context_x = diagram.GetMutableSubsystemContext(plant_x, context)
		plant_context_y = diagram.GetMutableSubsystemContext(plant_y, context)
		# Then we can set the pendulum state, which is (x, y).
		z0 = X_0
		plant_context_x.get_mutable_continuous_state_vector().SetFromVector(z0)
		z0 = Y_0
		plant_context_y.get_mutable_continuous_state_vector().SetFromVector(z0)

		# The diagram has a single input port (port index 0), which is the desired_state.
		context.FixInputPort(0, [X_d]) # here we assume no perception, closing feedback on state X
		context.FixInputPort(1, [Z_d]) #Z_y, keep 3m away, basically we want to be at Y=0

		# run the simulation
		simulator.AdvanceTo(10)
		t = logger_x.sample_times()
		#import pdb; pdb.set_trace()
		# Plot the results.
		plt.figure()
		plot_system_graphviz(diagram, max_depth=2)

		plt.figure()
		plt.plot(t, logger_noise_x.data().transpose(), label='x_state_meas')
		plt.plot(t, logger_noise_y.data().transpose(), label='y_meas (z(y))')
		plt.plot(t, logger_x.data().transpose(), label='x_state')
		plt.plot(t, logger_y.data().transpose(), label='y_state')
		plt.xlabel('t')
		plt.ylabel('x(t),y(t)');
		plt.legend()
		plt.show(block=True)

		
if __name__ == "__main__":
	M = ClosedLoopModel()
