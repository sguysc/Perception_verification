#!/usr/bin/env python

"""
Created on Tue Sep 30 10:37:53 2020

@author: Guy

Implements a map for the robot

"""

import numpy as np
from scipy.linalg import block_diag
try:
	from pydrake.solvers import MathematicalProgram
	from pydrake.solvers.mathematicalprogram import Solve
except:
	from pydrake.all import MathematicalProgram
	from pydrake.all import Solve


class NoiseOptSolver():
	# for x[k+1] = Ax[k] + Bu[k]
	#     y[k]   = Cx[k] + n[k]
	#     u[k]   = -Ky[k]
	# ==> 
	#     x[k+1] = Ax[k] - BKy[k] = Ax[k] - BK(Cx[k] + n[k])  = (A - BKC)x[k] - BKn[k]
	# it is assumed for now that y is the full state -> C=eye(n)
	
	def __init__(self, A, B, C, K, T, Sigma, x0, x_lb, x_ub, check_states=[], noise_trunc=[]):
		n = A.shape[0] # n states
		
		if(len(check_states)==0):
			check_states = range(0, n)
		if(len(noise_trunc)==0):
			# set default to 3 std's
			noise_trunc = np.diag(Sigma)**(0.5)*3.
		
		invsig = np.linalg.inv(Sigma)
		#Q = block_diag(*([invsig]*T))  #  [n1,n2,...,nL]*[Q]*[n1;n2;...;nL]
		b = np.zeros((n, 1))  #n*T
		
		costs = []
		eval_n_is = []
		
		for prob_len in range(1,T):
			for bound in ['lo', 'hi']:
				for state in check_states:
					self.prog  = MathematicalProgram()
					# the noise decision variables, we search for the first time it falsifies.
					# to not overconstrain the problem, if we are checking to falsify at t=i,
					# we do not want more constraints in i:T because maybe a better probability cost will be
					# that i,i+1,i+2 will go out of bounds, and then i+3 comes back, etc ...
					self.n_i = self.prog.NewContinuousVariables(n*prob_len,'n')  
					# the states as a function of time
					self.x   = self.prog.NewIndeterminates(n*prob_len,'x')
					
					# the maximum value allowed for a noise measurement (truncated normal distribution)
					for i, noise in enumerate(self.n_i):
						self.prog.AddConstraint(noise, -noise_trunc[i%n], noise_trunc[i%n])
						
					# simulate the dynamics forward to obtain the constraints on the states
					# Drake will convert them to constraints on the decision variables
					x_t = x0.copy()
					for t in range(1, prob_len+1):
						# which noise variables affect this step
						ind = np.arange((t-1)*n, t*n)
						#import pdb; pdb.set_trace()
						x_t = (A-B*K.dot(C)).dot(x_t) - B.dot(K).dot(self.n_i[ind].reshape(n,-1))
						# the falsifying constraint:
						if(prob_len == t):
							for state_i in range(n):
								dv_exist = (len(x_t[state_i][0].GetVariables()) > 0)
								if(state_i == state and dv_exist):
									if(bound == 'lo'):
										#print('adding a lower violation constraint state=%d t=%d' %(state_i, t))
										# hits the lower bound
										self.prog.AddConstraint(x_t[state_i][0] <= x_lb[state_i][0])
									else:
										#print('adding an upper violation constraint state=%d t=%d' %(state_i, t))
										# hits the upper bound
										self.prog.AddConstraint(x_t[state_i][0] >= x_ub[state_i][0])
								elif(state_i != state and dv_exist):
									# keep it within the bounds (satisfying)
									self.prog.AddConstraint(x_t[state_i][0] >= x_lb[state_i][0])
									self.prog.AddConstraint(x_t[state_i][0] <= x_ub[state_i][0])
								else:
									# dv_exist == False =>
									# drake cannot get a constraint in the form of const <=> const
									pass
						else:
							for state_i in range(n):
								dv_exist = (len(x_t[state_i][0].GetVariables()) > 0)
								if(dv_exist):
									# the "box"/corridor constraint
									self.prog.AddConstraint(x_t[state_i][0], x_lb[state_i][0], x_ub[state_i][0])
						
						# quadratic cost of the log likelihood 
						self.prog.AddQuadraticCost(invsig, b, self.n_i[ind])
						
					# solve the mathematical program
					result = Solve(self.prog)
					
					if(result.is_success()):
						eval_n_i = result.GetSolution(self.n_i)
						eval_n_is.append(eval_n_i.reshape(prob_len,n).T)
						# minus because the real cost is -1/2*x'Sx and for the prog I multiply by -1 to get max
						# this is the true multivariate pdf
						Q = block_diag(*([invsig]*prob_len))  #  [n1,n2,...,nL]*[Q]*[n1;n2;...;nL]
						cost = (2.*np.pi)**(-(n*prob_len)/2.) * np.linalg.det(Q)**(-0.5) * \
								np.exp(-0.5*eval_n_i.T.dot(Q).dot(eval_n_i))
						costs.append(cost)
						print('found solution (lh/step/state/cost %s/%d/%d/%.2e)' %(bound, prob_len, state, cost))
					else:
						print('nope :( (lh/step/state %s/%d/%d)' %(bound, prob_len, state))
			
		costs = np.array(costs)
		if(len(costs)>0):
			ind = np.argmax(costs)
			# the index step, the actual cost and the whole series of noises
			self.ind = ind
			self.max_cost = costs[ind]
			self.noise_seq = eval_n_is[ind]
		else:
			self.ind = -1
			self.max_cost = -1
			self.noise_seq = np.array([])


if __name__ == "__main__":
	# definitions of parameters
	m = 1.
	dt = 1. #sec
	# discrete system of double integrator with time step 1sec
	A = np.array([[1.,dt], \
				  [0.,1.]])
	B = np.array([[dt*dt/(2.*m)], \
				  [dt/m]])
	C = np.eye(2)
	K = np.array([[1.2657, 1.4622]]) # with lqrd matlab
	T = 10 #steps
	Sigma= np.array([[0.5, 0.], \
					 [0.0, 0.001]])
	x0   = np.array([[0.], [0.]]) # y, ydot
	x_lb = np.array([[-3.], [-100.]]) # y, ydot
	x_ub = np.array([[+3.], [+100.]]) # y, ydot
	
	N = NoiseOptSolver(A, B, C, K, T, Sigma, x0, x_lb, x_ub, check_states=[0], noise_trunc=[10., 10.])
	
	print('maximum likelihood = %.3e\nwhere the noise sequence is:' %(N.max_cost))
	print(N.noise_seq)
	
	
