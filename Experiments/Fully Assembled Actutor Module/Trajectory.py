import numpy as np

class trajectory:
	def __init__(self, q_i, q_f, t_i, t_f, max_v=[], max_a=[]):
		self.q_i = q_i
		self.q_f = q_f
		self.t_i = t_i
		self.t_f = t_f
		self.max_v = max_v
		self.max_a = max_a

		self.D = q_f - q_i

	def r_quintic(self, t_c):
		'''
		Linear and Cubic are useless
		Interpolates between t_i and t_f with a quintic polynomial
		Also calculates the derivative, to be used for expected velocity
		'''

		current_step = t_c/self.t_f
		r = 10 * current_step**3 - 15 * current_step**4 + 6 * current_step**5
		r_prime = 30 * ((t_c**2)/(self.t_f**3)) - 60 * ((t_c**3)/(self.t_f**4)) + 30 * ((t_c**4)/(self.t_f**5))
		
		return r, r_prime
		
	def step(self, t):
		t_c = t - self.t_i
		r, r_prime = self.r_bang_bang(t_c)

		q = self.q_i + r * self.D
		q_prime = r_prime * self.D

		return q, q_prime

	def r_bang_bang(self, t):

		"""
		Into the room
		"""
		current_step = t/self.t_f
		if t <= (self.t_f/2):
			r = 2 * current_step**2
			r_prime = 4 * t/(self.t_f**2)

		elif t >= (self.t_f/2) and t <= self.t_f:
			r = -1 + 4 * current_step - 2 * current_step**2
			r_prime = 4/self.t_f - 4 * t/(self.t_f**2)

		return r, r_prime