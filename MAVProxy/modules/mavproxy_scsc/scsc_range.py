
import threading, multiprocessing, Queue
import time

import hokuyo

class RelPositionController:
	def __init__(self, mpstate):

		self._mpstate = mpstate
		self.target_dist = 0

		self.pitch_dist_p = 0
		self.pitch_dist_i = 0
		self.pitch_dist_d = 0

		self.roll_dist_p = 0
		self.roll_dist_i = 0
		self.roll_dist_d = 0

		self.pitch_rate_p = 0
		self.pitch_rate_i = 0
		self.pitch_rate_d = 0

		self.roll_rate_p = 0
		self.roll_rate_i = 0
		self.roll_rate_d = 0

		self.pitch_channel = 1
		self.roll_channel = 2

		self.mid_rc = 1500

		self.control_period_ms = 10

		self._engaged_lock = threading.Lock()
		# Lock is created unlocked, take it so the controller thread
		# starts up paused
		self._engaged_lock.acquire()

		self._rc_queue = multiprocessing.Queue()

		self._terminate = False

		self._ctrl_running = False

		self._ctrl_thread = threading.Thread(target=self.control_thread)
		self._ctrl_thread.start()

	def engage(self):
		'''Release the engaged lock so the controller thread
		   can take it and go running'''
		if not self._ctrl_running:
			self._ctrl_running = True
			self._engaged_lock.release()

	def disengage(self):
		''' Take the engaged lock, this blocks the controller
		    thread'''
		if self._ctrl_running:
			self._engaged_lock.acquire()
			self._ctrl_running = False

	def process_commands(self):
		'''Called from the MAVProxy main loop.
		   Should check for updated RC values from the controller
		   pipe and load them in to the mpstate overrides. Don't block!
		'''
		overrides = None
		try:
			overrides = self._rc_queue.get(False)
		except Queue.Empty:
			pass
		else:
			for i in range(len(self._mpstate.status.override)):
				if i + 1 in overrides:
					self._mpstate.status.override[i] = overrides[i + 1]
				else:
					self._mpstate.override[i] = 0 # Or 65535, unclear..

			self._mpstate.override_period.force()


	def handle_message(self, m):
		pass

	def set_param(self, param, val):
		''' Update module parameters, such as gains and targets.
		    Implemented in this way so the param can easily be a string
		    passed in from the user.  Don't have to take any locks for
		    these as the granularity imposed by the interpretter lock
		    is good enough.'''
		if param in self.__dict__:
			self.__dict__[param] = val
		else:
			raise AttributeError(param)

	def get_params(self):
		public_params = {}

		for k in self.__dict__:
			if not k.startswith('_'):
				public_params[k] = self.__dict__[k]

		return public_params

	@staticmethod
	def _constrain(d, low, high):
		if d < low:
			return low
		elif d > high:
			return high
		else:
			return d

	def control_thread(self):
		from math import cos, sin
		'''Main control thread'''

		p_error = 0
		p_last_dist = 0
		p_rate_target = 0
		p_rate_current = 0
		p_rate_error = 0
		p_ctrl = 0

		r_error = 0
		r_last_dist = 0
		r_rate_target = 0
		r_rate_current = 0
		r_rate_error = 0
		r_ctrl = 0

		self._engaged_lock.acquire()

		last_time = time.time()

		ranger_dist = 0
		ranger_bear = 0

		while not self._terminate:
			dt = time.time() - last_time
			last_time = time.time()

			if dt > 2 * self.control_period_ms / 1000:
				print("WARNING: Control period slip {}".format(dt))
				# TODO: Probably reset integrators here, or at least
				# degrade them

			# Block until we get the first reading, then update non-blocking
			# from there.  TODO: Check timestamp for sanity.
			if ranger_dist == 0:
				(t, ranger_dist, ranger_bear) = self._laser_queue.get()
			else:
				(t, ranger_dist, ranger_bear) = self._laser_queue.get(False)


			p_error = (self.target_dist - ranger_dist) * cos(ranger_bear)
			# Return a target speed, positive away from target, negative towards wall.
			p_rate_target = p_error * self.pitch_dist_p
			# Constrain target to 100 cm/s  for sanity.
			p_rate_target = RelPositionController._constrain(p_rate_target, -100, 100)
        
			# Current speed, in cm/second. Positive away from wall. Negative towards wall.
			p_rate_current = (ranger_dist*cos(ranger_bear) - p_last_dist) / dt
			# Speed Error, in cm/seconds. Positive away from wall.
			p_rate_error = p_rate_target - p_rate_current
			p_rate_error = RelPositionController._constrain(p_rate_error, -400, 400)

			# Return positive pitch (nose up) to accelerate away from wall.
			# TODO: I, D
			p_ctrl = p_rate_error * self.pitch_rate_p


			r_error = self.target_dist * ranger_bear * cos(ranger_bear)
			r_rate_target = r_error * self.roll_dist_p
			r_rate_target = RelPositionController._constrain(r_rate_target, -100, 100)
        
			r_rate_current = (ranger_dist * cos(ranger_bear) - r_last_dist) / dt
			r_rate_error = r_rate_target - r_rate_current
			r_rate_error = RelPositionController._constrain(r_rate_error, -400, 400)

			# TODO: I, D
			r_ctrl = r_rate_error * self.roll_rate_p

			controls = {self.pitch_channel: p_ctrl + self.mid_rc,
			            self.roll_channel : r_ctrl + self.mid_rc }

			self._rc_queue.put(controls)

			# Horrid sleeping pattern, I've become everything I've ever hated..
			time.sleep(last_time - time.time() + (self.control_period_ms / 1000))

			self._engaged_lock.release()

			# We aquire immediately before the termination condition is checked
			# This means that if we are asked to terminate then unlocked in order
			# to check that condition, we don't complete a single unexpected
			# control loop iteration
			self._engaged_lock.acquire()








