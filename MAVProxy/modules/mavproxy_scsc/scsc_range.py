
import threading, multiprocessing, Queue
import time

import hokuyo

class RelPositionController:
	def __init__(self, mpstate):

		self._mpstate = mpstate
		self.target_dist_m = 4

		self.ranger_max_m = 5.5
		self.ranger_min_m  = 0.2

		self.pitch_dist_p = 0.5
		self.pitch_dist_i = 0
		self.pitch_dist_d = 0

		self.roll_dist_p = 0.5
		self.roll_dist_i = 0
		self.roll_dist_d = 0

		self.pitch_rate_p = 100
		self.pitch_rate_i = 0
		self.pitch_rate_d = 0

		self.roll_rate_p = 100
		self.roll_rate_i = 0
		self.roll_rate_d = 0

		self.pitch_channel = 2
		self.roll_channel = 1

		self.mid_rc = 1500

		self.control_period_ms = 50 # 20 Hz

		self.mav_len = 3
		self.r_mav = []
		self.b_mav = []

		self._engaged_lock = threading.Lock()
		# Lock is created unlocked, take it so the controller thread
		# starts up paused
		self._engaged_lock.acquire()

		self._rc_queue = multiprocessing.Queue()
		self._ranger_queue = multiprocessing.Queue()

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
					self._mpstate.status.override[i] = 0 # Or 65535, unclear..

			self._mpstate.override_period.force()


	def handle_message(self, m):
		if m.get_type() == 'RANGEFINDER':
			# TODO: Hacked bearing in radians in to voltage field,
			# should have its own message type

			self.r_mav.append(m.distance)
			if len(self.r_mav) > self.mav_len:
				self.r_mav.pop(0)

			self.b_mav.append(m.voltage)
			if len(self.b_mav) > self.mav_len:
				self.b_mav.pop(0)

			r = sum(self.r_mav) / len(self.r_mav)
			b = sum(self.b_mav) / len(self.b_mav)

			sample = (r, b)
			self._ranger_queue.put(sample)

	def set_param(self, param, val):
		''' Update module parameters, such as gains and targets.
		    Implemented in this way so the param can easily be a string
		    passed in from the user.  Don't have to take any locks for
		    these as the granularity imposed by the interpretter lock
		    is good enough.'''
		if param in self.__dict__:
			self.__dict__[param] = float(val)
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

		# Blocked here until we're engaged (by the module code above
		# releasing this lock)
		self._engaged_lock.acquire()
		print("SCSC Engaged")
		last_time = time.time()

		while not self._terminate:

			#try:
			ranger_dist, ranger_bear = self._ranger_queue.get(True)
			print(ranger_dist, ranger_bear)
			#except Queue.Empty:
			#	pass

			if ranger_dist < self.ranger_min_m or ranger_dist > self.ranger_max_m:
				# Wait until we actually have valid ranger data
				#time.sleep(0.5)
				print("Waiting for range ({})".format(ranger_dist))
				continue

			dt = time.time() - last_time

			if last_time == 0:
				p_last_dist = ranger_dist
				r_last_dist = 0
				last_time = time.time()
				continue

			last_time = time.time()

			#if dt > 2.0 * self.control_period_ms / 1000.0:
			#	print("WARNING: Control period slip {}".format(dt))
				# TODO: Probably reset integrators here, or at least
				# degrade them (when we have integrators!)

			p_error = (self.target_dist_m - ranger_dist) * cos(ranger_bear)
			# Return a target speed, positive away from target, negative towards wall.
			p_rate_target = p_error * self.pitch_dist_p
			# Constrain target to 1 m/s  for sanity.
			p_rate_target = RelPositionController._constrain(p_rate_target, -1, 1)
        
			# Current speed, in m/s. Positive away from wall. Negative towards wall.
			p_rate_current = (ranger_dist * cos(ranger_bear) - p_last_dist) / dt

			if p_rate_current > 4:
				print("Pitch rate whacky: {} {} {} {} {} {}".format(p_error, p_rate_target, p_rate_current, ranger_dist * cos(ranger_bear), p_last_dist, dt))

			# Speed Error, in m/seconds. Positive away from wall.
			p_rate_error = p_rate_target - p_rate_current
			p_rate_error = RelPositionController._constrain(p_rate_error, -4, 4)

			# Return positive pitch (nose up) to accelerate away from wall.
			# TODO: I, D
			p_ctrl = p_rate_error * self.pitch_rate_p
			p_ctrl = RelPositionController._constrain(p_ctrl, -500, 500)
			p_last_dist = ranger_dist * cos(ranger_bear)

			r_error = ranger_dist * sin(-ranger_bear) #* cos(ranger_bear)
			r_rate_target = r_error * self.roll_dist_p
			r_rate_target = RelPositionController._constrain(r_rate_target, -1, 1)
        
			r_rate_current = (ranger_dist * sin(ranger_bear) - r_last_dist) / dt
			r_rate_error = r_rate_target - r_rate_current
			r_rate_error = RelPositionController._constrain(r_rate_error, -4, 4)

			# TODO: I, D
			r_ctrl = r_rate_error * self.roll_rate_p
			r_ctrl = RelPositionController._constrain(r_ctrl, -500, 500)
			r_last_dist = ranger_dist * sin(ranger_bear)

			print("R: {} {} {} {} {} {}".format(r_error, r_rate_target, r_rate_current, r_rate_error, r_ctrl, r_last_dist))

			controls = {self.pitch_channel: p_ctrl + self.mid_rc,
			            self.roll_channel : r_ctrl + self.mid_rc }

			self._rc_queue.put(controls)

			#print(p_error, r_error, p_rate_error, r_rate_error, controls)

			# Horrid sleeping pattern, I've become everything I've ever hated..
			to_sleep = last_time - time.time() + (self.control_period_ms / 1000.0)
			time.sleep(to_sleep if to_sleep > 0 else 0)

			self._engaged_lock.release()

			# We aquire immediately before the termination condition is checked
			# This means that if we are asked to terminate then unlocked in order
			# to check that condition, we don't complete a single unexpected
			# control loop iteration
			self._engaged_lock.acquire()








