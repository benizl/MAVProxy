
import hokuyo
import threading, multiprocessing, Queue
import time

mpstate = None

def name():
    '''return module name'''
    return "hokuyo"

def description():
    '''return module description'''
    return "Hokuyo Laser Rangefinder message generator"

def print_usage():
    print("Usage: hokuyo <show|set>")

def init(_mpstate):
	global mpstate
	mpstate = _mpstate
	mpstate.laser_ranger = laser_ranger()
	mpstate.command_map['hokuyo'] = (cmd_hokuyo, "Control Hokuyo Rangefinder driver")

def idle_task():
	mpstate.laser_ranger.process_ranges()

def unload():
	mpstate.laser_ranger.unload()


def cmd_hokuyo(args):
	if len(args) > 0:
		if args[0] == 'show':
			p = mpstate.laser_ranger.get_params()

			for k in sorted(p):
				print("\t{:<20}: {}".format(k, p[k]))
		elif args[0] == 'set':
			if len(args) == 3:
				try:
					mpstate.state.controller.set_param(args[1], args[2])
				except AttributeError:
					print("Can't find parameter {}".format(args[1]))
			else:
				print("usage: hokuyo set <param> <val>")
		else:
			print_usage()
	else:
		print_usage()

class laser_ranger:

	def __init__(self):

		# TODO: Auto-detect path, allow manual override etc
		self._laser = hokuyo.HokuyoURG('/dev/ttyACM0')

		self.min_laser_index = self._laser.steps / 4
		self.max_laser_index = self._laser.steps * 3 / 4

		self.cluster_thresh = 200   # 20cm
		self.cluster_min_dist = 100 # 10cm
		self.cluster_min_count = 10

		self._terminate = False

		self._laser_queue = multiprocessing.Queue()

		self._laser_thread = threading.Thread(target=self.laser_thread)
		self._laser_thread.start()


	def process_ranges(self):
		try:
			r = self._laser_queue.get(False)
		except Queue.Empty:
			pass
		else:
			# Inject a RANGEFINDER message in to MAVProxy's incoming
			# queue, emulating a packet from the APM.  In this way
			# other modules are agnostic of whether the RANGEFINDER
			# message originates on the APM or the MAVProxy host
			# TODO: Tridge suggests that the callback should be added
			# to mpstate.functions as this is the right way to expose
			# MAVProxy internals to modules
			master = mpstate.master()
			msg = master.mav.rangefinder_encode(
				r[1] / 1000.0,
				r[2])
			master.mav.callback(msg, master)
			

	def laser_thread(self):
		''' Laser thread.
		    Sets up scans, calculates range and bearing to object and
		    pushes this tuple to the laser queue to be read by the controller'''
		self._laser.start_scan()

		while not self._terminate:
			min_dist = 5600
			min_angle = 0
			cluster_count = 0
			d = self._laser.read_scan()

			for i in range(self.min_laser_index, self.max_laser_index):
				if abs(d[i] - d[i-1]) < self.cluster_thresh:
					cluster_count += 1
					if d[i] < min_dist and d[i] > self.cluster_min_dist:
						min_cluster_dist = d[i]
						min_cluster_angle = self._laser.index_to_radians(i)
				elif cluster_count > self.cluster_min_count:
					if min_cluster_dist < min_dist:
						min_dist = min_cluster_dist
						min_angle = min_cluster_angle
					cluster_count = 0
				else:
					cluster_count = 0

			# If we finished within a cluster we have to check minima here too
			if min_cluster_dist < min_dist:
				min_dist = min_cluster_dist
				min_angle = min_cluster_angle

			sample = (time.time(), min_dist, min_angle)

			self._laser_queue.put(sample)

		self._laser.end_scan()

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

	def unload(self):
		self._terminate = True
		self._laser_thread.join(5)
		if self._laser_thread.is_alive():
			print("WARNING: Couldn't stop Hokuyo thread")


