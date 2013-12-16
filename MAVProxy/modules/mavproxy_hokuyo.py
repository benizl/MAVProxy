
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

def dist(x1, y1, x2, y2):
	from math import sqrt
	return sqrt((x1 - x2)**2 + (y1 - y2)**2)

def circle_func(ps, x, y, r):
	''' Circle function. Take negative sqrt as the circle we're
	    fitting is generated from LOS measurements in the positive
	    y direction'''
	ys = []
	for p in ps:
		rs = r**2 - (p - x)**2
		if rs < 0:
			rs = 0
		ys.append(y - sqrt(rs))

	return numpy.array(ys)

def cmd_hokuyo(args):
	if len(args) > 0:
		if args[0] == 'show':
			p = mpstate.laser_ranger.get_params()

			for k in sorted(p):
				print("\t{:<20}: {}".format(k, p[k]))
		elif args[0] == 'set':
			if len(args) == 3:
				try:
					mpstate.laser_ranger.set_param(args[1], args[2])
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
		self._laser = hokuyo.HokuyoURG('/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00', initial_baud=115200, run_baud=115200)

		self.min_laser_index = self._laser.steps / 4
		self.max_laser_index = self._laser.steps * 3 / 4

		self.cluster_thresh = 200   # 20cm
		self.cluster_min_dist = 100 # 10cm
		self.cluster_min_count = 10

		self.laser_cluster = 1

		self.find_circles = 1
		self.use_regression = 0

		self._terminate = False

		self._circ_lr = 0
		self._circ_lb = 0
		self._loss_count = 0

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

			#print("Msg sent")
			

	def clustered_min(self, d):
		min_cluster_dist = 5600
		min_cluster_angle = 0
		min_dist = 5600
		min_angle = 0
		cluster_count = 0
		print("C MIN")
		for i in range(self.min_laser_index // self.laser_cluster,
		               self.max_laser_index // self.laser_cluster):
			if abs(d[i] - d[i-1]) < self.cluster_thresh:
				cluster_count += 1
				if d[i] < min_cluster_dist and d[i] > self.cluster_min_dist:
					min_cluster_dist = d[i]
					min_cluster_angle = self._laser.index_to_radians(i)
			elif cluster_count > self.cluster_min_count:
				if min_cluster_dist < min_dist:
					min_dist = min_cluster_dist
					min_angle = min_cluster_angle
				cluster_count = 0
				min_cluster_dist = 5600
				min_cluster_angle = 0
			else:
				cluster_count = 0
				min_cluster_dist = 5600
				min_cluster_angle = 0

		# If we finished within a cluster we have to check minima here too
		if min_cluster_dist < min_dist:
			min_dist = min_cluster_dist
			min_angle = min_cluster_angle

		return (time.time(), min_dist, min_angle)


	def find_circle_regressive(self, cd, cb):
		#print("CIR REG")
		circles = []
		cx = [ r * sin(bear) for r, bear in zip(cd, cb)]
		cy = [ r * cos(bear) for r, bear in zip(cd, cb)]

		centx0 = (cx[0] + cx[-1]) / 2.0
		centy0 = (cy[0] + cy[-1]) / 2.0

		r0 = dist(cx[0], cx[-1], cy[0], cy[-1]) / 2.0

		try:
			popt, pcov = scipy.optimize.curve_fit(circle_func, cx, cy,
				p0=(centx0, centy0, r0), xtol=0.001, ftol=0.001)
		except RuntimeError:
			return None #didn't converge

		try:
			popt[2] = abs(popt[2]) # Could have converge to -ve radius

			# Accept circles with radius between 10cm and under 3 metres
			# centroid variance better than 20cm and radial variance better
			# than 20cm. These must be a little larger than you might expect
			# do to point data corruption during rapid movement
			if popt[2] > 50 and popt[2] < 3000 and pcov[0,0] < 200 and pcov[1,1] < 200 and pcov[2,2] < 200:
				return popt
		except TypeError:
			pass # pcov is inf if the system didn't converge (and inf can't be subscripted)

		return None

	def find_circle_geometric(self, cd, cb):
		from math import sqrt, sin, cos, pi
		#print("CIR GEO")

		# This algorithm assumes that the end points are tangent to the circle,
		# but in fact it's very common for the absolute endpoints to be especially
		# noisy.  As such we use the samples one in from each end to calculate the
		# circle parameters; they're close enough to being tangent that it still works
		a, b = cd[1], cd[-2]
		ta, tb = cb[1], cb[-2]
		theta = tb - ta

		r = sqrt(a**2 + b**2 - 2 * a * b * cos(theta)) / 2
		r = r / (cos(theta / 2))

		xc = a * sin(ta) + r * sin(ta - pi/2)
		yc = a * cos(ta) + r * cos(ta - pi/2)

		xo = [ i * sin(j) for i, j in zip(cd, cb)]
		yo = [ i * cos(j) for i, j in zip(cd, cb)]

		ressqr = [(dist(xc, yc, x, y) - r)**2 for x, y in zip(xo, yo)]
		score = sqrt(sum(ressqr) / len(ressqr))

		# The score is the RMS residual to the circle in mm. The
		# threshold is set empirically
		if r > 50 and r < 3000 and score < 40:
			return (xc, yc, r)
		else:
			return None

	def clustered_circles(self, d, regressive=False):
		from math import atan2
		ranges, bearings = [],[]
		circles = []
		cluster_count = 0
		for i in range(self.min_laser_index // self.laser_cluster,
		               self.max_laser_index // self.laser_cluster):
			if abs(d[i] - d[i-1]) < self.cluster_thresh and d[i] > 100:
				cluster_count += 1
				ranges.append(d[i])
				bearings.append(-self._laser.index_to_radians(i))
			elif cluster_count > self.cluster_min_count:
				if regressive:
					c = self.find_circle_regressive(ranges, bearings)
				else:
					c = self.find_circle_geometric(ranges, bearings)
				if c is not None:
					circles.append(c)
				ranges, bearings = [], []
				cluster_count = 0
			else:
				cluster_count = 0
				ranges, bearings = [], []

		# If we finished within a cluster we have to check minima here too
		c = None
		if cluster_count > self.cluster_min_count:
			if regressive:
				c = self.find_circle_regressive(ranges, bearings)
			else:
				c = self.find_circle_geometric(ranges, bearings)

		if c is not None:
			circles.append(c)

		# If we didn't find any circles, return the last sample.  Not that
		# nice, but hopefully doesn't happen much..
		if len(circles) == 0:
			if self._loss_count < 5:
				self._loss_count += 1
				return (time.time(), self._circ_lr, self._circ_lb)
			else:
				return (time.time(), 0, 0)

		# Bearing to centre of closest circle, range to the edge of that
		# circle along that bearing
		d_min = 5600
		chosen = None
		for c in circles:
			d = dist(c[0], c[1], 0, 0)
			if d < d_min:
				d_min = d
				chosen = c

		bear = atan2(chosen[0], chosen[1])
		rang = d_min - chosen[2]

		self._circ_lr, self._circ_lb = rang, bear
		self._loss_count = 0

		return (time.time(), rang, bear)


	def laser_thread(self):
		''' Laser thread.
		    Sets up scans, calculates range and bearing to object and
		    pushes this tuple to the laser queue to be read by the controller'''

		self._laser.start_scan(cluster=self.laser_cluster)

		while not self._terminate:
			min_dist = 5600
			min_angle = 0
			cluster_count = 0
			d = self._laser.read_scan()

			#d = ([0] * 367) + [592, 574, 559, 554, 546, 540, 537, 533, 529, 527, 527, 524, 509, 508, 507, 502, 497, 497, 497, 495, 494, 495, 495, 494, 494, 493, 491, 491, 491, 491, 495, 495, 487, 493, 486, 486, 490, 493, 500, 508, 510, 511, 512, 513, 515, 523, 524, 531, 546, 546, 547, 552, 572, 573, 620] + ([0] * 347)

			if d is None:
			#	print("Scan failed")
				continue
			#else:
			#	print("Scan")

			if self.find_circles:
				sample = self.clustered_circles(d)
			else:
				sample = self.clustered_min(d)
	

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

		self._laser.close()


