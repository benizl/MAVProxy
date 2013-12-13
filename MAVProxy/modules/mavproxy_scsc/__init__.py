#!/usr/bin/env python
'''SCSC inspection flight script

   based on sensor test flight module, mavproxy_test.py

'''

import time

import scsc_range

mpstate = None
controller = None


### Control parameters

rf_min = 1             # Minimum rangefinder range considered valid
rf_max = 5             # Maximum rangefinder range considered valid
rf_top_count = 20      # Number of consecutive samples above max before recognizing the top of the chimney

safety_ceiling = 100   # Height in metres at which we will abort the search for chimney top
safety_climb = 1       # Height in metres above the top of the chimney to climb to before placing sensor

yaw_angle = 90         # Angle to yaw through to place the sensor (if the sign is changed,
                       # the yaw functions below should be too)

yaw_tolerance = 5      # Tolerance with which to try and reach the above angle

sample_time = 30       # Time in seconds spent sampling at the top of the chimney

land_alt = 3           # Altitude in meters to switch from loiter descent to LAND mode

### Control parameters


### Craft control functions

def climb():
    mpstate.state.controller.engage()
    mpstate.functions.process_stdin("rc 3 1650")

def stop_climb():
    mpstate.state.controller.disengage()
    mpstate.functions.process_stdin("rc 3 1400")

def yaw_in():
    mpstate.functions.process_stdin("rc 4 1575")

def stop_yaw():
    mpstate.functions.process_stdin("rc 4 1500")

def yaw_out():
    mpstate.functions.process_stdin("rc 4 1425")

def descend():
    mpstate.state.controller.engage()
    mpstate.functions.process_stdin("rc 3 1200")

def land():
    mpstate.state.controller.disengage()
    mpstate.functions.process_stdin("mode land")

def abort():
    # On abort, stop whatever we're doing and enforce loiter
    # (in case what we were doing was landing)
    stop_climb()
    stop_yaw()
    mpstate.state.controller.disengage() # Done by stop_climb also..
    mpstate.functions.process_stdin("loiter")

### End craft control functions

def name():
    '''return module name'''
    return "scsc"

def description():
    '''return module description'''
    return "SCSC inspection flight script"

def print_usage():
    print("Usage: scsc <start|abort|show|set|force_ctrl>")

def enum(**enums):
    return type('Enum', (), enums)

InspectState = enum(INIT=0, WAIT_LOITER=1, WAIT_RANGE=2, CLIMB=3, CONFIRM_TOP=4, SAFETY_CLIMB=5, PLACE_SENSOR=6, SAMPLE=7, REMOVE_SENSOR=8, DESCEND=9, LAND=10, SLEEP=11)


class inspect_state(object):
    def __init__(self):
        self.state = InspectState.INIT
        self.dist = -1
        self.cur_yaw = 0
        self.tcount = 0
        self.chim_yaw = 0
        self.chim_top = 0
        self.sample_time = 0

	self.ctrl_in_alt_hold = False

def init(_mpstate):
    '''initialise module'''
    global mpstate, controller
    mpstate = _mpstate
    mpstate.state = inspect_state()
    mpstate.command_map['scsc'] = (cmd_scsc, "Control SCSC inspection program")

    # TODO: Autodetect laser and/or use the correct by-id device
    mpstate.state.controller = scsc_range.RelPositionController(mpstate)

    print("SCSC Inspection loaded.  Begin program with 'scsc start'")

def idle_task():
    mpstate.state.controller.process_commands()

def cmd_scsc(args):
    if len(args) > 0:
        if args[0] == 'start':
            if mpstate.state.state == InspectState.INIT:
                print("Starting SCSC inspection program")
                mpstate.state.state = InspectState.WAIT_LOITER
            else:
                print("Already started!")
        elif args[0] == 'abort':
            mpstate.state.state = InspectState.INIT
            mpstate.state.ctrl_in_alt_hold = False
            abort()
            print("SCSC program aborted")
        elif args[0] == 'show':
            p = mpstate.state.controller.get_params()

            for k in sorted(p):
                print("\t{:<20}: {}".format(k, p[k]))
        elif args[0] == 'set':
            if len(args) == 3:
                try:
                    mpstate.state.controller.set_param(args[1], args[2])
                except AttributeError:
                    print("Can't find parameter {}".format(args[1]))
            else:
                print("usage: scsc set <param> <val>")
        elif args[0] == 'force_ctrl':
            mpstate.state.controller.engage()
        elif args[0] == 'ctrl_ah':
            mpstate.state.ctrl_in_alt_hold = True
        else:
            print_usage()
    else:
        print_usage()

def angle_diff(angle1, angle2):
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''

    mpstate.state.controller.handle_message(m)

    if (mpstate.status.flightmode == 'ALT_HOLD' and
       mpstate.state.ctrl_in_alt_hold and not mpstate.state.controller.engaged()):
        mpstate.state.controller.engage()

    if (mpstate.status.flightmode != 'ALT_HOLD' and
       mpstate.state.ctrl_in_alt_hold and mpstate.state.controller.engaged()):
        mpstate.state.controller.disengage()

    if m.get_type() == "RANGEFINDER":
        mpstate.state.dist = m.distance

    if m.get_type() == "GLOBAL_POSITION_INT":
        mpstate.state.cur_yaw = m.hdg / 100

    if (mpstate.state.state > InspectState.WAIT_LOITER and
      mpstate.state.state < InspectState.LAND and
      mpstate.status.flightmode != "LOITER"):

        print("User switched out of loiter, aborting")
        mpstate.state.state = InspectState.INIT

    if mpstate.state.state == InspectState.WAIT_LOITER:
        # Wait for the pilot to select loiter mode
        if mpstate.status.flightmode == "LOITER":
            print("Entered Loiter, waiting for range valid")
            mpstate.state.state = InspectState.WAIT_RANGE

    if mpstate.state.state == InspectState.WAIT_RANGE:
        # Wait for the rangefinder to show a valid distance then start climbing
        if rf_min < mpstate.state.dist < rf_max:
            print("Range valid, climbing")
            climb()
            mpstate.state.state = InspectState.CLIMB

    if mpstate.state.state == InspectState.CLIMB:
        # Climb until we reach the top of the chimney or a safety ceiling
        if mpstate.status.altitude > safety_ceiling:
            print("Ceiling breached!  Landing")
            land()
            mpstate.state.state = InspectState.LAND
        elif mpstate.state.dist > rf_max:
            print("This might be the top")
            stop_climb()
            mpstate.state.state = InspectState.CONFIRM_TOP

    if mpstate.state.state == InspectState.CONFIRM_TOP:
        # Count 10 successive range measurements over the 5m threshold.
        # If we get them, record our orientation and start yawing through
        # 90 degrees.  If not, restart the climb
        if mpstate.state.dist > rf_max:
            # Count the number of RANGEFINDER updates only
            if m.get_type() == 'RANGEFINDER':
                mpstate.state.tcount += 1

            if mpstate.state.tcount >  rf_top_count:
                print("Yep, it is.  Climbing a little for safety")
                climb()
                mpstate.state.chim_yaw = mpstate.state.cur_yaw
                mpstate.state.chim_top = mpstate.status.altitude
                mpstate.state.state = InspectState.SAFETY_CLIMB
        else:
            print("False trigger, climbing some more")
            mpstate.state.tcount = 0
            climb()
            mpstate.state.state = InspectState.CLIMB

    if mpstate.state.state == InspectState.SAFETY_CLIMB:
        if mpstate.status.altitude >= mpstate.state.chim_top + safety_climb:
            print("Safe distance reached, placing sensor")
            stop_climb()
            yaw_in()
            mpstate.state.state = InspectState.PLACE_SENSOR

    if mpstate.state.state == InspectState.PLACE_SENSOR:
        # Wait to yaw around to face the chimney (within tolerance)
        if abs(angle_diff(mpstate.state.cur_yaw, mpstate.state.chim_yaw) - yaw_angle) < yaw_tolerance:
            print("Sensor in position")
            stop_yaw()
            mpstate.state.sample_time = time.time()
            mpstate.state.state = InspectState.SAMPLE

    if mpstate.state.state == InspectState.SAMPLE:
        # Wait for the sample time to expire
        if time.time() - mpstate.state.sample_time >= sample_time:
            print("Sample completed, yawing back")
            yaw_out()
            mpstate.state.state = InspectState.REMOVE_SENSOR

    if mpstate.state.state == InspectState.REMOVE_SENSOR:
        # Wait to yaw back to the previous orientation (within tolerance) then land
        if abs(angle_diff(mpstate.state.cur_yaw, mpstate.state.chim_yaw)) < yaw_tolerance:
            print("Descending")
            stop_yaw()
            descend()
            mpstate.state.state = InspectState.DESCEND

    if mpstate.state.state == InspectState.DESCEND:
        if mpstate.status.altitude < land_alt:
            print("Landing")
            land()
            mpstate.state.state = InspectState.LAND

    if mpstate.state.state == InspectState.LAND:
        if mpstate.status.altitude < 0.5:
            print("Great success")
            mpstate.state.state = InspectState.SLEEP



