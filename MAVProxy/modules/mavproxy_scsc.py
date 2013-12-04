#!/usr/bin/env python
'''SCSC inspection flight script

   based on sensor test flight module, mavproxy_test.py

'''

import time

mpstate = None



### Control parameters

rf_min = 1             # Minimum rangefinder range considered valid
rf_max = 5             # Maximum rangefinder range considered valid
rf_top_count = 20      # Number of consecutive samples above max before recognizing the top of the chimney

safety_ceiling = 100   # Height in metres at which we will abort the search for chimney top
safety_climb = 1       # Height in metres above the top of the chimney to climb to before placing sensor

yaw_angle = 90         # Angle to yaw through to place the sensor (if the sign is changed,
                       # the yaw functions below should be too)

yaw_tolerance = 10     # Tolerance with which to try and reach the above angle

sample_time = 30       # Time in seconds spent sampling at the top of the chimney

### Control parameters


### Craft control functions

def climb():
    mpstate.functions.process_stdin("rc 3 1600")

def stop_climb():
    mpstate.functions.process_stdin("rc 3 1400")

def yaw_in():
    mpstate.functions.process_stdin("rc 4 1600")

def stop_yaw():
    mpstate.functions.process_stdin("rc 4 1500")

def yaw_out():
    mpstate.functions.process_stdin("rc 4 1400")

def land():
    mpstate.functions.process_stdin("mode land")

### End craft control functions

def name():
    '''return module name'''
    return "scsc"

def description():
    '''return module description'''
    return "SCSC inspection flight script"

def enum(**enums):
    return type('Enum', (), enums)

InspectState = enum(INIT=0, WAIT_LOITER=1, WAIT_RANGE=2, CLIMB=3, CONFIRM_TOP=4, PLACE_SENSOR=5, SAMPLE=6, REMOVE_SENSOR=7, LAND=8, SLEEP=9)


class inspect_state(object):
    def __init__(self):
        self.state = InspectState.INIT
        self.dist = -1
        self.tcount = 0
        self.chim_yaw = 0
        self.sample_time = 0

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.state = inspect_state()
    mpstate.command_map['scsc'] = (cmd_scsc, "Control SCSC inspection program")
    print("Module SCSC loaded")

def cmd_scsc(args):
    if len(args) == 1:
        if args[0] == 'start':
            if mpstate.state.state == InspectState.INIT:
                print("Starting SCSC inspection program")
                mpstate.state.state = InspectState.WAIT_LOITER
            else:
                print("Already started!")
        elif args[0] == 'abort':
            mpstate.state.state = InspectState.INIT
            print("SCSC program aborted")
    else:
        print("Usage: scsc <start|abort>")

def angle_diff(angle1, angle2):
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret


def mavlink_packet(m):
    '''handle an incoming mavlink packet'''

    if "RANGEFINDER" in mpstate.status.msgs:
        mpstate.state.dist = mpstate.status.msgs['RANGEFINDER'].distance

    if "GLOBAL_POSITION_INT" in mpstate.status.msgs:
        cur_yaw = mpstate.status.msgs['GLOBAL_POSITION_INT'].hdg / 100
    else:
        cur_yaw = 0

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
            mpstate.state.tcount += 1

            if mpstate.state.tcount >  rf_top_count:
                print("Yep, it is.  Placing sensor")
                yaw_in()
                mpstate.state.chim_yaw = cur_yaw
                mpstate.state.state = InspectState.PLACE_SENSOR
        else:
            print("False trigger, climbing some more")
            mpstate.state.tcount = 0
            climb()
            mpstate.state.state = InspectState.CLIMB
    if mpstate.state.state == InspectState.PLACE_SENSOR:
        # Wait to yaw around to face the chimney (within tolerance)
        if abs(angle_diff(cur_yaw, mpstate.state.chim_yaw) - yaw_angle) < yaw_tolerance:
            print("Sensor in position")
            stop_yaw()
            mpstate.state.sample_time = time.time()
            mpstate.state.state = InspectState.SAMPLE

    if mpstate.state.state == InspectState.SAMPLE:
        # Wait for the sample time to expire
        if time.time() - mpstate.state.sample_time >= sample_time:
            print("Sample completed")
            yaw_out()
            mpstate.state.state = InspectState.REMOVE_SENSOR

    if mpstate.state.state == InspectState.REMOVE_SENSOR:
        # Wait to yaw back to the previous orientation (within tolerance) then land
        if abs(angle_diff(cur_yaw, mpstate.state.chim_yaw)) < yaw_tolerance:
            print("Yaw back, ready to land")
            stop_yaw()
            land()
            mpstate.state.state = InspectState.LAND

    if mpstate.state.state == InspectState.LAND:
        if mpstate.status.altitude < 1:
            print("Great success")
            mpstate.state.state = InspectState.SLEEP



