#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
# http://cars9.uchicago.edu/software/python/pyepics3/devices.html#module-motor
#
# m-epics-singlemotion/src/main/test/singlemotion_test.py
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
import math
import time
###

polltime = 0.2

def waitForStartAndDone(motor, tc_no, wait_for_done):
    wait_for_start = 2
    while wait_for_start > 0:
        wait_for_start -= polltime
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_start=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_start, dmov, movn, motor.get_position(readback=True,dial=True))
        if movn and not dmov:
           break
        time.sleep(polltime)

    wait_for_done = math.fabs(wait_for_done) #negative becomes positive
    wait_for_done += 1 # One extra second for rounding
    while wait_for_done > 0:
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_done=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_done, dmov, movn, motor.get_position(readback=True,dial=True))
        if dmov and not movn:
            return True
        time.sleep(polltime)
        wait_for_done -= polltime
    return False

def jogDirection(motor, tc_no, direction, jogging_velocity, acceleration):
    if direction > 0:
        destination = motor.DHLM
        motor.put('JOGF', 1)
    else:
        destination = motor.DLLM
        motor.put('JOGR', 1)

    time_to_wait = 30
    if jogging_velocity > 0:
        if motor.DHLM != motor.DLLM:
            distance = math.fabs(motor.get_position(readback=True,dial=True) - destination)
            time_to_wait += distance / jogging_velocity + 2 * acceleration
    done = waitForStartAndDone(motor, tc_no, time_to_wait)

    if direction > 0:
        motor.put('JOGF', 0)
    else:
        motor.put('JOGR', 0)

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def moveDialPosition(motor, tc_no, destination, velocity, acceleration):
    time_to_wait = 30
    if velocity > 0:
        distance = math.fabs(motor.get_position(readback=True,dial=True) - destination)
        time_to_wait += distance / velocity + 2 * acceleration
    motor.put('DVAL', destination)
    done = waitForStartAndDone(motor, tc_no, time_to_wait)


class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = epics.Motor(os.getenv("TESTEDMOTORAXIS"))

    middle_dialPosition  = round((m1.DLLM + m1.DHLM) / 2)
    range_postion    = m1.DHLM - m1.DLLM
    homing_velocity  = m1.HVEL
    jogging_velocity = m1.JVEL
    moving_velocity  = m1.VELO
    acceleration     = m1.ACCL

    saved_high_limit = m1.get('DHLM')
    saved_low_limit = m1.get('DLLM')

    print "m1.DLLM=%d m1.DHLM=%d middle_dialPosition=%d" % (m1.DLLM, m1.DHLM, middle_dialPosition)

    def setUp(self):
        print 'set up'
        mymiddle_dialPosition  = int((self.m1.DLLM + self.m1.DHLM) / 2)
        print 'self.m1.DLLM=%f self.m1.DHLM=%f self.middle_dialPosition=%f' % (self.m1.DLLM, self.m1.DHLM, self.middle_dialPosition)
        print 'self.m1.DLLM=%f self.m1.DHLM=%f mymiddle_dialPosition=%f' % (self.m1.get('DLLM'), self.m1.get('DHLM'), mymiddle_dialPosition)

    def tearDown(self):
        print 'clean up'
        self.m1.put('STOP', 1)
        if self.saved_high_limit > self.saved_low_limit:
            self.m1.put('DHLM', self.saved_high_limit)
            self.m1.put('DLLM', self.saved_low_limit)



    # Home the motor
    def test_TC_100(self):
        tc_no = "TC-100"
        print '%s Home the motor forward' % tc_no
        self.m1.put('HOMF', 1)
        time_to_wait = 30
        if self.range_postion > 0 and self.homing_velocity > 0:
            time_to_wait = 1 + self.range_postion / self.homing_velocity + 2 * self.acceleration

        # Homing velocity not implemented, wait longer
        time_to_wait = 180
        done = waitForStartAndDone(self.m1, tc_no, time_to_wait)

        msta = int(self.m1.get('MSTA'))
        self.assertEqual(True, done, 'done = True')
        self.assertNotEqual(0, msta & self.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')


