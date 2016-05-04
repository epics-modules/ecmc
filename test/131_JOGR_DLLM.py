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
    MSTA_BIT_PROBLEM  = 1 << (10 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = epics.Motor(os.getenv("TESTEDMOTORAXIS"))

    per10_dialPosition  = round((9 * m1.DLLM + 1 * m1.DHLM) / 10)

    range_postion    = m1.DHLM - m1.DLLM
    homing_velocity  = m1.HVEL
    jogging_velocity = m1.JVEL
    moving_velocity  = m1.VELO
    acceleration     = m1.ACCL
    msta             = int(m1.MSTA)

    print "m1.DLLM=%d m1.DHLM=%d per10_dialPosition=%d" % (m1.DLLM, m1.DHLM, per10_dialPosition)

    def setUp(self):
        print 'set up'
        print 'self.m1.DLLM=%f self.m1.DHLM=%f self.per10_dialPosition=%f' % (self.m1.DLLM, self.m1.DHLM, self.per10_dialPosition)

    def tearDown(self):
        print 'clean up'
        self.m1.put('STOP', 1)

    # Assert if motor is not homed
    def test_TC_1311(self):
        tc_no = "TC-1311"
        if not (self.msta & self.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')


    # per10 dialPosition
    def test_TC_1312(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1312-30-percent-dialPosition"
            print '%s' % tc_no
            destination = self.per10_dialPosition
            moveDialPosition(self.m1, tc_no, destination,
                             self.moving_velocity, self.acceleration)

            dialPosition = self.m1.get_position(readback=True,dial=True)
            print '%s postion=%f per10_dialPosition=%f' % (
                tc_no, dialPosition, self.per10_dialPosition)
            assert calcAlmostEqual(self.m1, tc_no, destination, dialPosition, 2)

    # Low soft limit JOGR
    def test_TC_1313(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1313-low-soft-limit JOGR"
            print '%s' % tc_no
            jogDirection(self.m1, tc_no, 0, self.jogging_velocity, self.acceleration)
            lvio = int(self.m1.get('LVIO'))
            msta = int(self.m1.get('MSTA'))

            self.assertEqual(0, msta & self.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGR')
            self.assertEqual(0, msta & self.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGR')
            self.assertEqual(0, msta & self.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGR')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGR')
            self.m1.put('JOGF', 0)


    # per10 dialPosition
    def test_TC_1314(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1314-30-percent-dialPosition"
            print '%s' % tc_no
            destination = self.per10_dialPosition
            moveDialPosition(self.m1, tc_no, destination,
                             self.moving_velocity, self.acceleration)

            dialPosition = self.m1.get_position(readback=True,dial=True)
            print '%s postion=%f per10_dialPosition=%f' % (
                tc_no, dialPosition, self.per10_dialPosition)
            assert calcAlmostEqual(self.m1, tc_no, destination, dialPosition, 2)

    # Low soft limit JOGF + DIR
    def test_TC_1315(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1315-low-soft-limit JOGF DIR"
            print '%s' % tc_no
            saved_DIR = self.m1.get('DIR')
            self.m1.put('DIR', 1)
            jogDirection(self.m1, tc_no, 1, self.jogging_velocity, self.acceleration)

            lvio = int(self.m1.get('LVIO'))
            msta = int(self.m1.get('MSTA'))

            self.assertEqual(0, msta & self.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGF DIR')
            self.assertEqual(0, msta & self.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF DIR')
            self.assertEqual(0, msta & self.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGR DIR')
            ### commit  4efe15e76cefdc060e14dbc3 needed self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            self.m1.put('JOGF', 0)
            self.m1.put('DIR', saved_DIR)



