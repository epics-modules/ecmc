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

class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_PROBLEM  = 1 << (10 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = epics.Motor(os.getenv("TESTEDMOTORAXIS"))

    range_postion    = m1.DHLM - m1.DLLM
    homing_velocity  = m1.HVEL
    jogging_velocity = m1.JVEL
    moving_velocity  = m1.VELO
    acceleration     = m1.ACCL
    msta             = int(m1.MSTA)

    def setUp(self):
        print 'set up'

    def tearDown(self):
        print 'clean up'
        self.m1.put('STOP', 1)

    # Assert if motor is not homed
    def test_TC_1221(self):
        tc_no = "TC-1221"
        if not (self.msta & self.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')



    # high limit switch
    def test_TC_1222(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1222-high-limit-switch"
            print '%s' % tc_no
            old_high_limit = self.m1.get('DHLM')
            old_low_limit = self.m1.get('DLLM')
            #switch off the soft limits. Depending on the postion
            # low or high must be set to 0 first
            if self.m1.get_position(readback=True,dial=True) > 0:
                self.m1.put('DLLM', 0.0)
                self.m1.put('DHLM', 0.0)
            else:
                self.m1.put('DHLM', 0.0)
                self.m1.put('DLLM', 0.0)

            jogDirection(self.m1, tc_no, 1, self.jogging_velocity, self.acceleration)
            # Get values, check them later
            lvio = int(self.m1.get('LVIO'))
            msta = int(self.m1.get('MSTA'))
            #Go away from limit switch
            self.m1.move(val=old_high_limit, wait=True)
            self.m1.put('DLLM', old_low_limit)
            self.m1.put('DHLM', old_high_limit)

            self.assertEqual(0, lvio, 'LVIO == 0')
            self.assertEqual(0, msta & self.MSTA_BIT_PROBLEM,    'No Error MSTA.Problem at PLUS_LS')
            self.assertEqual(0, msta & self.MSTA_BIT_MINUS_LS,   'Minus hard limit switch not active')
            self.assertNotEqual(0, msta & self.MSTA_BIT_PLUS_LS, 'Plus hard limit switch active')


