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

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def waitForStart(motor, tc_no, wait_for_start):
    while wait_for_start > 0:
        wait_for_start -= polltime
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_start=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_start, dmov, movn, motor.get_position(readback=True,dial=True))
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for_start -= polltime
    return False

def waitForStop(motor, tc_no, wait_for_stop):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_stop=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_stop, dmov, movn, motor.get_position(readback=True,dial=True))
        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for_stop -= polltime
    return False

def getAcceleration(tc_no):
    print '%s: getAcceleration' % (tc_no)
    # TODO: MC_CPU1 needs to be a parameter
    epics.caput(os.getenv("TESTEDMCUASYN") + ".PORT", "MC_CPU1")

    epics.caput(os.getenv("TESTEDMCUASYN") + ".AOUT", "Main.M1.fAcceleration?")
    res = epics.caget (os.getenv("TESTEDMCUASYN") + ".AINP", as_string=True)
    print '%s: getAcceleration res=(%s)' % (tc_no, res)
    if res == "":
        time.sleep(polltime)
        res = epics.caget (os.getenv("TESTEDMCUASYN") + ".AINP", as_string=True)
        print '%s: getAcceleration res=(%s)' % (tc_no, res)
    return float(res + "0")

class Test(unittest.TestCase):
    MSTA_BIT_MOVING   = 1 << (11 -1)

    motm1   = epics.Motor(os.getenv("TESTEDMOTORAXIS"))
    pv_MSTA = epics.PV(os.getenv("TESTEDMOTORAXIS") + ".MSTA")
    pv_MCU_AOUT = epics.PV(os.getenv("TESTEDMCUASYN") + ".AOUT")
    pv_MCU_AINP = epics.PV(os.getenv("TESTEDMCUASYN") + ".AINP")

    saved_DHLM = motm1.get('DHLM')
    saved_DLLM = motm1.get('DLLM')
    saved_CNEN = motm1.get('CNEN')

    def setUp(self):
        print 'set up'
        self.motm1.put('CNEN', 1)


    def tearDown(self):
        print 'clean up'
        self.motm1.put('CNEN', self.saved_CNEN)


    # 10% dialPosition
    def test_TC_301(self):
        tc_no = "TC-301-10-percent-dialPosition"
        print '%s' % tc_no
        dval =  (self.saved_DHLM + 9 * self.saved_DLLM) / 10
        ret = self.motm1.move(dval, dial=True, wait=True)
        assert (ret == 0)
        drbv = self.motm1.get_position(readback=True,dial=True)
        print '%s dval=%f drbv=%f' % (tc_no, dval, drbv)
        assert calcAlmostEqual(self.motm1, tc_no, dval, drbv, 2)

    # 20% dialPosition
    def test_TC_302(self):
        tc_no = "TC-302-20-percent-dialPosition"
        print '%s' % tc_no
        motm1 = self.motm1
        saved_ACCL = motm1.get('ACCL')
        used_ACCL = saved_ACCL + 1.0 # Make sure we have an acceleration != 0
        motm1.put('ACCL', used_ACCL)

        dval =  (2 * self.saved_DHLM + 8 * self.saved_DLLM) / 10
        ret = self.motm1.move(dval, dial=True, wait=True)
        drbv = self.motm1.get_position(readback=True,dial=True)
        print '%s dval=%f drbv=%f' % (tc_no, dval, drbv)
        motm1.put('ACCL', saved_ACCL)
        saved_ACCL = None

        saved_VELO = motm1.get('VELO')
        expacc = saved_VELO / used_ACCL
        resacc = float(getAcceleration(tc_no))
        print '%s ACCL=%f VELO=%f expacc=%f resacc=%f' % (tc_no,used_ACCL,saved_VELO,expacc,resacc)
        assert calcAlmostEqual(self.motm1, tc_no, expacc, resacc, 2)
        assert (ret == 0)
        assert calcAlmostEqual(self.motm1, tc_no, dval, drbv, 2)


    # 30% dialPosition
    def test_TC_303(self):
        tc_no = "TC-303-30-percent-dialPosition"
        print '%s' % tc_no
        motm1 = self.motm1
        saved_ACCL = motm1.get('ACCL')
        used_ACCL = saved_ACCL + 2.0 # Make sure we have an acceleration != 0
        motm1.put('ACCL', used_ACCL)

        dval =  (3 * self.saved_DHLM + 7 * self.saved_DLLM) / 10
        ret = self.motm1.move(dval, dial=True, wait=True)
        drbv = self.motm1.get_position(readback=True,dial=True)
        print '%s dval=%f drbv=%f' % (tc_no, dval, drbv)
        motm1.put('ACCL', saved_ACCL)
        saved_ACCL = None

        saved_VELO = motm1.get('VELO')
        expacc = saved_VELO / used_ACCL
        resacc = float(getAcceleration(tc_no))
        print '%s ACCL=%f VELO=%f expacc=%f resacc=%f' % (tc_no,used_ACCL,saved_VELO,expacc,resacc)
        assert calcAlmostEqual(self.motm1, tc_no, expacc, resacc, 2)
        assert (ret == 0)
        assert calcAlmostEqual(self.motm1, tc_no, dval, drbv, 2)


    # Jog, wait for start, stop. check fAcceleration
    def test_TC_304(self):
        tc_no = "TC-304-JOG-fAcceleration"
        print '%s' % tc_no
        motm1 = self.motm1
        saved_ACCL = motm1.get('ACCL')
        used_ACCL = saved_ACCL + 0.75 # Make sure we have an acceleration != 0
        motm1.put('ACCL', used_ACCL)

        motm1.put('JOGF', 1)
        ret = waitForStart(self.motm1, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStart return True')

        motm1.put('JOGF', 0)
        ret = waitForStop(self.motm1, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStop return True')
        motm1.put('ACCL', saved_ACCL)
        saved_ACCL = None

        expacc = motm1.get('JAR')
        resacc = float(getAcceleration(tc_no))
        print '%s expacc=%f resacc=%f' % (tc_no,expacc,resacc)
        assert calcAlmostEqual(self.motm1, tc_no, expacc, resacc, 2)

    # Jog, wait for start, stop. check fAcceleration
    def test_TC_305(self):
        tc_no = "TC-305-JOG-fAcceleration"
        print '%s' % tc_no
        motm1 = self.motm1
        saved_JAR = motm1.get('JAR')
        used_JAR = saved_JAR + 0.5 # Make sure we have an acceleration != 0
        motm1.put('JAR', used_JAR)

        motm1.put('JOGF', 1)
        ret = waitForStart(self.motm1, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStart return True')

        motm1.put('JOGF', 0)
        ret = waitForStop(self.motm1, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStop return True')
        motm1.put('JAR', saved_JAR)
        saved_JAR = None

        expacc = used_JAR
        resacc = float(getAcceleration(tc_no))
        print '%s expacc=%f resacc=%f' % (tc_no,expacc,resacc)
        assert calcAlmostEqual(self.motm1, tc_no, expacc, resacc, 2)

#    # HOMF
# TODO: Send acceleration when homing in Axis.cpp and test it here
# 
#    def test_TC_306(self):
#        tc_no = "TC-306-20-HOMF-acceleration"
#        print '%s' % tc_no
#        motm1 = self.motm1
#        saved_ACCL = motm1.get('ACCL')
#        used_ACCL = saved_ACCL + 1.5 # Make sure we have an acceleration != 0
#        motm1.put('ACCL', used_ACCL)
#        motm1.put('HOMF', 1)
#        ret = waitForStart(self.motm1, tc_no, 2.0)
#        self.assertEqual(True, ret, 'waitForStart return True')
#
#        time_to_wait = 180
#        ret = waitForStop(self.motm1, tc_no, time_to_wait)
#        self.assertEqual(True, ret, 'waitForStop return True')
#        motm1.put('HOMF', 0)
#
#        motm1.put('ACCL', saved_ACCL)
#        saved_ACCL = None
#
#        used_HVEL = motm1.get('HVEL')
#        expacc = used_HVEL / used_ACCL
#        resacc = float(getAcceleration(tc_no))
#        print '%s ACCL=%f HVEL=%f expacc=%f resacc=%f' % (tc_no,used_ACCL,used_HVEL,expacc,resacc)
#        assert calcAlmostEqual(self.motm1, tc_no, expacc, resacc, 2)


