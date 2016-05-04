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

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    motm1   = epics.Motor(os.getenv("TESTEDMOTORAXIS"))
    pvm1 = epics.PV(os.getenv("TESTEDMOTORAXIS"))
    pvm1DLY  = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.DLY')
    pvm1ACCL = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.ACCL')
    pvm1VELO = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.VELO')
    pvm1DVAL = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.DVAL')

    range_postion    = motm1.DHLM - motm1.DLLM
    homing_velocity  = motm1.HVEL
    jogging_velocity = motm1.JVEL
    moving_velocity  = motm1.VELO
    acceleration     = motm1.ACCL

    saved_DHLM = motm1.get('DHLM')
    saved_DLLM = motm1.get('DLLM')
    saved_DLY = motm1.get('DLY')
    saved_VELO = motm1.get('VELO')
    saved_ACCL = motm1.get('ACCL')
    
    print 'motm1.DLLM=%d motm1.DHLM=%d' % (motm1.DLLM, motm1.DHLM)

    def setUp(self):
        print 'set up'
        print 'self.motm1.DLLM=%f self.motm1.DHLM=%f' % (self.motm1.DLLM, self.motm1.DHLM)
        print 'self.motm1.DLLM=%f self.motm1.DHLM=%f' % (self.motm1.get('DLLM'), self.motm1.get('DHLM'))

    def tearDown(self):
        print 'clean up'
        if self.saved_DHLM > self.saved_DLLM:
            self.motm1.put('DHLM', self.saved_DHLM)
            self.motm1.put('DLLM', self.saved_DLLM)
            self.motm1.put('DLY',  self.saved_DLY)
            self.motm1.put('VELO', self.saved_VELO)
            self.motm1.put('ACCL', self.saved_ACCL)

    
    # 10% dialPosition
    def test_TC_501(self):
        tc_no = "TC_501-10-percent-dialPosition"
        print '%s' % tc_no
        dval =  (self.saved_DHLM + 9 * self.saved_DLLM) / 10
        ret = self.motm1.move(dval, dial=True, wait=True)
        assert (ret == 0)
        drbv = self.motm1.get_position(readback=True,dial=True)
        print '%s postion=%f' % (tc_no, drbv)
        assert calcAlmostEqual(self.motm1, tc_no, dval, drbv, 2)
        
    # 10% dialPosition + X
    def test_TC_502(self):
        tc_no = "TC_502-10-percent-plus-1"
        print '%s' % tc_no
        dval = self.motm1.get_position(readback=True,dial=True) + 1
        self.motm1.DLY = 5.2
        self.motm1.VELO = 1
        self.motm1.ACCL = 1
        self.motm1.DVAL = dval

        time.sleep(4.0)
        movn1 = self.motm1.get('MOVN')
        self.motm1.STOP = 1
        time.sleep(7.0)
        dmov2 = self.motm1.get('DMOV')
        self.motm1.SPMG = 0
        self.motm1.SPMG = 3
        time.sleep(4.0)
        dmov3 = self.motm1.get('DMOV')
        print '%s: movn1=%d dmov2=%d dmov3=%d' % (tc_no, movn1, dmov2, dmov3)
        assert(dmov3)
        assert(dmov2)
