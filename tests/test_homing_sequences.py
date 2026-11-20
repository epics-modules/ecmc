#!/usr/bin/env python3
"""Step production homing and position compare without an IOC or motor record."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SUP = ROOT / 'devEcmcSup'
OFFLINE = ROOT / 'tools/plc-test/offline'
MOTION = (
    'ecmcAxisBase', 'ecmcAxisVirt', 'ecmcAxisData', 'ecmcAxisSequencer',
    'ecmcEncoder', 'ecmcMonitor', 'ecmcTrajectoryBase', 'ecmcTrajectoryTrapetz',
    'ecmcFilter', 'ecmcMotionUtils', 'ecmcPositionCompare', 'ecmcPIDController',
    'ecmcAxisPVTSequence',
)

with tempfile.TemporaryDirectory(prefix='ecmc-homing-') as directory:
    executable = Path(directory) / 'homingTests'
    command = [os.environ.get('CXX', 'c++'), '-std=c++17', '-O0', '-g0',
               '-Wall', '-Wextra', '-UNDEBUG', '-fsanitize=address,undefined,vptr',
               '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
    command += ['-I' + str(OFFLINE)]
    command += ['-I' + str(SUP / name)
                for name in ('motion', 'main', 'misc', 'com', 'ethercat', 'sequence')]
    command += [str(ROOT / 'tests/homing/homing_checks.cpp'),
                str(OFFLINE / 'motion_support.cpp')]
    command += [str(SUP / name) for name in ('main/ecmcError.cpp', 'com/ecmcDataLink.cpp',
                                            'com/ecmcDataAccess.cpp', 'com/ecmcDataItem.cpp')]
    command += [str(SUP / 'motion' / (name + '.cpp')) for name in MOTION]
    subprocess.run([*command, '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
