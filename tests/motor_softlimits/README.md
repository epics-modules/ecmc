# Motor / ECMC soft-limit propagation

Run `python3 tests/test_motor_softlimit_propagation.py`, or the normal
`python3 tests/run_tests.py`. The separate CMake target compiles the real monitor,
encoder/axis data and error support under ASan/UBSan/vptr. Current motor-driver
limit methods are extracted on every run. Only parameter storage, base-class
callbacks, transaction endpoints and logging/publication are adapters. Normal
IOC build settings and runtime are unchanged.

## What the checks establish

| Requested mode | Checked behavior |
| --- | --- |
| ECMC limits disabled, motor limits enabled, no sync | Motor callbacks and polls do not enable/change ECMC limits. Config readbacks remain available. |
| ECMC limits enabled, motor limits disabled, no sync | Driver polls preserve an independently supplied zero motor pair; the real ECMC monitor still interlocks out-of-range targets. Startup caveat below applies. |
| Both enabled with a narrower motor range, no sync | The driver preserves the independent pairs. A wider motor pair does not widen ECMC's range; native enforcement remains active. This does not promise automatic clamping of motor fields to ECMC limits. |
| Sync enabled | Motor callbacks change ECMC limits; ECMC readbacks update motor RO parameters; flag changes, zero-pair disabling/re-enabling and leaving sync are exercised. Regular readback updates RO parameters, not the writable command cache directly. |

Independent layers are INFERRED from the sync option's purpose. Exact startup
seeding, readback and flag policy are COMPAT. The test does not reinterpret all
existing policy as an agreed requirement. See [EXPECTATIONS.md](../EXPECTATIONS.md).

## Actual motor-record OFF / DIR code

To execute the actual ESS record/device-support conversion and limit helpers,
provide a source directory containing `motorRecord.cc`, `motorDevSup.c` and
`devMotorAsyn.c`:

```sh
ECMC_TEST_MOTOR_SOURCE=/path/to/motor/motorApp/MotorSrc python3 tests/test_motor_softlimit_propagation.py
```

The script also recognizes the conventional adjacent `motor/` checkout and this
workspace's sibling `motorECMC_7.2.8.1/motor/` checkout. Missing optional sources
produce an explicit SKIP for the record section; core driver/monitor checks still
run. An explicitly configured missing/incompatible source fails. GitHub Actions
checks out [ESS motor commit 7da30b10](https://github.com/EuropeanSpallationSource/motor/commit/7da30b10b9ea160647e312b8ccef4b67fd32f82c)
and sets the path, so this section runs on both CI compilers. No installed EPICS
or motor library is needed.

The tests extract the real user/dial limit helpers, `special()` OFF/DIR cases,
native-unit conversion, limit transaction code and read-only-limit reception.
They check independently calculated ranges for OFF=0/+100/-100, DIR=Pos/Neg,
MRES=0.1/1/2.5/-0.1 and sync on/off. OFF changes user limits without shifting dial
or ECMC limits. DIR swaps/signs the user endpoints; Frozen and Variable OFF
behavior are exercised. MRES magnitude does not rescale EGU, while its sign
reverses the native direction. The zero dial pair remains a disabled motor range
even with nonzero OFF. See the [motor-record coordinate documentation](https://epics-modules.github.io/motor/motorRecord.html).

These are actual production conversion functions, not a second implementation
of their equations. Record storage, event publication, the full asyn transaction
transport and LVIO recalculation are outside the fixture. Native-limit readback
reception is exercised with positive MRES; negative-MRES writes/restoration are
covered, but full negative-MRES readback/IOC processing is not established here.

## Startup and read-only-limit caveats

With `restoreMotorSoftLims=0`, `initializeSoftLimits()` forces a one-time ECMC-to-
motor seed of the *writable* motor limits even when sync is disabled. Thus no-sync
is not a promise that motor limits are untouched throughout startup. With
restoration requested and sync disabled, initialization remains pending instead.
Both behaviors are checked.

Motor-local restore mode (`restoreMotorSoftLims=2`) suppresses ECMC read-only
limit publication while synchronization remains disabled. The ESS motor record
copies a newly published read-only pair into `DLLM`/`DHLM`; publishing the YAML
pair would therefore replace the independently autosave-restored motor limits.
ECMC continues enforcing its native limits. Enabling synchronization later
resumes normal synchronized readback behavior.

The driver asserts a valid read-only dial range only while soft-limit
synchronization is enabled; the startup seed and the sync on->off transition both
clear the range instead. This matters because the ESS motor record latches any
valid read-only range it receives and clamps later dial limit edits to it,
including the zero pair that disables the motor limits. The actual record helpers
are still checked against an explicitly latched range (device with its own travel
limits) and against an absent range. The [ESS soft-limit design
notes](https://github.com/EuropeanSpallationSource/motor/blob/7da30b10b9ea160647e312b8ccef4b67fd32c/docs/Softlimits.txt)
describe the separate read-only and read/write modes.

## Live slit acceptance cases

The ecmccfg repository contains `slit/cases/softlimit_modes.py` for the simulator
IOC and `slit_hardware/softlimits.py` for the hardware IOC. Both execute shared
four-mode and OFF/DIR verdicts. The hardware group is selected explicitly as
`--case softlimits` because it temporarily disables individual limit layers.
It performs configuration checks with drives disabled and issues no movement,
homing or position-loading commands. All original independent pairs, flags,
OFF/DIR/FOFF and state-machine enable are restored and acknowledged.

Acceptance failures remain visible if an IOC cannot establish the requested
independent zero pair or propagate limits. Cached read-only state and sync-flag-
only transitions are possible reasons; the cases do not silently enable sync or
replace their expected values with the observed values. Full motor-record
motion rejection, scheduling and hardware response still require the live IOC
cases and existing limit/motion groups.
