# Cpp Logic Helpers

This page gives a compact reference for the helper headers under
[`devEcmcSup/logic/`](./).

It complements the higher-level IOC/user docs in `ecmccfg` and focuses on the
C++ helper API itself.

## Main Headers

- [`ecmcCppLogic.hpp`](./ecmcCppLogic.hpp)
  Core binding/export interface, host services, and registration macros.
- [`ecmcCppMotion.hpp`](./ecmcCppMotion.hpp)
  `MC_*` style motion wrappers on top of the existing `ecmc` motion backend.
- [`ecmcCppControl.hpp`](./ecmcCppControl.hpp)
  Control-oriented helpers such as `ecmcCpp::Pid`.
- [`ecmcCppUtils.hpp`](./ecmcCppUtils.hpp)
  Utility helpers for triggers, timers, latches, filtering, basic state
  handling, and EtherCAT status decoding.
- [`ecmcCppTrace.hpp`](./ecmcCppTrace.hpp)
  Reusable fixed-capacity triggered trace helper for exported waveforms.
- [`ecmcCppPersist.hpp`](./ecmcCppPersist.hpp)
  Small retained-value helper for infrequent load/save of trivially copyable
  parameters and state.

## Logic Base And Registration

Normal user code derives from `ecmcCpp::LogicBase` and finishes with:

```cpp
ECMC_CPP_LOGIC_REGISTER_DEFAULT(MyLogic)
```

Inside the logic class:

- `ecmc...` binds to live `ecmc` items
- `epics...` exports values on the dedicated C++ logic asyn port
- override `enterRealtime()` for setup that should run once when RT starts
- override `exitRealtime()` for one-time cleanup when RT stops
- keep `run()` for cyclic realtime work only

## ecmcCppLogic.hpp

Logic lifecycle:

- constructor: normal object construction and binding/export declarations
- `enterRealtime()`: called once when the logic enters RT
- `run()`: called cyclically at the configured execution rate
- `exitRealtime()`: called once when the logic leaves RT

Common scalar binding helpers:

- `ecmc.input("item", value)`
- `ecmc.output("item", value)`
- `epics.readOnly("name", value)`
- `epics.writable("name", value)`

Configuration text helper:

- `ecmcCpp::getMacrosString()`
- `ecmcCpp::getMacroValue(...)`
- `ecmcCpp::getMacroValueString(...)`
- `ecmcCpp::getMacroValueInt(...)`
- `ecmcCpp::getMacroValueDouble(...)`
- `ecmcCpp::setCreateErrorMessage(...)`

This returns the optional `MACROS` string passed from the IOC startup path.
Typical use is to read it once in the constructor and interpret it as a small
configuration string for the logic module. `getMacroValue(...)` and
`getMacroValueString(...)` return one macro value as a string. The typed helpers
return a caller-provided default value when the key is missing or the text
cannot be parsed.

Use `setCreateErrorMessage(...)` during construction when the logic cannot be
created. The C++ logic ABI returns an error code from `createInstance()` and
passes the created object through an out pointer. If creation fails,
`Cfg.LoadCppLogic` returns the message in its command error text.

With the default registration macro, user logic can reject creation after the
object has been constructed by adding a validation hook:

```cpp
int32_t validateCreation(std::string* errorMessage) {
  if (bad_config) {
    if (errorMessage) {
      *errorMessage = "bad config: missing AXIS_ID";
    }
    return ECMC_CPP_LOGIC_CREATE_INSTANCE_FAIL;
  }
  return 0;
}
```

If `validateCreation()` returns an error, the default adapter deletes the
object, returns the error code over the ABI, and passes the message to
`Cfg.LoadCppLogic`.

Array/buffer helpers:

- `ecmc.inputArray(...)`
- `ecmc.outputArray(...)`
- `ecmc.inputBytes(...)`
- `ecmc.outputBytes(...)`
- `ecmc.inputAutoArray(...)`
- `ecmc.outputAutoArray(...)`
- `epics.readOnlyArray(...)`
- `epics.writableArray(...)`
- `epics.readOnlyBytes(...)`
- `epics.writableBytes(...)`

## ecmcCppControl.hpp

Current control helper:

- `ecmcCpp::Pid`

Highlights:

- `Kp`, `Ki`, `Kd`
- feed-forward via `FF` and `Kff`
- output limiting with `OutMin` / `OutMax`
- integral limiting with `IMin` / `IMax`
- optional derivative filtering via `DFilterTau`

## ecmcCppUtils.hpp

### Triggers and timers

- `ecmcCpp::RTrig`
- `ecmcCpp::FTrig`
- `ecmcCpp::Ton`
- `ecmcCpp::Tof`
- `ecmcCpp::Tp`

These are intended to feel familiar to users coming from IEC/ST helper blocks.

### Latches and toggles

- `ecmcCpp::Sr`
- `ecmcCpp::Rs`
- `ecmcCpp::FlipFlop`

### Simple runtime utilities

- `ecmcCpp::Blink`
- `ecmcCpp::StateTimer<T>`
- `ecmcCpp::DebounceBool`
- `ecmcCpp::StartupDelay`
- `ecmcCpp::RateLimiter`
- `ecmcCpp::FirstOrderFilter`
- `ecmcCpp::HysteresisBool`
- `ecmcCpp::Integrator`
- `ecmcCpp::MoveAverage`
- `ecmcCpp::MinMaxHold`

### Scalar helper functions

- `ecmcCpp::applyDeadband(...)`
- `ecmcCpp::clampValue(...)`
- `ecmcCpp::inWindow(...)`
- `ecmcCpp::readBit(...)`
- `ecmcCpp::writeBit(...)`
- `ecmcCpp::setBit(...)`
- `ecmcCpp::clearBit(...)`
- `ecmcCpp::toggleBit(...)`

The bit helpers work on integral word types such as `uint16_t`, `int32_t`, and
`uint64_t`. Out-of-range bit indexes are handled without undefined shifts:
`readBit(...)` returns `false`, while the write-style helpers return the
unchanged input value.

### EtherCAT status wrappers

- `ecmcCpp::getEcTimeNs()`
- `ecmcCpp::getEcTimeOffsetNs()`
- `ecmcCpp::getEcLastReceiveTimeNs()`
- `ecmcCpp::getEcLastSendTimeNs()`
- `ecmcCpp::getEcDomainState(...)`
- `ecmcCpp::getEcStatusOK()`
- `ecmcCpp::EcMasterStatus`
- `ecmcCpp::EcSlaveStatus`

## ecmcCppTrace.hpp

Current trace helper:

- `ecmcCpp::TriggeredTrace<T, Capacity>`

Highlights:

- rolling pre-trigger history
- capture of pre-trigger, trigger, and post-trigger samples
- fixed-capacity output array suitable for `epics.readOnlyArray(...)`
- explicit arming and ready state

## ecmcCppPersist.hpp

Current persistence helper:

- `ecmcCpp::RetainedValue<T>`

Highlights:

- binary load/save of trivially copyable values
- explicit `restore()` and `store()` calls
- useful for infrequent retained parameters such as setpoints or tunings
- intended for startup/manual save patterns, not continuous per-cycle file I/O
- prefer `enterRealtime()` for startup restore instead of doing file I/O from `run()`

## ecmcCppMotion.hpp

Current motion wrappers include:

- `ecmcCpp::McPower`
- `ecmcCpp::McReset`
- `ecmcCpp::McMoveAbsolute`
- `ecmcCpp::McMoveRelative`
- `ecmcCpp::McHome`
- `ecmcCpp::McMoveVelocity`
- `ecmcCpp::McStop`
- `ecmcCpp::McReadStatus`
- `ecmcCpp::McReadActualPosition`
- `ecmcCpp::McReadActualVelocity`

Selected encoder helpers are also available through `ecmcCppLogic.hpp`:

- `ecmcCpp::axisGetEncoderActualPos(axis, encoder)`
- `ecmcCpp::axisSetEncoderActualPos(axis, encoder, position)`
- `ecmcCpp::axisGetEncoderHomed(axis, encoder)`
- `ecmcCpp::axisSetEncoderHomed(axis, encoder, homed)`
- `ecmcCpp::axisGetEncoderReady(axis, encoder)`
- `ecmcCpp::axisGetPrimaryEncoder(axis)`
- `ecmcCpp::axisSelectPrimaryEncoder(axis, encoder)`
- `ecmcCpp::axisGetStatus(axis, status)`
- `ecmcCpp::axisGetStatus(axis)`

The encoder index follows the ECMC motion API convention: the first encoder is
`1`.

`axisGetStatus(...)` fills or returns an `ecmcCpp::AxisStatus` snapshot with the
axis motion status fields. The native motion bitfield is exposed as the stable
`status_word` mask, using flags such as `ECMC_CPP_AXIS_STATUS_ENABLED`,
`ECMC_CPP_AXIS_STATUS_BUSY`, `ECMC_CPP_AXIS_STATUS_HOMED`, and
`ECMC_CPP_AXIS_STATUS_LIMIT_FWD`.

Motion group helpers are available through `ecmcCppLogic.hpp`:

- `ecmcCpp::axisGroupGetEnable(group)`
- `ecmcCpp::axisGroupGetAnyEnable(group)`
- `ecmcCpp::axisGroupGetEnabled(group)`
- `ecmcCpp::axisGroupGetAnyEnabled(group)`
- `ecmcCpp::axisGroupGetBusy(group)`
- `ecmcCpp::axisGroupGetAnyBusy(group)`
- `ecmcCpp::axisGroupGetAnyErrorId(group)`
- `ecmcCpp::axisGroupSetEnable(group, enable)`
- `ecmcCpp::axisGroupSetTrajSource(group, source)`
- `ecmcCpp::axisGroupSetEncSource(group, source)`
- `ecmcCpp::axisGroupResetError(group)`
- `ecmcCpp::axisGroupSetError(group, error_id)`
- `ecmcCpp::axisGroupSetSlavedAxisInError(group)`
- `ecmcCpp::axisGroupHalt(group)`
- `ecmcCpp::axisGroupAxisInGroup(group, axis)`
- `ecmcCpp::axisGroupSize(group)`
- `ecmcCpp::axisGroupGetTrajSourceExt(group)`
- `ecmcCpp::axisGroupGetAnyTrajSourceExt(group)`
- `ecmcCpp::axisGroupSetAllowSourceChangeWhenEnabled(group, allow)`
- `ecmcCpp::axisGroupSetMrSync(group, sync)`
- `ecmcCpp::axisGroupSetMrStop(group, stop)`
- `ecmcCpp::axisGroupSetMrCnen(group, enable)`
- `ecmcCpp::axisGroupSetAutoEnable(group, enable)`
- `ecmcCpp::axisGroupSetAutoDisable(group, enable)`
- `ecmcCpp::axisGroupSetCtrlWithinDeadband(group, within)`
- `ecmcCpp::axisGroupSetIgnoreMrStatusCheckAtDisable(group, ignore)`
- `ecmcCpp::axisGroupGetAnyAtForwardLimit(group)`
- `ecmcCpp::axisGroupGetAnyAtBackwardLimit(group)`
- `ecmcCpp::axisGroupGetAnyAtLimit(group)`
- `ecmcCpp::axisGroupGetAnyInterlocked(group)`
- `ecmcCpp::axisGroupSetSlavedAxisInterlocked(group)`
- `ecmcCpp::axisGroupGetCtrlWithinDeadband(group)`

Master-to-master shared-memory helpers are available through `ecmcCppLogic.hpp`:

- `ecmcCpp::m2mWrite(index, value)`
- `ecmcCpp::m2mRead(index)`
- `ecmcCpp::m2mStatus()`
- `ecmcCpp::m2mResetError()`
- `ecmcCpp::m2mGetError()`
- `ecmcCpp::m2mIocRun(master_index)`
- `ecmcCpp::m2mIocEcOk(master_index)`

Data storage helpers are available through `ecmcCppLogic.hpp`. In addition to
single-value access, the C++ helpers support `std::vector<double>` for bulk
read/write/append:

- `ecmcCpp::dataStorageClear(storage)`
- `ecmcCpp::dataStorageAppend(storage, value)`
- `ecmcCpp::dataStorageGet(storage, index)`
- `ecmcCpp::dataStorageSet(storage, index, value)`
- `ecmcCpp::dataStorageGetIndex(storage)`
- `ecmcCpp::dataStorageSetIndex(storage, index)`
- `ecmcCpp::dataStorageRead(storage, values)`
- `ecmcCpp::dataStorageWrite(storage, values)`
- `ecmcCpp::dataStorageAppend(storage, values)`
- `ecmcCpp::dataStorageAppendFromStorage(from_storage, from_index, elements, to_storage)`
- `ecmcCpp::dataStorageIsFull(storage)`
- `ecmcCpp::dataStorageGetSize(storage)`
- `ecmcCpp::dataStoragePushAsyn(storage)`
- `ecmcCpp::dataStorageGetAvg(storage)`
- `ecmcCpp::dataStorageGetMin(storage)`
- `ecmcCpp::dataStorageGetMax(storage)`
- `ecmcCpp::dataStorageGetError()`
- `ecmcCpp::dataStorageResetError()`

## Examples

See:

- [`examples/README.md`](./examples/README.md)
- [`examples/cpp_logic_control/main.cpp`](./examples/cpp_logic_control/main.cpp)
- [`examples/cpp_logic_arrays/main.cpp`](./examples/cpp_logic_arrays/main.cpp)
- [`examples/cpp_logic_trace/main.cpp`](./examples/cpp_logic_trace/main.cpp)
- [`examples/cpp_logic_retained/main.cpp`](./examples/cpp_logic_retained/main.cpp)

The array example is the best starting point for:

- `inputAutoArray(...)`
- `outputArray(...)`
- `epics.readOnlyArray(...)`
- byte-buffer exports
