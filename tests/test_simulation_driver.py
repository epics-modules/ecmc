#!/usr/bin/env python3
"""Test the model-free simulation driver's actual memory and typed bindings.

Uses a publication fixture rather than EPICS. No physical transport or drive model.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SUP = ROOT / 'devEcmcSup'
checks = r'''
#include <cassert>
#include <cstring>
#include "ecmcSimulationDriver.h"
#include "ecmcAsynPortDriver.h"
#include "ecmcDataLink.h"
#include "ecmcErrorsList.h"
// DOC: simulation-io.md; bit layouts match ecmcEc/Slave/Domain and ecmccfg records.
uint32_t diagnostic(ecmcAsynPortDriver &port, const char *name) {
  auto *parameter = port.find(name);
  assert(parameter && !parameter->writable && parameter->bytes == sizeof(uint32_t));
  uint32_t value = 0;
  memcpy(&value, parameter->data, sizeof(value));
  return value;
}
int main() {
  {
    ecmcAsynPortDriver itemPort;
    ecmcSimulationDriver sim(0, &itemPort);
    assert(sim.addEntry(2, 123, 456, 1, 2, 0x1602, 0x2067, 0,
                        ECMC_EC_S32, "tmlComIF01", true) == 0);
    assert(sim.addDataItem(2, "tmlComIF01", 2, 0, 1, ECMC_EC_U16, "cfg") == 0);
    assert(sim.addDataItem(2, "tmlComIF01", 0, 0, 1, ECMC_EC_S16, "vel") == 0);
    assert(sim.addDataItem(2, "tmlComIF01", 3, 0, 1, ECMC_EC_U16, "outside") != 0);
    assert(sim.addDataItem(2, "tmlComIF01", 0, 0, 2, ECMC_EC_S16, "wrongDir") != 0);
    assert(sim.addDataItem(2, "tmlComIF01", 0, 1, 1, ECMC_EC_U16, "unaligned") != 0);
    assert(sim.addDataItem(2, "tmlComIF01", 0, 0, 1, ECMC_EC_S16, "vel") != 0);
    assert(itemPort.find("ec0.s2.cfg") && itemPort.find("ec0.s2.vel"));
    assert(sim.setValue(2, "tmlComIF01", "0x1234ffff") == 0);
    auto cfg = sim.findEntry(2, "cfg")->access;
    auto vel = sim.findEntry(2, "vel")->access;
    double number = 0;
    assert(vel->readNumber(&number) == 0 && number == -1);
    assert(cfg->readNumber(&number) == 0 && number == 0x1234);
    assert(sim.setValue(2, "cfg", "0x2345") == 0);
    assert(vel->writeDouble(-2) == 0);
    assert(sim.findEntry(2, "tmlComIF01")->access->readNumber(&number) == 0 &&
           number == 0x2345fffe);
  }

  ecmcAsynPortDriver port;
  auto driver = std::make_unique<ecmcSimulationDriver>(0, &port);
  assert(driver->configurationError() == 0);
  assert(diagnostic(port,"ec0.ok") == 0);
  assert(driver->addDevice(0, -1, 0, 0) == 0);
  assert(driver->addDevice(0, 2, 123, 456) == 0);
  auto *slaveStatusPV = port.find("ec0.s2.slavestatus");
  auto *masterStatusPV = port.find("ec0.masterstatus");
  assert(driver->addEntry(2, 123, 456, 2, 3, 0x1a00, 0x6000, 1, ECMC_EC_S32, "positionActual01", true) == 0);
  assert(driver->addEntry(2, 123, 456, 1, 2, 0x1600, 0x7000, 1, ECMC_EC_U16, "controlWord01", true) == 0);
  auto input = driver->findEntry(2, "positionActual01")->access;
  auto output = driver->findEntry(2, "controlWord01")->access;
  assert(driver->addAlias(2, "positionActual01", "positionAlias") == 0);
  assert(driver->findEntry(2, "positionAlias")->access == input);
  assert(driver->setValue(2, "positionActual01", "-123") == 0);
  assert(driver->setValue(2, "controlWord01", "65536") != 0);
  ecmcDataLink encoder, drive;
  assert(encoder.setDataAtIndex(input, 0, -1) == 0);
  assert(drive.setDataAtIndex(output, 0, -1) == 0);
  assert(driver->validate() == 0 && driver->isActive());
  driver->slowExecute();
  assert(diagnostic(port,"ec0.ok") == 1);
  assert(diagnostic(port,"ec0.masterstatus") == ((1u << 16) | 17u));
  assert(diagnostic(port,"ec0.s2.slavestatus") == ((2u << 16) | 35u));
  assert(diagnostic(port,"ec0.slavecounter") == 1);
  assert(diagnostic(port,"ec0.entrycounter") == 2);
  assert(diagnostic(port,"ec0.memmapcounter") == 0);
  assert(diagnostic(port,"ec0.dom0.domainstatus") == 8);
  assert(diagnostic(port,"ec0.dom0.domainfailcountertotal") == 0);
  driver->readInputs({1,2}, {3,4});
  double position = 0;
  assert(encoder.readDataValueDouble(0, &position) == 0 && position == -123);
  assert(drive.writeDataValue(0, 15) == 0);
  driver->writeOutputs({3,4});
  assert(driver->findEntry(2, "controlWord01")->value == 15);
  // No baked-in behaviour: output commands never change input feedback.
  driver->readInputs({2,2}, {3,4});
  assert(encoder.readDataValueDouble(0, &position) == 0 && position == -123);
  auto *pv = port.find("ec0.s2.positionActual01");
  assert(pv && pv->writable && pv->bytes == sizeof(int32_t));
  const int32_t injected = 654;
  memcpy(pv->data, &injected, sizeof(injected));
  driver->readInputs({3,2}, {3,4});
  assert(encoder.readDataValueDouble(0, &position) == 0 && position == 654);
  auto *healthPV = port.find("ec0.simulation.healthy");
  assert(healthPV && healthPV->writable && healthPV->bytes == sizeof(int));
  const int unhealthy = 0;
  memcpy(healthPV->data, &unhealthy, sizeof(unhealthy)); position = 77;
  assert(!driver->updateIoStatus() && !driver->timingReady());
  driver->slowExecute();
  assert(diagnostic(port,"ec0.ok") == 0);
  assert(diagnostic(port,"ec0.masterstatus") == 0);
  assert(diagnostic(port,"ec0.s2.slavestatus") == (2u << 16));
  assert(diagnostic(port,"ec0.dom0.domainstatus") == 2);
  assert(diagnostic(port,"ec0.dom0.domainfailcountertotal") == 1);
  assert(encoder.readDataValueDouble(0, &position) != 0 && position == 77);
  driver->resetStartupErrors(); assert(!input->valid());
  driver->setHealthy(true); assert(input->valid());
  assert(driver->updateIoStatus());
  driver->slowExecute();
  assert(diagnostic(port,"ec0.ok") == 1);
  assert(diagnostic(port,"ec0.masterstatus") == ((1u << 16) | 17u));
  assert(diagnostic(port,"ec0.s2.slavestatus") == ((2u << 16) | 35u));
  assert(diagnostic(port,"ec0.dom0.domainstatus") == 8);
  assert(diagnostic(port,"ec0.dom0.domainfailcountertotal") == 1);
  assert(driver->setObjectTyped(2, 0x8000, 1, ECMC_EC_U64, "18446744073709551615") == 0);
  uint64_t object = 0;
  assert(driver->getObject(2, 0x8000, 1, &object, 8) == 0 && object == UINT64_MAX);
  assert(driver->getObject(2, 0x8000, 2, &object, 8) != 0);
  assert(driver->setSetting(2, "dc", {0x300, 2000000, -1000}) == 0);
  assert(driver->findDevice(2)->settings["dc"][2] == -1000);
  driver.reset();
  assert(!input->data && !output->data && !input->valid());
  assert(!pv->data && pv->bytes == 0);
  assert(!healthPV->data);
  assert(!slaveStatusPV->data && !masterStatusPV->data);
  // Existing typo-correction aliases must be available to EPICS as well as links.
  auto latch = std::make_unique<ecmcSimulationDriver>(2, &port);
  assert(latch->addDevice(0,7,123,456) == 0);
  assert(latch->addEntry(7,123,456,2,3,0x1a00,0x6000,0x12,ECMC_EC_U32,"encoderLatchPostion01",true) == 0);
  assert(latch->addEntry(7,123,456,2,3,0x1a00,0x6000,0x13,ECMC_EC_U16,"encoderLatchPostion02",true) == 0);
  for (const char *suffix : {"01", "02"}) {
    const std::string oldName = std::string("encoderLatchPostion") + suffix;
    const std::string alias = std::string("encoderLatchPosition") + suffix;
    assert(latch->addAlias(7,oldName.c_str(),alias.c_str()) == 0);
    assert(latch->addAlias(7,oldName.c_str(),alias.c_str()) == 0); // Idempotent.
    auto *original = port.find(("ec2.s7." + oldName).c_str());
    auto *corrected = port.find(("ec2.s7." + alias).c_str());
    assert(original && original == corrected); // Same conversions, publication and buffer.
    assert(latch->findEntry(7,oldName.c_str()) == latch->findEntry(7,alias.c_str()));
  }
  assert(latch->addAlias(7,"encoderLatchPosition01","latchAlias") == 0);
  assert(port.find("ec2.s7.latchAlias") == port.find("ec2.s7.encoderLatchPostion01"));
  assert(latch->addAlias(7,"encoderLatchPostion02","encoderLatchPosition01") != 0);
  assert(latch->addAlias(7,"encoderLatchPostion01","slavestatus") != 0);
  assert(!latch->findEntry(7,"slavestatus")); // Failed asyn alias leaves no data alias.
  assert(latch->setValue(7,"encoderLatchPosition01","123456") == 0);
  auto *corrected = port.find("ec2.s7.encoderLatchPosition01");
  uint32_t latchValue = 0;
  memcpy(&latchValue,corrected->data,sizeof(latchValue));
  assert(latchValue == 123456);
  const uint32_t injectedLatch = 654321;
  memcpy(corrected->data,&injectedLatch,sizeof(injectedLatch));
  ecmcDataLink latchInput;
  assert(latchInput.setDataAtIndex(latch->findEntry(7,"encoderLatchPostion01")->access,0,-1) == 0);
  assert(latchInput.readDataValueDouble(0,&position) == 0 && position == 654321);
  assert(latch->validate() == 0);
  latch->slowExecute();
  assert(diagnostic(port,"ec2.entrycounter") == 2); // Aliases never create entries.
  latch.reset();
  assert(!corrected->data);
  // 1 kHz RT, 10 ms records: exactly one sampling tick per phase per cycle.
  // A ten-cycle enable pulse must be observable, even without slow work.
  ecmcSimulationDriver timing(3, &port);
  assert(timing.addEntry(14,123,456,2,3,0,0,0,ECMC_EC_U16,"driveStatus01",true) == 0);
  assert(timing.addEntry(14,123,456,1,2,0,0,0,ECMC_EC_U16,"driveControl01",true) == 0);
  auto *status = port.find("ec3.s14.driveStatus01");
  auto *control = port.find("ec3.s14.driveControl01");
  status->sampleCycles = control->sampleCycles = 10;
  assert(timing.validate() == 0);
  ecmcDataLink command;
  assert(command.setDataAtIndex(timing.findEntry(14,"driveControl01")->access,0,-1) == 0);
  for (int cycle = 1; cycle <= 100; ++cycle) {
    timing.readInputs({cycle,0},{});
    timing.updateIoStatus();
    assert(status->rtCalls == cycle && control->rtCalls == cycle-1);
    const uint64_t word = cycle >= 11 && cycle <= 20 ? 1 : 0;
    assert(command.writeDataValue(0,word) == 0);
    assert(command.writeDataValue(0,word) == 0); // Multiple producers do not accelerate sampling.
    assert(control->rtCalls == cycle-1);
    timing.writeOutputs({});
    assert(control->rtCalls == cycle);
    if (cycle == 20) assert(control->lastRTValue == 1);
    if (cycle % 100 == 0) timing.slowExecute();
  }
  assert(status->rtPublications == 10 && control->rtPublications == 10);
  assert(port.find("ec3.ok")->rtCalls == 100);
  assert(port.find("ec3.simulation.healthy")->rtCalls == 100);
  assert(port.find("ec3.dom0.domainstatus")->rtCalls == 100);
  // A 31-device configuration retains stable status addresses while growing.
  ecmcSimulationDriver many(1, &port);
  assert(many.addDevice(0, -1, 0, 0) == 0);
  for (int i = 1; i <= 31; ++i) assert(many.addDevice(0, i, 123, 456) == 0);
  assert(many.addDevice(0, 1, 123, 456) == 0); // Verify/redeclare does not double-count.
  assert(many.addEntry(31,123,456,2,3,0x1a00,0x6000,1,ECMC_EC_S32,"position",true) == 0);
  assert(many.validate() == 0);
  many.slowExecute();
  assert(diagnostic(port,"ec1.slavecounter") == 31);
  assert(diagnostic(port,"ec1.entrycounter") == 1);
  assert(diagnostic(port,"ec1.masterstatus") == ((31u << 16) | 17u));
  assert(diagnostic(port,"ec1.s1.slavestatus") == 35u);
  assert(diagnostic(port,"ec1.s31.slavestatus") == ((1u << 16) | 35u));
}
'''
with tempfile.TemporaryDirectory(prefix='ecmc-simulation-') as directory:
    cpp, exe = Path(directory)/'simulation.cpp', Path(directory)/'simulation'
    cpp.write_text(checks)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O2', '-Wall', '-Wextra',
        '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
        '-I'+str(ROOT/'tools/plc-test/offline'), '-I'+str(SUP/'com'), '-I'+str(SUP/'main'), '-I'+str(SUP/'misc'), '-I'+str(SUP/'motion'),
        str(cpp), str(SUP/'misc/ecmcSimulationDriver.cpp'), str(SUP/'com/ecmcDataAccess.cpp'),
        str(SUP/'com/ecmcDataItem.cpp'), str(SUP/'com/ecmcDataLink.cpp'),
        str(SUP/'main/ecmcError.cpp'), str(ROOT/'tests/monitor/support.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
