#!/usr/bin/env python3
"""Compile real startup coordination and target-slave verification dispatch offline."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root/'devEcmcSup/com/ecmcAsynPortDriver.cpp').read_text()
start = source.index('int startQueuedSdosThenRuntime() {')
worker = source[start:source.index('\n}\n}', start)+2]
source = (root/'devEcmcSup/ethercat/ecmcEc.cpp').read_text()
start = source.index('  // Appended only here, after component/plugin configuration and autosave.')
activate = source[start:source.index('  ECMC_RT_LOG_ETHERCAT_DEBUG', start)]
start = source.index('  // Startup reads must follow EtherLab')
dispatch = source[start:source.index('  // Keep for later investigation', start)]

stubs = r'''
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <vector>
#include <string>
std::vector<std::string> events;
const int ERROR_EC_SDO_STARTUP_PENDING=100;
bool iocRunning=true;
int sleeps=0, collections=0, activationResult=0, modeCalls=0;
int exitRequests=0, requestedExitCode=0;
int ecmcIocRunning() { return iocRunning; }
int requestIocExitFromRt(int code) { ++exitRequests; requestedExitCode=code; return 0; }
void epicsThreadSleep(double) { ++sleeps; }
struct Port {
 bool held=false;
 void lock() { assert(!held); held=true; events.push_back("lock"); }
 void unlock() { assert(held); held=false; events.push_back("unlock"); }
} port;
Port *ecmcAsynPortObj=&port;
struct Queue {
 bool active=false, targetOperational=false, inFlight=false;
 int prepareError=0, prepareCalls=0, polls=0, error=0;
 size_t remaining=0;
 bool running() const { return active; }
 int prepareStartup() { ++prepareCalls; events.push_back("append-restored-last"); return prepareError; }
 size_t startupRemaining() const { return remaining; }
 size_t pending() const { return remaining; }
 int startupError() const { return error; }
 void poll() {
  ++polls;
  if(inFlight) { --remaining; inFlight=false; }
  if(remaining && targetOperational) inFlight=true;
 }
};
struct ec_master_state_t { int al_states=2; };
struct Ethercat {
 Queue sdoQueue_;
 bool sdoStartupFailureFatal_=false;
 Queue& sdoQueue() { return sdoQueue_; }
 ec_master_state_t masterState_;
 void *master_=nullptr;
 int activate();
 void receiveVerification();
} ethercat;
Ethercat *ec=&ethercat;
void ecrt_master_state(void*,ec_master_state_t *state) { *state=ec->masterState_; }
struct Sdo { bool explicitWrite=false; bool hasStartupWrite() const { return explicitWrite; } } sdo;
struct Record { Sdo *sdo; int collectionError; };
std::vector<Record> sdoRestoreRecords;
bool sdoRestoreCollected=false;
int collectSdoRestoreRecords() { assert(port.held); ++collections; sdoRestoreCollected=true; return 0; }
int setAppMode(int mode) {
 assert(mode==1 && port.held); ++modeCalls;
 events.push_back("component-plugin-settings");
 int error=ec->activate();
 if(!error) events.push_back("master-activate-and-rt");
 return error ? error : activationResult;
}
int logs=0;
void ecmcRtLoggerLogError(const char*,...) { ++logs; }
void ecmcRtLoggerLogInfo(const char*,...) { ++logs; }
'''
body = 'int Ethercat::activate() {\n' + activate + '\n return 0;\n}\n'
body += 'void Ethercat::receiveVerification() {\n' + dispatch + '\n}\n'
body += worker + r'''
void reset() {
 assert(!port.held);
 ethercat=Ethercat{}; sdo=Sdo{}; sdoRestoreRecords.clear();
 sdoRestoreCollected=false; iocRunning=true;
 sleeps=collections=activationResult=modeCalls=logs=0;
 exitRequests=requestedExitCode=0; events.clear();
}
int main() {
 reset();
 assert(startQueuedSdosThenRuntime()==0 && collections==1 && modeCalls==1 && !port.held);
 assert(events==std::vector<std::string>({"lock","component-plugin-settings","append-restored-last","master-activate-and-rt","unlock"}));
 reset(); iocRunning=false;
 assert(startQueuedSdosThenRuntime()==ERROR_EC_SDO_STARTUP_PENDING && sleeps==1000 && modeCalls==0 && !port.held);
 reset(); sdoRestoreRecords.push_back({&sdo,77});
 assert(startQueuedSdosThenRuntime()==77 && modeCalls==0 && !port.held);
 reset(); sdo.explicitWrite=true; sdoRestoreRecords.push_back({&sdo,77});
 assert(startQueuedSdosThenRuntime()==0 && modeCalls==1 && !port.held);
 reset(); ethercat.sdoQueue_.active=true;
 assert(startQueuedSdosThenRuntime()==ERROR_EC_SDO_STARTUP_PENDING && modeCalls==0 && !port.held);
 reset(); ethercat.sdoQueue_.prepareError=88;
 assert(startQueuedSdosThenRuntime()==88 && !port.held);
 assert(events==std::vector<std::string>({"lock","component-plugin-settings","append-restored-last","unlock"}));

 reset(); ethercat.sdoQueue_.remaining=2;
 ethercat.masterState_.al_states=8;
 ethercat.receiveVerification(); // Master summary alone does not authorize target I/O.
 assert(!ethercat.sdoQueue_.inFlight && ethercat.sdoQueue_.remaining==2 && logs==0);
 ethercat.masterState_.al_states=10; // Unconfigured PREOP slaves plus configured OP slave.
 ethercat.sdoQueue_.targetOperational=true;
 ethercat.receiveVerification();
 assert(ethercat.sdoQueue_.inFlight && ethercat.sdoQueue_.remaining==2 && logs==0);
 ethercat.receiveVerification();
 assert(ethercat.sdoQueue_.remaining==1 && logs==0);
 ethercat.sdoQueue_.targetOperational=false; ethercat.masterState_.al_states=2;
 ethercat.receiveVerification(); // Complete in-flight result even after leaving OP.
 assert(!ethercat.sdoQueue_.remaining && logs==1);
 ethercat.receiveVerification(); assert(logs==1);

 reset(); ethercat.sdoQueue_.remaining=1; ethercat.sdoQueue_.inFlight=true;
 ethercat.sdoQueue_.error=91;
 ethercat.receiveVerification();
 assert(logs==1 && exitRequests==0);
 reset(); ethercat.sdoQueue_.remaining=1; ethercat.sdoQueue_.inFlight=true;
 ethercat.sdoQueue_.error=92; ethercat.sdoStartupFailureFatal_=true;
 ethercat.receiveVerification();
 assert(logs==1 && exitRequests==1 && requestedExitCode==EXIT_FAILURE);

 puts("SDO startup coordination: restore collection, final append ordering, activation errors, producer locking and mixed PREOP/OP and in-flight readback dispatch passed");
}
'''
with tempfile.TemporaryDirectory() as directory:
    cpp=Path(directory)/'sdo_startup_execution.cpp'
    exe=Path(directory)/'sdo_startup_execution'
    cpp.write_text(stubs+body)
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++11','-O2','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
