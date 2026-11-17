#!/usr/bin/env python3
"""Compile the production async SDO class against fake EtherCAT/asyn endpoints.

Exercises byte payloads and request lifecycle without EPICS or hardware.
"""
# Expectation basis: DOC: SDO feature contract; COMPAT: exact codes/call counts.
# Rationale, sources and open questions: EXPECTATIONS.md
from pathlib import Path
import os
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
header = (root / 'devEcmcSup/ethercat/ecmcSdoQueue.h').read_text() + (root / 'devEcmcSup/ethercat/ecmcEcAsyncSDO.h').read_text()
source = (root / 'devEcmcSup/ethercat/ecmcEcAsyncSDO.cpp').read_text()
header = (root / 'devEcmcSup/ethercat/ecmcSdoErrors.h').read_text() + header
header = re.sub(r'^#include ".*"\n', '', header, flags=re.M)
source = re.sub(r'^#include ".*"\n', '', source, flags=re.M)
stubs = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <stdexcept>
enum asynStatus { asynSuccess, asynError };
enum asynParamType { asynParamInt32, asynParamUInt32Digital, asynParamFloat64,
                     asynParamInt8Array, asynParamInt64 };
enum ecmcEcDataType { ECMC_EC_NONE, ECMC_EC_B1, ECMC_EC_B2, ECMC_EC_B3, ECMC_EC_B4,
 ECMC_EC_U8, ECMC_EC_S8, ECMC_EC_S8_TO_U8, ECMC_EC_U16, ECMC_EC_S16,
 ECMC_EC_S16_TO_U16, ECMC_EC_U32, ECMC_EC_S32, ECMC_EC_S32_TO_U32,
 ECMC_EC_U64, ECMC_EC_S64, ECMC_EC_S64_TO_U64, ECMC_EC_F32, ECMC_EC_F64 };
int getEcDataTypeByteSize(ecmcEcDataType t) { return t == ECMC_EC_U32 ? 4 : 1; }
int getEcDataTypeBits(ecmcEcDataType t) { return 8 * getEcDataTypeByteSize(t); }
int getEcDataTypeMinVal(ecmcEcDataType) { return 0; }
int getEcDataTypeMaxVal(ecmcEcDataType) { return 255; }
#define EC_MAX_OBJECT_PATH_CHAR_LENGTH 256
#define ECMC_EC_STR "ec"
#define ECMC_SLAVE_CHAR "s"
#define ECMC_SDO_STR "sdo"
#define ECMC_VALUE_STR "value"
#define ECMC_ERROR_STR "error"
#define ECMC_READCMD_STR "readcmd"
#define ECMC_WRITECMD_STR "writecmd"
#define ECMC_BUSY_STR "busy"
#define ECMC_ASYN_DEFAULT_LIST 0
#define ECMC_ASYN_DEFAULT_ADDR 0
#define ERROR_MAIN_ASYN_CREATE_PARAM_FAIL 123
#define ECMC_ASYN_ASYNPARAMINT64
struct ecmcError {
 template<class... T> int setErrorID(T... args) { return last(args...); }
 int last(int a) { return a; }
 template<class A, class... T> int last(A, T... t) { return last(t...); }
};
void ecmcRtLoggerLogError(const char*, ...) {}
void ecmcRtLoggerLogInfo(const char*, ...) {}
struct ecmcAsynDataItem {
 uint8_t *data; size_t size; bool writable = false; int callbacks = 0;
 using Callback = asynStatus(*)(void*, size_t, asynParamType, void*);
 Callback callback = nullptr; void *user = nullptr;
 void setExeCmdFunctPtr(Callback cb, void *obj) { callback=cb; user=obj; }
 void setAllowWriteToEcmc(bool v) { writable=v; }
 void addSupportedAsynType(asynParamType) {}
 void setEcmcDataSize(size_t v) { size=v; }
 void setEcmcBitCount(int) {}
 void setEcmcMinValueInt(int) {}
 void setEcmcMaxValueInt(int) {}
 int refreshParam(int) { ++callbacks; return 0; }
 int refreshParam(int f, size_t bytes) { size=bytes; return refreshParam(f); }
 int refreshParamRT(int f) { return refreshParam(f); }
 int32_t integer() { int32_t v; memcpy(&v,data,4); return v; }
 asynStatus trigger(int32_t v) { return callback(&v,4,asynParamInt32,user); }
};
struct ecmcAsynPortDriver {
 std::map<std::string,std::unique_ptr<ecmcAsynDataItem>> items;
 ecmcAsynDataItem *addNewAvailParam(const char *n, asynParamType,
 uint8_t *data, size_t size, ecmcEcDataType, int) {
   assert(!items.count(n));
   items[n]=std::unique_ptr<ecmcAsynDataItem>(new ecmcAsynDataItem{data,size});
   return items[n].get();
 }
 void callParamCallbacks(int,int) {}
 ecmcAsynDataItem& get(const char *suffix) { return *items.at(std::string("ec0.s3.sdo.payload.")+suffix); }
};
enum ec_request_state_t { EC_REQUEST_UNUSED, EC_REQUEST_BUSY, EC_REQUEST_SUCCESS, EC_REQUEST_ERROR };
struct ec_sdo_request_t {
 std::vector<uint8_t> data; ec_request_state_t state=EC_REQUEST_UNUSED;
 int reads=0, writes=0;
};
struct ec_slave_config_t {
 struct Startup { uint16_t index; uint8_t sub; std::vector<uint8_t> data; };
 std::vector<std::unique_ptr<ec_sdo_request_t>> requests;
 std::vector<Startup> startup;
 int configError=0;
 bool operational=true;
};
struct ec_slave_config_state_t { bool operational=false; };
void ecrt_slave_config_state(ec_slave_config_t *sc, ec_slave_config_state_t *state) {
 state->operational=sc->operational;
}
int ecrt_slave_config_sdo(ec_slave_config_t *sc, uint16_t index, uint8_t sub,
                         const uint8_t *data, size_t size) {
 if(sc->configError) return sc->configError;
 sc->startup.push_back({index,sub,std::vector<uint8_t>(data,data+size)});
 return 0;
}
struct ec_master_t {
  int state=2, downloadError=0, uploadError=0, downloadCalls=0, uploadCalls=0;
  bool mismatch=false;
  std::vector<uint8_t> value;
};
struct ec_slave_info_t { int al_state=2; };
#define EC_AL_STATE_PREOP 2
int ecrt_master_get_slave(ec_master_t *m,int,ec_slave_info_t *info) { info->al_state=m->state; return 0; }
int ecrt_master_sdo_download(ec_master_t *m,int,uint16_t,uint8_t,const uint8_t *data,size_t size,uint32_t*) {
 ++m->downloadCalls; if(m->downloadError) return -1; m->value.assign(data,data+size); return 0;
}
int ecrt_master_sdo_upload(ec_master_t *m,int,uint16_t,uint8_t,uint8_t *data,size_t capacity,size_t *size,uint32_t*) {
 ++m->uploadCalls; if(m->uploadError) return -1; *size=m->value.size();
 memcpy(data,m->value.data(),std::min(capacity,*size));
 if(m->mismatch && capacity) data[0]^=1;
 return 0;
}

ec_sdo_request_t *ecrt_slave_config_create_sdo_request(ec_slave_config_t *s,
 uint16_t,uint8_t,size_t bytes) {
 auto r=std::unique_ptr<ec_sdo_request_t>(new ec_sdo_request_t);
 r->data.resize(bytes); auto ptr=r.get(); s->requests.push_back(std::move(r)); return ptr;
}
void ecrt_sdo_request_timeout(ec_sdo_request_t*,int) {}
ec_request_state_t ecrt_sdo_request_state(ec_sdo_request_t *r) { return r->state; }
uint8_t *ecrt_sdo_request_data(ec_sdo_request_t *r) { assert(r->state != EC_REQUEST_BUSY); return r->data.data(); }
size_t ecrt_sdo_request_data_size(ec_sdo_request_t *r) { return r->data.size(); }
void ecrt_sdo_request_write(ec_sdo_request_t *r) { assert(r->state!=EC_REQUEST_BUSY); r->state=EC_REQUEST_BUSY; ++r->writes; }
void ecrt_sdo_request_read(ec_sdo_request_t *r) { assert(r->state!=EC_REQUEST_BUSY); r->state=EC_REQUEST_BUSY; ++r->reads; }
template<class T> T readLE(const uint8_t *p) { T v; memcpy(&v,p,sizeof v); return v; }
template<class T> void writeLE(uint8_t *p,T v) { memcpy(p,&v,sizeof v); }
'''
for suffix, typ in [('U8','uint8_t'),('S8','int8_t'),('U16','uint16_t'),('S16','int16_t'),('U32','uint32_t'),('S32','int32_t'),('U64','uint64_t'),('S64','int64_t'),('REAL','float'),('LREAL','double')]:
    stubs += f'\n#define EC_READ_{suffix}(p) readLE<{typ}>(p)\n#define EC_WRITE_{suffix}(p,v) writeLE<{typ}>(p,v)\n'
# Compile the actual lookup, C API, PLC adapters and C++ wrappers as well.
api_header = (root / 'devEcmcSup/ethercat/ecmcSdoApi.h').read_text()
api_source = (root / 'devEcmcSup/ethercat/ecmcSdoApi.cpp').read_text()
api_source = re.sub(r'^#include ".*"\n', '', api_source, flags=re.M)
slave_source = (root / 'devEcmcSup/ethercat/ecmcEcSlave.cpp').read_text()
lookup_source = slave_source[slave_source.index('ecmcEcAsyncSDO *ecmcEcSlave::getSDOAsync('):
                            slave_source.index('int ecmcEcSlave::activate()')]
plc_source = (root / 'devEcmcSup/plc/ecmcPLCTask_libEc.inc').read_text()
plc_adapters = plc_source[plc_source.index('inline bool ec_sdo_valid_index('):
                         plc_source.index('inline double ec_set_bit(')]
cpp_wrappers = (root / 'devEcmcSup/logic/ecmcCppSdo.hpp').read_text()
cpp_wrappers = re.sub(r'^#include ".*"\n', '', cpp_wrappers, flags=re.M)
registry_stubs = r'''
#define ERROR_MAIN_EC_MASTER_NULL 1001
#define ERROR_MAIN_EC_SLAVE_NULL 1002
struct ecmcEcSlave {
  std::vector<ecmcEcAsyncSDO *> asyncSDOvector_;
  int asyncSDOCounter_ = 0;
  ecmcEcAsyncSDO *getSDOAsync(int index);
  int findSDOAsyncIndex(const char *name, int *index);
};
struct ecmcEc {
  ecmcEcSlave *slave = nullptr;
  ecmcEcSlave *findSlave(int id) { return id == 3 ? slave : nullptr; }
};
ecmcEc *ec = nullptr;
static int ec_errorCode = 0;
'''
checks = r'''
int main() {
 ecmcAsynPortDriver port; ec_slave_config_t slave;
 ecmcEcAsyncSDO sdo(&port,0,3,&slave,0x2000,1,ECMC_EC_U8,"payload",4096);
 assert(slave.requests.size()==2);
 auto &download=*slave.requests[0], &upload=*slave.requests[1];
 auto &read=port.get("readcmd"), &write=port.get("writecmd");
 auto &busy=port.get("busy"), &error=port.get("error");
 auto &value=port.get("value"), &staged=port.get("writevalue");
 assert(!value.writable && staged.writable && value.size==0);
 // No download of uninitialized staging memory.
 assert(write.trigger(1)==asynError && download.writes==0);
 write.trigger(0);
 std::vector<uint8_t> bytes(4096);
 for (size_t i=0;i<bytes.size();++i) bytes[i]=i%251;
 assert(staged.callback(bytes.data(),4095,asynParamInt8Array,staged.user)==asynError);
 assert(staged.callback(bytes.data(),4097,asynParamInt8Array,staged.user)==asynError);
 assert(staged.callback(bytes.data(),4096,asynParamInt32,staged.user)==asynError);
 assert(staged.callback(bytes.data(),4096,asynParamInt8Array,staged.user)==asynSuccess);
 assert(write.trigger(1)==asynSuccess && download.data==bytes && error.integer()==0);
 assert(busy.integer()==1);
 sdo.execute(); assert(busy.integer()==1);
 assert(staged.callback(bytes.data(),4096,asynParamInt8Array,staged.user)==asynError);
 assert(read.trigger(1)==asynError && upload.reads==0);
 // Request finished, but execute has not consumed it: still reject another request.
 download.state=EC_REQUEST_SUCCESS;
 read.trigger(0); assert(read.trigger(1)==asynError);
 sdo.execute(); assert(busy.integer()==0);
 read.trigger(0); assert(read.trigger(1)==asynSuccess && error.integer()==0);
 assert(read.trigger(1)==asynSuccess && upload.reads==1); // no new edge
 upload.data={1,2,3}; upload.state=EC_REQUEST_SUCCESS;
 sdo.execute();
 assert(value.size==3 && value.data[2]==3 && port.get("bytes").integer()==3);
 assert(busy.integer()==0 && std::equal(bytes.begin(),bytes.end(),staged.data));
 // Short upload must not alter the next download's size or bytes.
 write.trigger(0); assert(write.trigger(1)==asynSuccess);
 assert(download.data==bytes && download.data.size()==4096);
 download.state=EC_REQUEST_ERROR; sdo.execute();
 assert(error.integer()==ERROR_EC_SDO_ASYNC_ERROR && busy.integer()==0);
 // Oversized response preserves previous data and reports actual size/error.
 read.trigger(0); assert(read.trigger(1)==asynSuccess && error.integer()==0);
 upload.data.assign(4097,42); upload.state=EC_REQUEST_SUCCESS;
 sdo.execute();
 assert(error.integer()==ERROR_EC_SDO_ASYNC_SIZE);
 assert(value.size==3 && value.data[0]==1 && port.get("bytes").integer()==4097);
 // Full-length and zero-length uploads.
 read.trigger(0); read.trigger(1); upload.data=bytes; upload.state=EC_REQUEST_SUCCESS; sdo.execute();
 assert(value.size==4096 && std::equal(bytes.begin(),bytes.end(),value.data));
 read.trigger(0); read.trigger(1); upload.data.clear(); upload.state=EC_REQUEST_SUCCESS; sdo.execute();
 assert(value.size==0 && port.get("bytes").integer()==0);
 // Scalar upload and download use independent request state.
 ecmcAsynPortDriver scalarPort; ec_slave_config_t scalarSlave;
 ecmcEcAsyncSDO scalar(&scalarPort,0,3,&scalarSlave,0x2000,1,ECMC_EC_U32,"payload");
 assert(scalarSlave.requests.size()==2);
 scalarPort.get("readcmd").trigger(1);
 auto &w=*scalarSlave.requests[0], &r=*scalarSlave.requests[1];
 r.state=EC_REQUEST_SUCCESS;
 EC_WRITE_U32(r.data.data(),0x12345678); scalar.execute();
 assert(scalarPort.get("value").integer()==0x12345678);
 scalarPort.get("value").data[0]=0x42;
 scalarPort.get("writecmd").trigger(1); assert(w.data[0]==0x42);
 // Validate size before allocation.
 try {
   ecmcEcAsyncSDO invalid(&port,0,3,&slave,1,1,ECMC_EC_U8,"invalid",ECMC_SDO_ASYNC_MAX_BYTES+1);
   assert(false);
 } catch(const std::invalid_argument&) {}

 // Lookup and runtime calls operate on exactly the existing objects.
 ecmcEcSlave registry; ecmcEc master; master.slave=&registry;
 int objectIndex=99;
 assert(ecGetSdoRTObjectIndex(3,"payload",&objectIndex)==ERROR_MAIN_EC_MASTER_NULL && objectIndex==-1);
 ec=&master;
 registry.asyncSDOvector_={&sdo,&scalar}; registry.asyncSDOCounter_=2;
 assert(ecGetSdoRTObjectIndex(3,"payload",&objectIndex)==ERROR_EC_SDO_ASYNC_NAME_AMBIGUOUS && objectIndex==-1);
 registry.asyncSDOvector_={&sdo}; registry.asyncSDOCounter_=1;
 assert(ecGetSdoRTObjectIndex(3,"missing",&objectIndex)==ERROR_EC_SDO_ASYNC_NOT_FOUND && objectIndex==-1);
 assert(ecGetSdoRTObjectIndex(3,"payload",&objectIndex)==0 && objectIndex==0);
 assert(ecGetSdoRTObjectIndex(-1,"payload",&objectIndex)==ERROR_EC_SDO_ASYNC_VALUE);
 assert(ecSdoRead(3,-1)==ERROR_EC_SDO_ASYNC_NOT_FOUND);
 assert(ecSdoRead(4,0)==ERROR_MAIN_EC_SLAVE_NULL);
 ecmcCpp::SdoRef ref;
 assert(ecmcCpp::sdoFind(3,"payload",ref)==0 && ref.objectIndex==0 && ref.slave==3);
 assert(ecmcCpp::sdoFindRTObjectIndex(3,"payload",objectIndex)==0 && objectIndex==0);
 int capacity=0;
 assert(ecmcCpp::sdoGetCapacity(ref,capacity)==0 && capacity==4096);
 assert(ecmcCpp::sdoRead(ref)==0);
 assert(ec_sdo_read(3,0)==ERROR_EC_SDO_ASYNC_BUSY);
 assert(ec_sdo_get_busy(3,0)==1);
 upload.data={7,8}; upload.state=EC_REQUEST_SUCCESS; sdo.execute();
 assert(ec_sdo_get_size(3,0)==2);
 assert(ec_sdo_get_value(3,0,1)==8 && ec_errorCode==0);
 assert(ec_sdo_get_value(3,0,2)==0 && ec_errorCode==ERROR_EC_SDO_ASYNC_SIZE);
 assert(ec_sdo_read(3.5,0)==ERROR_EC_SDO_ASYNC_VALUE);
 assert(ec_sdo_read(3,std::numeric_limits<double>::infinity())==ERROR_EC_SDO_ASYNC_VALUE);
 assert(ec_sdo_set_value(3,0,0,256)==ERROR_EC_SDO_ASYNC_VALUE);
 assert(ec_sdo_set_value(3,0,-1,1)==ERROR_EC_SDO_ASYNC_SIZE);
 assert(ec_sdo_set_value(3,0,0,1.5)==ERROR_EC_SDO_ASYNC_VALUE);
 assert(ec_sdo_set_value(3,0,0,55)==0);
 assert(ec_sdo_get_value(3,0,0)==7); // Staging never modifies readback.
 assert(ec_sdo_write(3,0)==0 && download.data[0]==55 && download.data.size()==4096);
 download.state=EC_REQUEST_SUCCESS; sdo.execute();
 assert(ec_sdo_get_error(3,0)==0);
 // All bytes must have been initialized once before an element-staged download.
 ecmcAsynPortDriver smallPort; ec_slave_config_t smallSlave;
 ecmcEcAsyncSDO small(&smallPort,0,3,&smallSlave,1,1,ECMC_EC_U8,"payload",4);
 assert(small.setValue(0,255)==0 && small.setValue(0,255)==0);
 assert(small.requestWrite()==ERROR_EC_SDO_ASYNC_SIZE);
 for (int i=1;i<4;++i) assert(small.setValue(i,i)==0);
 assert(small.setValue(4,0)==ERROR_EC_SDO_ASYNC_SIZE);
 assert(small.requestWrite()==0);
 assert(smallSlave.requests[0]->data==std::vector<uint8_t>({255,1,2,3}));
 assert(small.setValue(0,0)==ERROR_EC_SDO_ASYNC_BUSY);
 smallSlave.requests[0]->state=EC_REQUEST_SUCCESS; small.execute();
 assert(smallPort.get("readcmd").trigger(1)==asynSuccess);
 assert(small.requestRead()==ERROR_EC_SDO_ASYNC_BUSY);
 // Scalar element 0 and integer validation (no narrowing of invalid doubles).
 w.state=EC_REQUEST_SUCCESS; scalar.execute();
 assert(scalar.setValue(0,42)==0);
 double numeric=0;
 assert(scalar.getValue(0,&numeric)==0 && numeric==42);
 assert(scalar.setValue(1,42)==ERROR_EC_SDO_ASYNC_SIZE);
 assert(scalar.setValue(0,-1)==ERROR_EC_SDO_ASYNC_VALUE);
 assert(scalar.setValue(0,4294967296.0)==ERROR_EC_SDO_ASYNC_VALUE);
 assert(scalar.setValue(0,std::numeric_limits<double>::quiet_NaN())==ERROR_EC_SDO_ASYNC_VALUE);
 assert(scalar.getValue(0,&numeric)==0 && numeric==42);


 // Ordinary scalar endpoints restore accepted requests without PREOP-only policy.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver port; ec_slave_config_t slave;
  ecmcEcAsyncSDO original(&port,0,3,&slave,0x2000,1,ECMC_EC_U32,"payload");
  assert(original.configureQueue(&queue,false,false)==0);
  assert(original.enableStartupData()==0);
  const uint8_t empty[32]={};
  assert(original.restoreStartupData(empty,32)==0 && !queue.pending());
  assert(original.setValue(0,123)==0 && !queue.pending());
  assert(!original.hasStartupWrite()); // Manual Val staging alone is not autosaved.
  assert(memcmp(port.get("startup").data,empty,32)==0);
  assert(original.requestWrite()==0 && queue.pending()==1);
  uint8_t saved[32]; memcpy(saved,port.get("startup").data,32);
  assert(memcmp(saved,"ESDO",4)==0);
  ecmcSdoQueue::Request job; assert(queue.claim(&job));
  ec_master_t master; uint64_t actual=0; bool valid=false;
  assert(original.transferBeforeActivation(&master,job.write,job.value,&actual,&valid)==0 && valid);
  original.publishQueuedRead(actual); queue.complete(0);
  // Next boot: valid saved data queues exactly the previous committed payload.
  ecmcSdoQueue nextQueue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver nextPort; ec_slave_config_t nextSlave;
  ecmcEcAsyncSDO restored(&nextPort,0,3,&nextSlave,0x2000,1,ECMC_EC_U32,"payload");
  assert(restored.configureQueue(&nextQueue,false,false)==0 && restored.enableStartupData()==0);
  assert(restored.restoreStartupData(saved,32)==0 && nextQueue.pending()==1);
  assert(nextQueue.claim(&job) && job.write && job.value==123);
  nextQueue.complete(0);
  // A manual runtime write and read still work with automatic restore support.
  nextQueue.setRunning(true);
  assert(restored.setValue(0,456)==0 && nextQueue.pending()==0);
  assert(restored.requestWrite()==0 && restored.requestRead()==0);
  assert(nextQueue.claim(&job) && job.write && job.value==456); nextQueue.complete(0);
  assert(nextQueue.claim(&job) && !job.write); nextQueue.complete(0);
  assert(!nextQueue.pending());
  // Explicit zero is a valid write; it is distinct from an untouched waveform.
  assert(original.setValue(0,0)==0 && original.requestWrite()==0);
  memcpy(saved,port.get("startup").data,32);
  ecmcSdoQueue zeroQueue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver zeroPort; ec_slave_config_t zeroSlave;
  ecmcEcAsyncSDO zero(&zeroPort,0,3,&zeroSlave,0x2000,1,ECMC_EC_U32,"payload");
  assert(zero.configureQueue(&zeroQueue,false,false)==0 && zero.enableStartupData()==0);
  saved[20]^=1;
  assert(zero.restoreStartupData(saved,32)==ERROR_EC_SDO_CONFIG_CONFLICT && !zeroQueue.pending());
  saved[20]^=1;
  assert(zero.restoreStartupData(saved,32)==0 && zeroQueue.pending()==1);
  assert(zeroQueue.claim(&job) && job.write && job.value==0); zeroQueue.complete(0);
 }


 // Component defaults first, accepted restored writes last. Verification waits
 // until the master has applied the entire list, and checks only final values.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver port; ec_slave_config_t slave;
  uint8_t baseline[4]; EC_WRITE_U32(baseline,500);
  assert(ecrt_slave_config_sdo(&slave,0x8010,1,baseline,4)==0);
  ecmcEcAsyncSDO restored(&port,0,3,&slave,0x8010,1,ECMC_EC_U32,"payload");
  assert(restored.configureQueue(&queue,false,true)==0);
  assert(restored.setValue(0,700)==0 && restored.requestWrite()==0);
  assert(restored.setValue(0,900)==0 && restored.requestWrite()==0);
  assert(queue.prepareStartup()==0 && queue.startupHeld() && queue.startupRemaining()==2);
  assert(slave.startup.size()==3);
  assert(EC_READ_U32(slave.startup[0].data.data())==500);
  assert(EC_READ_U32(slave.startup[1].data.data())==700);
  assert(EC_READ_U32(slave.startup[2].data.data())==900);
  assert(slave.requests[0]->writes==0 && slave.requests[0]->reads==0);
  // Simulate successful EtherLab startup list execution in list order.
  std::vector<uint8_t> hardware;
  for(const auto &write : slave.startup) hardware=write.data;
  assert(EC_READ_U32(hardware.data())==900);
  queue.setRunning(true);
  for(int i=0;i<2;++i) {
   queue.poll();
   auto &read=*slave.requests[1];
   assert(read.writes==0 && read.reads==i+1 && queue.startupHeld());
   read.data=hardware; read.state=EC_REQUEST_SUCCESS;
   queue.poll();
   // poll may already have begun the next verification.
   if(i==0) {
    assert(queue.startupHeld() && read.reads==2);
    read.data=hardware; read.state=EC_REQUEST_SUCCESS;
    queue.poll(); break;
   }
  }
  assert(!queue.startupHeld() && !restored.getBusy() && !restored.getSdoError());
  assert(EC_READ_U32(port.get("readvalue").data)==900);
  assert(restored.requestRead()==0); // PREOP-only settings still allow runtime reads.
  assert(restored.requestWrite()==ERROR_EC_SDO_PREOP_ONLY);
  assert(queue.prepareStartup()==0 && slave.startup.size()==3); // No append on runtime reentry.
 }

 // Different aliases for one address must also verify the last write, not an
 // earlier alias's superseded value.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver aPort,bPort; ec_slave_config_t slave;
  ecmcEcAsyncSDO a(&aPort,0,3,&slave,0x8010,1,ECMC_EC_U32,"first");
  ecmcEcAsyncSDO b(&bPort,0,3,&slave,0x8010,1,ECMC_EC_U32,"second");
  assert(a.configureQueue(&queue,false,false)==0 && b.configureQueue(&queue,false,false)==0);
  assert(a.setValue(0,100)==0 && a.requestWrite()==0);
  assert(b.setValue(0,200)==0 && b.requestWrite()==0 && queue.prepareStartup()==0);
  queue.setRunning(true); queue.poll();
  EC_WRITE_U32(slave.requests[1]->data.data(),200); slave.requests[1]->state=EC_REQUEST_SUCCESS;
  queue.poll();
  EC_WRITE_U32(slave.requests[3]->data.data(),200); slave.requests[3]->state=EC_REQUEST_SUCCESS;
  queue.poll();
  assert(!queue.startupHeld() && !a.getSdoError() && !b.getSdoError());
  // Subsequent runtime errors do not change the completed startup latch.
  assert(b.requestRead()==0); queue.poll();
  slave.requests[3]->state=EC_REQUEST_ERROR; queue.poll();
  assert(b.getSdoError()==ERROR_EC_SDO_ASYNC_ERROR && !queue.startupHeld());
 }


 // Slave-local readiness: wait for the SDO target to reach OP, while unrelated
 // slaves may remain PREOP. If the target drops out, still complete active errors.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver port; ec_slave_config_t slave;
  ecmcEcAsyncSDO restored(&port,0,3,&slave,0x8010,1,ECMC_EC_U32,"payload");
  assert(restored.configureQueue(&queue,false,false)==0);
  assert(restored.setValue(0,700)==0 && restored.requestWrite()==0);
  assert(restored.setValue(0,800)==0 && restored.requestWrite()==0);
  assert(queue.prepareStartup()==0);
  queue.setRunning(true); slave.operational=false;
  queue.poll(); assert(slave.requests[1]->reads==0 && queue.startupRemaining()==2);
  slave.operational=true; queue.poll();
  assert(slave.requests[1]->reads==1); // Begins as soon as the target is OP.
  slave.operational=false; slave.requests[1]->state=EC_REQUEST_ERROR;
  queue.poll();
  // Verification uploads tolerate five transient errors. The active request
  // continues to be polled after its target leaves OP, while readiness still
  // prevents the following transaction from starting.
  assert(queue.startupRemaining()==2 && !restored.getSdoError());
  assert(slave.requests[1]->reads==2);
  for (int retry=1; retry<5; ++retry) {
    slave.requests[1]->state=EC_REQUEST_ERROR;
    queue.poll();
    assert(queue.startupRemaining()==2 && !restored.getSdoError());
    assert(slave.requests[1]->reads==retry+2);
  }
  slave.requests[1]->state=EC_REQUEST_ERROR;
  queue.poll();
  assert(queue.startupRemaining()==1 && restored.getSdoError()==ERROR_EC_SDO_ASYNC_ERROR);
  assert(slave.requests[1]->reads==6); // No next transaction while target is PREOP.
  slave.operational=true; queue.poll();
  assert(slave.requests[1]->reads==7);
  EC_WRITE_U32(slave.requests[1]->data.data(),800); slave.requests[1]->state=EC_REQUEST_SUCCESS;
  queue.poll();
  assert(!queue.startupRemaining() && queue.startupHeld()); // Error remains latched.
 }

 // No saved/requested write: preserve the component default, without appending.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver port; ec_slave_config_t slave;
  uint8_t baseline[4]; EC_WRITE_U32(baseline,500);
  assert(ecrt_slave_config_sdo(&slave,0x8010,1,baseline,4)==0);
  ecmcEcAsyncSDO untouched(&port,0,3,&slave,0x8010,1,ECMC_EC_U32,"payload");
  assert(untouched.configureQueue(&queue,false,false)==0);
  assert(queue.prepareStartup()==0 && slave.startup.size()==1 && !queue.startupHeld());
 }
 // A failed final verification is sticky: a later ordinary read must not release motion.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver port; ec_slave_config_t slave;
  ecmcEcAsyncSDO restored(&port,0,3,&slave,0x8010,1,ECMC_EC_U32,"payload");
  assert(restored.configureQueue(&queue,false,false)==0);
  assert(restored.setValue(0,700)==0 && restored.requestWrite()==0 && queue.prepareStartup()==0);
  queue.setRunning(true); queue.poll();
  auto &read=*slave.requests[1]; EC_WRITE_U32(read.data.data(),500); read.state=EC_REQUEST_SUCCESS;
  queue.poll();
  assert(queue.startupHeld() && !queue.startupRemaining() && queue.startupError()==ERROR_EC_SDO_VERIFY);
  assert(restored.getSdoError()==ERROR_EC_SDO_VERIFY);
  assert(restored.requestRead()==0); queue.poll();
  EC_WRITE_U32(read.data.data(),700); read.state=EC_REQUEST_SUCCESS; queue.poll();
  assert(queue.startupHeld() && queue.startupError()==ERROR_EC_SDO_VERIFY);
  assert(queue.prepareStartup()==0 && queue.startupHeld());
 }
 // Registration errors fail before activation. PDO mapping overrides remain rejected.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver port; ec_slave_config_t slave;
  ecmcEcAsyncSDO restored(&port,0,3,&slave,0x8010,1,ECMC_EC_U32,"payload");
  assert(restored.configureQueue(&queue,false,false)==0);
  assert(restored.setValue(0,700)==0 && restored.requestWrite()==0);
  slave.configError=-1;
  assert(queue.prepareStartup()==ERROR_EC_SDO_ASYNC_ERROR && slave.startup.empty());
  ecmcSdoQueue mappingQueue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver mappingPort; ec_slave_config_t mappingSlave;
  ecmcEcAsyncSDO mapping(&mappingPort,0,3,&mappingSlave,0x1600,1,ECMC_EC_U32,"payload");
  assert(mapping.configureQueue(&mappingQueue,false,false)==0);
  assert(mapping.requestWrite()==0 && mappingQueue.prepareStartup()==ERROR_EC_SDO_CONFIG_CONFLICT);
  assert(mappingSlave.startup.empty());
 }

 // New mode: command writes snapshot staged values while busy; no overlap,
 // and each download owns its verification upload before the next request.
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL);
  ecmcAsynPortDriver aPort,bPort; ec_slave_config_t aSlave,bSlave;
  ecmcEcAsyncSDO a(&aPort,0,3,&aSlave,0x2000,1,ECMC_EC_U32,"payload");
  ecmcEcAsyncSDO b(&bPort,0,4,&bSlave,0x2000,2,ECMC_EC_U32,"payload");
  assert(a.configureQueue(&queue,false,false)==0 && b.configureQueue(&queue,false,false)==0);
  assert(a.setValue(0,11)==0 && queue.pending()==0); // Manual staging still needs WrtCmd.
  assert(a.requestWrite()==0 && a.setValue(0,22)==0 && a.requestWrite()==0);
  assert(b.setValue(0,33)==0 && b.requestWrite()==0 && a.requestRead()==0);
  assert(queue.pending()==4 && a.getBusy()==1 && aSlave.requests[0]->writes==0);
  queue.setRunning(true); queue.poll();
  auto &aWrite=*aSlave.requests[0], &aRead=*aSlave.requests[1];
  auto &bWrite=*bSlave.requests[0], &bRead=*bSlave.requests[1];
  assert(EC_READ_U32(aWrite.data.data())==11 && bWrite.writes==0);
  aWrite.state=EC_REQUEST_SUCCESS; queue.poll();
  assert(aRead.reads==1 && bWrite.writes==0 && a.getBusy());
  EC_WRITE_U32(aRead.data.data(),11); aRead.state=EC_REQUEST_SUCCESS; queue.poll();
  assert(EC_READ_U32(aWrite.data.data())==22 && aWrite.writes==2 && bWrite.writes==0);
  assert(EC_READ_U32(aPort.get("readvalue").data)==11);
  aWrite.state=EC_REQUEST_SUCCESS; queue.poll();
  // Verification mismatch is retained across the rest of this object's burst.
  EC_WRITE_U32(aRead.data.data(),23); aRead.state=EC_REQUEST_SUCCESS; queue.poll();
  assert(a.getSdoError()==ERROR_EC_SDO_VERIFY && bWrite.writes==1 && a.getBusy());
  bWrite.state=EC_REQUEST_SUCCESS; queue.poll();
  EC_WRITE_U32(bRead.data.data(),33); bRead.state=EC_REQUEST_SUCCESS; queue.poll();
  assert(!b.getBusy() && aRead.reads==3);
  EC_WRITE_U32(aRead.data.data(),24); aRead.state=EC_REQUEST_SUCCESS; queue.poll();
  assert(!a.getBusy() && a.getSdoError()==ERROR_EC_SDO_VERIFY && !queue.pending());
  assert(a.getValue(0,&numeric)==0 && numeric==24);
  assert(a.setValue(0,25)==0 && a.requestWrite()==0 && a.getSdoError()==0);
  queue.poll(); aWrite.state=EC_REQUEST_ERROR; queue.poll();
  assert(!a.getBusy() && a.getSdoError()==ERROR_EC_SDO_ASYNC_ERROR);
 }
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL); ecmcAsynPortDriver port; ec_slave_config_t slave;
  ecmcEcAsyncSDO queued(&port,0,3,&slave,0x2000,1,ECMC_EC_U32,"payload");
  assert(queued.configureQueue(&queue,true,true)==0);
  // Ordinary value writes can opt in; each accepted write owns an immutable snapshot.
  int32_t first=41, second=42;
  assert(queued.asynWriteScalar(&first,4,asynParamInt32)==asynSuccess);
  assert(queued.asynWriteScalar(&second,4,asynParamInt32)==asynSuccess && queue.pending()==2);
  ec_master_t master;
  for(uint32_t expected : {41u,42u}) {
   ecmcSdoQueue::Request job; assert(queue.claim(&job));
   uint64_t actual=0; bool valid=false;
   assert(queued.transferBeforeActivation(&master,job.write,job.value,&actual,&valid)==0 && valid);
   assert(EC_READ_U32(master.value.data())==expected);
   queued.publishQueuedRead(actual); queue.complete(0);
  }
  assert(!queued.getBusy() && queued.getValue(0,&numeric)==0 && numeric==42);
  uint64_t actual=0; bool valid=false;
  master.state=8;
  assert(queued.transferBeforeActivation(&master,true,43,&actual,&valid)==ERROR_EC_SDO_PREOP_ONLY);
  master.state=2; master.mismatch=true;
  assert(queued.transferBeforeActivation(&master,true,43,&actual,&valid)==ERROR_EC_SDO_VERIFY && valid);
  master.mismatch=false; master.downloadError=1;
  assert(queued.transferBeforeActivation(&master,true,43,&actual,&valid)==ERROR_EC_SDO_ASYNC_ERROR && !valid);
  master.downloadError=0; master.uploadError=1;
  assert(queued.transferBeforeActivation(&master,true,43,&actual,&valid)==ERROR_EC_SDO_ASYNC_ERROR && !valid);
  queue.setRunning(true);
  assert(queued.requestWrite()==ERROR_EC_SDO_PREOP_ONLY && queue.pending()==0);
  assert(queued.requestRead()==0); // Reads remain available after activation.
 }
 {
  ecmcSdoQueue queue(ERROR_EC_SDO_QUEUE_FULL); ecmcAsynPortDriver port; ec_slave_config_t slave;
  ecmcEcAsyncSDO queued(&port,0,3,&slave,0x2000,1,ECMC_EC_U32,"payload");
  assert(queued.configureQueue(&queue,false,false)==0);
  for(size_t i=0;i<queue.CAPACITY;++i) {
   assert(queued.setValue(0,i)==0 && queued.requestWrite()==0);
  }
  assert(queued.requestWrite()==ERROR_EC_SDO_QUEUE_FULL && queue.pending()==queue.CAPACITY);
  for(size_t i=0;i<queue.CAPACITY;++i) {
   ecmcSdoQueue::Request job; assert(queue.claim(&job));
   assert(job.value==i); queue.complete(0);
  }
  assert(!queued.getBusy() && queued.getSdoError()==ERROR_EC_SDO_QUEUE_FULL);
 }
 puts("async SDO scalar/array, lookup, PLC and C++ interface tests passed");
}
'''
with tempfile.TemporaryDirectory() as tmp:
    cpp=Path(tmp)/'test.cpp'; binary=Path(tmp)/'test'
    cpp.write_text(stubs+header+source+api_header+registry_stubs+lookup_source+api_source+plc_adapters+cpp_wrappers+checks)
    subprocess.run([os.environ.get('CXX', 'c++'),'-std=c++17','-Wall','-Wextra','-fsanitize=address,undefined','-g',
                    '-I' + str(root / 'devEcmcSup/ethercat'), str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
