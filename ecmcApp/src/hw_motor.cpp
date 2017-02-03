
#define __STDC_FORMAT_MACROS  //for printf uint_64_t
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <assert.h>
#include <sched.h>
#include <pthread.h>

#include <stdlib.h>
#include <math.h>
#include "cmd.h"
#include "hw_motor.h"
#include <inttypes.h>
#include <algorithm>

#include "ecrt.h"
#include "messages.h"
#include <time.h>

//General
#include "ecmcDefinitions.h"
#include "ecmcErrorsList.h"

//Hardware
#include "ecmcEcPdo.h"
#include "ecmcEcSlave.h"
#include "ecmcEcSyncManager.h"
#include "ecmcEcEntry.h"
#include "ecmcEc.h"

//Motion
#include "ecmcAxisBase.h"      //Abstract class for all axis types
#include "ecmcAxisReal.h"      //Normal axis (controller, drive, encoder, trajectory, monitor, sequencer)
#include "ecmcAxisVirt.h"      //Axis without drive and controller
#include "ecmcDriveBase.hpp"
#include "ecmcTrajectoryTrapetz.hpp"
#include "ecmcPIDController.hpp"
#include "ecmcEncoder.h"
#include "ecmcMonitor.hpp"
#include "ecmcCommandTransform.h"
#include "ecmcEvent.h"
#include "ecmcDataRecorder.h"
#include "ecmcDataStorage.h"
#include "ecmcCommandList.h"


/****************************************************************************/
#define CHECK_AXIS_RETURN_IF_ERROR(axisIndex) {if(axisIndex>=ECMC_MAX_AXES || axisIndex<=0){LOGERR("ERROR: Axis index out of range.\n");return ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;}if(axes[axisIndex]==NULL){LOGERR("ERROR: Axis object NULL\n");return ERROR_MAIN_AXIS_OBJECT_NULL;}}
#define CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getEnc()==NULL){LOGERR("ERROR: Encoder object NULL.\n");return ERROR_MAIN_ENCODER_OBJECT_NULL;}}
#define CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getCntrl()==NULL){LOGERR("ERROR: Controller object NULL.\n");return ERROR_MAIN_CONTROLLER_OBJECT_NULL;}}
#define CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getDrv()==NULL){LOGERR("ERROR: Drive object NULL.\n");return ERROR_MAIN_DRIVE_OBJECT_NULL;}}
#define CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getTraj()==NULL){LOGERR("ERROR: Trajectory object NULL.\n");return ERROR_MAIN_TRAJECTORY_OBJECT_NULL;}}
#define CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getSeq()==NULL){LOGERR("ERROR: Sequence object NULL.\n");return ERROR_MAIN_SEQUENCE_OBJECT_NULL;}}
#define CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getMon()==NULL){LOGERR("ERROR: Monitor object NULL.\n");return ERROR_MAIN_MONITOR_OBJECT_NULL;}}
#define CHECK_AXIS_TRAJ_TRANSFORM_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getExternalTrajIF()->getExtInputTransform()==NULL){LOGERR("ERROR: Trajectory transform object NULL.\n");return ERROR_MAIN_TRAJ_TRANSFORM_OBJECT_NULL;}}
#define CHECK_AXIS_ENC_TRANSFORM_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getExternalEncIF()->getExtInputTransform()==NULL){LOGERR("ERROR: Encoder transform object NULL.\n");return ERROR_MAIN_ENC_TRANSFORM_OBJECT_NULL;}}
#define CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex) {  if(indexCommandList>=ECMC_MAX_COMMANDS_LISTS || indexCommandList<0){LOGERR("ERROR: Command list index out of range.\n");return ERROR_COMMAND_LIST_INDEX_OUT_OF_RANGE;}if(commandLists[indexCommandList]==NULL){LOGERR("ERROR: Command list object NULL.\n");return ERROR_COMMAND_LIST_NULL;}}
#define CHECK_EVENT_RETURN_IF_ERROR(indexEvent) {if(indexEvent>=ECMC_MAX_EVENT_OBJECTS || indexEvent<0){LOGERR("ERROR: Event index out of range.\n");return ERROR_MAIN_EVENT_INDEX_OUT_OF_RANGE;}if(events[indexEvent]==NULL){LOGERR("ERROR: Event object NULL.\n");return ERROR_MAIN_EVENT_NULL;}}
#define CHECK_STORAGE_RETURN_IF_ERROR(indexStorage) { if(indexStorage>=ECMC_MAX_DATA_STORAGE_OBJECTS || indexStorage<0){LOGERR("ERROR: Data storage index out of range.\n");return ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE;}if(dataStorages[indexStorage]==NULL){LOGERR("ERROR: Data storage object NULL.\n");return ERROR_MAIN_DATA_STORAGE_NULL;}}
#define CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder) {if(indexRecorder>=ECMC_MAX_DATA_RECORDERS_OBJECTS || indexRecorder<0){LOGERR("ERROR: Data recorder index out of range.\n");return ERROR_MAIN_DATA_RECORDER_INDEX_OUT_OF_RANGE;}if(dataRecorders[indexRecorder]==NULL){LOGERR("ERROR: Data recorder object NULL.\n");return ERROR_MAIN_DATA_RECORDER_NULL;}}

/****************************************************************************/
static ecmcAxisBase     *axes[ECMC_MAX_AXES];
static ecmcEc           ec;
static int              axisDiagIndex;
static int              axisDiagFreq;
static int              controllerError=0;
static app_mode_type    appMode,appModeOld;
static unsigned int     counter = 0;
static ecmcEvent        *events[ECMC_MAX_EVENT_OBJECTS];
static ecmcDataRecorder *dataRecorders[ECMC_MAX_DATA_RECORDERS_OBJECTS];
static ecmcDataStorage  *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
static ecmcCommandList  *commandLists[ECMC_MAX_COMMANDS_LISTS];
static struct timespec  masterActivationTimeMonotonic={};
static struct timespec  masterActivationTimeOffset={};
static struct timespec  masterActivationTimeRealtime={};

/****************************************************************************/

const struct timespec cycletime = {0, MCU_PERIOD_NS};

/*****************************************************************************/

void printStatus()
{
  //Print axis diagnostics to screen
  if(PRINT_STDOUT_BIT12() && axisDiagIndex<ECMC_MAX_AXES && axisDiagIndex>=0){
    if(axes[axisDiagIndex]!=NULL){
      axes[axisDiagIndex]->printStatus();
    }
  }
}

//****** Threading
typedef void (*rtTHREADFUNC)(void *parm);

struct rtThreadOSD
{
  pthread_t thread;
  pthread_attr_t attr;
  void * usr;
  rtTHREADFUNC start;
};

typedef struct rtThreadOSD *rtThreadId;

static void * start_routine(void *arg)
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  rtThreadId thread = (rtThreadId)arg;
  thread->start(thread->usr);
  return NULL;
}

rtThreadId rtThreadCreate (
    const char * name, unsigned int priority, unsigned int stackSize,
    rtTHREADFUNC funptr, void * parm)
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  struct sched_param sched = {0};
  rtThreadId thread =(rtThreadId)calloc(1, sizeof(struct rtThreadOSD));
  assert(thread != NULL);
  sched.sched_priority = priority;
  assert(pthread_attr_init(&thread->attr) == 0);
  if(priority){
    assert(pthread_attr_setinheritsched(&thread->attr,
                PTHREAD_EXPLICIT_SCHED) == 0);
    assert(pthread_attr_setschedpolicy(&thread->attr,
                SCHED_FIFO) == 0);
    assert(pthread_attr_setschedparam(&thread->attr,
                &sched) == 0);
  }
  thread->start = funptr;
  thread->usr = parm;
  int result = pthread_create(&thread->thread, &thread->attr, start_routine, thread);
  if( result == 0){
    return thread;
  }
  else{
    switch(result){
      case EAGAIN: LOGERR("rtThreadCreate: EAGAIN (%d)\n", result);
        break;
      case EINVAL: LOGERR("rtThreadCreate: EINVAL (%d)\n", result);
        break;
      case EPERM: LOGERR("rtThreadCreate: EPERM (%d)\n", result);
        break;
      default:
        LOGERR("rtThreadCreate: other error %d\n", result );
        break;
    }
    return NULL;
  }
}

//******

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= MCU_NSEC_PER_SEC) {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - MCU_NSEC_PER_SEC;
  } else {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }
  return result;
}

struct timespec timespec_sub(struct timespec time1, struct timespec time2)
{
  struct timespec result;

  result.tv_sec = time1.tv_sec -time2.tv_sec;
  result.tv_nsec = time1.tv_nsec -time2.tv_nsec;
  if ((time1.tv_nsec - time2.tv_nsec) < 0) {
    result.tv_sec = result.tv_sec -1;
    result.tv_nsec = result.tv_nsec + MCU_NSEC_PER_SEC;
  }
  return result;
}

void cyclic_task(void * usr)
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  int i=0;
  struct timespec wakeupTime , sendTime, lastSendTime= {};
  struct timespec startTime, endTime, lastStartTime= {};
  struct timespec offsetStartTime= {};
  uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,sendperiod_ns = 0,
           latency_min_ns = 0, latency_max_ns = 0,
           period_min_ns = 0, period_max_ns = 0,
           exec_min_ns = 0, exec_max_ns = 0,
           send_min_ns = 0, send_max_ns = 0;

  offsetStartTime.tv_nsec=49*MCU_PERIOD_NS;
  offsetStartTime.tv_sec=0;

  // get current time
  wakeupTime=timespec_add(masterActivationTimeMonotonic,offsetStartTime); // start 50 (49+1) cycle times after master activate
  while(appMode==ECMC_MODE_RUNTIME)
  {
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

    clock_gettime(CLOCK_MONOTONIC, &startTime);
    latency_ns = DIFF_NS(wakeupTime, startTime);
    period_ns = DIFF_NS(lastStartTime, startTime);
    exec_ns = DIFF_NS(lastStartTime, endTime);
    sendperiod_ns = DIFF_NS(lastSendTime, sendTime);
    lastStartTime = startTime;
    lastSendTime=sendTime;

    if (latency_ns > latency_max_ns) {
      latency_max_ns = latency_ns;
    }
    if (latency_ns < latency_min_ns) {
      latency_min_ns = latency_ns;
    }
    if (period_ns > period_max_ns) {
      period_max_ns = period_ns;
    }
    if (period_ns < period_min_ns) {
      period_min_ns= period_ns;
    }
    if (exec_ns > exec_max_ns) {
      exec_max_ns = exec_ns;
    }
    if (exec_ns < exec_min_ns) {
      exec_min_ns = exec_ns;
    }
    if (sendperiod_ns > send_max_ns) {
	send_max_ns = sendperiod_ns;
    }
    if (sendperiod_ns < send_min_ns) {
	send_min_ns = sendperiod_ns;
    }

    ec.receive();
    ec.checkDomainState();

    //Motion
    for( i=0;i<ECMC_MAX_AXES;i++){
      if(axes[i]!=NULL){
        axes[i]->execute(ec.statusOK());
      }
    }

    //Data events
    for( i=0;i<ECMC_MAX_EVENT_OBJECTS;i++){
      if(events[i]!=NULL){
	events[i]->execute(ec.statusOK());
      }
    }

    if (counter) {
      counter--;
    }
    else{ //Lower freq
      counter = MCU_FREQUENCY/axisDiagFreq;
      ec.checkState();
      ec.checkSlavesConfState();
      printStatus();
      ec.printStatus();

      if(PRINT_STDOUT_BIT13()){
        struct timespec testtime;
        clock_gettime(CLOCK_MONOTONIC, &testtime);
        LOGINFO("\nCLOCK: %ld, %ld\n", testtime.tv_sec,testtime.tv_nsec);
        LOGINFO("period     %10u ... %10u\n",
        		period_min_ns, period_max_ns);
        LOGINFO("exec       %10u ... %10u\n",
                exec_min_ns, exec_max_ns);
        LOGINFO("latency    %10u ... %10u\n",
                latency_min_ns, latency_max_ns);
        LOGINFO("send       %10u ... %10u\n",
		send_min_ns, send_max_ns);
        period_max_ns = 0;
        period_min_ns = 0xffffffff;
        exec_max_ns = 0;
        exec_min_ns = 0xffffffff;
        latency_max_ns = 0;
        latency_min_ns = 0xffffffff;
        send_max_ns=0;
        send_min_ns=0xffffffff;
        sendperiod_ns=0;
      }
    }
    clock_gettime(CLOCK_MONOTONIC, &sendTime);
    ec.send(masterActivationTimeOffset);
    clock_gettime(CLOCK_MONOTONIC, &endTime);
  }
}

/****************************************************************************/

int  hw_motor_global_init(void){
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  appMode=ECMC_MODE_CONFIG;
  appModeOld=appMode;
  LOGINFO("\nESS Open Source EtherCAT Motion Control Unit Initializes......\n");
  LOGINFO("\nMode: Configuration\n");

  axisDiagIndex=0;
  axisDiagFreq=10;
  setDiagAxisEnable(1);

  for(int i=0; i<ECMC_MAX_AXES;i++){
    axes[i]=NULL;
  }

  for(int i=0; i<ECMC_MAX_EVENT_OBJECTS;i++){
    events[i]=NULL;
  }

  for(int i=0; i<ECMC_MAX_DATA_RECORDERS_OBJECTS;i++){
    dataRecorders[i]=NULL;
  }

  for(int i=0; i<ECMC_MAX_DATA_STORAGE_OBJECTS;i++){
    dataStorages[i]=NULL;
  }

  for(int i=0; i<ECMC_MAX_COMMANDS_LISTS;i++){
    commandLists[i]=NULL;
  }

  return 0;
}

/****************************************************************************/

void startRTthread()
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  int prio = ECMC_PRIO_HIGH;
  if(rtThreadCreate("cyclic", prio, 0, cyclic_task, NULL) == NULL){
    LOGERR("ERROR: Can't create high priority thread, fallback to low priority\n");
    prio = ECMC_PRIO_LOW;
    assert(rtThreadCreate("cyclic", prio, 0, cyclic_task, NULL) != NULL);
  }
  else{
    LOGINFO4("INFO:\t\tCreated high priority thread for cyclic task\n");
  }
}

int waitForEtherCATtoStart(int timeoutSeconds)
{
  struct timespec timeToPause;
  timeToPause.tv_sec = 1;
  timeToPause.tv_nsec = 0;
  for(int i=0;i<timeoutSeconds;i++){
    LOGINFO("Starting up EtherCAT bus: %d second(s).\n",i);
    clock_nanosleep(CLOCK_MONOTONIC, 0, &timeToPause, NULL);
    if(!ec.getError() && ec.statusOK()){
      clock_nanosleep(CLOCK_MONOTONIC, 0, &timeToPause, NULL);
      LOGINFO("EtherCAT bus started!\n");
      return 0;
    }
  }
  LOGERR("Timeout error: EtherCAT bus did not start correctly in %ds.\n",timeoutSeconds);
  return ec.getErrorID();
}

int setAppModeCfg(int mode)
{
  LOGINFO4("INFO:\t\tApplication in configuration mode.\n");
  appModeOld = appMode;
  appMode=(app_mode_type)mode;
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      axes[i]->setRealTimeStarted(false);
    }
  }
  return 0;
}

int setAppModeRun(int mode)
{
  appModeOld=appMode;
  appMode=(app_mode_type)mode;

  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      axes[i]->setInStartupPhase(true);
    }
  }

  if(appModeOld!=ECMC_MODE_CONFIG){
    return ERROR_MAIN_APP_MODE_ALREADY_RUNTIME;
  }

  int errorCode=validateConfig();
  if (errorCode){
    return errorCode;
  }

  clock_gettime(CLOCK_MONOTONIC, &masterActivationTimeMonotonic);
  clock_gettime(CLOCK_REALTIME, &masterActivationTimeRealtime); // absolute clock (epoch)

  masterActivationTimeOffset=timespec_sub(masterActivationTimeRealtime,masterActivationTimeMonotonic);
  ecrt_master_application_time(ec.getMaster(), TIMESPEC2NS(masterActivationTimeRealtime));


  if(ec.activate()){
    LOGERR("INFO:\t\tActivation of master failed.\n");
    return ERROR_MAIN_EC_ACTIVATE_FAILED;
  }
  LOGINFO4("INFO:\t\tApplication in runtime mode.\n");
  startRTthread();
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      axes[i]->setRealTimeStarted(true);
    }
  }

  errorCode=waitForEtherCATtoStart(30);
  if(errorCode){
    return errorCode;
  }
  return 0;
}

int setAppMode(int mode)
{
  LOGINFO4("%s/%s:%d mode=%d\n",__FILE__, __FUNCTION__, __LINE__,mode);
  switch((app_mode_type)mode){
    case ECMC_MODE_CONFIG:
      return setAppModeCfg(mode);
      break;
    case ECMC_MODE_RUNTIME:
       return setAppModeRun(mode);
      break;
    default:
      LOGERR("WARNING: Mode %d not implemented. (Config=%d, Runtime=%d).\n",mode, ECMC_MODE_CONFIG,ECMC_MODE_RUNTIME);
      return ERROR_MAIN_APP_MODE_NOT_SUPPORTED;
      break;
  }
  return 0;
}

int prepareForRuntime(){
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  //Update input sources for all trajectories and encoders (transforms)
  for(int i=0;i<ECMC_MAX_AXES;i++){
    for(int j=0;j<ECMC_MAX_AXES;j++){
      if(axes[i] !=NULL && axes[j] !=NULL){
        //Trajectory
        if(axes[i]->getExternalTrajIF()!=NULL){
          if(axes[j]->getExternalTrajIF()!=NULL){
            axes[j]->getExternalTrajIF()->addInputDataInterface(axes[i]->getExternalTrajIF()->getOutputDataInterface(),i);
          }
          if(axes[j]->getExternalEncIF()!=NULL){
            axes[j]->getExternalEncIF()->addInputDataInterface(axes[i]->getExternalTrajIF()->getOutputDataInterface(),i);
          }
        }
        //Encoder
        if(axes[i]->getExternalEncIF()!=NULL){
          if(axes[j]->getExternalTrajIF()!=NULL){
            axes[j]->getExternalTrajIF()->addInputDataInterface(axes[i]->getExternalEncIF()->getOutputDataInterface(),i+ECMC_MAX_AXES);
          }
          if(axes[j]->getExternalEncIF()!=NULL){
            axes[j]->getExternalEncIF()->addInputDataInterface(axes[i]->getExternalEncIF()->getOutputDataInterface(),i+ECMC_MAX_AXES);
          }
        }
        axes[i]->setAxisArrayPointer(axes[j],j);
      }
    }
  }
  return 0;
}

int validateConfig(){
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  prepareForRuntime();
  int errorCode=0;
  int axisCount=0;
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      axisCount++;
      errorCode=axes[i]->validate();
      if(errorCode){
        LOGERR("ERROR: Validation failed on axis %d with error code %d.",i,errorCode);
        return errorCode;
      }
    }
  }

  for(int i=0; i<ECMC_MAX_EVENT_OBJECTS;i++){
    if(events[i]!=NULL){
      errorCode=events[i]->validate();
      if(errorCode){
        LOGERR("ERROR: Validation failed on event %d with error code %d.",i,errorCode);
        return errorCode;
      }
    }
  }

  for(int i=0; i<ECMC_MAX_DATA_RECORDERS_OBJECTS;i++){
    dataRecorders[i]=NULL;
    if(dataRecorders[i]!=NULL){
      errorCode=dataRecorders[i]->validate();
      if(errorCode){
        LOGERR("ERROR: Validation failed on data recorder %d with error code %d.",i,errorCode);
        return errorCode;
      }
    }
  }

  return 0;
}

int ecApplyConfig(int masterIndex)
{
  //Master index not used right now..
  LOGINFO4("%s/%s:%d value=%d\n",__FILE__, __FUNCTION__, __LINE__,masterIndex);

  int errorCode=0;
  if((errorCode=ec.writeAndVerifySDOs())){
    LOGERR("ERROR:\tSDO write and verify failed\n");
    return errorCode;
  }
  if((errorCode=ec.compileRegInfo())){
    LOGERR("ERROR:\tCompileRegInfo failed\n");
    return errorCode;
  }
  return 0;
}

int ecSetDiagnostics(int value)
{  //Set diagnostics mode
  LOGINFO4("%s/%s:%d value=%d\n",__FILE__, __FUNCTION__, __LINE__,value);

  return ec.setDiagnostics(value);
}

int ecSetDomainFailedCyclesLimit(int value)
{
  LOGINFO4("%s/%s:%d value=%d\n",__FILE__, __FUNCTION__, __LINE__,value);

  return ec.setDomainFailedCyclesLimitInterlock(value);
}


int ecEnablePrintouts(int value)
{
  LOGINFO4("%s/%s:%d value=%d\n",__FILE__, __FUNCTION__, __LINE__,value);

  WRITE_DIAG_BIT(FUNCTION_ETHERCAT_DIAGNOSTICS_BIT,value);

  return 0;
}

int setEnableFunctionCallDiag(int value)
{
  LOGINFO4("%s/%s:%d value=%d\n",__FILE__, __FUNCTION__, __LINE__,value);

  WRITE_DIAG_BIT(FUNCTION_CALL_DIAGNOSTICS_BIT,value);

  return 0;
}

/****************************************************************************/

int setAxisExecute(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(value &&axes[axisIndex]->getError()){ //Axis needs to be reset before new command is executed (however allow execute=0)
    return ERROR_MAIN_AXIS_ERROR_EXECUTE_INTERLOCKED;
  }

  return axes[axisIndex]->setExecute(value);
}

int setAxisCommand(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setCommand((motionCommandTypes)value);
  return 0;
}

int setAxisCmdData(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setCmdData(value);
  return 0;
}

int setAxisSeqTimeout(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setSequenceTimeout(value*MCU_FREQUENCY);
  return 0;
}


int getAxisCommand(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value= (int)axes[axisIndex]->getSeq()->getCommand();
  return 0;
}

int getAxisCmdData(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getCmdData();
  return 0;
}

int getAxisError(int axisIndex)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getError();;
}

int getAxisErrorID(int axisIndex)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getErrorID();
}

const char *getErrorString(int error_number)
{
  LOGINFO4("%s/%s:%d error_no=%d\n",__FILE__, __FUNCTION__, __LINE__, error_number);

  return ecmcError::convertErrorIdToString(error_number);
}

int getAxisDebugInfoData(int axisIndex,char *buffer, int bufferByteSize)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  ecmcAxisStatusPrintOutType data;
  int error=axes[axisIndex]->getDebugInfoData(&data);
  if(error){
    return error;
  }

  int ret=snprintf(buffer,bufferByteSize,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d",
       data.positionSetpoint,
       data.positionActual,
       data.cntrlError,
       data.cntrlOutput,
       data.positionError,
       data.velocityActual,
       data.velocitySetpoint,
       data.velocityFFRaw,
       data.velocitySetpointRaw,
       data.error,
       data.enable,
       data.execute,
       data.busy,
       data.seqState,
       data.atTarget,
       data.trajInterlock,
       data.limitFwd,
       data.limitBwd,
       data.homeSwitch
       );

  if(ret>=bufferByteSize || ret <=0){
    return ERROR_MAIN_PRINT_TO_BUFFER_FAIL;
  }

  return 0;
}

int setAxisEnable(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(!value){
    axes[axisIndex]->setExecute(value);
  }

  return axes[axisIndex]->setEnable(value);
}

int getAxisEnable(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value= axes[axisIndex]->getEnable();
  return 0;
}

int setAxisEnableAlarmAtHardLimits(int axisIndex,int enable)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, enable);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  int error=axes[axisIndex]->getMon()->setEnableHardLimitBWDAlarm(enable);
  if (error){
    return error;
  }

  error=axes[axisIndex]->getMon()->setEnableHardLimitFWDAlarm(enable);
  if (error){
    return error;
  }
  return 0;
}

int getAxisEnableAlarmAtHardLimits(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  *value= axes[axisIndex]->getMon()->getEnableAlarmAtHardLimit();
  return 0;
}

int getAxisEnabled(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getEnabled();
  return 0;
}

int getAxisTrajSource(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  *value=(int)axes[axisIndex]->getExternalTrajIF()->getDataSourceType();
  return 0;
}

int getAxisEncSource(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  *value=(int)axes[axisIndex]->getExternalEncIF()->getDataSourceType();
  return 0;
}

int getAxisEnableCommandsFromOtherAxis(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=(int)axes[axisIndex]->getCascadedCommandsEnabled();
  return 0;
}

int getAxisEnableCommandsTransform(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=(int)axes[axisIndex]->getEnableCommandsTransform();
  return 0;
}

int getAxisType(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getAxisType();
  return 0;

}

int setAxisOpMode(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setOpMode((operationMode)value);
}

int getAxisOpMode(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value= axes[axisIndex]->getOpMode();
  return 0;
}

int setAxisEnableSoftLimitBwd(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setEnableSoftLimitBwd(value);
  return 0;
}

int setAxisEnableSoftLimitFwd(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setEnableSoftLimitFwd(value);
  return 0;
}

int setAxisSoftLimitPosBwd(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setSoftLimitBwd(value);
  return 0;
}

int setAxisSoftLimitPosFwd(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setSoftLimitFwd(value);
  return 0;
}

int getAxisSoftLimitPosBwd(int axisIndex, double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getMon()->getSoftLimitBwd();
  return 0;
}

int getAxisSoftLimitPosFwd(int axisIndex, double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getMon()->getSoftLimitFwd();
  return 0;
}

int getAxisEnableSoftLimitBwd(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getMon()->getEnableSoftLimitBwd();
  return 0;
}

int getAxisEnableSoftLimitFwd(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getMon()->getEnableSoftLimitFwd();
  return 0;
}

int getAxisBusy(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getBusy();
  return 0;
}

int setAxisAcceleration(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setAcc(value);
  return 0;
}

int setAxisDeceleration(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setDec(value);
  return 0;
}

int setAxisEmergDeceleration(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setEmergDec(value);
  return 0;
}

int setAxisJerk(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setJerk(value);  //TODO not implemented in trajectory generator
  return 0;
}

int setAxisTargetPos(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setTargetPos(value);
  return 0;
}

int setAxisTargetVel(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)
  //CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getSeq()->setTargetVel(value);
  //axes[axisIndex]->getMon()->setTargetVel(value);

  return 0;
}

int setAxisJogVel(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setJogVel(value);
  return 0;
}

int setAxisHomeVelTwordsCam(int axisIndex,double dVel)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, dVel);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setHomeVelTwordsCam(dVel);
}

int setAxisHomeVelOffCam(int axisIndex,double dVel)
{

  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, dVel);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setHomeVelOffCam(dVel);
}

int setAxisHomePos(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setHomePosition(value);
  return 0;
}

int setAxisHomeDir(int axisIndex,int nDir)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, nDir);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setHomeDir((motionDirection)nDir);
}

int setAxisEnableCommandsFromOtherAxis(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEnableCascadedCommands(value);
}

int setAxisEnableCommandsTransform(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEnableCommandsTransform(value);
}

int axisErrorReset(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex,value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  axes[axisIndex]->setReset(value);
  return 0;
}

int setAxisGearRatio(int axisIndex,double ratioNum,double ratioDenom)
{
  LOGINFO4("%s/%s:%d axisIndex=%d num=%lf denom=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, ratioNum,ratioDenom);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getExternalTrajIF()->setGearRatio(ratioNum,ratioDenom);
}

int setAxisEncScaleNum(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  double temp=0;
  int errorCode=axes[axisIndex]->getEncScaleNum(&temp);
  if(errorCode){
    return errorCode;
  }

  if(temp==value){
    return 0;
  }

  if(axes[axisIndex]->getEnable()){  //Change of encoder scale while axis enabled is not allowed (jump in position)
    return ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED;
  }
  axes[axisIndex]->setEncScaleNum(value);

  return 0;
}

int setAxisEncScaleDenom(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  double temp=0;
  int errorCode=axes[axisIndex]->getEncScaleDenom(&temp);
  if(errorCode)  {
    return errorCode;
  }
  if(temp==value){
    return 0;
  }

  if(axes[axisIndex]->getEnable()){  //Change of encoder scale while axis enabled is not allowed (jump in position)
    return ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED;
  }
  axes[axisIndex]->setEncScaleDenom(value);

  return 0;
}

int setAxisTrajTransExpr(int axisIndex, char *expr)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, expr);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_TRANSFORM_RETURN_IF_ERROR(axisIndex)

  std::string tempExpr=expr;

  return axes[axisIndex]->setTrajTransformExpression(tempExpr);
}

int setAxisTransformCommandExpr(int axisIndex,char *expr)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__,axisIndex, expr);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  std::string tempExpr=expr;

  return axes[axisIndex]->setCommandsTransformExpression(tempExpr);
}

int setAxisTrajExtVelFilterEnable(int axisIndex, int enable)
{
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, enable);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_TRANSFORM_RETURN_IF_ERROR(axisIndex)

  ecmcMasterSlaveIF *tempIf=axes[axisIndex]->getExternalTrajIF();
  if(!tempIf){
    return ERROR_MAIN_MASTER_SLAVE_IF_NULL;
  }
  return tempIf->setEnableVelFilter(enable);
}

int setAxisEncTransExpr(int axisIndex, char *expr)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, expr);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENC_TRANSFORM_RETURN_IF_ERROR(axisIndex)

  std::string tempExpr=expr;

  return axes[axisIndex]->setEncTransformExpression(tempExpr);
}

int setAxisEncExtVelFilterEnable(int axisIndex, int enable)
{
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, enable);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENC_TRANSFORM_RETURN_IF_ERROR(axisIndex)

  ecmcMasterSlaveIF *tempIf=axes[axisIndex]->getExternalEncIF();
  if(!tempIf){
    return ERROR_MAIN_MASTER_SLAVE_IF_NULL;
  }
  return tempIf->setEnableVelFilter(enable);
}

const char* getAxisTrajTransExpr(int axisIndex, int *error)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  if(axisIndex>=ECMC_MAX_AXES || axisIndex<=0){
    LOGERR("ERROR: Axis index out of range.\n");
    *error=ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    return "";
  }
  if(axes[axisIndex]==NULL){
    LOGERR("ERROR: Axis object NULL\n");
    *error=ERROR_MAIN_AXIS_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getTraj()==NULL){
    LOGERR("ERROR: Trajectory object NULL.\n");
    *error=ERROR_MAIN_TRAJECTORY_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getExternalTrajIF()->getExtInputTransform()==NULL){
    LOGERR("ERROR: Trajectory transform object NULL.\n");
    *error=ERROR_MAIN_TRAJ_TRANSFORM_OBJECT_NULL;
    return "";
  }
  std::string *sExpr=axes[axisIndex]->getExternalTrajIF()->getExtInputTransform()->getExpression();
  *error=0;
  return sExpr->c_str();
}

const char* getAxisEncTransExpr(int axisIndex, int *error)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  if(axisIndex>=ECMC_MAX_AXES || axisIndex<=0){
    LOGERR("ERROR: Axis index out of range.\n");
    *error=ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    return "";
  }
  if(axes[axisIndex]==NULL){
    LOGERR("ERROR: Axis object NULL\n");
    *error=ERROR_MAIN_AXIS_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getEnc()==NULL){
    LOGERR("ERROR: Encoder object NULL.\n");
    *error=ERROR_MAIN_ENCODER_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getExternalEncIF()->getExtInputTransform()==NULL){
    LOGERR("ERROR: Encoder transform object NULL.\n");
    *error=ERROR_MAIN_ENC_TRANSFORM_OBJECT_NULL;
    return "";
  }
  std::string *sExpr=axes[axisIndex]->getExternalEncIF()->getExtInputTransform()->getExpression();
  *error=0;
  return sExpr->c_str();
}

const char* getAxisTransformCommandExpr(int axisIndex, int *error)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  if(axisIndex>=ECMC_MAX_AXES || axisIndex<=0){
    LOGERR("ERROR: Axis index out of range.\n");
    *error=ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    return "";
  }
  if(axes[axisIndex]==NULL){
    LOGERR("ERROR: Axis object NULL\n");
    *error=ERROR_MAIN_AXIS_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getCommandTransform()==NULL){
    LOGERR("ERROR: Axis command transform object NULL.\n");
    *error=ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED;
    return "";
  }
  std::string *sExpr=axes[axisIndex]->getCommandTransform()->getExpression();
  *error=0;
  return sExpr->c_str();

}

int setAxisTrajSource(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setTrajDataSourceType((dataSource)value);
}

int setAxisEncSource(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEncDataSourceType((dataSource)value);
}

int setAxisTrajStartPos(int axisIndex,double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setStartPos(value);
  return 0;
}

//*****GET*********
int getAxisAcceleration(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getAcc();
  return 0;
}

int getAxisDeceleration(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getDec();
  return 0;
}

int getAxisTargetPos(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  return axes[axisIndex]->getPosSet(value);;
}

int getAxisTargetVel(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getTargetVel();
  return 0;
}

int getAxisDone(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=!axes[axisIndex]->getSeq()->getBusy();
  return 0;
}

int getAxisPosSet(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getPosSet(value);
}

int getAxisVelFF(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getVel();
  return 0;
}

int getAxisExecute(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getExecute();
  return 0;
}

int getAxisReset(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=axes[axisIndex]->getReset();
  return 0;
}

int getAxisID(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=axes[axisIndex]->getAxisID();
  return 0;
}

int getAxisGearRatio(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]-> getExternalTrajIF()->getGearRatio(value);
}

int getAxisAtHardFwd(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getMon()->getHardLimitFwd();
  return 0;
}

int getAxisAtHardBwd(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getMon()->getHardLimitBwd();
  return 0;
}

int getAxisEncHomed(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  bool tempHomed=0;
  int errorCode=axes[axisIndex]->getAxisHomed(&tempHomed);
  if(errorCode){
    return errorCode;
  }

  *value=tempHomed;
  return 0;
}

int getAxisEncPosAct(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

    if(int iRet=axes[axisIndex]->getPosAct(value)){
      value=0;
      return iRet;
    }
  return 0;
}

int getAxisEncVelAct(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(int iRet=axes[axisIndex]->getVelAct(value)){
    *value=0;
    return iRet;
  }

  return 0;
}

int getAxisAtHome(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(axes[axisIndex]->getMon()==NULL){
    return ERROR_MAIN_MONITOR_OBJECT_NULL;
  }

  *value=axes[axisIndex]->getMon()->getHomeSwitch();
  return 0;
}

int getAxisCntrlError(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(int iRet=axes[axisIndex]->getCntrlError(value)){
    *value=0;
    return iRet;
  }
  return 0;
}

int getAxisHomeVelOffCam(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getSeq()->getHomeVelOffCam();
  return 0;
}

int getAxisHomeVelTwordsCam(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getSeq()->getHomeVelTwordsCam();
  return 0;
}

int getAxisEncScaleNum(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  if(int iRet=axes[axisIndex]->getEncScaleNum(value)){
    value=0;
    return iRet;
  }
  return 0;
}

int getAxisEncScaleDenom(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  if(int iRet=axes[axisIndex]->getEncScaleDenom(value)){
    *value=0;
    return iRet;
  }
  return 0;
}

int getAxisEncPosRaw(int axisIndex,int64_t *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  if(int iRet=axes[axisIndex]->getEncPosRaw(value)){
    *value=0;
    return iRet;
  }
  return 0;
}

/****************************************************************************/

int setAxisCntrlKp(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKp(value);
  return 0;
}

int setAxisCntrlKi(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKi(value);
  return 0;
}

int setAxisCntrlKd(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKd(value);
  return 0;
}

int setAxisCntrlKff(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);
  //CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKff(value);
  //axes[axisIndex]->getMon()->setCntrlKff(value);
  return 0;
}

int setAxisCntrlOutHL(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setOutMax(value);
  return 0;
}

int setAxisCntrlOutLL(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setOutMin(value);
  return 0;
}

int setAxisCntrlIpartHL(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setIOutMax(value);
  return 0;
}

int setAxisCntrlIpartLL(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setIOutMin(value);
  return 0;
}

//Cntrl GET
/*int getAxisCntrlEnable(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getCntrl()->getEnable();
  return 0;
}*/

int getAxisCntrlOutPpart(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getCntrl()->getOutPPart();
  return 0;
}

int getAxisCntrlOutIpart(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getCntrl()->getOutIPart();
  return 0;
}

int getAxisCntrlOutDpart(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getCntrl()->getOutDPart();
  return 0;
}

int getAxisCntrlOutFFpart(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getCntrl()->getOutFFPart();
  return 0;
}

int getAxisCntrlOutput(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getCntrl()->getOutTot();
  return 0;
}

int setAxisEncOffset(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getEnc()->setOffset(value);
  return 0;
}

int setAxisEncBits(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getEnc()->setBits(value);
  return 0;
}

int setAxisEncType(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getEnc()->setType((encoderType)value);
}

/****************************************************************************/
//Drv SET
int setAxisDrvScaleNum(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getDrv()->setScaleNum(value);
  return 0;
}

int setAxisDrvScaleDenom(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setScaleDenom(value);
}

int setAxisDrvEnable(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setEnable(value);
}

int setAxisDrvVelSet(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setVelSet(value);
}

int setAxisDrvVelSetRaw(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setVelSetRaw(value);
}


int setAxisDrvBrakeEnable(int axisIndex, int enable)
{
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, enable);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setEnableBrake(enable);
}

int setAxisDrvReduceTorqueEnable(int axisIndex, int enable)
{
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, enable);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setEnableReduceTorque(enable);
}

int setAxisDrvType(int axisIndex, int type)
{
  LOGINFO4("%s/%s:%d axisIndex=%d type=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, type);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->setDriveType((ecmcDriveTypes)type);
}


//Drv GET
int getAxisDrvScale(int axisIndex,double *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getDrv()->getScale();;
  return 0;
}

int getAxisDrvEnable(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getDrv()->getEnable();;
  return 0;
}

/****************************************************************************/
//Mon SET
int setAxisMonAtTargetTol(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setAtTargetTol(value);
  return 0;
}

int setAxisMonAtTargetTime(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setAtTargetTime(value);
  return 0;
}

int setAxisMonEnableAtTargetMon(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setEnableAtTargetMon(value);
  return 0;
}

int setAxisMonPosLagTol(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setPosLagTol(value);
  return 0;
}

int setAxisMonPosLagTime(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setPosLagTime(value);
  return 0;
}

int setAxisMonEnableLagMon(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setEnableLagMon(value);
  return 0;
}

int setAxisMonMaxVel(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setMaxVel(value);;
}

int setAxisMonEnableMaxVel(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableMaxVelMon(value);
}

int setAxisMonMaxVelDriveILDelay(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setMaxVelDriveTime(value);
}

int setAxisMonMaxVelTrajILDelay(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setMaxVelTrajTime(value);
}

int setAxisMonEnableExternalInterlock(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableHardwareInterlock(value);
}

int setAxisMonEnableCntrlOutHLMon(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableCntrlHLMon(value);
}

int setAxisMonEnableVelocityDiff(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableVelocityDiffMon(value);
}

int setAxisMonVelDiffTol(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setVelDiffMaxDifference(value);
}

int setAxisMonVelDiffTrajILDelay(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setVelDiffTimeTraj(value);
}

int setAxisMonVelDiffDriveILDelay(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setVelDiffTimeDrive(value);
}

int setAxisMonCntrlOutHL(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setCntrlOutputHL(value);
}

//Mon GET
int getAxisMonAtTarget(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value=axes[axisIndex]->getMon()->getAtTarget();
  return 0;
}

/****************************************************************************/
//Configuration procedures

int createAxis(int index, int type)
{
  LOGINFO4("%s/%s:%d axisIndex=%d type:%d\n",__FILE__, __FUNCTION__, __LINE__,index,type);

  if(index<0 && index>=ECMC_MAX_AXES){
    return ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
  }

  switch((axisType)type){

    case ECMC_AXIS_TYPE_REAL:
      if(axes[index]!=NULL){
        delete axes[index];
      }
      axes[index]=new ecmcAxisReal(index,1/MCU_FREQUENCY);
      break;

    case ECMC_AXIS_TYPE_VIRTUAL:
      if(axes[index]!=NULL){
        delete axes[index];
      }
      axes[index]=new ecmcAxisVirt(index,1/MCU_FREQUENCY);
      break;
    default:
      return ERROR_MAIN_AXIS_TYPE_UNKNOWN;
  }

  axisDiagIndex=index; //Always printout last axis added
  return 0;
}

int ecSetMaster(int masterIndex)
{
  LOGINFO4("%s/%s:%d master index=%d \n",__FILE__, __FUNCTION__, __LINE__, masterIndex);

  return ec.init(masterIndex);
}

int ecResetMaster(int masterIndex)
{
  LOGINFO4("%s/%s:%d master index=%d \n",__FILE__, __FUNCTION__, __LINE__, masterIndex);
  ///todo  master index not used. Only there for future use.
  return ec.reset();
}

int ecResetError()
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);

  ec.errorReset();
  return 0;
}

int ecAddSlave(uint16_t alias, uint16_t position, uint32_t vendorId,uint32_t productCode)
{
  LOGINFO4("%s/%s:%d alias=%d position=%d vendor_id=%d product_code=%d\n",__FILE__, __FUNCTION__, __LINE__, alias,position,vendorId,productCode);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  ec.addSlave(alias,position,vendorId,productCode);
  return 0;
}

int ecSlaveConfigDC(
    int slaveBusPosition,
    uint16_t assignActivate, /**< AssignActivate word. */
    uint32_t sync0Cycle, /**< SYNC0 cycle time [ns]. */
    int32_t sync0Shift, /**< SYNC0 shift time [ns]. */
    uint32_t sync1Cycle, /**< SYNC1 cycle time [ns]. */
    int32_t sync1Shift /**< SYNC1 shift time [ns]. */)
{
  LOGINFO4("%s/%s:%d position=%d assign_axtive=%x sync0_cycle=%d sync0_shift=%d sync1_cycle=%d sync1_shift=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveBusPosition,assignActivate,sync0Cycle,sync0Shift,sync1Cycle,sync1Shift);

  ecmcEcSlave *slave=ec.findSlave(slaveBusPosition);
  if(slave==NULL){
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  return slave->configDC(assignActivate,sync0Cycle,sync0Shift,sync1Cycle,sync1Shift);
}

int ecSelectReferenceDC(int masterIndex,int slaveBusPosition)
{
  LOGINFO4("%s/%s:%d master=%d position=%d\n",__FILE__, __FUNCTION__, __LINE__,masterIndex, slaveBusPosition);

  ecmcEcSlave *slave=ec.findSlave(slaveBusPosition);
  if(slave==NULL){
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  return slave->selectAsReferenceDC();
}

int ecAddEntryComplete(
    uint16_t position,
    uint32_t vendorId,
    uint32_t productCode,
    int direction,
    uint8_t syncMangerIndex,
    uint16_t pdoIndex,
    uint16_t entryIndex,
    uint8_t  entrySubIndex,
    uint8_t bits,
    char *entryIDString
    )
{
  std::string id=entryIDString;

  LOGINFO4("%s/%s:%d slave=%d vendor=%d productcode=%d direction=%d sm=%d pdoindex=%d entry_index=%d entry_subindex=%d bits=%d id=%s\n",__FILE__, __FUNCTION__, __LINE__, position,vendorId,productCode,direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,entryIDString);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return ec.addEntry(position,vendorId,productCode,(ec_direction_t)direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,id);
}

int ecAddPdo(int slaveIndex,int syncManager,uint16_t pdoIndex)
{
  LOGINFO4("%s/%s:%d slave=%d sm=%d pdo_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,syncManager,pdoIndex);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  if(ec.getSlave(slaveIndex)==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;
  if(ec.getSlave(slaveIndex)->getSyncManager(syncManager)==NULL)
    return ERROR_MAIN_EC_SM_NULL;
  return ec.getSlave(slaveIndex)->getSyncManager(syncManager)->addPdo(pdoIndex);
}

int ecAddSyncManager(int slaveIndex,int direction,uint8_t syncMangerIndex)
{
  LOGINFO4("%s/%s:%d slave=%d direction=%d sm_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,direction,syncMangerIndex);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  if(ec.getSlave(slaveIndex)==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;
  return ec.getSlave(slaveIndex)->addSyncManager((ec_direction_t)direction,syncMangerIndex);;
}

int ecAddSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize)
{
  LOGINFO4("%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d value=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,sdoSubIndex,value,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return (int)ec.addSDOWrite(slavePosition,sdoIndex,sdoSubIndex,value,byteSize);
}

int ecWriteSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value,int byteSize)
{
  LOGINFO4("%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d value=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,sdoSubIndex,value,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec.writeSDO(slavePosition,sdoIndex,sdoSubIndex,value,byteSize);
}

uint32_t ecReadSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,int byteSize)
{
  LOGINFO4("%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,sdoSubIndex,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return ec.readSDO(slavePosition,sdoIndex,sdoSubIndex,byteSize);
}

int ecSlaveConfigWatchDog(int slaveBusPosition,int watchdogDivider,int watchdogIntervals)
{
  LOGINFO4("%s/%s:%d position=%d, watchdogDivider=%d watchdogIntervals=%d\n",__FILE__, __FUNCTION__, __LINE__,slaveBusPosition,watchdogDivider,watchdogIntervals);

  ecmcEcSlave *slave=ec.findSlave(slaveBusPosition);
  if(slave==NULL){
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  return slave->setWatchDogConfig(watchdogDivider,watchdogIntervals);
}

int linkEcEntryToAxisEnc(int slaveIndex, char *entryIDString,int axisIndex,int encoderEntryIndex, int bitIndex)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s encoder=%d encoder_entry=%d bit_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex,encoderEntryIndex, bitIndex);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slaveIndex>=0){
    slave=ec.findSlave(slaveIndex);
  }
  else{ //simulation slave
    slave=ec.getSlave(slaveIndex);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);


  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  if(encoderEntryIndex>=MaxEcEntryLinks && encoderEntryIndex<0)
    return ERROR_MAIN_ENCODER_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getEnc()->setEntryAtIndex(entry,encoderEntryIndex,bitIndex);
}

int linkEcEntryToAxisDrv(int slaveIndex,char *entryIDString,int axisIndex,int driveEntryIndex, int bitIndex)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s drive=%d drive_entry=%d bit_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex,driveEntryIndex,bitIndex);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slaveIndex>=0){
    slave=ec.findSlave(slaveIndex);
  }
  else
  {
    slave=ec.getSlave(slaveIndex);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  if(driveEntryIndex>=MaxEcEntryLinks && driveEntryIndex<0)
    return ERROR_MAIN_DRIVE_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getDrv()->setEntryAtIndex(entry,driveEntryIndex,bitIndex);
}

int linkEcEntryToAxisMon(int slaveIndex,char *entryIDString,int axisIndex,int monitorEntryIndex, int bitIndex)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s monitor=%d monitor_entry=%d bit_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex,monitorEntryIndex,bitIndex);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slaveIndex>=0){
    slave=ec.findSlave(slaveIndex);
  }
  else
  {
    slave=ec.getSlave(slaveIndex);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;


  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  if(monitorEntryIndex>=MaxEcEntryLinks && monitorEntryIndex<0)
    return ERROR_MAIN_MONITOR_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getMon()->setEntryAtIndex(entry,monitorEntryIndex,bitIndex);
}

int writeEcEntry(int slaveIndex, int entryIndex,uint64_t value)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%d value=%llu \n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIndex,(long long unsigned int)value);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  if(ec.getSlave(slaveIndex)==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;
  if(ec.getSlave(slaveIndex)->getEntry(entryIndex)==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;
  return ec.getSlave(slaveIndex)->getEntry(entryIndex)->writeValue(value);
}

int writeEcEntryIDString(int slavePosition,char * entryIDString,uint64_t value)
{
  LOGINFO4("%s/%s:%d slave_position=%d entry=%s value=%llu \n",__FILE__, __FUNCTION__, __LINE__, slavePosition,entryIDString,(long long unsigned int)value);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slavePosition>=0){
    slave=ec.findSlave(slavePosition);
  }
  else{ //simulation slave
    slave=ec.getSlave(slavePosition);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;

  return entry->writeValue(value);;
}

int readEcEntry(int slaveIndex, int entryIndex,uint64_t *value)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIndex);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  if(ec.getSlave(slaveIndex)==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;
  if(ec.getSlave(slaveIndex)->getEntry(entryIndex)==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;
  return ec.getSlave(slaveIndex)->getEntry(entryIndex)->readValue(value);
}

int readEcEntryIDString(int slavePosition,char *entryIDString,uint64_t *value)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,entryIDString);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slavePosition>=0){
    slave=ec.findSlave(slavePosition);
  }
  else{ //simulation slave
    slave=ec.getSlave(slavePosition);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string tempEntryIDString=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(tempEntryIDString);

  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;

  return entry->readValue(value);
}

int readEcEntryIndexIDString(int slavePosition,char *entryIDString,int *value)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,entryIDString);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slavePosition>=0){
    slave=ec.findSlave(slavePosition);
  }
  else{ //simulation slave
    slave=ec.getSlave(slavePosition);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string tempEntryIDString=entryIDString;

  int entryIndex=slave->findEntryIndex(tempEntryIDString);

  if(entryIndex<0)
    return ERROR_MAIN_EC_ENTRY_NULL;

  *value=entryIndex;
  return 0;
}

int readEcSlaveIndex(int slavePosition,int *value)
{
  LOGINFO4("%s/%s:%d slave_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec.findSlaveIndex(slavePosition,value);
}

int setDiagAxisIndex(int axisIndex)
{
  LOGINFO4("%s/%s:%d axisIndex=%d \n",__FILE__, __FUNCTION__, __LINE__,axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  axisDiagIndex=axisIndex;
  return 0;
}

int setDiagAxisFreq(int value)
{
  LOGINFO4("%s/%s:%d frequency=%d \n",__FILE__, __FUNCTION__, __LINE__,value);

  if(value<1 || value>500)
    return ERROR_MAIN_DIAG_AXIS_FREQ_OUT_OF_RANGE;

  axisDiagFreq=value;
  return 0;
}

int setDiagAxisEnable(int value)
{
  LOGINFO4("%s/%s:%d enable=%d \n",__FILE__, __FUNCTION__, __LINE__,value);

  WRITE_DIAG_BIT(FUNCTION_HW_MOTOR_AXIS_DIAGNOSTICS_BIT,value);

  return 0;
}

int setEnableTimeDiag(int value)
{
  LOGINFO4("%s/%s:%d enable=%d \n",__FILE__, __FUNCTION__, __LINE__,value);

  WRITE_DIAG_BIT(FUNCTION_TIMING_DIAGNOSTICS_BIT,value);

  return 0;
}

int createEvent(int indexEvent)
{
  LOGINFO4("%s/%s:%d indexEvent=%d \n",__FILE__, __FUNCTION__, __LINE__,indexEvent);

  if(indexEvent>=ECMC_MAX_EVENT_OBJECTS || indexEvent<0){
    return ERROR_MAIN_EVENT_INDEX_OUT_OF_RANGE;
  }

  delete events[indexEvent];
  events[indexEvent]=new ecmcEvent(1/MCU_FREQUENCY,indexEvent);
  return 0;
}

int createDataStorage(int index, int elements, int bufferType)
{
  LOGINFO4("%s/%s:%d index=%d elements=%d \n",__FILE__, __FUNCTION__, __LINE__,index,elements);

  if(index>=ECMC_MAX_DATA_STORAGE_OBJECTS || index<0){
    return ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE;
  }

  if(elements<=0){
    return ERROR_MAIN_DATA_STORAGE_INVALID_SIZE;
  }

  delete dataStorages[index];
  dataStorages[index]=new ecmcDataStorage(index,elements,(storageType)bufferType);
  return 0;
}

int linkStorageToRecorder(int indexStorage,int indexRecorder)
{
  LOGINFO4("%s/%s:%d indexStorage=%d indexRecorder=%d \n",__FILE__, __FUNCTION__, __LINE__,indexStorage,indexRecorder);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);
  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->setDataStorage(dataStorages[indexStorage]);
}

int linkEcEntryToEvent(int indexEvent,int eventEntryIndex,int slaveIndex,char *entryIDString,int bitIndex)
{
  LOGINFO4("%s/%s:%d indexEvent=%d eventEntryIndex=%d slave_index=%d entry=%s bitIndex=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent, eventEntryIndex,slaveIndex,entryIDString,bitIndex);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slaveIndex>=0){
    slave=ec.findSlave(slaveIndex);
  }
  else{ //simulation slave
    slave=ec.getSlave(slaveIndex);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;

  return events[indexEvent]->setEntryAtIndex(entry,eventEntryIndex,bitIndex);
}

int setEventType(int indexEvent,int type)
{
  LOGINFO4("%s/%s:%d indexEvent=%d recordingType=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent, type);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setEventType((eventType)type);
}

int setEventSampleTime(int indexEvent,int sampleTime)
{
  LOGINFO4("%s/%s:%d indexEvent=%d sampleTime=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent, sampleTime);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setDataSampleTime(sampleTime);
}

int setEventExecute(int indexEvent,int execute)
{
  LOGINFO4("%s/%s:%d indexEvent=%d execute=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent, execute);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setExecute(execute);
}

int clearStorage(int indexStorage)
{
  LOGINFO4("%s/%s:%d indexStorage=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->clearBuffer();
}

int setStorageEnablePrintouts(int indexStorage,int enable)
{
  LOGINFO4("%s/%s:%d indexStorage=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage,enable);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  WRITE_DIAG_BIT(FUNCTION_DATA_STORAGE_DIAGNOSTICS_BIT,enable);
  return 0;
}

int printStorageBuffer(int indexStorage)
{
  LOGINFO4("%s/%s:%d indexStorage=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->printBuffer();
}

int readStorageBuffer(int indexStorage, double **data, int* size)
{
  LOGINFO4("%s/%s:%d indexStorage=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->getData(data,size);
}

int writeStorageBuffer(int indexStorage, double *data, int size)
{
  LOGINFO4("%s/%s:%d indexStorage=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->setData(data,size);
}

int appendStorageBuffer(int indexStorage, double *data, int size)
{
  LOGINFO4("%s/%s:%d indexStorage=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->appendData(data,size);
}

int setDataStorageCurrentDataIndex(int indexStorage,int position)
{
  LOGINFO4("%s/%s:%d indexStorage=%d position=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage, position);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->setCurrentPosition(position);
}

int setEventTriggerEdge(int indexEvent,int triggerEdge)
{
  LOGINFO4("%s/%s:%d indexEvent=%d triggerEdge=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent, triggerEdge);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setTriggerEdge((triggerEdgeType)triggerEdge);
}

int setEventEnableArmSequence(int indexEvent,int enable)
{
  LOGINFO4("%s/%s:%d indexEvent=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent, enable);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setEnableArmSequence(enable);
}

int setEventEnablePrintouts(int indexEvent,int enable)
{
  LOGINFO4("%s/%s:%d indexEvent=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent,enable);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  WRITE_DIAG_BIT(FUNCTION_EVENTS_DIAGNOSTICS_BIT,enable);
  return 0;
}

int triggerEvent(int indexEvent)
{
  LOGINFO4("%s/%s:%d indexEvent=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->triggerEvent(ec.statusOK());
}

int armEvent(int indexEvent)
{
  LOGINFO4("%s/%s:%d indexEvent=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->arm();
}

int createRecorder(int indexRecorder)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d \n",__FILE__, __FUNCTION__, __LINE__,indexRecorder);

  if(indexRecorder>=ECMC_MAX_DATA_RECORDERS_OBJECTS || indexRecorder<0){
    return ERROR_MAIN_DATA_RECORDER_INDEX_OUT_OF_RANGE;
  }

  delete dataRecorders[indexRecorder];
  dataRecorders[indexRecorder]=new ecmcDataRecorder(indexRecorder);
  return 0;

}

int linkEcEntryToRecorder(int indexRecorder,int recorderEntryIndex,int slaveIndex,char *entryIDString,int bitIndex)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d recorderEntryIndex=%d slave_index=%d entry=%s bitIndex=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder, recorderEntryIndex,slaveIndex,entryIDString,bitIndex);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave=NULL;
  if(slaveIndex>=0){
    slave=ec.findSlave(slaveIndex);
  }
  else{ //simulation slave
    slave=ec.getSlave(slaveIndex);
  }

  if(slave==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL)
    return ERROR_MAIN_EC_ENTRY_NULL;

  return dataRecorders[indexRecorder]->setEntryAtIndex(entry,recorderEntryIndex,bitIndex);
}

int setRecorderEnablePrintouts(int indexRecorder,int enable)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder,enable);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  WRITE_DIAG_BIT(FUNCTION_DATA_RECORDER_DIAGNOSTICS_BIT,enable);
  return 0;
}

int setRecorderExecute(int indexRecorder,int execute)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d execute=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder, execute);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->setExecute(execute);
}

int linkRecorderToEvent(int indexRecorder,int indexEvent, int consumerIndex)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d indexEvent=%d consumerIndex=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder, indexEvent,consumerIndex);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);
  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);
  return events[indexEvent]->linkEventConsumer(dataRecorders[indexRecorder],consumerIndex);
}

int triggerRecorder(int indexRecorder)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->executeEvent(ec.statusOK());
}

int getControllerError()
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);

  //EtherCAT errors
  if(ec.getError()){
    return ec.getErrorID();
  }

  //Event errors
  for(int i=0; i< ECMC_MAX_EVENT_OBJECTS;i++){
    if(events[i]!=NULL){
      if(events[i]->getError()){
        return events[i]->getErrorID();
      }
    }
  }

  //DataRecorders
  for(int i=0; i< ECMC_MAX_DATA_RECORDERS_OBJECTS;i++){
    if(dataRecorders[i]!=NULL){
      if(dataRecorders[i]->getError()){
        return dataRecorders[i]->getErrorID();
      }
    }
  }

  //Data Storages
  for(int i=0; i< ECMC_MAX_DATA_STORAGE_OBJECTS;i++){
    if(dataStorages[i]!=NULL){
      if(dataStorages[i]->getError()){
        return dataStorages[i]->getErrorID();
      }
    }
  }

  //CommandLists
  for(int i=0; i< ECMC_MAX_COMMANDS_LISTS;i++){
    if(commandLists[i]!=NULL){
      if(commandLists[i]->getError()){
        return commandLists[i]->getErrorID();
      }
    }
  }

  //Axes
  for(int i=0; i< ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      if(axes[i]->getError()){
        return axes[i]->getErrorID();
      }
    }
  }

  return controllerError;
}

int controllerErrorReset()
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);

  //EtherCAT errors
  ec.errorReset();

  //Event errors
  for(int i=0; i< ECMC_MAX_EVENT_OBJECTS;i++){
    if(events[i]!=NULL){
      events[i]->errorReset();
    }
  }

  //DataRecorders
  for(int i=0; i< ECMC_MAX_DATA_RECORDERS_OBJECTS;i++){
    if(dataRecorders[i]!=NULL){
      dataRecorders[i]->errorReset();
    }
  }

  //Data Storages
  for(int i=0; i< ECMC_MAX_DATA_STORAGE_OBJECTS;i++){
    if(dataStorages[i]!=NULL){
      dataStorages[i]->errorReset();
    }
  }

  //CommandLists
  for(int i=0; i< ECMC_MAX_COMMANDS_LISTS;i++){
    if(commandLists[i]!=NULL){
      commandLists[i]->errorReset();
    }
  }

  //Axes
  for(int i=0; i< ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      axes[i]->errorReset();
    }
  }

  return 0;
}

int createCommandList(int indexCommandList)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d \n",__FILE__, __FUNCTION__, __LINE__,indexCommandList);

  if(indexCommandList>=ECMC_MAX_COMMANDS_LISTS || indexCommandList<0){
    return ERROR_COMMAND_LIST_INDEX_OUT_OF_RANGE;
  }

  delete commandLists[indexCommandList];
  commandLists[indexCommandList]=new ecmcCommandList(indexCommandList);
  return 0;
}

int linkCommandListToEvent(int indexCommandList,int indexEvent, int consumerIndex)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d indexEvent=%d consumerIndex=%d\n",__FILE__, __FUNCTION__, __LINE__,indexCommandList, indexEvent,consumerIndex);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);
  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->linkEventConsumer(commandLists[indexCommandList],consumerIndex);
}

int setCommandListExecute(int indexCommandList,int execute)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d execute=%d\n",__FILE__, __FUNCTION__, __LINE__,indexCommandList, execute);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  return commandLists[indexCommandList]->setExecute(execute);
}

int setCommandListEnablePrintouts(int indexCommandList,int enable)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexCommandList,enable);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  WRITE_DIAG_BIT(FUNCTION_COMMAND_LIST_DIAGNOSTICS_BIT,enable);
  return 0;
}

int addCommandListCommand(int indexCommandList,char *expr)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__, indexCommandList, expr);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  std::string tempExpr=expr;

  return commandLists[indexCommandList]->addCommand(tempExpr);
}

int triggerCommandList(int indexCommandList)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d\n",__FILE__, __FUNCTION__, __LINE__,indexCommandList);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  return commandLists[indexCommandList]->executeEvent(1); //No need for state of ethercat master
}


//Summary functions
int moveAbsolutePosition(int axisIndex,double positionSet, double velocitySet, double accelerationSet, double decelerationSet)
{
  LOGINFO4("%s/%s:%d axisIndex=%d, positionSet=%lf, velocitySet=%lf, accelerationSet=%lf, decelerationSet=%lf\n",__FILE__, __FUNCTION__, __LINE__,axisIndex,positionSet,velocitySet,accelerationSet,decelerationSet);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  int errorCode=axes[axisIndex]->getErrorID();
  if(errorCode){
    return errorCode;
  }
  errorCode=axes[axisIndex]->setExecute(0);
  if(errorCode){
    return errorCode;
  }
  errorCode=axes[axisIndex]->setCommand(ECMC_CMD_MOVEABS);
  if(errorCode){
    return errorCode;
  }
  errorCode=axes[axisIndex]->setCmdData(0);
  if(errorCode){
    return errorCode;
  }
  axes[axisIndex]->getSeq()->setTargetPos(positionSet);
  axes[axisIndex]->getSeq()->setTargetVel(velocitySet);
  axes[axisIndex]->getTraj()->setAcc(accelerationSet);
  axes[axisIndex]->getTraj()->setDec(decelerationSet);
  errorCode=axes[axisIndex]->setExecute(1);
  if(errorCode){
    return errorCode;
  }
  return 0;
}

int moveRelativePosition(int axisIndex,double positionSet, double velocitySet, double accelerationSet, double decelerationSet)
{
  LOGINFO4("%s/%s:%d axisIndex=%d, positionSet=%lf, velocitySet=%lf, accelerationSet=%lf, decelerationSet=%lf\n",__FILE__, __FUNCTION__, __LINE__,axisIndex,positionSet,velocitySet,accelerationSet,decelerationSet);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  int errorCode=axes[axisIndex]->getErrorID();
  if(errorCode){
    return errorCode;
  }
  errorCode=axes[axisIndex]->setExecute(0);
  if(errorCode){
    return errorCode;
  }
  errorCode=axes[axisIndex]->setCommand(ECMC_CMD_MOVEREL);
  if(errorCode){
    return errorCode;
  }
  errorCode=axes[axisIndex]->setCmdData(0);
  if(errorCode){
    return errorCode;
  }
  axes[axisIndex]->getSeq()->setTargetPos(positionSet);
  axes[axisIndex]->getSeq()->setTargetVel(velocitySet);
  axes[axisIndex]->getTraj()->setAcc(accelerationSet);
  axes[axisIndex]->getTraj()->setDec(decelerationSet);
  errorCode=axes[axisIndex]->setExecute(1);
  if(errorCode){
    return errorCode;
  }
  return 0;
}

int moveVelocity(int axisIndex,double velocitySet, double accelerationSet, double decelerationSet)
{
   LOGINFO4("%s/%s:%d axisIndex=%d, velocitySet=%lf, accelerationSet=%lf, decelerationSet=%lf\n",__FILE__, __FUNCTION__, __LINE__,axisIndex,velocitySet,accelerationSet,decelerationSet);

   CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
   CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
   CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

   int errorCode=axes[axisIndex]->getErrorID();
   if(errorCode){
     return errorCode;
   }
   errorCode=axes[axisIndex]->setExecute(0);
   if(errorCode){
     return errorCode;
   }
   errorCode=axes[axisIndex]->setCommand(ECMC_CMD_MOVEVEL);
   if(errorCode){
     return errorCode;
   }
   errorCode=axes[axisIndex]->setCmdData(0);
   if(errorCode){
     return errorCode;
   }
   axes[axisIndex]->getSeq()->setTargetVel(velocitySet);
   axes[axisIndex]->getTraj()->setAcc(accelerationSet);
   axes[axisIndex]->getTraj()->setAcc(decelerationSet);
   errorCode=axes[axisIndex]->setExecute(1);
   if(errorCode){
     return errorCode;
   }
   return 0;
}

int stopMotion(int axisIndex,int killAmplifier)
{
   LOGINFO4("%s/%s:%d axisIndex=%d, killAmplifier=%d\n",__FILE__, __FUNCTION__, __LINE__,axisIndex,killAmplifier);

   CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

   if(killAmplifier){
     int errorCode=axes[axisIndex]->setEnable(0);
     if(errorCode){
       return errorCode;
     }
   }

   int errorCode=axes[axisIndex]->setExecute(0);
   if(errorCode){
     return errorCode;
   }

   return 0;
}
