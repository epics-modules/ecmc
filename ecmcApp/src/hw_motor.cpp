
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
#include "ecmcEc.h"
#include "ecmcEcPdo.h"
#include "ecmcEcSlave.h"
#include "ecmcEcSyncManager.h"
#include "ecmcEcEntry.h"

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
#include "ecmcAsynPortDriver.h"
#include "ecmcPLC.h"


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
#define CHECK_PLC_RETURN_IF_ERROR(index) {if(index>=ECMC_MAX_PLCS || index<0){LOGERR("ERROR: PLC index out of range.\n");return ERROR_MAIN_PLC_INDEX_OUT_OF_RANGE;}if(plcs[index]==NULL){LOGERR("ERROR: PLC object NULL.\n");return ERROR_MAIN_PLC_OBJECT_NULL;}}
/****************************************************************************/
static ecmcAxisBase     *axes[ECMC_MAX_AXES];
static int              axisDiagIndex;
static int              axisDiagFreq;
static int              controllerError=-1;
static const char*      controllerErrorMsg="NO_ERROR";
static app_mode_type    appModeCmd,appModeCmdOld,appModeStat;
static unsigned int     counter = 0;
static ecmcEc           ec;
static ecmcEvent        *events[ECMC_MAX_EVENT_OBJECTS];
static ecmcDataRecorder *dataRecorders[ECMC_MAX_DATA_RECORDERS_OBJECTS];
static ecmcDataStorage  *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
static ecmcCommandList  *commandLists[ECMC_MAX_COMMANDS_LISTS];
static struct timespec  masterActivationTimeMonotonic={};
static struct timespec  masterActivationTimeOffset={};
static struct timespec  masterActivationTimeRealtime={};
static ecmcAsynPortDriver *asynPort=NULL;
static ecmcPLC          *plcs[ECMC_MAX_PLCS];

//Default asyn params
static int asynParIdLatencyMin=0;
static int asynParIdLatencyMax=0;
static int asynParIdExecuteMin=0;
static int asynParIdExecuteMax=0;
static int asynParIdPeriodMin=0;
static int asynParIdPeriodMax=0;
static int asynParIdSendMin=0;
static int asynParIdSendMax=0;
static int asynParIdEcmcAppMode=0;
static int asynParIdEcmcErrorId=0;
static int asynParIdEcmcErrorMsg=0;
static int asynParIdEcmcErrorReset=0;
static int asynSkipCyclesThread=0;
static int asynUpdateCounterThread=0;
static int asynThreadParamsEnable=0;

static int asynSkipCyclesFastest=-1;
static int asynSkipUpdateCounterFastest=0;

/****************************************************************************/

const struct timespec cycletime = {0, MCU_PERIOD_NS};

/*****************************************************************************/

void printStatus()
{
  //Print axis diagnostics to screen
  if(PRINT_STDOUT_BIT12() && axisDiagIndex<ECMC_MAX_AXES && axisDiagIndex>=0){
    if(axes[axisDiagIndex]!=NULL){
      axes[axisDiagIndex]->printAxisStatus();
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

/*Available strings:
 *  ec<masterId>.s<slaveId>.<alias>  (defaults complete ecentry)
 *  ec<masterId>.s<slaveId>.<alias>.<bit> (only one bit)
*/

static int parseEcPath(char* ecPath, int *master,int *slave, char*alias,int *bit)
{
  int masterId=0;
  int slaveId=0;
  int bitId=0;
  int nvals=0;

  nvals = sscanf(ecPath, "ec%d.s%d.%[^.].%d",&masterId,&slaveId,alias,&bitId);
  if (nvals == 4){
	*master=masterId;
	*slave=slaveId;
	*bit=bitId;
    return 0;
  }
  nvals = sscanf(ecPath, "ec%d.s%d.%s",&masterId,&slaveId,alias);
  if (nvals == 3){
	*master=masterId;
	*slave=slaveId;
	*bit=-1;
    return 0;
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

static int parseObjectPath(char* objPath,int *axis, motionObjectType *objectType,int *objectFunction)
{
  int axisId=0;
  char objectTypeStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int nvals=0;
  *objectType=ECMC_OBJ_INVALID;

  //Axis sub objects
  nvals = sscanf(objPath, ECMC_AX_STR"%d.%[^.].%s",&axisId,objectTypeStr,objectFunctionStr);
  if (nvals == 3){
	*axis=axisId;
	//Drive
	nvals=strcmp(objectTypeStr,ECMC_DRV_STR);
	if( nvals==0 ){
	  *objectType=ECMC_OBJ_DRIVE;
	  //Enable
	  nvals=strcmp(objectFunctionStr,ECMC_DRV_ENABLE_STR);
	  if(nvals==0){
		*objectFunction=ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD;
        return 0;
	  }
	  //Velocity
	  nvals=strcmp(objectFunctionStr,ECMC_DRV_VELOCITY_STR);
	  if(nvals==0){
		*objectFunction=ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT;
        return 0;
	  }
	  //Enabled
	  nvals=strcmp(objectFunctionStr,ECMC_DRV_ENABLED_STR);
      if(nvals==0){
		*objectFunction=ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD;
        return 0;
	  }
	  //Break
	  nvals=strcmp(objectFunctionStr,ECMC_DRV_BREAK_STR);
	  if(nvals==0){
		*objectFunction=ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT;
        return 0;
	  }
	  //Reduce Torque
	  nvals=strcmp(objectFunctionStr,ECMC_DRV_REDUCETORQUE_STR);
	  if(nvals==0){
		*objectFunction=ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT;
        return 0;
	  }
	  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
	}

	//Encoder
	nvals=strcmp(objectTypeStr,ECMC_ENC_STR);
	if( nvals==0 ){
	  *objectType=ECMC_OBJ_ENCODER;
	  //Enable
      nvals=strcmp(objectFunctionStr,ECMC_ENC_ACTPOS_STR);
	  if(nvals==0){
		*objectFunction=ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION;
        return 0;
	  }
	  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
	}

	//Monitor
	nvals=strcmp(objectTypeStr,ECMC_MON_STR);
	if( nvals==0 ){
	  *objectType=ECMC_OBJ_MONITOR;
	  //Lowlim
	  nvals=strcmp(objectFunctionStr,ECMC_MON_LOWLIM_STR);
	  if(nvals==0){
	    *objectFunction=ECMC_MON_ENTRY_INDEX_LOWLIM;
        return 0;
	  }
	  //Highlim
	  nvals=strcmp(objectFunctionStr,ECMC_MON_HIGHLIM_STR);
	  if(nvals==0){
		*objectFunction=ECMC_MON_ENTRY_INDEX_HIGHLIM;
        return 0;
	  }
	  //Home sensor
	  nvals=strcmp(objectFunctionStr,ECMC_MON_HOMESENSOR_STR);
	  if(nvals==0){
		*objectFunction=ECMC_MON_ENTRY_INDEX_HOMESENSOR;
        return 0;
	  }
	  //ExternalInterupt
	  nvals=strcmp(objectFunctionStr,ECMC_MON_EXTINTERLOCK_STR);
	  if(nvals==0){
		*objectFunction=ECMC_MON_ENTRY_INDEX_EXTINTERLOCK;
        return 0;
	  }
	  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
	}
  }

  //Axis object only
  nvals = sscanf(objPath, ECMC_AX_STR"%d.%s",&axisId,objectFunctionStr);
  if (nvals == 2){
	*objectType=ECMC_OBJ_AXIS;
	*axis=axisId;
	//Health
    nvals=strcmp(objectFunctionStr,ECMC_AX_HEALTH_STR);
	if(nvals==0){
	  *objectFunction=ECMC_AXIS_ENTRY_INDEX_HEALTH;
	  return 0;
	}
	return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
  }

  //Ec object
  int masterId=0;
  nvals = sscanf(objPath, ECMC_EC_STR"%d.%s",&masterId,objectFunctionStr);
  if (nvals == 2){
	*objectType=ECMC_OBJ_EC;
	//Health
    nvals=strcmp(objectFunctionStr,ECMC_EC_HEALTH_STR);
	if(nvals==0){
	  *objectFunction=ECMC_EC_ENTRY_INDEX_HEALTH;
	  return 0;
	}

	return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
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
  while(appModeCmd==ECMC_MODE_RUNTIME)
  {
    wakeupTime = timespec_add(wakeupTime, cycletime);

    /* Only lock asyn port when ec is started
     * otherwise deadlock in stratup phase
     * (sleep in waitforstartup() this is called
     * in asyn thread) .
     * */
    if(asynPort && appModeStat==ECMC_MODE_RUNTIME){
      asynPort->unlock();
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);
    if(asynPort && appModeStat==ECMC_MODE_RUNTIME){
      asynPort->lock();
    }
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

    //PLCs
    for( i=0;i<ECMC_MAX_PLCS;i++){
      if(plcs[i]!=NULL){
	plcs[i]->refresh();
      }
    }


    //Update Asyn thread diagnostics parameters
    if(asynUpdateCounterThread){
      asynUpdateCounterThread--;
    }
    else{

      asynUpdateCounterThread=asynSkipCyclesThread;

      if(asynPort && asynSkipCyclesThread>=0 && asynThreadParamsEnable){
	if(asynPort->getAllowRtThreadCom()){
          asynPort-> setIntegerParam(asynParIdLatencyMin,latency_min_ns);
          asynPort-> setIntegerParam(asynParIdLatencyMax,latency_max_ns);
          asynPort-> setIntegerParam(asynParIdExecuteMin,exec_min_ns);
          asynPort-> setIntegerParam(asynParIdExecuteMax,exec_max_ns);
          asynPort-> setIntegerParam(asynParIdPeriodMin,period_min_ns);
          asynPort-> setIntegerParam(asynParIdPeriodMax,period_max_ns);
          asynPort-> setIntegerParam(asynParIdSendMin,send_min_ns);
          asynPort-> setIntegerParam(asynParIdSendMax,send_max_ns);
          controllerError=getControllerError();
          asynPort->setIntegerParam(asynParIdEcmcErrorId,controllerError);
          controllerErrorMsg=getErrorString(controllerError);
          asynPort->doCallbacksInt8Array((epicsInt8*)controllerErrorMsg,(int)strlen(controllerErrorMsg)+1, asynParIdEcmcErrorMsg,0);
	}
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

    if (counter) {
      counter--;
    }
    else{ //Lower freq
      if(axisDiagFreq>0){
        counter = MCU_FREQUENCY/axisDiagFreq;
        ec.checkState();
        ec.checkSlavesConfState();
        printStatus();
        for(int i=0;i<ECMC_MAX_AXES;i++){
          if(axes[i]!=NULL){
            axes[i]->slowExecute();
          }
        }
        ec.slowExecute();
      }
    }

    //Asyn callbacks for all parameters (except arrays)
    if(asynSkipUpdateCounterFastest){
      asynSkipUpdateCounterFastest--;
    }
    else{
      asynSkipUpdateCounterFastest=asynSkipCyclesFastest;
      if(asynPort){
        if(asynPort->getAllowRtThreadCom()){
          asynPort-> callParamCallbacks();
        }
      }
    }

    clock_gettime(CLOCK_MONOTONIC, &sendTime);
    ec.send(masterActivationTimeOffset);
    clock_gettime(CLOCK_MONOTONIC, &endTime);
  }
  appModeStat=ECMC_MODE_CONFIG;
}

/****************************************************************************/

int  hw_motor_global_init(void){
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  LOGINFO("\nESS Open Source EtherCAT Motion Control Unit Initializes......\n");
  LOGINFO("\nMode: Configuration\n");

  appModeStat=ECMC_MODE_CONFIG;
  appModeCmd=ECMC_MODE_CONFIG;
  appModeCmdOld=appModeCmd;

  axisDiagIndex=0;
  axisDiagFreq=10;
  setDiagAxisEnable(1);

  for(int i=0; i<ECMC_MAX_AXES;i++){
    axes[i]=NULL;
  }

  for(int i=0; i<ECMC_MAX_PLCS;i++){
    plcs[i]=NULL;
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
    if(/*!ec.getError() && */ec.statusOK()){
      clock_nanosleep(CLOCK_MONOTONIC, 0, &timeToPause, NULL);
      LOGINFO("EtherCAT bus started!\n");
      return 0;
    }
  }
  LOGERR("Timeout error: EtherCAT bus did not start correctly in %ds.\n",timeoutSeconds);
  setAppMode(0);
  return ec.getErrorID();
}

int setAppModeCfg(int mode)
{
  LOGINFO4("INFO:\t\tApplication in configuration mode.\n");
  appModeCmdOld = appModeCmd;
  appModeCmd=(app_mode_type)mode;

  if(asynPort){
    asynPort->setAllowRtThreadCom(false);
  }

  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      axes[i]->setRealTimeStarted(false);
    }
  }
  return 0;
}

int setAppModeRun(int mode)
{

  if(appModeStat==ECMC_MODE_RUNTIME){
    return ERROR_MAIN_APP_MODE_ALREADY_RUNTIME;
  }

  asynPort->setAllowRtThreadCom(false);  //Block rt communication during startup (since sleep in waitForEtherCATtoStart())

  appModeCmdOld=appModeCmd;
  appModeCmd=(app_mode_type)mode;

  appModeStat=ECMC_MODE_STARTUP;

  if(asynPort && asynThreadParamsEnable){
    asynPort-> setIntegerParam(asynParIdEcmcAppMode,mode);
    asynPort-> callParamCallbacks();
  }

  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes[i]!=NULL){
      axes[i]->setInStartupPhase(true);
    }
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
  appModeStat=ECMC_MODE_RUNTIME;

  if(asynPort){
    asynPort->setAllowRtThreadCom(true);
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
  /*if((errorCode=ec.writeAndVerifySDOs())){
    LOGERR("ERROR:\tSDO write and verify failed\n");
    return errorCode;
  }*/
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

int ecPrintAllHardware()
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  return ec.printAllConfig();
}

int ecPrintSlaveConfig(int slaveIndex)
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  return ec.printSlaveConfig(slaveIndex);
}

int ecAutoConfigSlave(int slaveIndex,int addAsynParams)
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  return ec.autoConfigSlave(slaveIndex,addAsynParams);
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
  *value=0;
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

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getErrorID();
}

const char *getErrorString(int error_number)
{
  LOGINFO4("%s/%s:%d error_no=%d\n",__FILE__, __FUNCTION__, __LINE__, error_number);

  return ecmcError::convertErrorIdToString(error_number);
}

int getAxisCycleCounter(int axisIndex,int *counter)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  *counter=axes[axisIndex]->getCycleCounter();
  return 0;
}

int getAxisDebugInfoData(int axisIndex,char *buffer, int bufferByteSize)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  int bytesUsed=0;
  return axes[axisIndex]->getAxisDebugInfoData(buffer,bufferByteSize,&bytesUsed);
}

int getAxisStatusStructV2(int axisIndex,char *buffer, int bufferByteSize)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  ecmcAxisStatusType data;
  int error=axes[axisIndex]->getDebugInfoData(&data);
  if(error){
    return error;
  }

  // (Ax,PosSet,PosAct,PosErr,PosTarg,DistLeft,CntrOut,VelFFSet,VelAct,VelFFRaw,VelRaw,CycleCounter,Error,Co,CD,St,IL,TS,ES,En,Ena,Ex,Bu,Ta,L-,L+,Ho");
  int ret=snprintf(buffer,bufferByteSize,
       "Main.M%d.stAxisStatusV2=%g,%g,%" PRId64 ",%g,%g,%g,%g,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
       axisIndex,
       data.onChangeData.positionTarget,
       data.onChangeData.positionActual,
       data.onChangeData.positionRaw,  //UINT64_t
       data.onChangeData.velocitySetpoint,
       data.onChangeData.velocityActual,
       data.acceleration,
       data.deceleration,
       data.cycleCounter,
       0,  //EtherCAT time low32 not available yet
       0,  //EtherCAT time high32 not available yet
       data.onChangeData.enable,
       data.onChangeData.enabled,
       data.onChangeData.execute,
       data.onChangeData.command,
       data.onChangeData.cmdData,
       data.onChangeData.limitBwd,
       data.onChangeData.limitFwd,
       data.onChangeData.homeSwitch,
       data.onChangeData.error>0,
       data.onChangeData.error,
       data.reset,
       data.onChangeData.homed,
       data.onChangeData.busy,
       data.onChangeData.atTarget,
       data.moving,
       data.stall
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
  *value=0;
  *value= axes[axisIndex]->getEnable()>0;
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
  *value=0;
  *value= axes[axisIndex]->getMon()->getEnableAlarmAtHardLimit()>0;
  return 0;
}

int getAxisEnabled(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=0;
  *value=axes[axisIndex]->getEnabled()>0;
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
  *value=0;
  *value=(int)axes[axisIndex]->getCascadedCommandsEnabled()>0;
  return 0;
}

int getAxisEnableCommandsTransform(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=0;
  *value=(int)axes[axisIndex]->getEnableCommandsTransform()>0;
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
  *value=0;
  *value=axes[axisIndex]->getMon()->getEnableSoftLimitBwd()>0;
  return 0;
}

int getAxisEnableSoftLimitFwd(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)
  *value=0;
  *value=axes[axisIndex]->getMon()->getEnableSoftLimitFwd()>0;
  return 0;
}

int getAxisBusy(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=0;
  *value=axes[axisIndex]->getBusy()>0;
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

  axes[axisIndex]->getSeq()->setTargetVel(value);

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

int setPLCExpr(int index,char *expr)
{
  LOGINFO4("%s/%s:%d index=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__,index, expr);

  CHECK_PLC_RETURN_IF_ERROR(index)

  return plcs[index]->setExpression(expr);
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
  *value=0;
  *value=!axes[axisIndex]->getSeq()->getBusy()>0;
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
  *value=0;
  *value=axes[axisIndex]->getExecute()>0;
  return 0;
}

int getAxisReset(int axisIndex, int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=0;
  *value=axes[axisIndex]->getReset()>0;
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
  *value=0;
  *value=axes[axisIndex]->getMon()->getHardLimitFwd()>0;
  return 0;
}

int getAxisAtHardBwd(int axisIndex,int *value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)
  *value=0;
  *value=axes[axisIndex]->getMon()->getHardLimitBwd()>0;
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
  *value=0;
  *value=tempHomed>0;
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
  *value=0;
  *value=axes[axisIndex]->getMon()->getHomeSwitch()>0;
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

  return axes[axisIndex]->getEnc()->setOffset(value);
}

int setAxisEncBits(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getEnc()->setBits(value);
}

int setAxisEncAbsBits(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getEnc()->setAbsBits(value);
}

int setAxisEncRawMask(int axisIndex, uint64_t rawMask)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, (uint)rawMask);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getEnc()->setRawMask(rawMask);
}

int setAxisEncType(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getEnc()->setType((encoderType)value);
}

/****************************************************************************/
//Drv GET
int getAxisDrvScaleNum(int axisIndex, double *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getDrv()->getScaleNum();
  return 0;
}

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

int setAxisDrvBrakeOpenDelayTime(int axisIndex, int delayTime)
{
  LOGINFO4("%s/%s:%d axisIndex=%d delayTime=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, delayTime);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setBrakeOpenDelayTime(delayTime);
}

int setAxisDrvBrakeCloseAheadTime(int axisIndex, int aheadTime)
{
  LOGINFO4("%s/%s:%d axisIndex=%d aheadTime=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, aheadTime);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setBrakeCloseAheadTime(aheadTime);
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
  *value=0;
  *value=axes[axisIndex]->getDrv()->getEnable()>0;
  return 0;
}

/****************************************************************************/
//Mon GET
int getAxisMonAtTargetTol(int axisIndex, double *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getAtTargetTol();
  return 0;
}

//Mon SET
int setAxisMonAtTargetTol(int axisIndex, double value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setAtTargetTol(value);
  return 0;
}

int getAxisMonAtTargetTime(int axisIndex, int *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getAtTargetTime();
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

int getAxisMonEnableAtTargetMon(int axisIndex, int *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getEnableAtTargetMon();
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

int setAxisMonExtHWInterlockPolarity(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setHardwareInterlockPolarity((externalHWInterlockPolarity)value);
  return 0;
}


int getAxisMonPosLagTol(int axisIndex, double *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getPosLagTol();
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

int getAxisMonPosLagTime(int axisIndex, int *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getPosLagTime();
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

int getAxisMonEnableLagMon(int axisIndex, int *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getEnableLagMon();
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

int getAxisMonMaxVel(int axisIndex, double *value)
{

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getMaxVel();
  return 0;
}

int setAxisMonEnableMaxVel(int axisIndex, int value)
{
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableMaxVelMon(value);
}

int getAxisMonEnableMaxVel(int axisIndex, int *value)
{
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getEnableMaxVelMon();
  return 0;
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
  *value=0;
  *value=axes[axisIndex]->getMon()->getAtTarget()>0;
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
      if(!axes[index]){
	LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR NORMAL AXIS OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
        exit(EXIT_FAILURE);
      }
      break;

    case ECMC_AXIS_TYPE_VIRTUAL:
      if(axes[index]!=NULL){
        delete axes[index];
      }
      axes[index]=new ecmcAxisVirt(index,1/MCU_FREQUENCY);
      if(!axes[index]){
	LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR VITRUAL AXIS OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
        exit(EXIT_FAILURE);
       }
      break;
    default:
      return ERROR_MAIN_AXIS_TYPE_UNKNOWN;
  }

  axisDiagIndex=index; //Always printout last axis added
  return 0;
}

int createPLC(int index)
{
  LOGINFO4("%s/%s:%d index=%d\n",__FILE__, __FUNCTION__, __LINE__,index);

  if(index<0 && index>=ECMC_MAX_PLCS){
    return ERROR_MAIN_PLC_INDEX_OUT_OF_RANGE;
  }

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  if(plcs[index]){
    delete plcs[index];
    plcs[index]=NULL;
  }

  plcs[index]=new ecmcPLC(&ec);

  //Set axes pointers (for the already configuered axes)
  for(int i=0; i<ECMC_MAX_AXES;i++){
    plcs[index]->setAxisArrayPointer(axes[i],i);
  }

  return plcs[index]->getErrorID();
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
    char *entryIDString,
    int signedValue
    )
{
  std::string id=entryIDString;

  LOGINFO4("%s/%s:%d slave=%d vendor=%d productcode=%d direction=%d sm=%d pdoindex=%d entry_index=%d entry_subindex=%d bits=%d id=%s\n",__FILE__, __FUNCTION__, __LINE__, position,vendorId,productCode,direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,entryIDString);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return ec.addEntry(position,vendorId,productCode,(ec_direction_t)direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,id,signedValue);
}

int ecSetEntryUpdateInRealtime(
    uint16_t slavePosition,
    char *entryIDString,
    int updateInRealtime
    )
{
  LOGINFO4("%s/%s:%d slave=%d id=%s updateInRealtime=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,entryIDString,updateInRealtime);
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

  return entry->setUpdateInRealtime(updateInRealtime);
}

int ecAddMemMap(
    uint16_t startEntryBusPosition,
    char *startEntryIDString,
    size_t byteSize,
    int direction,
    char *memMapIDString
    )
{
  std::string memMapId=memMapIDString;
  std::string startEntryId=startEntryIDString;

  LOGINFO4("%s/%s:%d startEntryBusPosition=%d, startEntryID=%s byteSize=%lu, direction=%d entryId=%s\n",__FILE__, __FUNCTION__, __LINE__,startEntryBusPosition,startEntryIDString,byteSize,direction,memMapIDString);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec.addMemMap(startEntryBusPosition,startEntryId,byteSize,0,(ec_direction_t)direction,memMapId);
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

int ecWriteSdoComplete(uint16_t slavePosition,uint16_t sdoIndex,uint32_t value,int byteSize)
{
  LOGINFO4("%s/%s:%d slave_position=%d sdo_index=%d value=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,value,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec.writeSDOComplete(slavePosition,sdoIndex,value,byteSize);
}

uint32_t ecReadSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,int byteSize, uint32_t *value)
{
  LOGINFO4("%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,sdoSubIndex,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return ec.readSDO(slavePosition,sdoIndex,sdoSubIndex,byteSize,value);
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

  if(encoderEntryIndex>=MaxEcEntryLinks || encoderEntryIndex<0)
    return ERROR_MAIN_ENCODER_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getEnc()->setEntryAtIndex(entry,encoderEntryIndex,bitIndex);
}


int linkEcEntryToAxisDrv(int slaveIndex,char *entryIDString,int axisIndex,int driveEntryIndex, int bitIndex)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s drive=%d drive_entry=%d bit_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex,driveEntryIndex,bitIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  //Disable brake output with empty string
  if(strlen(entryIDString)==0 && driveEntryIndex==ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT){
    return axes[axisIndex]->getDrv()->setEnableBrake(0);
  }

  //Disable reduce torque output with empty string
  if(strlen(entryIDString)==0 && driveEntryIndex==ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT){
    return axes[axisIndex]->getDrv()->setEnableReduceTorque(0);
  }

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

  if(driveEntryIndex>=MaxEcEntryLinks || driveEntryIndex<0){
    return ERROR_MAIN_DRIVE_ENTRY_INDEX_OUT_OF_RANGE;
  }

  int ret=axes[axisIndex]->getDrv()->setEntryAtIndex(entry,driveEntryIndex,bitIndex);
  if(ret){
    return ret;
  }

  //Auto enable break
  if(driveEntryIndex==ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT){
    ret=axes[axisIndex]->getDrv()->setEnableBrake(1);
    if(ret){
      return ret;
    }
  }

  //Auto enable reduce torque
  if(driveEntryIndex==ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT){
    ret=axes[axisIndex]->getDrv()->setEnableReduceTorque(1);
    if(ret){
      return ret;
    }
  }

  return 0;
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

  if(entry==NULL){
    return ERROR_MAIN_EC_ENTRY_NULL;
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  if(monitorEntryIndex>=MaxEcEntryLinks || monitorEntryIndex<0)
    return ERROR_MAIN_MONITOR_ENTRY_INDEX_OUT_OF_RANGE;

  int errorCode=axes[axisIndex]->getMon()->setEntryAtIndex(entry,monitorEntryIndex,bitIndex);
  if(errorCode){
	return errorCode;
  }
  if(monitorEntryIndex==ECMC_MON_ENTRY_INDEX_EXTINTERLOCK){
    return setAxisMonEnableExternalInterlock(axisIndex,1);
  }

  return 0;
}

int linkEcEntryToObject(char *ecPath,char *axPath)
{
  LOGINFO4("%s/%s:%d ecPath=%s axPath=%s\n",__FILE__, __FUNCTION__, __LINE__, ecPath,axPath);

  int masterId=-1;
  int slaveIndex=-1;
  char alias[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int bitIndex=-1;

  int errorCode=parseEcPath(ecPath, &masterId,&slaveIndex,alias,&bitIndex);
  if(errorCode){
    return errorCode;
  }

  int axisIndex=0;
  motionObjectType objectType=ECMC_OBJ_INVALID;
  int entryIndex=0;
  errorCode=parseObjectPath(axPath,&axisIndex,&objectType,&entryIndex);
  if(errorCode){
    return errorCode;
  }

  switch(objectType){
    case ECMC_OBJ_INVALID:
      return ERROR_MAIN_ECMC_LINK_INVALID;
      break;
    case ECMC_OBJ_DRIVE:
      return linkEcEntryToAxisDrv(slaveIndex,alias,axisIndex,entryIndex,bitIndex);
      break;
    case ECMC_OBJ_ENCODER:
      return linkEcEntryToAxisEnc(slaveIndex,alias,axisIndex,entryIndex,bitIndex);
      break;
    case ECMC_OBJ_MONITOR:
      return linkEcEntryToAxisMon(slaveIndex,alias,axisIndex,entryIndex,bitIndex);
      break;
    case ECMC_OBJ_CONTROLLER:
      return ERROR_MAIN_ECMC_LINK_INVALID;
      break;
    case ECMC_OBJ_TRAJECTORY:
      return ERROR_MAIN_ECMC_LINK_INVALID;
      break;
    case ECMC_OBJ_AXIS:
      if(entryIndex==ECMC_AXIS_ENTRY_INDEX_HEALTH){
        return linkEcEntryToAxisStatusOutput(slaveIndex,alias,axisIndex);
      }
      return ERROR_MAIN_ECMC_LINK_INVALID;
      break;
    case ECMC_OBJ_EC:
      if(entryIndex==ECMC_EC_ENTRY_INDEX_HEALTH){
        return linkEcEntryToEcStatusOutput(slaveIndex,alias);
      }
      return ERROR_MAIN_ECMC_LINK_INVALID;
      break;
  }
  return ERROR_MAIN_ECMC_LINK_INVALID;
}

int linkEcEntryToAxisStatusOutput(int slaveIndex,char *entryIDString,int axisIndex)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s, axisId=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex);

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

  return axes[axisIndex]->setEcStatusOutputEntry(entry);
}

int linkEcEntryToEcStatusOutput(int slaveIndex,char *entryIDString)
{
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString);

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

  return ec.setEcStatusOutputEntry(entry);
}

int linkEcEntryToAsynParameter(int masterIndex,int busPosition,const char *entryIDString, int asynParType,int skipCycles)
{
  LOGINFO4("%s/%s:%d masterIndex=%d busPosition=%d alias=%s type=%d,skipCycles=%d\n",__FILE__, __FUNCTION__, __LINE__,masterIndex,busPosition,entryIDString,asynParType,skipCycles);

  if(asynPort==NULL){
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  if(ec.getMasterIndex()!=masterIndex){
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  int err=ec.linkEcEntryToAsynParameter(asynPort,entryIDString,asynParType,skipCycles);
  if(err){
    return err;
  }

  if(skipCycles<asynSkipCyclesFastest || asynSkipCyclesFastest<0){
    asynSkipCyclesFastest=skipCycles;
  }

  return 0;
}

int linkEcMemMapToAsynParameter(int masterIndex,const char *memMapIDString, int asynParType,int skipCycles)
{
  LOGINFO4("%s/%s:%d masterIndex=%d alias=%s type=%d,skipCycles=%d\n",__FILE__, __FUNCTION__, __LINE__,masterIndex,memMapIDString,asynParType,skipCycles);

  if(asynPort==NULL){
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  if(ec.getMasterIndex()!=masterIndex){
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  int err=ec.linkEcMemMapToAsynParameter(asynPort,memMapIDString,asynParType,skipCycles);
  if(err){
    return err;
  }

  if(skipCycles<asynSkipCyclesFastest || asynSkipCyclesFastest<0){
    asynSkipCyclesFastest=skipCycles;
  }

   return 0;
}

int initEcmcAsyn(void* asynPortObject)
{
  LOGINFO4("%s/%s:%d\n",__FILE__, __FUNCTION__, __LINE__);
  asynPort=(ecmcAsynPortDriver*)asynPortObject;
  ec.setAsynPortDriver(asynPort);
  return 0;
}

int addDefaultAsynEc(int masterIndex,int regAsynParams,int skipCycles)
{
  LOGINFO4("%s/%s:%d masterIndex=%d regAsynParams=%d skipCycles=%d\n",__FILE__, __FUNCTION__, __LINE__,masterIndex,regAsynParams,skipCycles);

  if(asynPort==NULL){
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  if(ec.getMasterIndex()!=masterIndex){
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  int err=ec.initAsyn(asynPort,regAsynParams,skipCycles);
  if(err){
    return err;
  }

  if(skipCycles<asynSkipCyclesFastest || asynSkipCyclesFastest<0){
    asynSkipCyclesFastest=skipCycles;
  }

  return 0;
}

int addDefaultAsynEcSlave(int masterIndex,int busPosition,int regAsynParams,int skipCycles)
{
  LOGINFO4("%s/%s:%d masterIndex=%d busPosition=%d regAsynParams=%d skipCycles=%d\n",__FILE__, __FUNCTION__, __LINE__,masterIndex,busPosition,regAsynParams,skipCycles);

  if(asynPort==NULL){
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  if(ec.getMasterIndex()!=masterIndex){
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  ecmcEcSlave *tempSlave=ec.findSlave(busPosition);
  if(!tempSlave){
    return ERROR_MAIN_EC_SLAVE_NULL;
  }

  int err=tempSlave->initAsyn(asynPort,regAsynParams,skipCycles,masterIndex);
  if(err){
    return err;
  }

  if(skipCycles<asynSkipCyclesFastest || asynSkipCyclesFastest<0){
    asynSkipCyclesFastest=skipCycles;
  }

  return 0;
}

int addDefaultAsynAxis(int regAsynParams, int axisIndex,int skipCycles)
{
  LOGINFO4("%s/%s:%d regAsynParams=%d axisIndex=%d skipCycles=%d\n",__FILE__, __FUNCTION__, __LINE__,regAsynParams,axisIndex,skipCycles);

  if(asynPort==NULL){
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);


  int err=axes[axisIndex]->initAsyn(asynPort,regAsynParams,skipCycles);
  if(err){
    return err;
  }

  if(skipCycles<asynSkipCyclesFastest || asynSkipCyclesFastest<0){
    asynSkipCyclesFastest=skipCycles;
  }

  return 0;
}

int addDiagAsynAxis(int regAsynParams, int axisIndex,int skipCycles)
{
  LOGINFO4("%s/%s:%d regAsynParams=%d axisIndex=%d skipCycles=%d\n",__FILE__, __FUNCTION__, __LINE__,regAsynParams,axisIndex,skipCycles);

  if(asynPort==NULL){
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  //Array so updated in axis base object
  return axes[axisIndex]->initDiagAsyn(asynPort,regAsynParams,skipCycles);
}


int addDefaultAsynParams(int regAsynParams,int skipCycles)
{
  LOGINFO4("%s/%s:%d regAsynParams=%d skipCycles=%d\n",__FILE__, __FUNCTION__, __LINE__,regAsynParams,skipCycles);

  if(asynPort==NULL){
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  asynSkipCyclesThread=skipCycles;

  if(!regAsynParams ){
    return 0;
  }

  //Timing info (only updated in real time)!
  asynStatus status = asynPort->createParam("ecmc.thread.latency.min",asynParamInt32,&asynParIdLatencyMin);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.latency.min failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdLatencyMin,0);

  status = asynPort->createParam("ecmc.thread.latency.max",asynParamInt32,&asynParIdLatencyMax);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.latency.max failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdLatencyMax,0);

  status = asynPort->createParam("ecmc.thread.period.min",asynParamInt32,&asynParIdPeriodMin);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.period.min failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdPeriodMin,0);

  status = asynPort->createParam("ecmc.thread.period.max",asynParamInt32,&asynParIdPeriodMax);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.period.max failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdPeriodMax,0);

  status = asynPort->createParam("ecmc.thread.execute.min",asynParamInt32,&asynParIdExecuteMin);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.execute.min failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdExecuteMin,0);

  status = asynPort->createParam("ecmc.thread.execute.max",asynParamInt32,&asynParIdExecuteMax);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.execute.max failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdExecuteMax,0);

  status = asynPort->createParam("ecmc.thread.send.min",asynParamInt32,&asynParIdSendMin);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.send.min failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdSendMin,0);

  status = asynPort->createParam("ecmc.thread.send.max",asynParamInt32,&asynParIdSendMax);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter thread.send.max failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdSendMax,0);

  status = asynPort->createParam("ecmc.appmode",asynParamInt32,&asynParIdEcmcAppMode);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter appmode failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdEcmcAppMode,0);

  status = asynPort->createParam("ecmc.error.id",asynParamInt32,&asynParIdEcmcErrorId);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter ecmc.error.id failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdEcmcErrorId,0);

  status = asynPort->createParam("ecmc.error.reset",asynParamInt32,&asynParIdEcmcErrorReset);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter ecmc.error.reset failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }
  asynPort-> setIntegerParam(asynParIdEcmcErrorReset,0);

  status = asynPort->createParam("ecmc.error.msg",asynParamInt8Array,&asynParIdEcmcErrorMsg);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter ecmc.error.msg failed.\n",__FILE__,__FUNCTION__,__LINE__);
    return asynError;
  }

  asynPort->doCallbacksInt8Array((epicsInt8*)"NO_ERROR\0",9, asynParIdEcmcErrorMsg,0);

  asynPort-> callParamCallbacks();
  asynThreadParamsEnable=1;

  if(skipCycles<asynSkipCyclesFastest || asynSkipCyclesFastest<0){
    asynSkipCyclesFastest=skipCycles;
  }

  return 0;
}

int readEcMemMap(const char *memMapIDString,uint8_t *data,size_t bytesToRead, size_t *bytesRead)
{
  LOGINFO4("%s/%s:%d alias=%s bytesToRead=%lu\n",__FILE__, __FUNCTION__, __LINE__,memMapIDString,bytesToRead);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcMemMap *memMap=ec.findMemMap(memMapIDString);
  if(!memMap){
    return ERROR_MAIN_MEM_MAP_NULL;
  }

  return memMap->read(data,bytesToRead,bytesRead);
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
  return ec.getSlave(slaveIndex)->getEntry(entryIndex)->writeValueForce(value);
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

  return entry->writeValueForce(value);
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
  if(!events[indexEvent]){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR EVENT OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }
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
  if(!dataStorages[index]){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DATA STORAGE OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }

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

int setEventEnable(int indexEvent,int enable)
{
  LOGINFO4("%s/%s:%d indexEvent=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent, enable);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setEnable(enable);
}

int getEventEnabled(int indexEvent,int *enabled)
{
  LOGINFO4("%s/%s:%d indexEvent=%d\n",__FILE__, __FUNCTION__, __LINE__,indexEvent);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->getEnabled(enabled);
}

int clearStorage(int indexStorage)
{
  LOGINFO4("%s/%s:%d indexStorage=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->clearBuffer();
}

int getStorageDataIndex(int indexStorage,int *index)
{
  LOGINFO4("%s/%s:%d indexStorage=%d\n",__FILE__, __FUNCTION__, __LINE__,indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  *index=dataStorages[indexStorage]->getCurrentIndex();
  return 0;
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
  if(!dataRecorders[indexRecorder]){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DATA RECORDER OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }

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

  int error=dataRecorders[indexRecorder]->setEntryAtIndex(entry,recorderEntryIndex,bitIndex);
  if(error){
    return error;
  }

  //set source to EtherCAT
  return dataRecorders[indexRecorder]->setDataSourceType(ECMC_RECORDER_SOURCE_ETHERCAT);
}

int linkAxisDataToRecorder(int indexRecorder,int axisIndex,int dataToStore)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d axisIndex=%d dataToStore=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder, axisIndex,dataToStore);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  int error= dataRecorders[indexRecorder]->setAxisDataSource(axes[axisIndex]->getDebugInfoDataPointer(),(ecmcAxisDataType)dataToStore);
  if(error){
    return error;
  }
  //set source to Axis data
  return dataRecorders[indexRecorder]->setDataSourceType(ECMC_RECORDER_SOURCE_AXIS);
}

int setRecorderEnablePrintouts(int indexRecorder,int enable)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder,enable);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  WRITE_DIAG_BIT(FUNCTION_DATA_RECORDER_DIAGNOSTICS_BIT,enable);
  return 0;
}

int setRecorderEnable(int indexRecorder,int enable)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder, enable);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->setEnable(enable);
}

int getRecorderEnabled(int indexRecorder,int *enabled)
{
  LOGINFO4("%s/%s:%d indexRecorder=%d\n",__FILE__, __FUNCTION__, __LINE__,indexRecorder);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->getEnabled(enabled);
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

  return 0;
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
  if(!commandLists[indexCommandList]){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR COMAMND-LIST OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }

  return 0;
}

int linkCommandListToEvent(int indexCommandList,int indexEvent, int consumerIndex)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d indexEvent=%d consumerIndex=%d\n",__FILE__, __FUNCTION__, __LINE__,indexCommandList, indexEvent,consumerIndex);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);
  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->linkEventConsumer(commandLists[indexCommandList],consumerIndex);
}

int setCommandListEnable(int indexCommandList,int enable)
{
  LOGINFO4("%s/%s:%d indexCommandList=%d enable=%d\n",__FILE__, __FUNCTION__, __LINE__,indexCommandList, enable);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  return commandLists[indexCommandList]->setEnable(enable);
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
