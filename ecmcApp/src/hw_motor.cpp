
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

#include <stdlib.h>
#include <math.h>
#include "cmd.h"
#include "hw_motor.h"
#include <inttypes.h>
#include <algorithm>

#include "ecrt.h"
#include "./rtutilsSrc/rtutils.h"
#include "messages.h"
#include <time.h>

//General
#include "ecmcDefinitions.h"

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
#include "ecmcAxisTraj.h"      //Axis without drive, controller and encoder
#include "ecmcAxisEncoder.h"   //Axis without drive, controller and trajectory
#include "ecmcDrive.hpp"
#include "ecmcTrajectory.hpp"
#include "ecmcPIDController.hpp"
#include "ecmcEncoder.h"
#include "ecmcMonitor.hpp"
#include "ecmcCommandTransform.h"

/****************************************************************************/
#define CHECK_AXIS_RETURN_IF_ERROR(axisIndex) {if(axisIndex>=MAX_AXES || axisIndex<=0){printf("ERROR: Axis index out of range.\n");return ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;}if(axes[axisIndex]==NULL){printf("ERROR: Axis object NULL\n");return ERROR_MAIN_AXIS_OBJECT_NULL;}}
#define CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getEnc()==NULL){printf("ERROR: Encoder object NULL.\n");return ERROR_MAIN_ENCODER_OBJECT_NULL;}}
#define CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getCntrl()==NULL){printf("ERROR: Controller object NULL.\n");return ERROR_MAIN_CONTROLLER_OBJECT_NULL;}}
#define CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getDrv()==NULL){printf("ERROR: Drive object NULL.\n");return ERROR_MAIN_DRIVE_OBJECT_NULL;}}
#define CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getTraj()==NULL){printf("ERROR: Trajectory object NULL.\n");return ERROR_MAIN_TRAJECTORY_OBJECT_NULL;}}
#define CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getSeq()==NULL){printf("ERROR: Sequence object NULL.\n");return ERROR_MAIN_SEQUENCE_OBJECT_NULL;}}
#define CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getMon()==NULL){printf("ERROR: Monitor object NULL.\n");return ERROR_MAIN_MONITOR_OBJECT_NULL;}}
#define CHECK_AXIS_TRAJ_TRANSFORM_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getTraj()->getExtInputTransform()==NULL){printf("ERROR: Trajectory transform object NULL.\n");return ERROR_MAIN_TRAJ_TRANSFORM_OBJECT_NULL;}}
#define CHECK_AXIS_ENC_TRANSFORM_RETURN_IF_ERROR(axisIndex) {if(axes[axisIndex]->getEnc()->getExtInputTransform()==NULL){printf("ERROR: Encoder transform object NULL.\n");return ERROR_MAIN_ENC_TRANSFORM_OBJECT_NULL;}}

/****************************************************************************/
static ecmcAxisBase     *axes[MAX_AXES];
static ecmcEc           ec;
static int              axisDiagIndex;
static int              axisDiagFreq;
static bool             enableAxisDiag;
static app_mode_type    appMode,appModeOld;
static rtMessageQueueId rtq=NULL;
static bool             masterStateOK=0;
static bool             functionDiag=0;
static int              enableTimeDiag=0;
static unsigned int     counter = 0;
static int              printCounter=0;

/****************************************************************************/

const struct timespec cycletime = {0, MCU_PERIOD_NS};

/*****************************************************************************/

void printStatus()
{
  //Print axis diagnostics to screen
  if(enableAxisDiag && axisDiagIndex<MAX_AXES && axisDiagIndex>=0){
    if(axes[axisDiagIndex]!=NULL){
      if(printCounter==0){
        printf("\nAxis\tPos set\t\tPos act\t\tPos err\t\tCntrl out\tDist left\tVel act\t\tVel FF\t\tVel FFs\t\tDrv Vel\tError\tEn Ex Bu St Ta IL L+ L- Ho\n");
        printCounter=25;
      }
      axes[axisDiagIndex]->printStatus();
      printCounter--;
    }
  }
}

void cyclic_task(void * usr){
  int i=0;
  //EC_MESSAGE * msg = (EC_MESSAGE *)calloc(1, MAX_MESSAGE);
  struct timespec wakeupTime , time;
  struct timespec startTime, endTime, lastStartTime = {};
  uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
           latency_min_ns = 0, latency_max_ns = 0,
           period_min_ns = 0, period_max_ns = 0,
           exec_min_ns = 0, exec_max_ns = 0;

  // get current time
  clock_gettime(MCU_CLOCK_TO_USE, &wakeupTime);

  while(appMode==ECMC_MODE_RUNTIME)
  {
    if(rtq!=NULL) //todo remove traces from diamond since not used..
    {
      //rtMessageQueueReceive(rtq, msg,MAX_MESSAGE); //diff from diamond
      //if(msg->tag == MSG_TICK)
      //{
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(MCU_CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);
        //*******************
        clock_gettime(MCU_CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

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
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
//*******************
        ec.receive();
        ec.checkDomainState();

        for( i=0;i<MAX_AXES;i++){
          if(axes[i]!=NULL){
            axes[i]->execute(masterStateOK);
          }
        }

        if (counter) {
          counter--;
        }
        else{ //Lower freq
          counter = MCU_FREQUENCY/axisDiagFreq;
          masterStateOK=ec.checkState();
          ec.checkSlavesConfState();

          printStatus();
          if(enableTimeDiag){
            printf("period     %10u ... %10u\n",
                    period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                    exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n",
                    latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
          }
        }
        // write application time to master
        clock_gettime(MCU_CLOCK_TO_USE, &time);
        ecrt_master_application_time(ec.getMaster(), TIMESPEC2NS(time));

        ec.send();
        clock_gettime(MCU_CLOCK_TO_USE, &endTime);
      //}
    }
  }
  //TODO add nice close down here if needed
}

/****************************************************************************/

int  hw_motor_global_init(void){

  appMode=ECMC_MODE_CONFIG;
  appModeOld=appMode;
  printf("\n\nESS Open Source EtherCAT MCU\n\n");
  if(appMode==ECMC_MODE_CONFIG){
    printf("Mode: Configuration\n\n\n");
  }

  axisDiagIndex=0;
  axisDiagFreq=10;
  enableAxisDiag=true;

  for(int i=1; i<MAX_AXES;i++){
    axes[i]=NULL;
  }

  return 0;
}

/****************************************************************************/

void startRTthread(){
#if PRIORITY  //TODO: Remove never executed
  pid_t pid = getpid();
  if (setpriority(PRIO_PROCESS, pid, 10 /*-19*/))   //Range -20..20 (-20 highest prio)
    fprintf(stderr, "WARNING: Failed to set priority: %s\n",
            strerror(errno));
#endif

  int prio = ECMC_PRIO_HIGH;
  rtq = rtMessageQueueCreate(10, 10000);

  if(rtThreadCreate("cyclic", prio, 0, cyclic_task, NULL) == NULL){
    printf("WARNING: Can't create high priority thread, fallback to low priority\n");
    prio = ECMC_PRIO_LOW;
    assert(rtThreadCreate("cyclic", prio, 0, cyclic_task, NULL) != NULL);
  }
  else{
    printf("INFO:\t\tCreated high priority thread for cyclic task\n");
  }
  new_timer(1000000000/MCU_FREQUENCY, rtq, prio, MSG_TICK);
}

int setAppMode(int mode){
  int errorCode=0;
  switch((app_mode_type)mode){
    case ECMC_MODE_CONFIG:
      printf("INFO:\t\tApplication in configuration mode.\n");
      appModeOld = appMode;
      appMode=(app_mode_type)mode;
      break;
    case ECMC_MODE_RUNTIME:
      appModeOld=appMode;
      appMode=(app_mode_type)mode;

      if(appModeOld!=ECMC_MODE_CONFIG){
        return ERROR_MAIN_APP_MODE_ALREADY_RUNTIME;
      }
      errorCode=validateConfig();
      if (errorCode){
        return errorCode;
      }
      printf("INFO:\t\tApplication in runtime mode.\n");
      if(ec.activate()){
        printf("INFO:\t\tActivation of master failed.\n");
        return ERROR_MAIN_EC_ACTIVATE_FAILED;
      }
      startRTthread();
      break;
    default:
      printf("WARNING: Mode %d not implemented. (Config=%d, Runtime=%d).\n",mode, ECMC_MODE_CONFIG,ECMC_MODE_RUNTIME);
      return ERROR_MAIN_APP_MODE_NOT_SUPPORTED;
      break;
  }
  appModeOld=appMode;
  return 0;
}

int prepareForRuntime(){

  //Update input sources for all trajectories and encoders (transforms)
  for(int i=0;i<MAX_AXES;i++){
    for(int j=0;j<MAX_AXES;j++){
      if(axes[i] !=NULL && axes[j] !=NULL){
        //Trajectory
        if(axes[i]->getTraj()!=NULL){
          if(axes[j]->getTraj()!=NULL){
            axes[j]->getTraj()->addInputDataInterface(axes[i]->getTraj()->getOutputDataInterface(),i);
          }
          if(axes[j]->getEnc()!=NULL){
            axes[j]->getEnc()->addInputDataInterface(axes[i]->getTraj()->getOutputDataInterface(),i);
          }
        }
        //Encoder
        if(axes[i]->getEnc()!=NULL){
          if(axes[j]->getTraj()!=NULL){
            axes[j]->getTraj()->addInputDataInterface(axes[i]->getEnc()->getOutputDataInterface(),i+MAX_AXES);
          }
          if(axes[j]->getEnc()!=NULL){
            axes[j]->getEnc()->addInputDataInterface(axes[i]->getEnc()->getOutputDataInterface(),i+MAX_AXES);
          }
        }
        axes[i]->setAxisArrayPointer(axes[j],j);
      }
    }
  }

  for(int i=0;i<MAX_AXES;i++){


  }
  return 0;
}

int validateConfig(){
  prepareForRuntime();
  int errorCode=0;
  int axisCount=0;
  for(int i=0;i<MAX_AXES;i++){
    if(axes[i]!=NULL){
      axisCount++;
      errorCode=axes[i]->validate();
      if(errorCode){
        printf("ERROR: Validation failed on axis %d with error code %d.",i,errorCode);
        return errorCode;
      }
    }
  }
  if(axisCount==0){
    return ERROR_AXIS_CONFIGURED_COUNT_ZERO;
  }
  return 0;
}

int ecApplyConfig(int masterIndex)
{  //Master index not used right now..
  int errorCode=0;
  if((errorCode=ec.writeAndVerifySDOs())){
    printf("ERROR:\tSDO write and verify failed\n");
    return errorCode;
  }
  if((errorCode=ec.compileRegInfo())){
    printf("ERROR:\tCompileRegInfo failed\n");
    return errorCode;
  }
  return 0;
}

int ecSetDiagnostics(int value)
{  //Set diagnostics mode
  return ec.setDiagnostics(value);
}

int setEnableFunctionCallDiag(int value)
{  //Set diagnostics mode
  functionDiag=value;
  return 0;
}

/****************************************************************************/

#if 0
static void init_axis(int axisIndex){
// needed?
}

/*****************************************************************************/

void hw_motor_init(int axisIndex){
// needed?
}
#endif

/*****************************************************************************/

int setAxisExecute(int axisIndex, int value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(value &&axes[axisIndex]->getError()){ //Axis needs to be reset before new command is executed (however allow execute=0)
    return ERROR_MAIN_AXIS_ERROR_EXECUTE_INTERLOCKED;
  }

  return axes[axisIndex]->setExecute(value);
}

int setAxisCommand(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setCommand((motionCommandTypes)value);
  return 0;
}

int setAxisCmdData(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setCmdData(value);
  return 0;
}

int setAxisSeqTimeout(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setSequenceTimeout(value*MCU_FREQUENCY);
  return 0;
}


int getAxisCommand(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value= (int)axes[axisIndex]->getSeq()->getCommand();
  return 0;
}

int getAxisCmdData(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getCmdData();
  return 0;
}

int getAxisError(int axisIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getError();;
}

int getAxisErrorID(int axisIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getErrorID();
}

const char *getErrorString(int error_number)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d error_no=%d\n",__FILE__, __FUNCTION__, __LINE__, error_number);

  return ecmcError::convertErrorIdToString(error_number);
}


int setAxisEnable(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(!value){
    axes[axisIndex]->setExecute(value);
  }

  return axes[axisIndex]->setEnable(value);
}

int getAxisEnable(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value= axes[axisIndex]->getEnable();
  return 0;
}

int setAxisEnableAlarmAtHardLimits(int axisIndex,int enable)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, enable);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setEnableAlarmAtHardLimit(enable);
}

int getAxisEnableAlarmAtHardLimits(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value= axes[axisIndex]->getSeq()->getEnableAlarmAtHardLimit();
  return 0;
}

int getAxisEnabled(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getEnable();  //TODO check I/O card also in drive (should be implemented in _drv->getEnabled function)
  return 0;
}

int getAxisTrajSource(int axisIndex, int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  *value=(int)axes[axisIndex]->getTraj()->getDataSourceType();
  return 0;

}

int getAxisEncSource(int axisIndex, int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  *value=(int)axes[axisIndex]->getEnc()->getDataSourceType();
  return 0;

}

int getAxisEnableCommandsFromOtherAxis(int axisIndex, int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=(int)axes[axisIndex]->getCascadedCommandsEnabled();
  return 0;

}

int getAxisEnableCommandsTransform(int axisIndex, int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=(int)axes[axisIndex]->getEnableCommandsTransform();
  return 0;
}

int getAxisType(int axisIndex, int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getAxisType();
  return 0;

}

int setAxisOpMode(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setOpMode((operationMode)value);
}

int getAxisOpMode(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  *value= axes[axisIndex]->getOpMode();
  return 0;
}

int setAxisEnableSoftLimitBwd(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setEnableSoftLimitBwd(value);
  return 0;
}

int setAxisEnableSoftLimitFwd(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setEnableSoftLimitFwd(value);
  return 0;
}

int setAxisSoftLimitPosBwd(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setSoftLimitBwd(value);
  return 0;
}

int setAxisSoftLimitPosFwd(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setSoftLimitFwd(value);
  return 0;
}

int getAxisSoftLimitPosBwd(int axisIndex, double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getSoftLimitBwd();
  return 0;
}

int getAxisSoftLimitPosFwd(int axisIndex, double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getSoftLimitFwd();
  return 0;
}

int getAxisEnableSoftLimitBwd(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getEnableSoftLimitBwd();
  return 0;
}

int getAxisEnableSoftLimitFwd(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getEnableSoftLimitFwd();
  return 0;
}

int getAxisBusy(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getBusy();
  return 0;
}

int setAxisAcceleration(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setAcc(value);
  return 0;
}

int setAxisDeceleration(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setDec(value);
  return 0;
}

int setAxisEmergDeceleration(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setEmergDec(value);
  return 0;
}

int setAxisJerk(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setJerk(value);  //TODO not implemented in trajectory generator
  return 0;
}

int setAxisTargetPos(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setTargetPos(value);
  return 0;
}

int setAxisTargetVel(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setTargetVel(value);
  return 0;
}

int setAxisJogVel(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setJogVel(value);
  return 0;
}

int setAxisHomeVelTwordsCam(int axisIndex,double dVel)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, dVel);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setHomeVelTwordsCam(dVel);
}

int setAxisHomeVelOffCam(int axisIndex,double dVel)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, dVel);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setHomeVelOffCam(dVel);
}

int setAxisHomePos(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setHomePosition(value);
  return 0;
}

int setAxisHomeDir(int axisIndex,int nDir)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, nDir);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setHomeDir((motionDirection)nDir);
}

int setAxisEnableCommandsFromOtherAxis(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEnableCascadedCommands(value);
}

int setAxisEnableCommandsTransform(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEnableCommandsTransform(value);
}

int axisErrorReset(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex,value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  axes[axisIndex]->setReset(value);
  return 0;
}

int setAxisGearRatio(int axisIndex,double ratioNum,double ratioDenom)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d num=%lf denom=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, ratioNum,ratioDenom);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->setGearRatio(ratioNum,ratioDenom);
}

int setAxisEncScaleNum(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  if(axisIndex>=1000){ //Virtual axis
    return ERROR_MAIN_VIRT_AXIS_FUNCTION_NOT_SUPPORTED;
  }

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
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, expr);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_TRANSFORM_RETURN_IF_ERROR(axisIndex)

  std::string tempExpr=expr;

  //Ensure that "OUT" or "IL" variables are defined
  bool out=tempExpr.find(TRANSFORM_EXPR_OUTPUT_VAR_NAME)!=std::string::npos;
  bool il=tempExpr.find(TRANSFORM_EXPR_INTERLOCK_VAR_NAME)!=std::string::npos;

  if(!(out && il)){
    return ERROR_MAIN_TRANSFORM_OUTPUT_VAR_MISSING;
  }

  return axes[axisIndex]->getTraj()->getExtInputTransform()->setExpression(tempExpr);
}

int setAxisTransformCommandExpr(int axisIndex,char *expr)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__,axisIndex, expr);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  std::string tempExpr=expr;

  return axes[axisIndex]->setCommandsTransformExpression(tempExpr);
}

int setAxisEncTransExpr(int axisIndex, char *expr)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%s\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, expr);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENC_TRANSFORM_RETURN_IF_ERROR(axisIndex)

  std::string sTemp=expr;

  //Ensure that "OUT" or "IL" variables are defined
  bool out=sTemp.find(TRANSFORM_EXPR_OUTPUT_VAR_NAME)!=std::string::npos;
  bool il=sTemp.find(TRANSFORM_EXPR_INTERLOCK_VAR_NAME)!=std::string::npos;

  if(!(out && il)){
    return ERROR_MAIN_TRANSFORM_OUTPUT_VAR_MISSING;
  }

  return axes[axisIndex]->getEnc()->getExtInputTransform()->setExpression(sTemp);
}

const char* getAxisTrajTransExpr(int axisIndex, int *error)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  if(axisIndex>=MAX_AXES || axisIndex<=0){
    printf("ERROR: Axis index out of range.\n");
    *error=ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    return "";
  }
  if(axes[axisIndex]==NULL){
    printf("ERROR: Axis object NULL\n");
    *error=ERROR_MAIN_AXIS_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getTraj()==NULL){
    printf("ERROR: Trajectory object NULL.\n");
    *error=ERROR_MAIN_TRAJECTORY_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getTraj()->getExtInputTransform()==NULL){
    printf("ERROR: Trajectory transform object NULL.\n");
    *error=ERROR_MAIN_TRAJ_TRANSFORM_OBJECT_NULL;
    return "";
  }
  std::string *sExpr=axes[axisIndex]->getTraj()->getExtInputTransform()->getExpression();
  *error=0;
  return sExpr->c_str();
}

const char* getAxisEncTransExpr(int axisIndex, int *error)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  if(axisIndex>=MAX_AXES || axisIndex<=0){
    printf("ERROR: Axis index out of range.\n");
    *error=ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    return "";
  }
  if(axes[axisIndex]==NULL){
    printf("ERROR: Axis object NULL\n");
    *error=ERROR_MAIN_AXIS_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getEnc()==NULL){
    printf("ERROR: Encoder object NULL.\n");
    *error=ERROR_MAIN_ENCODER_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getEnc()->getExtInputTransform()==NULL){
    printf("ERROR: Encoder transform object NULL.\n");
    *error=ERROR_MAIN_ENC_TRANSFORM_OBJECT_NULL;
    return "";
  }
  std::string *sExpr=axes[axisIndex]->getEnc()->getExtInputTransform()->getExpression();
  *error=0;
  return sExpr->c_str();
}

const char* getAxisTransformCommandExpr(int axisIndex, int *error)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  if(axisIndex>=MAX_AXES || axisIndex<=0){
    printf("ERROR: Axis index out of range.\n");
    *error=ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    return "";
  }
  if(axes[axisIndex]==NULL){
    printf("ERROR: Axis object NULL\n");
    *error=ERROR_MAIN_AXIS_OBJECT_NULL;
    return "";
  }
  if(axes[axisIndex]->getCommandTransform()==NULL){
    printf("ERROR: Axis command transform object NULL.\n");
    *error=ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED;
    return "";
  }
  std::string *sExpr=axes[axisIndex]->getCommandTransform()->getExpression();
  *error=0;
  return sExpr->c_str();

}

int setAxisTrajSource(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  int errorCode=axes[axisIndex]->getTraj()->setDataSourceType((dataSource)value);
  if(errorCode){
    return errorCode;
  }

  return 0;
}

int setAxisEncSource(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)

  int iRet=axes[axisIndex]->getEnc()->setDataSourceType((dataSource)value);
  if(iRet){
    return iRet;
  }

  return 0;
}

int setAxisTrajStartPos(int axisIndex,double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setStartPos(value);
  return 0;

}

//*****GET*********
int getAxisAcceleration(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getAcc();
  return 0;
}

int getAxisDeceleration(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getDec();
  return 0;
}

int getAxisTargetPos(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)
  *value=axes[axisIndex]->getSeq()->getTargetPos();
  return 0;
}

int getAxisTargetVel(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getSeq()->getTargetVel();
  return 0;
}

int getAxisDone(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  *value=!axes[axisIndex]->getSeq()->getBusy();
  return 0;
}

int getAxisPosSet(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getCurrentPosSet();
  return 0;
}

int getAxisVelFF(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getVel();
  return 0;
}

int getAxisExecute(int axisIndex, int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=axes[axisIndex]->getExecute();
  return 0;
}

int getAxisReset(int axisIndex, int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=axes[axisIndex]->getReset();
  return 0;
}

int getAxisID(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value=axes[axisIndex]->getAxisID();
  return 0;
}

int getAxisGearRatio(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%i\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getSeq()->getGearRatio(value);
}

int getAxisAtHardFwd(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getHardLimitFwd();
  return 0;
}

int getAxisAtHardBwd(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getTraj()->getHardLimitBwd();
  return 0;
}

int getAxisEncHomed(int axisIndex,int *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

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
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

    if(int iRet=axes[axisIndex]->getActPos(value)){
      value=0;
      return iRet;
    }
  return 0;
}

int getAxisEncVelAct(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

    if(int iRet=axes[axisIndex]->getActVel(value)){
      *value=0;
      return iRet;
    }

  return 0;
}

int getAxisAtHome(int axisIndex,int *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

    if(axes[axisIndex]->getMon()==NULL){
      return ERROR_MAIN_MONITOR_OBJECT_NULL;
    }

  *value=axes[axisIndex]->getMon()->getHomeSwitch();
  return 0;
}

int getAxisCntrlError(int axisIndex,double *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

    if(int iRet=axes[axisIndex]->getCntrlError(value)){
      *value=0;
      return iRet;
    }
  return 0;
}

int getAxisHomeVelOffCam(int axisIndex,double *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

    *value=axes[axisIndex]->getSeq()->getHomeVelOffCam();
  return 0;
}

int getAxisHomeVelTwordsCam(int axisIndex,double *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

    *value=axes[axisIndex]->getSeq()->getHomeVelTwordsCam();
  return 0;
}

int getAxisEncScaleNum(int axisIndex,double *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

    if(int iRet=axes[axisIndex]->getEncScaleNum(value)){
      value=0;
      return iRet;
    }

  return 0;
}

int getAxisEncScaleDenom(int axisIndex,double *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

    if(int iRet=axes[axisIndex]->getEncScaleDenom(value)){
      *value=0;
      return iRet;
    }
  return 0;
}

int getAxisEncPosRaw(int axisIndex,int64_t *value)
{
  if(functionDiag){
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);
  }

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if(int iRet=axes[axisIndex]->getEncPosRaw(value)){
    *value=0;
    return iRet;
  }
  return 0;
}

/****************************************************************************/

int setAxisCntrlKp(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setKp(value);
  return 0;
}

int setAxisCntrlKi(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setKi(value);
  return 0;
}

int setAxisCntrlKd(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setKd(value);
  return 0;
}

int setAxisCntrlKff(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setKff(value);
  return 0;
}

int setAxisCntrlOutHL(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setOutMax(value);
  return 0;
}

int setAxisCntrlOutLL(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setOutMin(value);
  return 0;
}

int setAxisCntrlIpartHL(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setIOutMax(value);
  return 0;
}

int setAxisCntrlIpartLL(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getCntrl()->setIOutMin(value);
  return 0;
}

//Cntrl GET
int getAxisCntrlEnable(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getCntrl()->getEnable();
  return 0;
}

int getAxisCntrlOutPpart(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getCntrl()->getOutPPart();
  return 0;
}

int getAxisCntrlOutIpart(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getCntrl()->getOutIPart();
  return 0;
}

int getAxisCntrlOutDpart(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getCntrl()->getOutDPart();
  return 0;
}

int getAxisCntrlOutFFpart(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getCntrl()->getOutFFPart();
  return 0;
}

int getAxisCntrlOutput(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getCntrl()->getOutTot();
  return 0;
}

int getAxisCntrlVelSet(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getCntrl()->getOutTot();
  return 0;
}

int setAxisEncOffset(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getEnc()->setOffset(value);
  return 0;
}

int setAxisEncBits(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getEnc()->setBits(value);
  return 0;
}

int setAxisEncType(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getEnc()->setType((encoderType)value);
}

/****************************************************************************/
//Drv SET

int setAxisDrvScaleNum(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getDrv()->setScaleNum(value);
  return 0;
}

int setAxisDrvScaleDenom(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getDrv()->setScaleDenom(value);
}

int setAxisDrvEnable(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getDrv()->setEnable(value);
}

int setAxisDrvVelSet(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getDrv()->setVelSet(value);
}

int setAxisDrvVelSetRaw(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getDrv()->setVelSetRaw(value);
}

//Drv GET
int getAxisDrvScale(int axisIndex,double *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getDrv()->getScale();;
  return 0;
}

int getAxisDrvEnable(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  *value=axes[axisIndex]->getDrv()->getEnable();;
  return 0;
}

/****************************************************************************/
//Mon SET
int setAxisMonAtTargetTol(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    axes[axisIndex]->getMon()->setAtTargetTol(value);
  return 0;
}

int setAxisMonAtTargetTime(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    axes[axisIndex]->getMon()->setAtTargetTime(value);
  return 0;
}

int setAxisMonEnableAtTargetMon(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    axes[axisIndex]->getMon()->setEnableAtTargetMon(value);
  return 0;
}

int setAxisMonPosLagTol(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%f\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    axes[axisIndex]->getMon()->setPosLagTol(value);
  return 0;
}

int setAxisMonPosLagTime(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    axes[axisIndex]->getMon()->setPosLagTime(value);
  return 0;
}

int setAxisMonEnableLagMon(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    axes[axisIndex]->getMon()->setEnableLagMon(value);
  return 0;
}

int setAxisMonMaxVel(int axisIndex, double value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%lf\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    return axes[axisIndex]->getMon()->setMaxVel(value);;
}

int setAxisMonEnableMaxVel(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    return axes[axisIndex]->getMon()->setEnableMaxVelMon(value);
}

int setAxisMonMaxVelDriveILDelay(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    return axes[axisIndex]->getMon()->setMaxVelDriveTime(value);
}

int setAxisMonMaxVelTrajILDelay(int axisIndex, int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d value=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex, value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    return axes[axisIndex]->getMon()->setMaxVelTrajTime(value);
}

//Mon GET
int getAxisMonAtTarget(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    *value=axes[axisIndex]->getMon()->getAtTarget();
  return 0;
}

int getAxisMonAtHome(int axisIndex,int *value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d\n",__FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
    CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

    *value=axes[axisIndex]->getMon()->getHomeSwitch();
  return 0;
}

/****************************************************************************/
//Configuration procedures

int createAxis(int index, int type)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d type:%d\n",__FILE__, __FUNCTION__, __LINE__,index,type);

  if(index<0 && index>=MAX_AXES){
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

    case ECMC_AXIS_TYPE_TRAJECTORY:
      if(axes[index]!=NULL){
        delete axes[index];
      }
      axes[index]=new ecmcAxisTraj(index,1/MCU_FREQUENCY);
      break;

    case ECMC_AXIS_TYPE_ENCODER:
      if(axes[index]!=NULL){
        delete axes[index];
      }
      axes[index]=new ecmcAxisEncoder(index,1/MCU_FREQUENCY);
      break;

    default:
      return ERROR_MAIN_AXIS_TYPE_UNKNOWN;
  }

  axisDiagIndex=index; //Always printout last axis added
  return 0;
}

int ecSetMaster(int masterIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d master index=%d \n",__FILE__, __FUNCTION__, __LINE__, masterIndex);
  return ec.init(masterIndex);
}


int ecAddSlave(uint16_t alias, uint16_t position, uint32_t vendorId,uint32_t productCode)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d alias=%d position=%d vendor_id=%d product_code=%d\n",__FILE__, __FUNCTION__, __LINE__, alias,position,vendorId,productCode);

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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d position=%d assign_axtive=%x sync0_cycle=%d sync0_shift=%d sync1_cycle=%d sync1_shift=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveBusPosition,assignActivate,sync0Cycle,sync0Shift,sync1Cycle,sync1Shift);


  ecmcEcSlave *slave=ec.findSlave(slaveBusPosition);
  if(slave==NULL){
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  return slave->configDC(assignActivate,sync0Cycle,sync0Shift,sync1Cycle,sync1Shift);
}


/*int ecAddEntry(int slaveIndex,int nSyncManager,int nPdo,uint16_t nEntryIndex,uint8_t  nEntrySubIndex, uint8_t nBits)
{
  if(_nFunctionDiag)
    fprintf(stdlog,"%s/%s:%d slave=%d sm=%d pdo=%d entry_index=%d  entry_subindex=%d bits=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,nSyncManager,nPdo,nEntryIndex,nEntrySubIndex,nBits);

  if(!_ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  if(_ec.getSlave(slaveIndex)==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;
  if(_ec.getSlave(slaveIndex)->getSyncManager(nSyncManager)==NULL)
    return ERROR_MAIN_EC_SM_NULL;
  if(_ec.getSlave(slaveIndex)->getSyncManager(nSyncManager)->getPdo(nPdo)==NULL)
    return ERROR_MAIN_EC_PDO_NULL;

  return _ec.getSlave(slaveIndex)->getSyncManager(nSyncManager)->getPdo(nPdo)->addEntry(nEntryIndex,nEntrySubIndex,nBits);
}*/

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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave=%d vendor=%d productcode=%d direction=%d sm=%d pdoindex=%d entry_index=%d entry_subindex=%d bits=%d id=%s\n",__FILE__, __FUNCTION__, __LINE__, position,vendorId,productCode,direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,entryIDString);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return ec.addEntry(position,vendorId,productCode,(ec_direction_t)direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,id);
}



int ecAddPdo(int slaveIndex,int syncManager,uint16_t pdoIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave=%d sm=%d pdo_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,syncManager,pdoIndex);

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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave=%d direction=%d sm_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,direction,syncMangerIndex);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  if(ec.getSlave(slaveIndex)==NULL)
    return ERROR_MAIN_EC_SLAVE_NULL;
  return ec.getSlave(slaveIndex)->addSyncManager((ec_direction_t)direction,syncMangerIndex);;
}

int ecAddSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d value=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,sdoSubIndex,value,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return (int)ec.addSDOWrite(slavePosition,sdoIndex,sdoSubIndex,value,byteSize);
}

int ecWriteSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value,int byteSize)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d value=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,sdoSubIndex,value,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec.writeSDO(slavePosition,sdoIndex,sdoSubIndex,value,byteSize);
}

uint32_t ecReadSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,int byteSize)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d bytesize=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,sdoIndex,sdoSubIndex,byteSize);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;
  return ec.readSDO(slavePosition,sdoIndex,sdoSubIndex,byteSize);
}


int linkEcEntryToAxisEnc(int slaveIndex, char *entryIDString,int axisIndex,int encoderEntryIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_index=%d entry=%s encoder=%d encoder_entry=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex,encoderEntryIndex);

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

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)

  if(encoderEntryIndex>=MaxMcuEncoderEntries && encoderEntryIndex<0)
    return ERROR_MAIN_ENCODER_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getEnc()->setEntryAtIndex(entry,encoderEntryIndex);
}

int linkEcEntryToAxisDrv(int slaveIndex,char *entryIDString,int axisIndex,int driveEntryIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_index=%d entry=%s drive=%d drive_entry=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex,driveEntryIndex);

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

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex)

  if(driveEntryIndex>=MaxMcuDriveEntries && driveEntryIndex<0)
    return ERROR_MAIN_DRIVE_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getDrv()->setEntryAtIndex(entry,driveEntryIndex);
}

int linkEcEntryToAxisMon(int slaveIndex,char *entryIDString,int axisIndex,int monitorEntryIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_index=%d entry=%s monitor=%d monitor_entry=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIDString,axisIndex,monitorEntryIndex);
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

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  if(monitorEntryIndex>=MaxMcuDriveEntries && monitorEntryIndex<0)
    return ERROR_MAIN_MONITOR_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getMon()->setEntryAtIndex(entry,monitorEntryIndex);
}

int writeEcEntry(int slaveIndex, int entryIndex,uint64_t value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_index=%d entry=%d value=%llu \n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIndex,(long long unsigned int)value);

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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_position=%d entry=%s value=%llu \n",__FILE__, __FUNCTION__, __LINE__, slavePosition,entryIDString,(long long unsigned int)value);
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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_index=%d entry=%d\n",__FILE__, __FUNCTION__, __LINE__, slaveIndex,entryIndex);

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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_index=%d entry=%s\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,entryIDString);

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
  if(functionDiag)
      fprintf(stdlog,"%s/%s:%d slave_index=%d entry=%s\n",__FILE__, __FUNCTION__, __LINE__, slavePosition,entryIDString);

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
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d slave_index=%d\n",__FILE__, __FUNCTION__, __LINE__, slavePosition);

  if(!ec.getInitDone())
    return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec.findSlaveIndex(slavePosition,value);
}



int setDiagAxisIndex(int axisIndex)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d axisIndex=%d \n",__FILE__, __FUNCTION__, __LINE__,axisIndex);


  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  axisDiagIndex=axisIndex;
  return 0;
}

int setDiagAxisFreq(int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d frequency=%d \n",__FILE__, __FUNCTION__, __LINE__,value);

  if(value<1 || value>500)
    return ERROR_MAIN_DIAG_AXIS_FREQ_OUT_OF_RANGE;

  axisDiagFreq=value;
  return 0;
}

int setDiagAxisEnable(int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d enable=%d \n",__FILE__, __FUNCTION__, __LINE__,value);

  enableAxisDiag=value;  //Disabled if 0
  return 0;
}

int setEnableTimeDiag(int value)
{
  if(functionDiag)
    fprintf(stdlog,"%s/%s:%d enable=%d \n",__FILE__, __FUNCTION__, __LINE__,value);

  enableTimeDiag=value;  //Disabled if 0
  return 0;
}
