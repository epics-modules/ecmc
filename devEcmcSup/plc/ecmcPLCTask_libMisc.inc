/*************************************************************************\
* Copyright (c) 2019 Paul Sherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCTask_libMisc.inc
*
*  Created on: Oct 5, 2023
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ecmcPLC_libMisc_inc_
#define ecmcPLC_libMisc_inc_

#define CHECK_SHM_INDEX(shmIndex) {                        \
    m2m_errorCode = 0;                                     \
    if (shmIndex >= ECMC_SHM_ELEMENTS || shmIndex < 0) {   \
      m2m_errorCode = ERROR_SHM_INDEX_OUT_OF_RANGE;        \
      LOGERR("ERROR: SHM index out of range.\n");          \
    }                                                      \
    if (ecmcPLCTask::statShm_.valid == 0) {                \
      m2m_errorCode = ERROR_SHM_NULL;                      \
      LOGERR("ERROR: SHM object NULL.\n");                 \
    }                                                      \
}                                                          \

ecmcShm ecmcPLCTask::statShm_ = {0};

const char *miscLibCmdList[] = { "m2m_write(",
                                 "m2m_read(",
                                 "m2m_stat(",
                                 "m2m_err_rst(",
                                 "m2m_get_err(",
};

static int m2m_errorCode = 0;
static int misc_cmd_count = 5;

inline double m2m_write(double shmIndex, double data) {
  int index = (int)shmIndex;
  CHECK_SHM_INDEX(index);
  
  if(m2m_errorCode) {
    return (double)m2m_errorCode;
  }

  sem_wait(ecmcPLCTask::statShm_.sem);
  ecmcPLCTask::statShm_.dataPtr[index] = data;
  sem_post(ecmcPLCTask::statShm_.sem);
  return (double)m2m_errorCode;
}

inline double m2m_read(double shmIndex) {
  int index = (int)shmIndex;
  CHECK_SHM_INDEX(index);
  
  if(m2m_errorCode) {
    return 0;
  }
  
  sem_wait(ecmcPLCTask::statShm_.sem);
  double data = ecmcPLCTask::statShm_.dataPtr[index];
  sem_post(ecmcPLCTask::statShm_.sem);
  return data;
}

inline double m2m_stat() {
  
  if (ecmcPLCTask::statShm_.valid == 0) {
    m2m_errorCode = ERROR_SHM_NULL;
    LOGERR("ERROR: SHM object NULL.\n");
    return ERROR_SHM_NULL;
  }                 

  return (double)ecmcPLCTask::statShm_.valid;
}

// Reset error
inline double m2m_err_rst() {
  m2m_errorCode = 0;
  return 0;
}

// Get error
inline double m2m_get_err() {  
  return (double)m2m_errorCode;
}

#endif  /* ecmcPLC_libMisc_inc_ */
