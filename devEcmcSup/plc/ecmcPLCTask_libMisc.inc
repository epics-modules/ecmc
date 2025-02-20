/*************************************************************************\
* Copyright (c) 2023 Paul Sherrer Institute
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

#define CHECK_LUT_INDEX(lutIndex)\
        {\
          if (lutIndex >= ECMC_MAX_LUTS ||                       \
              lutIndex < 0) {                                    \
            LOGERR("ERROR: LUT list index out of range.\n");     \
            lut_errorCode = ERROR_LUT_INDEX_OUT_OF_RANGE;        \
          }                                                      \
          if (luts[lutIndex] == NULL) {                \
            lut_errorCode = ERROR_LUT_INDEX_OUT_OF_RANGE;        \
            LOGERR("ERROR: LUT object NULL.\n");                 \
          }                                                      \
        }\

extern ecmcLookupTable<double, double>  *luts[ECMC_MAX_LUTS];

ecmcShm ecmcPLCTask::statShm_ = {0};

const char *miscLibCmdList[] = { "m2m_write(",
                                 "m2m_read(",
                                 "m2m_stat(",
                                 "m2m_err_rst(",
                                 "m2m_get_err(",
                                 "m2m_ioc_run("
                                 "m2m_ioc_ec_ok(",
                                 "lut_get_value(",
};

static int m2m_errorCode = 0;
static int lut_errorCode = 0;
static int misc_cmd_count = 8;

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

inline double m2m_ioc_run(double mstIdx) {
  int index= (int) mstIdx;
  // allow master index -ECMC_SHM_MAX_MASTERS..+ECMC_SHM_MAX_MASTERS
  // negative master id: ioc without ethercat
  if(ecmcPLCTask::statShm_.valid == 0 || index >= ECMC_SHM_MAX_MASTERS || index <= -ECMC_SHM_MAX_MASTERS) {
    return -1;
  }

  if( index >= 0) { // ioc with ec master
    return (double) ecmcPLCTask::statShm_.mstPtr[index] > 0;
  } else { // ioc without ec master
    return (double) ecmcPLCTask::statShm_.simMstPtr[-index + ECMC_SHM_MAX_MASTERS] > 0;
  } 
  return -1.0; 
}

inline double m2m_ioc_ec_ok(double mstIdx) {
  int index= (int) mstIdx;
  // allow master index -ECMC_SHM_MAX_MASTERS..+ECMC_SHM_MAX_MASTERS
  // negative master id: ioc without ethercat
  if(ecmcPLCTask::statShm_.valid == 0 || index >= ECMC_SHM_MAX_MASTERS || index <= -ECMC_SHM_MAX_MASTERS) {
    return -1;
  }

  if( index >= 0) { // ioc with ec master
    return (double) ecmcPLCTask::statShm_.mstPtr[index] > 1;
  } else { // ioc without ec master
    return 0.0; // ioc:s without ec master cannot be ec.op
  } 
  return -1.0; 
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
    return -ERROR_SHM_NULL;
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

inline double lut_get_value(double lutObjIndex, double index) {
  int objIndex = (int)lutObjIndex;
  lut_errorCode = 0;
  CHECK_LUT_INDEX(objIndex);
  
  if(lut_errorCode) {
    return 0;
  }

  return luts[objIndex]->getValue(index); 
}

#endif  /* ecmcPLC_libMisc_inc_ */
