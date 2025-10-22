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
    if (shmObj.valid == 0) {                \
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
extern ecmcShm shmObj;
extern int allowCallbackEpicsState;

const char *miscLibCmdList[] = { "m2m_write(",
                                 "m2m_read(",
                                 "m2m_stat(",
                                 "m2m_err_rst(",
                                 "m2m_get_err(",
                                 "m2m_ioc_run("
                                 "m2m_ioc_ec_ok(",
                                 "lut_get_value(",
                                 "epics_running()",
};

static int m2m_errorCode = 0;
static int lut_errorCode = 0;
static int misc_cmd_count = 9;

inline double m2m_write(double shmIndex, double data) {
  int index = (int)shmIndex;
  CHECK_SHM_INDEX(index);
  
  if(m2m_errorCode) {
    return (double)m2m_errorCode;
  }

  sem_wait(shmObj.sem);
  shmObj.dataPtr[index] = data;
  sem_post(shmObj.sem);
  return (double)m2m_errorCode;
}

inline double m2m_ioc_run(double mstIdx) {
  int index= (int) mstIdx;
  // allow master index -ECMC_SHM_MAX_MASTERS..+ECMC_SHM_MAX_MASTERS
  // negative master id: ioc without ethercat
  if(shmObj.valid == 0 || index >= ECMC_SHM_MAX_MASTERS || index <= -ECMC_SHM_MAX_MASTERS) {
    return -1;
  }

  if( index >= 0) { // ioc with ec master
    return (double) shmObj.mstPtr[index] > 0;
  } else { // ioc without ec master
    return (double) shmObj.simMstPtr[-index + ECMC_SHM_MAX_MASTERS] > 0;
  } 
  return -1.0; 
}

inline double m2m_ioc_ec_ok(double mstIdx) {
  int index= (int) mstIdx;
  // allow master index -ECMC_SHM_MAX_MASTERS..+ECMC_SHM_MAX_MASTERS
  // negative master id: ioc without ethercat
  if(shmObj.valid == 0 || index >= ECMC_SHM_MAX_MASTERS || index <= -ECMC_SHM_MAX_MASTERS) {
    return -1;
  }

  if( index >= 0) { // ioc with ec master
    return (double) shmObj.mstPtr[index] > 1;
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
  
  sem_wait(shmObj.sem);
  double data = shmObj.dataPtr[index];
  sem_post(shmObj.sem);
  return data;
}

inline double m2m_stat() {
  
  if (shmObj.valid == 0) {
    m2m_errorCode = ERROR_SHM_NULL;
    LOGERR("ERROR: SHM object NULL.\n");
    return -ERROR_SHM_NULL;
  }                 

  return (double)shmObj.valid;
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

// Ensure epics is running
inline double epics_running() {
  return (double)allowCallbackEpicsState;
}

#endif  /* ecmcPLC_libMisc_inc_ */
