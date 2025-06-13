/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMisc.cpp
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcMisc.h"

#include "ecmcOctetIF.h"        // Log Macros
#include "ecmcErrorsList.h"
#include "ecmcDefinitions.h"
#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <semaphore.h>
#include <fcntl.h>

// TODO: REMOVE GLOBALS
#include "ecmcGlobalsExtern.h"

int createDataStorage(int index, int elements, int bufferType) {
  LOGINFO4("%s/%s:%d index=%d elements=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           elements);

  if ((index >= ECMC_MAX_DATA_STORAGE_OBJECTS) || (index < 0)) {
    return ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE;
  }

  if (elements <= 0) {
    return ERROR_MAIN_DATA_STORAGE_INVALID_SIZE;
  }

  // Sample rate fixed
  sampleRateChangeAllowed = 0;

  delete dataStorages[index];
  dataStorages[index] = new ecmcDataStorage(asynPort, index,
                                            elements,
                                            (ecmcDSBufferType)bufferType);

  if (!dataStorages[index]) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DATA STORAGE OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }

  return dataStorages[index]->getErrorID();
}


int clearStorage(int indexStorage) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->clearBuffer();
}

int getStorageDataIndex(int indexStorage, int *index) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  *index = dataStorages[indexStorage]->getCurrentIndex();
  return 0;
}

int setStorageEnablePrintouts(int indexStorage, int enable) {
  LOGINFO4("%s/%s:%d indexStorage=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage,
           enable);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  WRITE_DIAG_BIT(FUNCTION_DATA_STORAGE_DIAGNOSTICS_BIT, enable);
  return 0;
}

int printStorageBuffer(int indexStorage) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->printBuffer();
}

int readStorageBuffer(int indexStorage, double **data, int *size) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->getData(data, size);
}

int writeStorageBuffer(int indexStorage, double *data, int size) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->setData(data, size);
}

int appendStorageBuffer(int indexStorage, double *data, int size) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->appendData(data, size);
}

int setDataStorageCurrentDataIndex(int indexStorage, int position) {
  LOGINFO4("%s/%s:%d indexStorage=%d position=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage,
           position);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->setCurrentPosition(position);
}

int createShm() {
  LOGINFO4("%s/%s:%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  // first create or link to semaphore
  shmObj.sem = sem_open(ECMC_SEM_FILENAME, O_CREAT, 0666, 1);

  if (shmObj.sem == SEM_FAILED) {
    LOGERR("%s/%s:%d: ERROR: Failed create SEM (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__, ERROR_SEM_NULL);
    return ERROR_SEM_NULL;
  }

  printf("SEM object created: %p.\n", shmObj.sem);

  // ftok to generate unique key
  shmObj.key = (int)ftok(ECMC_SHM_FILENAME, ECMC_SHM_KEY);

  // shmget returns an identifier in shmid (add some extra bytes for ioc to ioc communication, not accessible via PLC)
  shmObj.shmid =
    shmget((key_t)shmObj.key,
           ECMC_SHM_ELEMENTS * sizeof(ECMC_SHM_TYPE) + ECMC_SHM_CONTROL_BYTES,
           0666 | IPC_CREAT);

  if (shmObj.shmid < 0) {
    LOGERR("%s/%s:%d: ERROR: Failed create SHM (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__, ERROR_SHMGET_ERROR);
    return ERROR_SHMGET_ERROR;
  }

  // shmat to attach to shared memory
  shmObj.memPtr = shmat(shmObj.shmid, (void *)0, 0);

  if (shmObj.memPtr == (ECMC_SHM_TYPE *)-1) {
    LOGERR("%s/%s:%d: ERROR: Failed attach SHM (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__, ERROR_SHMMAT_ERROR);
    return ERROR_SHMMAT_ERROR;
  }

  // Global data
  shmObj.dataPtr = (ECMC_SHM_TYPE *)shmObj.memPtr;

  // Info for iocs with ethercat master
  shmObj.mstPtr = (char *)shmObj.memPtr + ECMC_SHM_ELEMENTS *
                  sizeof(ECMC_SHM_TYPE);

  // Info for iocs without ethercat master
  shmObj.simMstPtr = (char *)shmObj.memPtr + ECMC_SHM_ELEMENTS *
                     sizeof(ECMC_SHM_TYPE) + ECMC_SHM_MAX_MASTERS;
  shmObj.size  = ECMC_SHM_ELEMENTS * sizeof(ECMC_SHM_TYPE);
  shmObj.valid = 1;
  return 0;
}

// LUTs
int loadLUT(int index, char *fileName) {
  LOGINFO4("%s/%s:%d index=%d fileName=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           fileName);
  CHECK_LUT_RETURN_IF_ERROR(index);

  try {
    if( luts[index] ) {
      delete luts[index];
    }
    luts[index] = new ecmcLookupTable<double,double>(fileName);
  }
  catch(...) {
    return ERROR_LUT_LOAD_ERROR;
  }
  return 0;
}
