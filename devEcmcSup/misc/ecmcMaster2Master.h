/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcMaster2Master.h
*
*  Created on: October 03, 2023
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCMASTER2MASTER_H_
#define ECMCMASTER2MASTER_H_

#include "ecmcShm.h"

class ecmcMaster2Master {
 public:
  ecmcMaster2Master(size_t count, int key);
  ~ecmcMaster2Master();
  int setValue(double value,  size_t index);
  int getValue(double *value, size_t index);

 private:
  key_t key_;
  size_t count_;
  ecmcShm<double> *shmObj_;
};

#endif  /* ECMCMASTER2MASTER_H_ */
