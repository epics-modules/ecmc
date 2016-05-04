/*
 * ecmcMasterSlaveData.h
 *
 *  Created on: Mar 16, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCMASTERSLAVEDATA_H_
#define ECMCMASTERSLAVEDATA_H_
#include "ecmcError.h"


class ecmcMasterSlaveData : public ecmcError
{
public:
  ecmcMasterSlaveData();
  ~ecmcMasterSlaveData();
  void setPosition(double pos);
  void setVelocity(double vel);
  double getPosition();
  double getVelocity();
private:
  void initVars();
  double pos_;
  double vel_;
};

#endif /* ECMCMASTERSLAVEDATA_H_ */
