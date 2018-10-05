/*
 * ecmPLCDataIF.h
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmPLCDataIF_H_
#define ecmPLCDataIF_H_

#include "ecmcAxisBase.h"
#include "ecmcEc.h"
#include "ecmcEcEntry.h"
#include "ecmcEcSlave.h"

#define ECMC_PLC_EC_ENTRY_INDEX 0

#define ERROR_PLC_AXIS_DATA_TYPE_ERROR 0x20600
#define ERROR_PLC_AXIS_NULL 0x20601
#define ERROR_PLC_EC_NULL 0x20602
#define ERROR_PLC_SOURCE_INVALID 0x20603
#define ERROR_PLC_EC_MASTER_INVALID 0x20604
#define ERROR_PLC_EC_SLAVE_NULL 0x20605
#define ERROR_PLC_EC_ENTRY_NULL 0x20606

class ecmPLCDataIF : public ecmcError : public ecmcEcEntryLink
{
public:
  ecmPLCDataIF(ecmcAxisBase *axis,std::string axisDataSource);
  ecmPLCDataIF(ecmcEc *ec,std::string ecDataSource);
  ~ecmPLCDataIF();
  int     read();
  int     write();
  double& getDataRef()
private:
  int                readAxis();
  int                writeAxis()
  int                readEc();
  int                writeEc()
  ecmcAxisDataType   parseAxisDataSource(std::string axisDataSource);
  int                parseAndLinkEcDataSource(std::string EcDataSource);
  int                parseEcPath(char* ecPath, int *master,int *slave, char*alias,int *bit);
  ecmcAxisBase       *axis_;
  ecmcEc             *ec_;
  double             data_;
  double             dataOld_;
  ecmcAxisDataType   dataSourceAxis_;
  ecmcDataSourceType source_;
};
#endif /* ecmPLCDataIF_H_ */
