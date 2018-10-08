/*
 * ecmcPLCDataIF.h
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLCDataIF_H_
#define ecmcPLCDataIF_H_

#include "ecmcAxisBase.h"
#include "ecmcEc.h"
#include "ecmcEcEntry.h"
#include "ecmcEcEntryLink.h"
#include "ecmcEcSlave.h"
#include "ecmcError.h"
#include <string.h>


#define ECMC_PLC_EC_ENTRY_INDEX 0

#define ERROR_PLC_AXIS_DATA_TYPE_ERROR 0x20600
#define ERROR_PLC_AXIS_NULL 0x20601
#define ERROR_PLC_EC_NULL 0x20602
#define ERROR_PLC_SOURCE_INVALID 0x20603
#define ERROR_PLC_EC_MASTER_INVALID 0x20604
#define ERROR_PLC_EC_SLAVE_NULL 0x20605
#define ERROR_PLC_EC_ENTRY_NULL 0x20606
#define ERROR_PLC_EC_VAR_NAME_INVALID 0x20607


class ecmcPLCDataIF : public ecmcEcEntryLink
{
public:
  ecmcPLCDataIF(ecmcAxisBase *axis,char *axisVarName);
  ecmcPLCDataIF(ecmcEc *ec,char *ecVarName);
  ecmcPLCDataIF(char *statVarName);
  ~ecmcPLCDataIF();
  int     read();
  int     write();
  double& getDataRef();
  const char *getVarName();
private:
  int                readAxis();
  int                writeAxis();
  int                readEc();
  int                writeEc();
  ecmcAxisDataType   parseAxisDataSource(char *axisVarName);
  int                parseAndLinkEcDataSource(char *ecVarName);
  int                parseEcPath(char* ecPath, int *master,int *slave, char*alias,int *bit);
  void               initVars();
  ecmcAxisBase       *axis_;
  ecmcEc             *ec_;
  double             data_;
  double             dataRead_;
  ecmcAxisDataType   dataSourceAxis_;
  ecmcDataSourceType source_;
  std::string        varName_;
};
#endif /* ecmcPLCDataIF_H_ */
