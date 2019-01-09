/*
 * ecmcPLCDataIF.h
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLCDataIF_H_
#define ecmcPLCDataIF_H_

#include "ecmcDefinitions.h"
#include "ecmcAxisBase.h"
#include "ecmcDataStorage.h"
#include "ethercat/ecmcEc.h"
#include "ethercat/ecmcEcEntry.h"
#include "ethercat/ecmcEcEntryLink.h"
#include "ethercat/ecmcEcSlave.h"
#include "ecmcError.h"
#include <string>
#include <sstream>

#define ECMC_PLC_EC_ENTRY_INDEX 0

#define ERROR_PLC_AXIS_DATA_TYPE_ERROR 0x20600
#define ERROR_PLC_AXIS_NULL 0x20601
#define ERROR_PLC_EC_NULL 0x20602
#define ERROR_PLC_SOURCE_INVALID 0x20603
#define ERROR_PLC_EC_MASTER_INVALID 0x20604
#define ERROR_PLC_EC_SLAVE_NULL 0x20605
#define ERROR_PLC_EC_ENTRY_NULL 0x20606
#define ERROR_PLC_EC_VAR_NAME_INVALID 0x20607
#define ERROR_PLC_TRAJ_NULL 0x20608
#define ERROR_PLC_MON_NULL 0x20609
#define ERROR_PLC_DATA_STORGAE_DATA_TYPE_ERROR 0x2060A
#define ERROR_PLC_DATA_STORAGE_NULL 0x2060B


class ecmcPLCDataIF : public ecmcEcEntryLink {
 public:
  ecmcPLCDataIF(ecmcAxisBase *axis,
                char         *axisVarName);
  ecmcPLCDataIF(ecmcDataStorage *ds,
                char            *dsVarName);
  ecmcPLCDataIF(ecmcEc *ec,
                char   *ecVarName);
  ecmcPLCDataIF(char              *varName,
                ecmcDataSourceType dataSource);
  ~ecmcPLCDataIF();
  int                 read();
  int                 write();
  double            & getDataRef();
  double              getData();
  void                setData(double data);
  const char        * getVarName();
  const char        * getExprTkVarName();
  int                 validate();
  int                 setReadOnly(int readOnly);

 private:
  int                 readAxis();
  int                 writeAxis();
  int                 readDs();
  int                 readEc();
  int                 writeDs();
  int                 writeEc();
  ecmcAxisDataType    parseAxisDataSource(char *axisVarName);
  ecmcDataStorageType parseDataStorageDataSource(char *axisVarName);
  int                 parseAndLinkEcDataSource(char *ecVarName);
  int                 parseEcPath(char *ecPath,
                                  int  *master,
                                  int  *slave,
                                  char *alias,
                                  int  *bit);
  void initVars();
  ecmcAxisBase *axis_;
  ecmcDataStorage *ds_;
  ecmcEc *ec_;
  double data_;
  double dataRead_;
  ecmcAxisDataType dataSourceAxis_;
  ecmcDataStorageType dataSourceDs_;
  ecmcDataSourceType source_;
  std::string varName_;
  std::string exprTkVarName_;
  int readOnly_;
};
#endif  /* ecmcPLCDataIF_H_ */
