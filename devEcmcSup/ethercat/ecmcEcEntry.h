/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcEntry.h
*
*  Created on: Dec 2, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECENTRY_H_
#define ECMCECENTRY_H_

#include <string>
#include <cmath>
#include "stdio.h"
#include "limits.h"
#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcEcDomain.h"
#include "ecmcOctetIF.h"
#include "ecmcAsynPortDriver.h"
#include "alarm.h"  //EPICS alarms

#include "asynPortDriver.h"
#ifndef VERSION_INT
#  define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#endif

#define VERSION_INT_4_37            VERSION_INT(4,37,0,0)
#define ECMC_ASYN_VERSION_INT VERSION_INT(ASYN_VERSION,ASYN_REVISION,ASYN_MODIFICATION,0)

#if ECMC_ASYN_VERSION_INT >= VERSION_INT_4_37
#define ECMC_ASYN_ASYNPARAMINT64
#endif

// ECENTRY ERRORS
#define ERROR_EC_ENTRY_DATA_POINTER_NULL 0x21000
#define ERROR_EC_ENTRY_INVALID_OFFSET 0x21001
#define ERROR_EC_ENTRY_INVALID_DOMAIN_ADR 0x21002
#define ERROR_EC_ENTRY_INVALID_BIT_LENGTH 0x21003
#define ERROR_EC_ENTRY_LINK_FAILED 0x21004
#define ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE 0x21005
#define ERROR_EC_ENTRY_INVALID_BIT_INDEX 0x21006
#define ERROR_EC_ENTRY_READ_FAIL 0x21007
#define ERROR_EC_ENTRY_WRITE_FAIL 0x21008
#define ERROR_EC_ENTRY_ASYN_TYPE_NOT_SUPPORTED 0x21009
#define ERROR_EC_ENTRY_ASSIGN_ADD_FAIL 0x2100A
#define ERROR_EC_ENTRY_REGISTER_FAIL 0x2100B
#define ERROR_EC_ENTRY_VALUE_OUT_OF_RANGE 0x2100C
#define ERROR_EC_ENTRY_SET_ALARM_STATE_FAIL 0x2100D
#define ERROR_EC_ENTRY_EC_DOMAIN_ERROR 0x2100E
#define ERROR_EC_ENTRY_DATATYPE_INVALID 0x2100F
#define ERROR_EC_ENTRY_SIZE_OUT_OF_RANGE 0x21010


class ecmcEcEntry : public ecmcError {
 public:
  ecmcEcEntry(ecmcAsynPortDriver *asynPortDriver,
              int masterId,
              int slaveId,
              ecmcEcDomain      *domain,
              ec_slave_config_t *slave,
              uint16_t           pdoIndex,
              uint16_t           entryIndex,
              uint8_t            entrySubIndex,
              ec_direction_t     nDirection,
              ecmcEcDataType     dt,
              std::string        id,
              int useInRealtime);
  
  // only used for simulation purpose              
  ecmcEcEntry(ecmcAsynPortDriver *asynPortDriver,
              int                masterId,
              int                slaveId,              
              uint8_t           *domainAdr,
              ecmcEcDataType     dt,
              std::string        id);
  virtual ~ecmcEcEntry();
  void        initVars();
  uint16_t    getEntryIndex();
  uint8_t     getEntrySubIndex();
  int         getBits();
  int         getEntryInfo(ec_pdo_entry_info_t *info);
  int         getByteOffset();
  ecmcEcDataType getDataType();

  // After activate
  int          activate();

  uint8_t     *getDomainAdr();
  int         writeValue(uint64_t value);
  int         writeDouble(double   value);
  int         writeValueForce(uint64_t value);
  int         writeBit(int      bitNumber,
                       uint64_t value);
  int         readValue(uint64_t *value);
  int         readDouble(double *value);
  int         readBit(int       bitNumber,
                      uint64_t *value);
  virtual int updateInputProcessImage();
  virtual int updateOutProcessImage();
  int         setUpdateInRealtime(int update);
  int         getUpdateInRealtime();
  std::string getIdentificationName();
  int         compileRegInfo();
  int         updateAsyn(bool force);
  bool        getSimEntry();
  virtual int validate();
  int         setComAlarm(bool alarm);
  int         getSlaveId();
  virtual int            getDomainOK();
  virtual ecmcEcDomain * getDomain();
  
 protected:
  void                setDomainAdr();  
  int                 initAsyn();
  uint8_t            *domainAdr_;
  uint8_t            *adr_;
  uint16_t            entryIndex_;
  uint8_t             entrySubIndex_;
  int16_t             pdoIndex_;
  uint                bitOffset_;
  bool                sim_;
  std::string         idString_;
  char               *idStringChar_;
  int                 updateInRealTime_;
  int                 masterId_;
  int                 slaveId_;
  int                 bitLength_;
  int                 byteOffset_;
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem   *entryAsynParam_;
  ecmcEcDataType      dataType_;
  ec_slave_config_t  *slave_;
  ecmcEcDomain       *domain_;
  ec_direction_t      direction_;
  uint64_t            buffer_;
  int8_t             *int8Ptr_;
  uint8_t            *uint8Ptr_;
  int16_t            *int16Ptr_;
  uint16_t           *uint16Ptr_;
  int32_t            *int32Ptr_;
  uint32_t           *uint32Ptr_;
  int64_t            *int64Ptr_;
  uint64_t           *uint64Ptr_;
  float              *float32Ptr_;
  double             *float64Ptr_;
  size_t              usedSizeBytes_;
};
#endif  /* ECMCECENTRY_H_ */
