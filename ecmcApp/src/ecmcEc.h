/*
 * ecmcEc.h
 *
 *  Created on: Dec 1, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCEC_H_
#define ECMCEC_H_
#include "ecrt.h"
#include "stdio.h"
#include <cmath>
#include <string>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcEcSDO.h"
#include "ecmcEcSlave.h"
#include "ecmcError.h"
#include "ecmcErrorsList.h"


class ecmcEc : public ecmcError
{
public:
  ecmcEc();
  ~ecmcEc();
  int init(int nMasterIndex);
  //void setMaster(ec_master_t *master);
  int addSlave(
      uint16_t alias, /**< Slave alias. */
      uint16_t position, /**< Slave position. */
      uint32_t vendorId, /**< Expected vendor ID. */
      uint32_t productCode /**< Expected product code. */);
  ecmcEcSlave *getSlave(int slave); //NOTE: index not bus position
  ec_domain_t *getDomain();
  ec_master_t *getMaster();
  bool getInitDone();
  void receive();
  void send();
  int compileRegInfo();
  void checkDomainState();
  int checkSlaveConfState(int slave);
  bool checkSlavesConfState();
  bool checkState();
  int activate();
  int setDiagnostics(bool diag);
  int addSDOWrite(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize);
  int writeAndVerifySDOs();
  uint32_t readSDO(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex, int byteSize);
  int writeSDO(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize);

  int addEntry(
      uint16_t       position, /**< Slave position. */
      uint32_t       vendorId, /**< Expected vendor ID. */
      uint32_t       productCode, /**< Expected product code. */
      ec_direction_t direction,
      uint8_t        syncMangerIndex,
      uint16_t       pdoIndex,
      uint16_t       entryIndex,
      uint8_t        entrySubIndex,
      uint8_t        bits,
      std::string    id
  );
  ecmcEcSlave *findSlave(int busPosition);
  int findSlaveIndex(int busPosition,int *slaveIndex);
  int updateTime();
  int printTimingInformation();
private:
  void initVars();
  int updateInputProcessImage();
  int updateOutProcessImage();

  ec_master_t *master_; /**< EtherCAT master */
  ec_domain_t *domain_;
  ec_domain_state_t dsOld_;
  ec_master_state_t msOld_;
  uint8_t *domainPd_ ;
  int slaveCounter_;
  int sdoCounter_;
  ecmcEcSlave *slaveArray_[EC_MAX_SLAVES];
  ec_pdo_entry_reg_t slaveEntriesReg_[EC_MAX_ENTRIES];
  unsigned int pdoByteOffsetArray_[EC_MAX_ENTRIES];
  unsigned int pdoBitOffsetArray_[EC_MAX_ENTRIES];
  ecmcEcSDO *sdoArray_[EC_MAX_ENTRIES];
  bool initDone_;
  bool diag_;
  ecmcEcSlave *simSlave_;
};
#endif /* ECMCEC_H_ */
