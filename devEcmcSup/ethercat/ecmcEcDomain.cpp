/*************************************************************************\
* Copyright (c) 2023 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcDomain.h
*
*  Created on: Sept 29, 2023
*      Author: anderssandstrom
*
\*************************************************************************/

#include <exception>
#include "ecmcEcDomain.h"

ecmcEcDomain::ecmcEcDomain(ecmcAsynPortDriver *asynPortDriver,
                           ec_master_t *master,
                           int masterIndex,
                           int objIndex,
                           int exeCycles,
                           int offsetCycles) {
  initVars();
  master_         = master;
  asynPortDriver_ = asynPortDriver;
  objIndex_       = objIndex;
  masterIndex_    = masterIndex;
  exeCycles_      = exeCycles_;
  offsetCycles_   = offsetCycles_;
  
  if(offsetCycles_ > exeCycles_) {
    offsetCycles_ = 0;
  }

  domain_ = ecrt_master_create_domain(master);
  
  if (!domain_) {
    LOGERR("%s/%s:%d: ERROR: EtherCAT create domain failed.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    throw std::bad_alloc();
  }
  initAsyn();
}

void ecmcEcDomain::initVars() {
  errorReset();
  domain_            = NULL;
  master_            = NULL;
  asynPortDriver_    = NULL; 
  masterIndex_       = 0;
  objIndex_          = 0;
  asynParStat_       = NULL;
  asynParFailCount_  = NULL;
  domainPd_          = 0;
  statusOk_          = 0;
  statusWordOld_       = 0;
  notOKCounter_      = 0;
  notOKCounterTotal_ = 0;
  notOKCyclesLimit_  = 0;
  notOKCounterMax_   = 0;
  size_              = 0;
  statusWord_        = 0;
  allowOffLine_      = 0;
  exeCycles_         = 0;
  offsetCycles_      = 0;
  cycleCounter_      = 0;
}

ecmcEcDomain::~ecmcEcDomain()
{

}

ec_domain_t * ecmcEcDomain::getDomain() {
  return domain_;
}

int ecmcEcDomain::setAllowOffline(int allow) {
  allowOffLine_ = allow;
  return 0;
}

int ecmcEcDomain::getAllowOffline() {
   return allowOffLine_;
}

int ecmcEcDomain::checkState() {

  ecrt_domain_state(domain_, &state_);

  // filter domainOK_ for some cycles
  if (state_.wc_state != EC_WC_COMPLETE) {
    if (notOKCounter_ <= notOKCyclesLimit_) {
      notOKCounter_++;
    }

    if (notOKCounter_ > notOKCounterMax_) {
      notOKCounterMax_ = notOKCounter_;
    }
    notOKCounterTotal_++;
  } else {
    notOKCounter_ = 0;
  }
  statusOk_ = notOKCounter_ <= notOKCyclesLimit_;

  //Build domain status word
  statusWord_ = 0;
  // bit 0
  statusWord_ = statusWord_ + (state_.redundancy_active > 0);
  // bit 1
  statusWord_ = statusWord_ + ((state_.wc_state ==  EC_WC_ZERO) << 1);
  // bit 2
  statusWord_ = statusWord_ + ((state_.wc_state ==  EC_WC_INCOMPLETE) << 2);
  // bit 3
  statusWord_ = statusWord_ + ((state_.wc_state ==  EC_WC_COMPLETE) << 3);
  // bit 16..31
  statusWord_ = statusWord_ + ((uint16_t)(state_.working_counter) << 16);

  // Set summary alarm for ethercat
  //statusOk_= state_.wc_state ==  EC_WC_COMPLETE;

  //if(statusWord_ != statusWordOld_) {
  //  LOGERR("%s/%s:%d: INFO: Domain[%d] status changed: %d.\n",
  //       __FILE__,
  //       __FUNCTION__,
  //       __LINE__,
  //       objIndex_,
  //       statusWord_);
  //}
  
  statusWordOld_= statusWord_;

  return statusOk_;
}

// For objects using data from teh domain (axes, plcs...)
int ecmcEcDomain::getOK() {
  return statusOk_;
}

void ecmcEcDomain::process() { 
  // recivie data
  if(cycleCounter_== offsetCycles_) {
    ecrt_domain_process(domain_);
  }
  cycleCounter_++;

  if(cycleCounter_ >= exeCycles_) {
    cycleCounter_ = 0;
  }
}

void ecmcEcDomain::queue() { 
  // send data
  if(cycleCounter_== offsetCycles_) {
      ecrt_domain_queue(domain_);
  }
}

void ecmcEcDomain::updateAsyn() {
  asynParFailCount_->refreshParamRT(0);
  asynParStat_->refreshParamRT(0);
}

uint8_t *ecmcEcDomain::getDataPtr() {
  
  domainPd_ = ecrt_domain_data(domain_);

  if (!domainPd_) {
    LOGERR("%s/%s:%d: ERROR: ecrt_domain_data() failed (0x%x).\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         ERROR_EC_MAIN_DOMAIN_DATA_FAILED);
  }

  return domainPd_;
}

int ecmcEcDomain::initAsyn(){

  // Asyn  parameters in domain
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name = buffer;
  ecmcAsynDataItem *paramTemp = NULL;

  // Status word domain
  size_t charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_DOMAIN "%d." ECMC_ASYN_EC_PAR_DOMAIN_STAT_NAME,
                       masterIndex_, objIndex_);
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(statusWord_),
                                         sizeof(statusWord_),
                                         ECMC_EC_U32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->addSupportedAsynType(asynParamInt32);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);    
  paramTemp->setAllowWriteToEcmc(false);  
  paramTemp->refreshParam(1);
  asynParStat_ = paramTemp;

  // Domain fail counter total
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_DOMAIN "%d." ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_TOT_NAME,
                       masterIndex_,objIndex_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(notOKCounterTotal_),
                                         sizeof(notOKCounterTotal_),
                                         ECMC_EC_S32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  asynParFailCount_ = paramTemp;

  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

size_t ecmcEcDomain::getSize() {
  size_ = ecrt_domain_size(domain_);
  return size_;
}

int ecmcEcDomain::setFailedCyclesLimitInterlock(int cycles) {
  notOKCyclesLimit_ = cycles;
  // do not allow to be ok at startup
  notOKCounter_ = cycles;
  return 0;
}

void ecmcEcDomain::slowExecute() {
  notOKCounterMax_ = 0;
}
