#include "ecmcDrive.hpp"

ecmcDrive::ecmcDrive()
{
  initVars();
}

ecmcDrive::ecmcDrive(double scale)
{
  initVars();
  scale_=scale;
}

void ecmcDrive::initVars()
{
  errorReset();
  interlock_=true;
  scale_=0;
  scaleNum_=0;
  scaleDenom_=0;
  velSetRaw_=0;
  velSet_=0;
  enable_=0;
  for(int i=0;i<MaxMcuDriveEntries;i++){
    entryArray_[i]=NULL;
  }
}

ecmcDrive::~ecmcDrive()
{
  ;
}

int ecmcDrive::setVelSet(double vel)
{
  if(interlock_){
    velSet_=0;
    velSetRaw_=0;
    return 0;
  }
  velSet_=vel;
  velSetRaw_=velSet_/scale_;
  return 0;
}

void ecmcDrive::setScaleNum(double scaleNum)
{
  scaleNum_=scaleNum;
  if(std::abs(scaleDenom_)>0){
    scale_=scaleNum_/scaleDenom_;
  }
}

int ecmcDrive::setScaleDenom(double scaleDenom)
{
  scaleDenom_=scaleDenom;
  if(scaleDenom_==0){
    return setErrorID(ERROR_DRV_SCALE_DENOM_ZERO);
  }
  scale_=scaleNum_/scaleDenom_;
  return 0;
}

double ecmcDrive::getScale()
{
  return scale_;
}

double ecmcDrive::getVelSet()
{
  return velSet_;
}

int ecmcDrive::getVelSetRaw()
{
  return velSetRaw_;
}

bool ecmcDrive::getEnable()
{
  return enable_;
}

void ecmcDrive::writeEntries()
{
  entryArray_[0]->writeValue(enable_);
  entryArray_[1]->writeValue(velSetRaw_);
}

int ecmcDrive::setEnable(bool enable){
  if(interlock_){
    return setErrorID(ERROR_DRV_DRIVE_INTERLOCKED);
  }
  enable_=enable;
  return 0;
}

int ecmcDrive::setEntryAtIndex(ecmcEcEntry *entry,int index)
{
  if(entry !=NULL && index< MaxMcuDriveEntries && index>=0){
    entryArray_[index]=entry;
    return 0;
  }
  else{
    printf("Assigning entry to cMcuDriv entry list failed. Index=%d \n",index);
    return setErrorID(ERROR_DRV_ASSIGN_ENTRY_FAILED);
  }
}

void ecmcDrive::setInterlock(bool interlock)
{
  interlock_=interlock;
}

bool ecmcDrive::getInterlock()
{
  return interlock_;
}

int ecmcDrive::setVelSetRaw(int vel)
{
  velSetRaw_=vel;
  return 0;
}

int ecmcDrive::validate()
{
  if(entryArray_[0]==NULL){//Enbale entry
    return setErrorID(ERROR_DRV_ENABLE_ENTRY_NULL);
  }

  if(entryArray_[1]==NULL){ //Velocity Setpoint entry
    return setErrorID(ERROR_DRV_VEL_SET_ENTRY_NULL);
  }

  if(scaleDenom_==0){
    return setErrorID(ERROR_DRV_SCALE_DENOM_ZERO);
  }
  return 0;
}
