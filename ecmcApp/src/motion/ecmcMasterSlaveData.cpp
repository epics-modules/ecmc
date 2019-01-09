/*
 * ecmcMasterSlaveData.cpp
 *
 *  Created on: Mar 16, 2016
 *      Author: anderssandstrom
 */

#include "ecmcMasterSlaveData.h"

ecmcMasterSlaveData::ecmcMasterSlaveData() {
  initVars();
}

ecmcMasterSlaveData::~ecmcMasterSlaveData()
{}

void ecmcMasterSlaveData::setPosition(double pos) {
  pos_ = pos;
}

void ecmcMasterSlaveData::setVelocity(double vel) {
  vel_ = vel;
}

double ecmcMasterSlaveData::getPosition() {
  return pos_;
}

double ecmcMasterSlaveData::getVelocity() {
  return vel_;
}

void ecmcMasterSlaveData::initVars() {
  pos_   = 0;
  vel_   = 0;
  ilock_ = false;
}

void ecmcMasterSlaveData::setInterlock(bool ilock) {
  ilock_ = ilock;
}

bool ecmcMasterSlaveData::getInterlock() {
  return ilock_;
}
