/*************************************************************************\
* Copyright (c) 2023 PaulScherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcDataIF.h
*
*  Created on: Oct 27, 2023
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECDATAIF_H_
#define ECMCECDATAIF_H_

/* 
 * Generic interface for both ecmcEcEntry and ecmcEcData classes
 * - ecmcEcEntry configures an ethercat entry and access the same data from process image
 * - ecmcEcData accesses data from ethercat process image based on byte and bit offsets
 */

WIP  WIP WIP WIP
class ecmcEcDataIF {
 public:
  ecmcEcDataIF();
  
  ~ecmcEcDataIF();
  int         getBits();
  int         getByteOffset();
  ecmcEcDataType getDataType();

  // After activate
  int         activate();

  int         writeValue(uint64_t value);
  int         writeDouble(double   value);
  int         writeValueForce(uint64_t value);
  int         writeBit(int      bitNumber,
                       uint64_t value);
  int         virtual readValue(uint64_t *value);
  int         virtual readDouble(double *value);
  int         virtual readBit(int       bitNumber,
                      uint64_t *value);
  int         virtual updateInputProcessImage();
  int         virtual updateOutProcessImage();
  std::string getIdentificationName();
  int         validate();
  int         setComAlarm(bool alarm);
  
 private:
};
#endif  /* ECMCECDATAIF_H_ */
