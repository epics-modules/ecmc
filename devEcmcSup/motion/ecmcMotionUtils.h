/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMotionUtils.h
*
*  Created on: Sept 05, 2022
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCMOTIONUTILS_H
#define ECMCMOTIONUTILS_H

class ecmcMotionUtils  {
public:
  ecmcMotionUtils();
  ~ecmcMotionUtils();

  // Help functions for modulo motion
  static double getPosErrorModWithSign(double set,
                                       double setOld,
                                       double act,
                                       double modRange);
  static double getPosErrorModAbs(double set,
                                  double act,
                                  double modRange);
};

#endif  /* ECMCMOTIONUTILS_H */
