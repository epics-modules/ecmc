/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEventConsumer.h
*
*  Created on: Jun 2, 2016
*      Author: anderssandstrom
*
\*************************************************************************/


#ifndef ECMCEVENTCONSUMER_H_
#define ECMCEVENTCONSUMER_H_

class ecmcEventConsumer {
public:
  ecmcEventConsumer();
  virtual ~ecmcEventConsumer();
  virtual int executeEvent(int masterOK) = 0;
};

#endif  /* ECMCEVENTCONSUMER_H_ */
