/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcMaster2Master.cpp
*
*  Created on: October 03, 2023
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcMaster2Master.h"

ecmcMaster2Master::ecmcMaster2Master() {
  int val = 50;
  ecmcShm<0x1234, int>::create(); 
  ecmcShm<0x1234, int> shm;
  shm.SetValue(&val);
}

ecmcMaster2Master::~ecmcMaster2Master(){

}

//template <key_t KEY, typename T, int COUNT> 
//int Shm<KEY, T, COUNT>::shmid_ = -1;
//
//int main(int argc, char ** argv)
//{
//  if(argc == 2)
//  {
//    if(std::string(argv[1]) == "server")
//    {
//      int val = 50;
//      Shm<0x1234, int>::create(); 
//      Shm<0x1234, int> shm;
//      shm.SetValue(&val);
//    }
//    else if(std::string(argv[1]) == "client")
//    {
//      Shm<0x1234, int> shm;
//      const int* ptr = shm.GetValue();
//      std::cout <<"Val = " << *ptr <<std::endl;
//      Shm<0x1234, int>::destroy(); 
//    }
//  }
//  else
//  {
//    std::cerr<<"Usage shm [server][client]"<<std::endl;
//  }
//
//  return 0;
//}
