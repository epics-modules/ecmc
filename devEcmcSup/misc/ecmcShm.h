/*************************************************************************\
* Copyright (c) 2023 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcShm.h
*
*  Created on: October 03, 2023
*      Author: anderssandstrom
* ref: https://stackoverflow.com/questions/33876164/shared-memory-in-linux
\*************************************************************************/

#ifndef ECMCSHM_H_
#define ECMCSHM_H_

#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <string.h>

/* For master to master communication */
template <key_t KEY, typename T, int COUNT = 1>
class ecmcShm
{

public:
  ecmcShm():shm_(0)
  {
    get();
    attach();
  }

  ~ecmcShm()
  {
    if(shm_ != NULL)
    {
      shmdt(shm_);
      shm_ = 0;
    }
  }

  //Set one element 
  void SetValue(const T* data, int count = 1)
  {
    if(sizeof(T)*count > sizeof(T) * COUNT)
    {
      throw std::runtime_error("Data size greater than shm size");
    }
    memcpy(shm_, data, sizeof(T)*count);
  }

  //Get pointer to element
  const T* GetValue()
  {
    T* ptr = new(shm_) T;
    return ptr;
  }


  static void create()
  {
    if ((shmid_ = shmget(KEY, COUNT*sizeof(T), IPC_CREAT | 0666)) < 0) 
    {
      throw std::runtime_error("Failed create shm");
    }
  }
  static void destroy()
  {
    get();
    if(shmctl(shmid_, IPC_RMID, NULL)<0)
    {
    perror("shctl");
    throw std::runtime_error("Error cannot remove shared memory");
    }
      shmid_ = -1;
  }

private:
  static void get()
  {
    if(shmid_ == -1)
    {
      if((shmid_ = shmget(KEY, COUNT*sizeof(T), 0666)) < 0)
      {
    perror("shmget");
    throw std::runtime_error("Shared memory not created");
      }
    }

  }

  void attach()
  {
    if ((shm_ = shmat(shmid_, NULL, 0)) == (char *) -1) 
    {
    throw std::runtime_error("Failed attach shm");
    }
  }
  void* shm_;
  static int shmid_;
};

#endif  /* ECMCSHM_H_ */
