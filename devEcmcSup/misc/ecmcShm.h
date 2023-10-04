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
template <typename T>
class ecmcShm
{

public:
  ecmcShm(key_t key, size_t count)
  {

  count_ = count;
  key_   = key;

    get(key,count);
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
  //void SetValue(const T* data, int count = 1)
  void SetValue(T data, int index)
  {
    if(sizeof(T)*index > sizeof(T) * count_)
    {
      throw std::runtime_error("Index pointing outside shm size");
    }

    T* dataPtr = (T*)shm_;
    dataPtr[index] = data;

    //memcpy(shm_, data, sizeof(T)*count);
  }

  //Get pointer to element
  T GetValue(size_t index)
  {
   if(sizeof(T)*index > sizeof(T) * count_)
    {
      throw std::runtime_error("Index pointing outside shm size");
    }

    T* ptr = new(shm_) T;
    return ptr[index];
  }

  static void create(key_t key, size_t count)
  {
    printf("CREATE shm\n");
    //count_ = count;
    //key_ = key;
    if ((shmid_ = shmget(key, count*sizeof(T), IPC_CREAT | 0666)) < 0) 
    {
      throw std::runtime_error("Failed create shm");
    }
  }

  static void destroy(key_t key, size_t count)
  {
    get(key,count);
    if(shmctl(shmid_, IPC_RMID, NULL)<0)
    {
    perror("shctl");
    throw std::runtime_error("Error cannot remove shared memory");
    }
      shmid_ = -1;
  }

private:
  static void get(key_t key, size_t count)
  {
    if(shmid_ == -1)
    {
      if((shmid_ = shmget(key, count*sizeof(T), 0666)) < 0)
      {
    perror("shmget");
    throw std::runtime_error("Shared memory not created");
      }
    }

  }

  static void attach()
  {
    if ((shm_ = shmat(shmid_, NULL, 0)) == (char *) -1) 
    {
    throw std::runtime_error("Failed attach shm");
    }
  }

  static void* shm_;
  static int shmid_;
  size_t count_;
  key_t key_;
};

#endif  /* ECMCSHM_H_ */
