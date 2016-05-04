
#ifdef START_WINSOCK2
#include <winsock2.h>
#include <stdio.h>

#include "startWinSock.h"


int startWinSock(void)
{
  WSADATA wsadata;
  if (WSAStartup(MAKEWORD(1,1), &wsadata) == SOCKET_ERROR)
  {
    printf("Error creating socket.");
    return -1;
  }
  return 0;
}

#endif
