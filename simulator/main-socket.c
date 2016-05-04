#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <libgen.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/time.h>

#include <errno.h>
#if (!defined _WIN32 && !defined __WIN32__ && !defined __CYGWIN__)
  #include <signal.h>
#endif

#ifdef USE_WINSOCK2
#include <winsock2.h>
#define SHUT_RDWR     SD_BOTH
#else
#include <sys/types.h>
#include <arpa/inet.h>   /* htons, ntohs .. */
#include <sys/socket.h>
#include <arpa/inet.h>   /* htons, ntohs .. */
#include <netdb.h>
#include <sys/select.h>
#endif

#ifdef START_WINSOCK2
#include "startWinSock.h"
#else
#define startWinSock() 0
#endif

#include "cmd.h"



/* defines */
#define NUM_CLIENT_CONS 5
#define CLIENT_CONS_BUFLEN 1024

/*****************************************************************************/

/* typedefs */
typedef struct client_con_type {
  size_t        len_used;
  unsigned char *buffer;
  time_t        last_active_sec;
  time_t        idleTimeout;
  int           fd;
} client_con_type;

/* prototypes */
void send_to_socket(int fd, const char *buf, unsigned len);

/*****************************************************************************/
/* Static variables */
/*****************************************************************************/
static client_con_type client_cons[NUM_CLIENT_CONS];
FILE *stdlog;

void init_client_cons(void)
{
  unsigned i;
  memset(client_cons, 0, sizeof(client_cons));
  for (i=0; i < NUM_CLIENT_CONS; i++) {
    client_cons[i].fd = -1; /* fd is closed */
    client_cons[i].buffer = malloc(CLIENT_CONS_BUFLEN);
  }
}

void add_client_con(int fd)
{
  unsigned int i;
  for (i=0; i < NUM_CLIENT_CONS; i++) {
    if (client_cons[i].fd < 0) {
      client_cons[i].fd = fd;
      client_cons[i].idleTimeout = 0;
      LOGINFO7("%s/%s:%d add i=%d fd=%d\n",
               __FILE__,__FUNCTION__, __LINE__, i, fd);
      return;
    }
  }
  LOGINFO7("%s/%s:%d add() and close() i=%d fd=%d\n",
           __FILE__,__FUNCTION__, __LINE__, i, fd);
  /* No more space, we should close the oldest not used,
       the most stale.
       But now we close the new one */
  close(fd);
}


int find_client_con(int fd)
{
  unsigned int i;
  for (i=0; i < NUM_CLIENT_CONS; i++) {
    if (client_cons[i].fd == fd) {
      LOGINFO7("%s/%s:%d add i=%d fd=%d\n",
               __FILE__,__FUNCTION__, __LINE__, i, fd);
      return i;
    }
  }
  return -1;
}

void close_and_remove_client_con_i(int i)
{
  if (i >= 0) {
    int fd = client_cons[i].fd;
    int res = close(fd);
    LOGINFO7("%s/%s:%d close i=%d fd=%d res=%d (%s)\n",
             __FILE__,__FUNCTION__, __LINE__,
             i, fd, res,
             res ? strerror(errno) : "");
    client_cons[i].fd = -1;
    return;
  }
  LOGINFO7("%s/%s:%d close i=%d\n",
           __FILE__,__FUNCTION__, __LINE__, i);
}


void close_and_remove_client_con_fd(int fd)
{
  close_and_remove_client_con_i(find_client_con(fd));
}

/*****************************************************************************/

int get_listen_socket(const char *listen_port_asc)
{
  enum bind_ok_status  {
    bind_ok_not_tried   = -1,
    bind_ok_ok          = 0,
    bind_ok_addr_in_use = 1,
    bind_ok_failed      = 2
  };
  int bind_ok = bind_ok_not_tried;
  int reuse_on = 1;
  int sockfd = -1;
  int socket_family = AF_INET;
  int socket_type = SOCK_STREAM;
  int socket_protocol = 0;
#ifndef USE_WINSOCK2
  struct addrinfo *ai = NULL;
  struct addrinfo hints;
  int gai;
#endif

  if (startWinSock()) {
    LOGERR_ERRNO("startWinSock() failed\n");
    exit(3);
  }
  init_client_cons();

#ifndef USE_WINSOCK2
  /* initialize the hints */
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET6;        /* Listen to both IPV4 and IPV6 */
  hints.ai_socktype = SOCK_STREAM;   /* Want TCP */
  hints.ai_flags = AI_PASSIVE;       /* PASSIVE means a "server" */

  gai = getaddrinfo(NULL, listen_port_asc, &hints, &ai);
  if (gai) {
    LOGERR("%s/%s:%d getaddrinfo() failed: %s\n",
          __FILE__, __FUNCTION__, __LINE__,
           gai_strerror(gai));
  }
  if (ai) {
    if (ai->ai_next) {
      LOGERR("More than socket available, ignored\n");
    }
    socket_family = ai->ai_family;
    socket_type = ai->ai_socktype;
    socket_protocol = ai->ai_protocol;
  }
#endif

  sockfd = socket(socket_family, socket_type, socket_protocol);
  if (sockfd < 0) {
    if (socket_family == AF_INET6) {
      /* Some systems have IPv6 compiled,
         but not activated: try with IPv4 */
      socket_family = AF_INET;
      socket_type = SOCK_STREAM;
      socket_protocol = 0;
    }
  }
  if (sockfd < 0) {
    sockfd = socket(socket_family, socket_type, socket_protocol);
    if (sockfd < 0) {
      LOGERR_ERRNO("socket(%d %d %d ) failed sockfd=%d\n",
                   socket_family, socket_type, socket_protocol, sockfd);
      goto freeandret;
    }
  }
  /* The following is needed to prevent the shut down and restart
     of the server needing a timeout from the socket layer */
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
                 (char*)&reuse_on, sizeof(reuse_on)))  {
    LOGERR("setsockopt() failed\n");
    goto error;
  }
  /* make the socket a server socket */
#ifndef USE_WINSOCK2
  if (bind(sockfd, ai->ai_addr, ai->ai_addrlen) == 0) {
    bind_ok = bind_ok_ok;
  } else if (errno == EADDRINUSE) {
    LOGERR("%s/%s:%d bind() failed (%s) (errno=%d)\n",
           __FILE__,__FUNCTION__, __LINE__,
           strerror(errno), errno);
    goto error;
  } else {
    LOGINFO7("%s/%s:%d bind() failed (%s) (errno=%d)\n",
             __FILE__,__FUNCTION__, __LINE__,
             strerror(errno), errno);
    bind_ok = bind_ok_failed;
  }
#endif

  switch (bind_ok) {
    case bind_ok_not_tried:
    case bind_ok_failed:
    {
      int res;
      unsigned port = (unsigned)atoi(listen_port_asc);
      struct sockaddr_in server_addr;
      memset(&server_addr, 0, sizeof(server_addr));
      server_addr.sin_family = AF_INET;
      server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
      server_addr.sin_port = htons(port);
      res = bind(sockfd, (struct sockaddr*)&server_addr, (unsigned int)sizeof(server_addr));
      if (res < 0) {
        LOGERR("%s/%s:%d bind() failed (%s) (errno=%d)\n",
             __FILE__,__FUNCTION__, __LINE__,
               strerror(errno), errno);
        goto error;
      }
    }
    break;
    case bind_ok_ok:
    case bind_ok_addr_in_use:
      break;
  }

  if ((sockfd >= 0) && (listen(sockfd, 1) < 0))
  {
    LOGERR("listen() failed\n");
    goto error;
  }
  goto freeandret;


  error:
  close(sockfd);
  sockfd = -1;

  freeandret:
#ifndef USE_WINSOCK2
  freeaddrinfo(ai);
#endif
  if (sockfd >= 0) {
    LOGINFO("listening on port %s\n", listen_port_asc);
  }
  return sockfd;
}
/*****************************************************************************/
extern int socket_set_timeout(int fd, int timeout)
{
  int i = find_client_con(fd);
  if (i >= 0) {
    time_t old = client_cons[i].idleTimeout;
    client_cons[i].idleTimeout = timeout;
    LOGINFO7("%s/%s:%d i=%d fd=%d timeout=%d (old=%lu)\n",
             __FILE__, __FUNCTION__, __LINE__, i, fd, timeout, (unsigned long)old);
    return 0;
  }
  LOGINFO7("%s/%s:%d i=%d fd=%d timeout=%d\n",
           __FILE__, __FUNCTION__, __LINE__, i, fd, timeout);
  return 1;
}
/*****************************************************************************/

static int handle_input_line_fd(int socket_fd, const char *input_line, int had_cr, int had_lf)
{
  const char *this_stSettings_iTimeOut_str_s = ".THIS.stSettings.iTimeOut=";
  static const char *seperator_seperator = ";";
  static const char *terminator_terminator = "\n";
  char *buf;
  size_t buflen;

  if (PRINT_STDOUT_BIT1() && stdlog) {
    fprintf(stdlog,"SIN=\"");
    cmd_dump_to_std(input_line, strlen(input_line));
    fprintf(stdlog,"%s%s\"\n",
            had_cr ? "\\r" : "",
            had_lf ? "\\n" : "");
  }


  if (!strncmp(input_line, this_stSettings_iTimeOut_str_s,
               strlen(this_stSettings_iTimeOut_str_s))) {
    const char *myarg_1 = &input_line[strlen(this_stSettings_iTimeOut_str_s)];
    int timeout;
    int nvals;
    nvals = sscanf(myarg_1, "%d", &timeout);
    if (nvals == 1) {
      int res = socket_set_timeout(socket_fd, timeout);
      cmd_buf_printf("%s%s%s",
                     res ? "Error" : "OK",
                     seperator_seperator,
                     terminator_terminator);
    } else {
      cmd_buf_printf("Error nvals=%d %s%s",
                     nvals,
                     seperator_seperator,
                     terminator_terminator);
    }
  } else {
    cmd_handle_input_line(input_line);
  }
  cmd_buf_printf("%s%s",
           had_cr ? "\r" : "",
           had_lf ? "\n" : "");

  buf = get_buf();
  buflen = strlen(buf);
  if (PRINT_STDOUT_BIT1() && stdlog) {
    fprintf(stdlog,"SOUT=\"");
    cmd_dump_to_std(buf, buflen);
    fprintf(stdlog,"\"\n");
  }

  send_to_socket(socket_fd, buf, buflen);
  clear_buf();

  return 0;
}
/*****************************************************************************/

static void handle_data_on_socket(int i, int fd)
{
  ssize_t read_res = 0;
  size_t len_used = client_cons[i].len_used;

  /* append received data to the end
     keep one place for the '\n'  */

  read_res = recv(fd, (char *)&client_cons[i].buffer[len_used],
                  CLIENT_CONS_BUFLEN - len_used - 1, 0);
  LOGINFO7("%s/%s:%d FD_ISSET fd=%d read_res=%ld\n",
           __FILE__, __FUNCTION__, __LINE__, fd, (long)read_res);
  if (read_res <= 0)  {
    if (read_res == 0) {
      close_and_remove_client_con_i(i);
      LOGINFO(" EOF i=%d fd=%d\n", i, fd);
    }
  } else {
    char *pNewline;
    len_used = client_cons[i].len_used + read_res;
    client_cons[i].len_used = len_used;
    client_cons[i].buffer[len_used] = '\0';
    pNewline = strchr((char *)client_cons[i].buffer, '\n');
    LOGINFO7("%s/%s:%d FD_ISSET i=%d fd=%d len_used=%lu pNewline=%d\n",
             __FILE__, __FUNCTION__, __LINE__, i, fd,
             (unsigned long)len_used, pNewline ? 1 : 0);
    if (pNewline) {
      size_t line_len = 1 + (void*)pNewline - (void*)client_cons[i].buffer;
      int had_cr = 0;
      LOGINFO7("%s/%s:%d FD_ISSET i=%d fd=%d line_len=%lu\n",
               __FILE__, __FUNCTION__, __LINE__, i, fd,
               (unsigned long)line_len);
      *pNewline = 0; /* Remove '\n' */
      if (line_len > 1) pNewline--;
      if (*pNewline == '\r') {
        had_cr = 1;
        *pNewline = '\0';
      }
      if (handle_input_line_fd(fd, (const char *)&client_cons[i].buffer[0], had_cr, 1)) {
        close_and_remove_client_con_i(i);
      }
      client_cons[i].len_used = 0;
    }
  }
}

/*****************************************************************************/
void handle_accepted_socket(int listen_socket, int accepted_socket)
{
  /* We come here if we have accepted a new connection */
  unsigned int i;
  int end_recv_loop = 0;
  int end_select_loop = 0;

  add_client_con(accepted_socket);
  do
  {
    do
    {
      int max_timeout = 2 * 60 * 60; /*  2 hours */
      int res;
      fd_set rfds;
      struct timeval tv_now;
      struct timeval tv_select;
      int maxfd = 0;

      (void)gettimeofday(&tv_now, NULL);

      FD_ZERO (&rfds);
      tv_select.tv_sec = max_timeout;
      tv_select.tv_usec = 0;

      FD_SET(listen_socket, &rfds);
      for (i=0; i < NUM_CLIENT_CONS; i++) {
        int fd = client_cons[i].fd;
        time_t idleTimeout = client_cons[i].idleTimeout;
        if (fd < 0) continue;
        if (idleTimeout) {
          time_t last_active_sec = client_cons[i].last_active_sec;
          if (tv_now.tv_sec - idleTimeout > last_active_sec) {
            LOGINFO7("%s/%s:%d timeout i=%d fd=%d\n",
                     __FILE__, __FUNCTION__, __LINE__, i, fd);
            close_and_remove_client_con_i(i);
            fd = -1;
          } else {
            /* Wait at least 1 second */
            time_t wait_now = 1 + idleTimeout + last_active_sec - tv_now.tv_sec;
            if (tv_select.tv_sec > wait_now) {
              tv_select.tv_sec = wait_now;
            }
          }
        }
        if (fd >= 0) {
          FD_SET(fd, &rfds);
          if (maxfd < fd) {
            maxfd = fd;
          }
        }
      }
      maxfd = listen_socket > maxfd ? listen_socket : maxfd;
      LOGINFO7("%s/%s:%d select(): maxfd=%d tv_sec=%lu\n",
               __FILE__, __FUNCTION__, __LINE__,
               maxfd, (unsigned long)tv_select.tv_sec);
      res = select (maxfd + 1, &rfds, NULL, NULL, &tv_select);
      LOGINFO7("%s/%s:%d maxfd=%d res(select)=%d %s\n",
               __FILE__, __FUNCTION__, __LINE__,
               maxfd,
               res,
               res < 0 ? strerror(errno) : "");
      (void)gettimeofday(&tv_now, NULL);
      if (res < 0) {
        end_select_loop = 1;
        end_recv_loop = 1;
      } else {
        if (FD_ISSET (listen_socket, &rfds)) {
          LOGINFO7("%s/%s:%d FD_ISSET (listen_socket)\n",
                   __FILE__, __FUNCTION__, __LINE__);
          end_recv_loop = 1;
        } else {
          unsigned int i;

          for (i=0; i < NUM_CLIENT_CONS; i++) {
            int fd = client_cons[i].fd;
            if (fd < 0) continue;
            if (FD_ISSET (fd, &rfds)) {
              LOGINFO7("%s/%s:%d FD_ISSET fd=%d\n",
                       __FILE__, __FUNCTION__, __LINE__, fd);
              client_cons[i].last_active_sec = tv_now.tv_sec;
              handle_data_on_socket(i, fd);
            } else {
            }
          }
        }
      }
    } while (!end_recv_loop && !end_select_loop);
  } while (!end_recv_loop);
  LOGINFO("End of loop\n");
}

/*****************************************************************************/
void send_to_socket(int fd, const char *buf, unsigned len)
{
  int res;
  errno = 0;
  res = send(fd, buf, len, 0);
#ifdef ENOTSOCK
  if (res == -1 && errno == ENOTSOCK) {
#else
  if (res == -1) {
#endif
    res = write(fd, buf, len);
    if (res != (int)len)
    {
      LOGERR_ERRNO("write(%u %d) failed, calling close()\n",
                   len, res);
      close_and_remove_client_con_fd(fd);
    }
  }
  else if (res != (int)len)
  {
    LOGERR_ERRNO("send(%u %d) failed, calling close()\n",
                 len, res);
    close_and_remove_client_con_fd(fd);
  }
}


/*****************************************************************************/
void socket_loop(void)
{
  static const char *listen_port_asc = "5024";
  int listen_socket;
  int accepted_socket;
  int stop_and_exit = 0;

  listen_socket = get_listen_socket(listen_port_asc);

  if (listen_socket < 0)
  {
    LOGERR_ERRNO("no listening socket!\n");
    exit(3);
  }

  while (!stop_and_exit)
  {
    accepted_socket = accept(listen_socket, NULL, NULL);
    if (accepted_socket < 0)
    {
      LOGERR("accept() failed\n");
      stop_and_exit = 1;
      continue;
    }

    LOGINFO("Connection accepted fd=%d\n", accepted_socket);
    handle_accepted_socket(listen_socket, accepted_socket);
    LOGINFO("Connection closed\n");
  }
}


/*****************************************************************************/
void help_and_exit(const char *msg)
{
  if (msg) {
    fprintf(stderr, "%s\n", msg);
  }

  fprintf(stderr,
          "Usage    telnet_motor\n"
          "Example: telnet_motor -v \n"
          "Example: telnet_motor -v   1 prints all data received\n"
          "Example: telnet_motor -v   2 prints all data send\n"
          "Example: telnet_motor -v   3 prints all data received or send\n"
          "Example: telnet_motor -v  64 prints the socket events\n"
          "Example: telnet_motor -v 128 prints all data received or send\n"
          "Example:\n");

  exit(1);
}

/*****************************************************************************/

/*****************************************************************************/
int main(int argc, char** argv)
{
#if (!defined _WIN32 && !defined __WIN32__ && !defined __CYGWIN__)
  (void)signal(SIGPIPE, SIG_IGN);
#endif

  if (argc == 3 &&
      !strcmp(argv[1], "-v")) {
    debug_print_flags = atoi(argv[2]);
    if (!debug_print_flags) {
      help_and_exit("debug_print_flags must not be 0");
    }
  } else if (argc == 1) {
    ;
  } else {
    fprintf(stderr, "argc=%d\n", argc);

    help_and_exit("wrong argc");
  }

  stdlog = stdout;
  socket_loop();

  LOGINFO("End %s\n", __FUNCTION__);
  fflush(stdlog);
  return 0;
}
