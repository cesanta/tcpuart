/*
 * Copyright (c) 2014-2016 Cesanta Software Limited
 * All rights reserved
 */

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char *argv[]) {
  unsigned char buf[2048];
  struct timeval tv;
  fd_set read_set, write_set, err_set;
  FD_ZERO(&write_set);
  FD_ZERO(&err_set);
  for (;;) {
    int r;
    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    r = select(STDOUT_FILENO, &read_set, &write_set, &err_set, &tv);
    if (r == 0) continue;
    if (r < 0) {
      perror("select");
      return 1;
    }
    while ((r = read(STDIN_FILENO, buf, sizeof(buf))) < 0 && errno == EINTR) {
    };
    if (r < 0) {
      perror("read");
      return 1;
    }
    if (r == 0) {
      continue;
    }
    write(STDOUT_FILENO, buf, r);
  }
  return 0;
}
