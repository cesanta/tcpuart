#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>

/*
 * Protocol is as follows:
 * 1) 1 byte - frame type. 0x01 - copy as is. 0x02 - generate a sequence of 'x'
 * 2) payload, depends on frame type
 *   - 0x01: 4 bytes big-endian length, N bytes of data to copy
 *   - 0x02: 4 bytes big-endian number of 'x' to generate
 */

void do_echo(int infd, int outfd);
void do_generate(int infd, int outfd);

size_t num_bytes = 0;
char next_payload_byte = 'x';

int main(int argc, char *argv[]) {
  unsigned char c = 0;
  struct timeval tv;
  fd_set read_set, write_set, err_set;
  FD_ZERO(&write_set);
  FD_ZERO(&err_set);
  for (;;) {
    int r;
    FD_ZERO(&read_set);
    FD_SET(0, &read_set);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    r = select(1, &read_set, &write_set, &err_set, &tv);
    if (r == 0) continue;
    if (r < 0) {
      perror("select");
      return 1;
    }
    while ((r = read(0, &c, 1)) < 0 && errno == EINTR) {
    };
    if (r < 0) {
      perror("read");
      return 1;
    }
    if (r == 0) {
      continue;
    }
    num_bytes++;
    switch (c) {
      case 1:
        do_echo(0, 1);
        break;
      case 2:
        do_generate(0, 1);
        break;
      default:
        fprintf(stderr, "Unknown frame type: 0x%02x\n", c);
        //        return 1;
    }
  }
  return 0;
}

#define MIN(a, b) ((a) < (b) ? (a) : (b))

void do_echo(int infd, int outfd) {
  uint32_t hdr_len = 1 + sizeof(double) + sizeof(uint32_t);
  unsigned char data[16384] = {1};
  uint32_t total_count = 1;
  int r;
  do {
    r = read(infd, data + total_count, hdr_len - total_count);
    if (r > 0) total_count += r;
  } while ((total_count < hdr_len && r >= 0) || (r < 0 && errno == EINTR));
  if (r < 0) {
    perror("read");
    return;
  }
  if (r == 0) {
    return;
  }
  num_bytes += total_count;
  unsigned char *p = data + 1 + sizeof(double);
  uint32_t payloadLen = *((uint32_t *) p);
  uint32_t i = 0;
  p += 4;
  while (i < payloadLen) {
    r = read(infd, p, payloadLen - i);
    if (r < 0) {
      if (errno == EINTR) continue;
      perror("read");
      return;
    }
    while (r > 0) {
      if (*p != next_payload_byte) {
        fprintf(stderr,
                "at byte %zu in stream (i = %u of %u, tc %u): %02x vs %02x\n",
                num_bytes, i, payloadLen, total_count, *p, next_payload_byte);
        exit(1);
      }
      p++;
      r--;
      i++;
      total_count++;
      num_bytes++;
    }
  }

  int bytes_written = 0;
  do {
    r = write(outfd, data + bytes_written, total_count - bytes_written);
    if (r > 0) bytes_written += r;
  } while ((r > 0 && bytes_written < total_count) || (r < 0 && errno == EINTR));
  if (r < 0) {
    perror("write");
    return;
  }
  if (r == 0) {
    return;
  }
}

void do_generate(int infd, int outfd) {
}
