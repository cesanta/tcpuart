/*
 * Copyright (c) 2014-2016 Cesanta Software Limited
 * All rights reserved
 */

#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "mongoose.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

struct {
  int bytes_sent;
  int bytes_received;
  double min_latency;
  double max_latency;
  double total_latency;
  int total_packets;
} stats;

struct timeval lastPrinted = {0, 0};

const suseconds_t oneSecond = 1000;
const int writeWatermark = 4 * 1024;
const char *address = NULL;
int listeningMode = 0;
int numProbes = -1;
int numSeconds = -1;
int probeLen = -1;
int maxNumProbesInFlight = -1;
long delayMillis = 0;
int doReconnect = 0;

double startTime = 0;
size_t numProbesSent = 0;
size_t numProbesRecvd = 0;

double probeTime = 0;
uint32_t payloadLen = 0;

double timediff_ms(const struct timeval t1, const struct timeval t2) {
  return ((t1.tv_sec - t2.tv_sec) * 1000000.0 + t1.tv_usec - t2.tv_usec) /
         1000.0;
}

void maybe_print_stats() {
  struct timeval cur;
  if (gettimeofday(&cur, NULL) < 0) {
    perror("gettimeofday");
    return;
  }
  if (lastPrinted.tv_sec == 0) {
    lastPrinted.tv_sec = cur.tv_sec;
    lastPrinted.tv_usec = cur.tv_usec;
    memset(&stats, 0, sizeof(stats));
    return;
  }
  suseconds_t d = timediff_ms(cur, lastPrinted);
  if (d < oneSecond) {
    return;
  }

  printf(
      "ospeed: %7d B/s, ispeed: %7d B/s, max_latency: %6.2lf ms, "
      "min_latency: %6.2lf ms, avg_latency: %6.2lf ms\n",
      (int) ((double) stats.bytes_sent / d * oneSecond),
      (int) ((double) stats.bytes_received / d * oneSecond), stats.max_latency,
      stats.min_latency, (double) stats.total_latency / stats.total_packets);

  lastPrinted.tv_sec = cur.tv_sec;
  lastPrinted.tv_usec = cur.tv_usec;
  memset(&stats, 0, sizeof(stats));
}

void maybe_send_probe(struct mg_connection *c) {
  if (c->send_mbuf.len >= writeWatermark) return;
  if (numProbes > 0 && numProbesSent == numProbes) return;
  if (maxNumProbesInFlight > 0 &&
      (numProbesSent - numProbesRecvd >= maxNumProbesInFlight)) {
    return;
  }
  if (delayMillis > 0) {
    struct timespec s;
    s.tv_sec = delayMillis / 1000;
    s.tv_nsec = (delayMillis % 1000) * 1000000;
    nanosleep(&s, NULL);
  }
  char *buf = calloc(1, probeLen);
  char *p = buf;
  double now = mg_time();
  *p++ = '\x01';
  memcpy(p, &now, sizeof(now));
  p += sizeof(now);
  uint32_t payloadLen = probeLen - 1 - sizeof(now) - sizeof(payloadLen);
  memcpy(p, &payloadLen, sizeof(payloadLen));
  p += sizeof(payloadLen);
  while (p - buf < probeLen) {
    *p++ = 'x';
  }
  mg_send(c, buf, probeLen);
  free(buf);
  stats.bytes_sent += probeLen;
  numProbesSent++;
}

void test_handler(struct mg_connection *c, int ev, void *data) {
  char addr[64];
  switch (ev) {
    case MG_EV_CONNECT:
      if (*((int *) data)) {
        printf("Connect failed, reconnecting...\n");
        mg_connect(c->mgr, address, test_handler);
        break;
      } else {
        printf("Connected\n");
      }
    /* fallthrough */
    case MG_EV_ACCEPT:
      if (ev == MG_EV_ACCEPT) {
        mg_sock_addr_to_str((union socket_address *) data, addr, sizeof(addr),
                            MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
        fprintf(stderr, "Connection from %s\n", addr);
      }
      c->user_data = NULL;
      startTime = cs_time();
    /* fallthrough */
    case MG_EV_POLL:
    case MG_EV_SEND:
      if (!(c->flags & MG_F_LISTENING)) {
        maybe_print_stats();
        maybe_send_probe(c);
      }
      break;
    case MG_EV_RECV: {
      struct mbuf *io = &c->recv_mbuf;
    next:
      // Read the payload, if any.
      if (payloadLen > 0) {
        int n = MIN(payloadLen, io->len);
        for (int i = 0; i < n; i++) {
          if (io->buf[i] != 'x') {
            fprintf(stderr, "want 'x', got 0x%02x\n",
                    (unsigned char) io->buf[i]);
            exit(1);
          }
        }
        mbuf_remove(io, n);
        stats.bytes_received += n;
        payloadLen -= n;
        if (payloadLen == 0 && probeTime > 0) {
          double latency = (mg_time() - probeTime) * 1000;
          if (stats.max_latency == 0 || latency > stats.max_latency)
            stats.max_latency = latency;
          if (stats.min_latency == 0 || latency < stats.min_latency)
            stats.min_latency = latency;
          stats.total_latency += latency;
          stats.total_packets++;
          numProbesRecvd++;
          // fprintf(stderr, "rec'd probe %d, latency %.2lf ms\n", (int)
          // numProbesRecvd, latency);
          if ((numProbes > 0 && numProbesRecvd == numProbes) ||
              (numSeconds > 0 && cs_time() - startTime >= numSeconds)) {
            printf("Done\n");
            exit(0);
          }
        }
      }
      // Read the header.
      size_t hdr_len = 1 + sizeof(probeTime) + sizeof(payloadLen);
      if (payloadLen > 0 || io->len < hdr_len) {
        maybe_send_probe(c);
        break;
      }
      char *p = io->buf + 1;
      memcpy(&probeTime, p, sizeof(probeTime));
      p += sizeof(probeTime);
      memcpy(&payloadLen, p, sizeof(payloadLen));
      p += sizeof(payloadLen);
      c->user_data = (void *) ((intptr_t) payloadLen);
      mbuf_remove(io, hdr_len);
      stats.bytes_received += hdr_len;

      maybe_send_probe(c);
      goto next;
    }
    case MG_EV_CLOSE:
      printf("Connection closed\n");
      if (listeningMode) {
        if (!doReconnect) {
          exit(1);
        }
        break;
      }
      c->user_data = NULL;
      if (doReconnect) {
        printf("Reconnecting...\n");
        sleep(1);
        mg_connect(c->mgr, address, test_handler);
      } else {
        exit(1);
      }
      break;
  }
}

void print_usage(FILE *f, const char *name) {
  fprintf(f, "Usage: %s [-h] [-l] [-s N] address\n", name);
  fprintf(f, "  -a - CA bundle to verify server/client with\n");
  fprintf(f, "  -c - certificate to present to the client/server\n");
  fprintf(f, "  -d - delay before each send, in milliseconds\n");
  fprintf(f, "  -h - print this message and exit\n");
  fprintf(f, "  -l - listening mode\n");
  fprintf(f, "  -n - exit after this many probes\n");
  fprintf(f, "  -N - exit after this many seconds of testing\n");
  fprintf(f, "  -r - reconnect on disconnection\n");
  fprintf(f, "  -s - number of bytes to send\n");
  fprintf(f, "  -t - latency test mode\n");
}

int main(int argc, char *argv[]) {
  int debugLevel = 0;
  const char *name = argv[0];
  const char *cert = NULL;
  const char *ca_bundle = NULL;

  int ch;
  while ((ch = getopt(argc, argv, "a:c:d:D:hln:N:rs:t")) != -1) {
    switch (ch) {
      case 'a':
        ca_bundle = optarg;
        break;
      case 'c':
        cert = optarg;
        break;
      case 'd':
        delayMillis = atoi(optarg);
        break;
      case 'D':
        debugLevel = atoi(optarg);
        break;
      case 'h':
        print_usage(stdout, name);
        return 0;
      case 'l':
        listeningMode = 1;
        break;
      case 'n':
        numProbes = atoi(optarg);
        break;
      case 'N':
        numSeconds = atoi(optarg);
        break;
      case 'r':
        doReconnect = 1;
        break;
      case 's':
        probeLen = atoi(optarg);
        break;
      case 't':
        maxNumProbesInFlight = 1;
        break;
      default:
        print_usage(stderr, name);
        return 1;
    }
  }
  argc -= optind;
  argv += optind;

  if (argc < 1) {
    print_usage(stderr, name);
    return 1;
  }

  probeLen = MAX(probeLen, 1 + sizeof(double) + 4);
  if (maxNumProbesInFlight == 1) {
    printf("Latency test mode, %d byte packets.\n", probeLen);
  }

  struct mg_mgr mgr;
  mg_mgr_init(&mgr, NULL);
  address = argv[0];
  cs_log_set_level(debugLevel);
  if (listeningMode) {
    struct mg_connection *nc;
    struct mg_bind_opts bopts;
    memset(&bopts, 0, sizeof(bopts));
    bopts.ssl_cert = cert;
    bopts.ssl_ca_cert = ca_bundle;
    if ((nc = mg_bind_opt(&mgr, address, test_handler, bopts)) == NULL) {
      fprintf(stderr, "failed to create listener\n");
      return 1;
    }
    fprintf(stderr, "Listening on %s\n", address);
    if (bopts.ssl_cert) fprintf(stderr, "Server cert %s\n", bopts.ssl_cert);
    if (bopts.ssl_ca_cert) {
      fprintf(stderr, "Verify clients with %s\n", bopts.ssl_ca_cert);
    }
  } else {
    struct mg_connection *nc;
    struct mg_connect_opts copts;
    const char *error_string = NULL;
    memset(&copts, 0, sizeof(copts));
    copts.ssl_cert = cert; /* Not really supported on the device side. */
    copts.ssl_ca_cert = ca_bundle;
    copts.ssl_server_name = "*";
    copts.error_string = &error_string;
    if ((nc = mg_connect_opt(&mgr, address, test_handler, copts)) == NULL) {
      fprintf(stderr, "failed to create connection: %s\n", error_string);
      return 1;
    }
    if (copts.ssl_cert) fprintf(stderr, "Client cert %s\n", copts.ssl_cert);
    if (copts.ssl_ca_cert) {
      fprintf(stderr, "Verify server with %s\n", copts.ssl_ca_cert);
    }
  }
  for (;;) {
    mg_mgr_poll(&mgr, 1000);
  }
  mg_mgr_free(&mgr);
  return 0;
}
