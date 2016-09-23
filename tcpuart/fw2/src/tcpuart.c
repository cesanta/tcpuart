/*
 * Copyright (c) 2014-2016 Cesanta Software Limited
 * All rights reserved
 */

#include "tcpuart.h"

#include <stdbool.h>

#include "common/cs_dbg.h"

#include "fw/src/mg_app.h"
#include "fw/src/mg_hal.h"
#include "fw/src/mg_mongoose.h"
#include "fw/src/mg_pwm.h"
#include "fw/src/mg_timers.h"
#include "fw/src/mg_sys_config.h"
#include "fw/src/mg_uart.h"

#if CS_PLATFORM == CS_P_ESP_LWIP
#include "user_interface.h"
#include "common/platforms/esp8266/esp_mg_net_if.h"
#include "fw/platforms/esp8266/user/esp_uart.h"
#elif CS_PLATFORM == CS_P_CC3200
#include "hw_types.h"
#include "driverlib/uart.h"
#include "fw/platforms/cc3200/src/cc3200_uart.h"
#else
#error Unsupported platform
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#ifndef IRAM
#define IRAM
#endif

static struct sys_config_tcp *s_tcfg = NULL;
static struct sys_config_uart *s_ucfg = NULL;
static struct sys_config_misc *s_mcfg = NULL;

static struct mg_uart_state *s_us = NULL;
static struct mg_connection *s_conn = NULL;
static struct mg_connection *s_client_conn = NULL;
static struct mg_connection *s_listener_conn = NULL;
static double s_last_connect_attempt = 0;
static struct mbuf s_tcp_rx_tail;
static double s_last_activity = 0;
static double s_last_tcp_status_report = 0;
static double s_last_uart_status_report = 0;
static struct mg_uart_stats s_prev_stats;

static void tu_conn_mgr(struct mg_connection *nc, int ev, void *ev_data);
static void tu_conn_mgr_timer_cb(void *arg);
static void tu_tcp_conn_handler(struct mg_connection *nc, int ev,
                                void *ev_data);
static void tu_ws_conn_handler(struct mg_connection *nc, int ev, void *ev_data);
static IRAM void tu_dispatcher(struct mg_uart_state *us);

tu_uart_processor_fn tu_uart_processor;

static int init_tcp(struct sys_config_tcp *cfg) {
  char listener_spec[100];
  struct mg_bind_opts bopts;
  memset(&bopts, 0, sizeof(bopts));
  if (cfg->listener.port > 0) {
    sprintf(listener_spec, "%d", cfg->listener.port);
    if (cfg->listener.tls.cert) {
      bopts.ssl_cert = cfg->listener.tls.cert;
    }
    LOG(LL_INFO, ("Listening on %s (%s, %s)", listener_spec,
                  (cfg->listener.ws.enable ? "WS" : ""),
                  (bopts.ssl_cert ? bopts.ssl_cert : "no SSL")));
    s_listener_conn =
        mg_bind_opt(mg_get_mgr(), listener_spec, tu_conn_mgr, bopts);
    if (s_listener_conn == NULL) {
      LOG(LL_ERROR, ("Failed to create listener"));
      return 0;
    }
    if (cfg->listener.ws.enable) {
      mg_set_protocol_http_websocket(s_listener_conn);
    }
  }
  mg_set_c_timer(200 /* ms */, true /* repeat */, tu_conn_mgr_timer_cb, NULL);
  s_tcfg = cfg;
  return 1;
}

static int init_uart(struct sys_config_uart *ucfg) {
  struct mg_uart_config *cfg = mg_uart_default_config();
  cfg->baud_rate = ucfg->baud_rate;
  cfg->rx_buf_size = ucfg->rx_buf_size;
  cfg->rx_fc_ena = ucfg->rx_fc_ena;
  cfg->rx_linger_micros = ucfg->rx_linger_micros;
  cfg->tx_buf_size = ucfg->tx_buf_size;
  cfg->tx_fc_ena = ucfg->tx_fc_ena;
#if CS_PLATFORM == CS_P_ESP_LWIP
  cfg->rx_fifo_full_thresh = ucfg->rx_fifo_full_thresh;
  cfg->rx_fifo_fc_thresh = ucfg->rx_fifo_fc_thresh;
  cfg->rx_fifo_alarm = ucfg->rx_fifo_alarm;
  cfg->tx_fifo_empty_thresh = ucfg->tx_fifo_empty_thresh;
  cfg->tx_fifo_full_thresh = ucfg->tx_fifo_full_thresh;
  cfg->swap_rxcts_txrts = ucfg->swap_rxcts_txrts;
#endif
  s_us = mg_uart_init(ucfg->uart_no, cfg, tu_dispatcher, NULL);
  if (s_us == NULL) {
    LOG(LL_ERROR, ("UART init failed"));
    free(cfg);
    return 0;
  }
  if (!ucfg->rx_throttle_when_no_net) {
    mg_uart_set_rx_enabled(ucfg->uart_no, true);
  }
  LOG(LL_INFO, ("UART%d configured: %d fc %d/%d", ucfg->uart_no, cfg->baud_rate,
                cfg->rx_fc_ena, cfg->tx_fc_ena));
  s_ucfg = ucfg;
  return 1;
}

size_t tu_dispatch_tcp_to_uart(struct mbuf *mb, struct mg_uart_state *us) {
  size_t len = 0;
  cs_rbuf_t *utxb = &us->tx_buf;
  len = MIN(mb->len, utxb->avail);
  if (len > 0) {
    cs_rbuf_append(utxb, mb->buf, len);
    mbuf_remove(mb, len);
    mg_uart_schedule_dispatcher(s_ucfg->uart_no);
  }
  return len;
}

void check_beeper(void) {
  static int beeping_on_gpio = -1;
  static double last_change = 0;
  if (beeping_on_gpio >= 0) {
    if (s_mcfg->beeper.timeout_seconds == 0 ||
        s_mcfg->beeper.gpio_no != beeping_on_gpio ||
        (mg_time() - last_change > 0.9)) {
      mg_pwm_set(beeping_on_gpio, 0, 0);
      beeping_on_gpio = -1;
      last_change = mg_time();
      return;
    }
    /* Continue beeping. */
    return;
  }
  /* Is beeping on inactivity enabled? */
  if (s_mcfg->beeper.timeout_seconds <= 0 || s_mcfg->beeper.gpio_no < 0) {
    return;
  }
  /* Should we be beeping? */
  const double now = mg_time();
  if ((now - s_last_activity > s_mcfg->beeper.timeout_seconds) &&
      (now - last_change > 0.9)) {
    beeping_on_gpio = s_mcfg->beeper.gpio_no;
    mg_pwm_set(beeping_on_gpio, 250, 125); /* BEEEP! (4 KHz) */
    last_change = now;
    LOG(LL_WARN,
        ("No activity for %d seconds - BEEP!", (int) (now - s_last_activity)));
  }
}

static int uart_cts(int uart_no) {
#if CS_PLATFORM == CS_P_ESP_LWIP
  return esp_uart_cts(uart_no);
#elif CS_PLATFORM == CS_P_CC3200
  return cc3200_uart_cts(uart_no);
#endif
}

static uint32_t uart_raw_ints(int uart_no) {
#if CS_PLATFORM == CS_P_ESP_LWIP
  return esp_uart_raw_ints(uart_no);
#elif CS_PLATFORM == CS_P_CC3200
  return cc3200_uart_raw_ints(uart_no);
#endif
}

static uint32_t uart_int_mask(int uart_no) {
#if CS_PLATFORM == CS_P_ESP_LWIP
  return esp_uart_int_mask(uart_no);
#elif CS_PLATFORM == CS_P_CC3200
  return cc3200_uart_int_mask(uart_no);
#endif
}

int uart_rx_fifo_len(int uart_no) {
#if CS_PLATFORM == CS_P_ESP_LWIP
  return esp_uart_rx_fifo_len(uart_no);
#elif CS_PLATFORM == CS_P_CC3200
  /* It's not possible to get exact FIFO length on CC3200. */
  return UARTCharsAvail(cc3200_uart_get_base(uart_no));
#endif
}

int uart_tx_fifo_len(int uart_no) {
#if CS_PLATFORM == CS_P_ESP_LWIP
  return esp_uart_tx_fifo_len(uart_no);
#elif CS_PLATFORM == CS_P_CC3200
  /* It's not possible to get exact FIFO length on CC3200. */
  return (UARTSpaceAvail(cc3200_uart_get_base(uart_no)) == 0);
#endif
}

static void report_status(struct mg_connection *nc, int force) {
  double now = mg_time();
  if (nc != NULL && s_tcfg->status_interval_ms > 0 &&
      (force ||
       (now - s_last_tcp_status_report) * 1000 >= s_tcfg->status_interval_ms)) {
    char addr[32];
    mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
                        MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
    fprintf(stderr, "TCP %p %s f %d rb %d sb %d\n", nc, addr, (int) nc->flags,
            (int) nc->recv_mbuf.len, (int) nc->send_mbuf.len);
    s_last_tcp_status_report = now;
  }
  if (s_us != NULL && s_ucfg->status_interval_ms > 0 &&
      (force ||
       (now - s_last_uart_status_report) * 1000 >=
           s_ucfg->status_interval_ms)) {
    struct mg_uart_state *us = s_us;
    struct mg_uart_stats *s = &us->stats;
    struct mg_uart_stats *ps = &s_prev_stats;
    int uart_no = us->uart_no;
    fprintf(stderr,
            "UART%d ints %u/%u/%u; rx en %d bytes %u buf %u fifo %u, ovf %u, "
            "lcs %u; "
            "tx %u %u %u, thr %u; hf %u i 0x%03x ie 0x%03x cts %d\n",
            uart_no, s->ints - ps->ints, s->rx_ints - ps->rx_ints,
            s->tx_ints - ps->tx_ints, us->rx_enabled,
            s->rx_bytes - ps->rx_bytes, us->rx_buf.used,
            uart_rx_fifo_len(uart_no), s->rx_overflows - ps->rx_overflows,
            s->rx_linger_conts - ps->rx_linger_conts,
            s->tx_bytes - ps->tx_bytes, us->tx_buf.used,
            uart_tx_fifo_len(uart_no), s->tx_throttles - ps->tx_throttles,
            mg_get_free_heap_size(), uart_raw_ints(uart_no),
            uart_int_mask(uart_no), uart_cts(uart_no));
    memcpy(ps, s, sizeof(*s));
    s_last_uart_status_report = now;
  }
}

static void tu_process_uart(struct mg_uart_state *us,
                            struct mg_connection *nc) {
  int num_sent = 0;
  if (nc == NULL) return;
  cs_rbuf_t *urxb = &us->rx_buf;
  int len = 0;
  while (urxb->used > 0 &&
         (len = (s_tcfg->tx_buf_size - nc->send_mbuf.len)) > 0) {
    uint8_t *data;
    len = cs_rbuf_get(urxb, len, &data);
    if (nc->flags & MG_F_IS_WEBSOCKET) {
      mg_send_websocket_frame(nc, WEBSOCKET_OP_BINARY, data, len);
    } else {
      mg_send(nc, data, len);
    }
    cs_rbuf_consume(urxb, len);
    num_sent += len;
  }
  if (num_sent > 0) {
    LOG(LL_DEBUG, ("UART -> %d -> %s", num_sent,
                   (nc->flags & MG_F_IS_WEBSOCKET ? "WS" : "TCP")));
    s_last_activity = mg_time();
  }
}

static IRAM void tu_dispatcher(struct mg_uart_state *us) {
  /* TCP -> UART */
  /* Drain buffer left from a previous connection, if any. */
  if (s_tcp_rx_tail.len > 0) {
    tu_dispatch_tcp_to_uart(&s_tcp_rx_tail, us);
    mbuf_trim(&s_tcp_rx_tail);
  }
  /* UART -> TCP */
  struct mg_connection *nc = s_conn;
  cs_rbuf_t *urxb = &us->rx_buf;
  if (urxb->used > 0) {
    tu_uart_processor(us, nc);
    if (s_conn != NULL) {
      /* See if we can unthrottle TCP RX */
      if (nc->recv_mbuf_limit == 0 && us->tx_buf.avail > 0) {
        if (s_tcfg->rx_buf_size > 0) {
          nc->recv_mbuf_limit = s_tcfg->rx_buf_size;
        } else {
          nc->recv_mbuf_limit = ~0;
        }
      }
    }
  }
}

static void tu_tcp_conn_handler(struct mg_connection *nc, int ev,
                                void *ev_data) {
  (void) ev_data;

  mg_wdt_feed();

  switch (ev) {
    case MG_EV_POLL:
    case MG_EV_RECV: {
      /* If there is a tail from previous conn, we need it to drain first. */
      if (s_tcp_rx_tail.len > 0) break;
      /* TCP -> UART */
      size_t len = tu_dispatch_tcp_to_uart(&nc->recv_mbuf, s_us);
      if (len > 0) {
        LOG(LL_DEBUG, ("UART <- %d <- TCP", (int) len));
        s_last_activity = mg_time();
      }
      break;
    }
    case MG_EV_SEND: {
      mg_uart_schedule_dispatcher(s_ucfg->uart_no);
      break;
    }
    case MG_EV_CLOSE: {
      LOG(LL_INFO, ("%p closed", nc));
      report_status(nc, 1 /* force */);
      if (nc == s_conn) {
        if (s_ucfg->rx_throttle_when_no_net) {
          mg_uart_set_rx_enabled(s_ucfg->uart_no, false);
        }
        if (nc->recv_mbuf.len > 0) {
          /* Rescue the bytes remaining in the rx buffer - if we have space. */
          if (s_tcp_rx_tail.len == 0) {
            mbuf_free(&s_tcp_rx_tail);
            s_tcp_rx_tail.len = nc->recv_mbuf.len;
            s_tcp_rx_tail.buf = nc->recv_mbuf.buf;
            nc->recv_mbuf.buf = NULL;
            nc->recv_mbuf.len = 0;
          } else {
            LOG(LL_WARN,
                ("Dropped %d bytes on the floor", (int) nc->recv_mbuf.len));
          }
        }
        s_conn = NULL;
      }
      if (nc == s_client_conn) {
        s_client_conn = NULL;
        s_last_connect_attempt = mg_time();
      }
      break;
    }
  }
}

static void tu_ws_conn_handler(struct mg_connection *nc, int ev,
                               void *ev_data) {
  mg_wdt_feed();

  switch (ev) {
    case MG_EV_WEBSOCKET_FRAME: {
      struct websocket_message *wm = (struct websocket_message *) ev_data;
      size_t len = 0;
      LOG(LL_DEBUG, ("ws frame %d", (int) wm->size));
      cs_rbuf_t *utxb = &s_us->tx_buf;
      len = MIN(wm->size, utxb->avail);
      if (len > 0) {
        cs_rbuf_append(utxb, wm->data, len);
        LOG(LL_DEBUG, ("UART <- %d <- WS", (int) len));
        s_last_activity = mg_time();
      }
      if (len < wm->size) {
        /* UART buffer is full. Save the rest of the frame and throttle RX. */
        size_t tail_len = (wm->size - len);
        LOG(LL_DEBUG, ("%d bytes added to tail", (int) tail_len));
        mbuf_append(&s_tcp_rx_tail, wm->data + len, tail_len);
        nc->recv_mbuf_limit = 0;
      }
      break;
    }
    case MG_EV_SEND: {
      mg_uart_schedule_dispatcher(s_ucfg->uart_no);
      break;
    }
    case MG_EV_CLOSE: {
      LOG(LL_INFO, ("%p closed", nc));
      report_status(nc, 1 /* force */);
      if (nc == s_conn) {
        if (s_ucfg->rx_throttle_when_no_net) {
          mg_uart_set_rx_enabled(s_ucfg->uart_no, false);
        }
        s_conn = NULL;
      }
      if (nc == s_client_conn) {
        s_client_conn = NULL;
        s_last_connect_attempt = mg_time();
      }
      break;
    }
  }
}

static void tu_set_conn(struct mg_connection *nc, bool ws) {
  LOG(LL_INFO, ("New conn: %p%s", nc, (ws ? " (WS)" : "")));
  if (s_conn != NULL) {
    if (s_tcfg->evict_old) {
      LOG(LL_INFO, ("Evicting %p", s_conn));
      s_conn->flags |= MG_F_SEND_AND_CLOSE;
      s_conn = NULL;
    } else {
      LOG(LL_INFO, ("%p already in place, dropping %p", s_conn, nc));
      nc->flags |= MG_F_CLOSE_IMMEDIATELY;
      return;
    }
  }
  nc->handler = (ws ? tu_ws_conn_handler : tu_tcp_conn_handler);
#if CS_PLATFORM == CS_P_ESP_LWIP
  mg_lwip_set_keepalive_params(nc, s_tcfg->keepalive.idle,
                               s_tcfg->keepalive.interval,
                               s_tcfg->keepalive.count);
#elif CS_PLATFORM == CS_P_CC3200
  /* On CC3200 keep-alive is enabled by default and can only be disabled. */
  SlSockKeepalive_t opt;
  opt.KeepaliveEnabled = (s_tcfg->keepalive.idle > 0);
  sl_SetSockOpt(nc->sock, SL_SOL_SOCKET, SL_SO_KEEPALIVE, (_u8 *) &opt,
                sizeof(opt));
#endif
  s_last_tcp_status_report = mg_time();
  if (s_tcfg->rx_buf_size > 0) nc->recv_mbuf_limit = s_tcfg->rx_buf_size;
  s_conn = nc;
  if (s_ucfg->rx_throttle_when_no_net) {
    mg_uart_set_rx_enabled(s_ucfg->uart_no, true);
  }
}

static void tu_conn_mgr(struct mg_connection *nc, int ev, void *ev_data) {
  switch (ev) {
    case MG_EV_ACCEPT: {
      char addr[32];
      mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
                          MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
      LOG(LL_INFO, ("%p Connection from %s", nc, addr));
      if (s_conn != NULL && !s_tcfg->evict_old) {
        LOG(LL_INFO, ("%p already in place, dropping %p", s_conn, nc));
        if (s_tcfg->listener.ws.enable) {
          mg_sock_addr_to_str(&s_conn->sa, addr, sizeof(addr),
                              MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
          mg_send_response_line(nc, 409, "Content-Type: text/plain\r\n");
          mg_printf(nc, "Existing connection: %s\r\n", addr);
          nc->flags |= MG_F_SEND_AND_CLOSE;
        } else {
          nc->flags |= MG_F_CLOSE_IMMEDIATELY;
        }
      } else {
        if (!s_tcfg->listener.ws.enable) {
          tu_set_conn(nc, false /* ws */);
        } else {
          /* Wait for handshake */
        }
      }
      break;
    }
    case MG_EV_WEBSOCKET_HANDSHAKE_REQUEST: {
      LOG(LL_INFO, ("%p WS handshake request", nc));
      break;
    }
    case MG_EV_WEBSOCKET_HANDSHAKE_DONE: {
      LOG(LL_INFO, ("%p WS handshake done", nc));
      tu_set_conn(nc, true /* ws */);
      break;
    }
    case MG_EV_CONNECT: {
      int res = *((int *) ev_data);
      LOG(LL_INFO, ("%p Connect result: %d", nc, res));
      if (res == 0) {
        if (s_conn == NULL) {
          if (!s_tcfg->client.ws.enable) {
            tu_set_conn(nc, false /* ws */);
          } else {
            char *uri = strdup(s_tcfg->client.ws.uri);
            mg_expand_mac_address_placeholders(uri);
            LOG(LL_INFO, ("%p Sending WS handshake to %s", nc, uri));
            mg_set_protocol_http_websocket(nc);
            mg_send_websocket_handshake2(nc, uri, s_tcfg->client.remote_addr,
                                         s_tcfg->client.ws.protocol, NULL);
            free(uri);
          }
        } else {
          /* We already have a connection (probably accepted one while
           * connecting), drop it. */
          LOG(LL_INFO, ("%p Already have %p, closing this one", nc, s_conn));
          nc->flags |= MG_F_CLOSE_IMMEDIATELY;
        }
      } else {
        /* Do nothing, wait for close event. */
      }
      break;
    }
    case MG_EV_CLOSE: {
      if (nc == s_client_conn) {
        LOG(LL_INFO, ("%p Closed", nc));
        s_client_conn = NULL;
        s_last_connect_attempt = mg_time();
      }
    }
  }
}

static void tu_conn_mgr_timer_cb(void *arg) {
  check_beeper();
  report_status(s_conn, 0 /* force */);
  /* Initiate outgoing connection, if configured. */
  if (s_conn == NULL && s_client_conn == NULL &&
      s_tcfg->client.remote_addr != NULL &&
      (mg_time() - s_last_connect_attempt) >=
          s_tcfg->client.reconnect_interval) {
    const char *error;
    struct mg_connect_opts copts;
    memset(&copts, 0, sizeof(copts));
    copts.ssl_cert = s_tcfg->client.tls.cert;
    copts.ssl_ca_cert = s_tcfg->client.tls.ca_cert;
    copts.ssl_server_name = s_tcfg->client.tls.server_name;
    copts.error_string = &error;
    LOG(LL_INFO,
        ("%p Connecting to %s (%s %s %s)", s_client_conn,
         s_tcfg->client.remote_addr, (copts.ssl_cert ? copts.ssl_cert : "-"),
         (copts.ssl_ca_cert ? copts.ssl_ca_cert : "-"),
         (copts.ssl_server_name ? copts.ssl_server_name : "-")));
    s_last_connect_attempt = mg_time();
    s_client_conn = mg_connect_opt(mg_get_mgr(), s_tcfg->client.remote_addr,
                                   tu_conn_mgr, copts);
    if (s_client_conn == NULL) {
      LOG(LL_ERROR, ("Connection error: %s", error));
    }
  }
  (void) arg;
}

#if CS_PLATFORM == CS_P_CC3200
int mg_pwm_set(int pin, int period, int duty) {
  /* TODO(rojer) */
  return 0;
}
#endif

enum mg_app_init_result tu_processor_init(void) __attribute__((weak));
enum mg_app_init_result tu_processor_init(void) {
  return MG_APP_INIT_SUCCESS;
}

enum mg_app_init_result mg_app_init(void) {
  s_mcfg = &get_cfg()->misc;
  s_last_activity = mg_time();
  LOG(LL_INFO, ("TCPUART init"));
  if (!init_tcp(&get_cfg()->tcp)) return MG_APP_INIT_ERROR;
  if (!init_uart(&get_cfg()->uart)) return MG_APP_INIT_ERROR;
  tu_uart_processor = tu_process_uart;
  return tu_processor_init();
}
