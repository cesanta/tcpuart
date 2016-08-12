#include "common/cs_dbg.h"

#include "user_interface.h"

#include "common/platforms/esp8266/esp_mg_net_if.h"
#include "fw/platforms/esp8266/user/esp_sj_uart.h"
#include "fw/src/sj_app.h"
#include "fw/src/sj_hal.h"
#include "fw/src/sj_mongoose.h"
#include "fw/src/sj_pwm.h"
#include "fw/src/sj_sys_config.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static struct sys_config_tcp *s_tcfg = NULL;
static struct sys_config_uart *s_ucfg = NULL;
static struct sys_config_misc *s_mcfg = NULL;

static struct mg_connection *s_conn = NULL;
static struct mg_connection *s_mgr_conn = NULL;
static struct mg_connection *s_client_conn = NULL;
static double s_last_connect_attempt = 0;
static struct mbuf s_tcp_rx_tail;
static double s_last_activity = 0;
static double s_last_status_report = 0;

static void tu_conn_mgr(struct mg_connection *nc, int ev, void *ev_data);
static void tu_conn_handler(struct mg_connection *nc, int ev, void *ev_data);
static IRAM void tu_dispatch_uart(int uart_no);

static int init_tcp(struct sys_config_tcp *cfg) {
  char listener_spec[100];
  struct mg_bind_opts bopts;
  memset(&bopts, 0, sizeof(bopts));
  if (cfg->listener.port > 0) {
    sprintf(listener_spec, "%d", cfg->listener.port);
    if (cfg->listener.tls.cert) {
      bopts.ssl_cert = cfg->listener.tls.cert;
    }
  } else {
    /*
     * User doesn't want us to listen on a port, but we need a persistent
     * connection to manage UART buffers when there isn't any active one.
     * I'm not proud of this, but this is the easiest way to achieve that:
     * listen on a port that nobody is going to reach from the outside.
     */
    strcpy(listener_spec, "127.0.0.1:1234");
  }
  LOG(LL_INFO, ("Listening on %s (%s)", listener_spec,
                (bopts.ssl_cert ? bopts.ssl_cert : "-")));
  s_mgr_conn = mg_bind_opt(&sj_mgr, listener_spec, tu_conn_mgr, bopts);
  if (s_mgr_conn == NULL) {
    LOG(LL_ERROR, ("Failed to create listener"));
    return 0;
  }
  s_tcfg = cfg;
  return 1;
}

static int init_uart(struct sys_config_uart *ucfg) {
  struct esp_uart_config *cfg = calloc(1, sizeof(*cfg));
  cfg->uart_no = ucfg->uart_no;
  cfg->dispatch_cb = tu_dispatch_uart;
  cfg->baud_rate = ucfg->baud_rate;
  cfg->rx_buf_size = ucfg->rx_buf_size;
  cfg->rx_fc_ena = ucfg->rx_fc_ena;
  cfg->rx_fifo_full_thresh = ucfg->rx_fifo_full_thresh;
  cfg->rx_fifo_fc_thresh = ucfg->rx_fifo_fc_thresh;
  cfg->rx_fifo_alarm = ucfg->rx_fifo_alarm;
  cfg->rx_linger_micros = ucfg->rx_linger_micros;
  cfg->tx_buf_size = ucfg->tx_buf_size;
  cfg->tx_fc_ena = ucfg->tx_fc_ena;
  cfg->tx_fifo_empty_thresh = ucfg->tx_fifo_empty_thresh;
  cfg->tx_fifo_full_thresh = ucfg->tx_fifo_full_thresh;
  cfg->swap_rxcts_txrts = ucfg->swap_rxcts_txrts;
  cfg->status_interval_ms = ucfg->status_interval_ms;
  if (!esp_uart_init(cfg)) {
    LOG(LL_ERROR, ("UART init failed"));
    free(cfg);
    return 0;
  }
  LOG(LL_INFO, ("UART%d configured: %d fc %d/%d", cfg->uart_no, cfg->baud_rate,
                cfg->rx_fc_ena, cfg->tx_fc_ena));
  s_ucfg = ucfg;
  return 1;
}

size_t tu_dispatch_tcp_to_uart(struct mbuf *mb, int uart_no) {
  size_t len = 0;
  cs_rbuf_t *utxb = esp_uart_tx_buf(uart_no);
  esp_uart_dispatch_tx_top(uart_no);
  len = MIN(mb->len, utxb->avail);
  if (len > 0) {
    cs_rbuf_append(utxb, mb->buf, len);
    mbuf_remove(mb, len);
  }
  return len;
}

void check_beeper() {
  static int beeping_on_gpio = -1;
  static double last_change = 0;
  if (beeping_on_gpio >= 0) {
    if (s_mcfg->beeper.timeout_seconds == 0 ||
        s_mcfg->beeper.gpio_no != beeping_on_gpio ||
        (mg_time() - last_change > 0.9)) {
      sj_pwm_set(beeping_on_gpio, 0, 0);
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
    sj_pwm_set(beeping_on_gpio, 250, 125); /* BEEEP! (4 KHz) */
    last_change = now;
    LOG(LL_WARN,
        ("No activity for %d seconds - BEEP!", (int) (now - s_last_activity)));
  }
}

static void tcp_report_status(struct mg_connection *nc, int force) {
  if (s_tcfg->status_interval_ms <= 0) return;
  double now = mg_time();
  if (force ||
      (now - s_last_status_report) * 1000 >= s_tcfg->status_interval_ms) {
    char addr[32];
    mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
                        MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
    fprintf(stderr, "TCP %p %s f %d rb %d sb %d\n", nc, addr, (int) nc->flags,
            (int) nc->recv_mbuf.len, (int) nc->send_mbuf.len);
    s_last_status_report = now;
  }
}

static void tu_conn_handler(struct mg_connection *nc, int ev, void *ev_data) {
  (void) ev_data;

  sj_wdt_feed();

  switch (ev) {
    case MG_EV_POLL:
    case MG_EV_RECV:
    case MG_EV_SEND: {
      int uart_no = s_ucfg->uart_no;
      size_t len = 0;
      /* UART -> TCP */
      {
        esp_uart_dispatch_rx_top(uart_no);
        cs_rbuf_t *urxb = esp_uart_rx_buf(uart_no);
        while (urxb->used > 0 &&
               (len = (s_tcfg->tx_buf_size - nc->send_mbuf.len)) > 0) {
          uint8_t *data;
          len = cs_rbuf_get(urxb, len, &data);
          mbuf_append(&nc->send_mbuf, data, len);
          cs_rbuf_consume(urxb, len);
        }
        if (len > 0) {
          LOG(LL_DEBUG, ("UART -> %d -> TCP %d %d", (int) len,
                         rx_fifo_len(uart_no), tx_fifo_len(uart_no)));
          s_last_activity = mg_time();
        }
      }
      /* TCP -> UART */
      len = tu_dispatch_tcp_to_uart(&nc->recv_mbuf, uart_no);
      if (len > 0) {
        LOG(LL_DEBUG, ("UART <- %d <- TCP %d %d", len, rx_fifo_len(uart_no),
                       tx_fifo_len(uart_no)));
        s_last_activity = mg_time();
      }
      esp_uart_dispatch_bottom(uart_no);
      tcp_report_status(nc, 0 /* force */);
      break;
    }
    case MG_EV_CLOSE: {
      LOG(LL_INFO, ("%p closed", nc));
      tcp_report_status(nc, 1 /* force */);
      if (nc == s_conn) {
        esp_uart_set_rx_enabled(s_ucfg->uart_no, 0);
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
      break;
    }
  }
}

static void tu_set_conn(struct mg_connection *nc) {
  LOG(LL_INFO, ("New conn: %p", nc));
  nc->handler = tu_conn_handler;
  mg_lwip_set_keepalive_params(nc, s_tcfg->keepalive.idle,
                               s_tcfg->keepalive.interval,
                               s_tcfg->keepalive.count);
  s_last_status_report = mg_time();
  s_conn = nc;
  esp_uart_set_rx_enabled(s_ucfg->uart_no, 1);
}

static void tu_conn_mgr(struct mg_connection *nc, int ev, void *ev_data) {
  switch (ev) {
    case MG_EV_ACCEPT: {
      char addr[32];
      mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
                          MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
      LOG(LL_INFO, ("%p Connection from %s", nc, addr));
      if (s_conn != NULL) {
        LOG(LL_INFO, ("Evicting %p", s_conn));
        s_conn->flags |= MG_F_SEND_AND_CLOSE;
      }
      tu_set_conn(nc);
      break;
    }
    case MG_EV_POLL: {
      int uart_no = s_ucfg->uart_no;
      check_beeper();
      if (s_conn == NULL) {
        /* Pump UART buffers if we have no active connection. */
        if (uart_no >= 0) {
          esp_uart_dispatch_rx_top(uart_no);
          if (s_tcp_rx_tail.len > 0) {
            tu_dispatch_tcp_to_uart(&s_tcp_rx_tail, uart_no);
            mbuf_trim(&s_tcp_rx_tail);
          }
          esp_uart_dispatch_tx_top(uart_no);
          esp_uart_dispatch_bottom(uart_no);
        }
        /* Initiate outgoing connection, if configured. */
        if (s_client_conn == NULL && s_tcfg->client.remote_addr != NULL &&
            (mg_time() - s_last_connect_attempt) >=
                s_tcfg->client.reconnect_interval) {
          const char *error;
          struct mg_connect_opts copts;
          memset(&copts, 0, sizeof(copts));
          copts.ssl_cert = s_tcfg->client.tls.cert;
          copts.ssl_ca_cert = s_tcfg->client.tls.ca_cert;
          copts.ssl_server_name = s_tcfg->client.tls.server_name;
          copts.error_string = &error;
          LOG(LL_INFO, ("%p Connecting to %s (%s %s %s)", s_client_conn,
                        s_tcfg->client.remote_addr,
                        (copts.ssl_cert ? copts.ssl_cert : "-"),
                        (copts.ssl_ca_cert ? copts.ssl_ca_cert : "-"),
                        (copts.ssl_server_name ? copts.ssl_server_name : "-")));
          s_last_connect_attempt = mg_time();
          s_client_conn = mg_connect_opt(nc->mgr, s_tcfg->client.remote_addr,
                                         tu_conn_mgr, copts);
          if (s_client_conn == NULL) {
            LOG(LL_ERROR, ("Connection error: %s", error));
          }
        }
      }
      break;
    }
    case MG_EV_CONNECT: {
      int res = *((int *) ev_data);
      LOG(LL_INFO, ("%p Connect result: %d", nc, res));
      if (res == 0) {
        if (s_conn == NULL) {
          tu_set_conn(nc);
        } else {
          /* We already have a connection (probably accepted one while
           * connecting), drop it. */
          LOG(LL_INFO, ("%p Already have %p, closing this one", nc, s_conn));
          nc->flags |= MG_F_CLOSE_IMMEDIATELY;
        }
      } else {
        /* Do nothing, wait for close event. */
      }
    }
    case MG_EV_CLOSE: {
      if (nc == s_client_conn) {
        s_client_conn = NULL;
        s_last_connect_attempt = mg_time();
      }
    }
  }
}

/* Note: runs in ISR context, whole thing (including subs) must be in IRAM. */
static IRAM void tu_dispatch_uart(int uart_no) {
  (void) uart_no;
  if (s_conn != NULL) {
    mg_lwip_mgr_schedule_poll(s_conn->mgr);
  } else if (s_mgr_conn != NULL) {
    mg_lwip_mgr_schedule_poll(s_mgr_conn->mgr);
  }
}

enum mg_app_init_result sj_app_init() {
  s_mcfg = &get_cfg()->misc;
  s_last_activity = mg_time();
  LOG(LL_INFO, ("TCPUART init, SDK %s", system_get_sdk_version()));
  if (!init_tcp(&get_cfg()->tcp)) return MG_APP_INIT_ERROR;
  if (!init_uart(&get_cfg()->uart)) return MG_APP_INIT_ERROR;
  return MG_APP_INIT_SUCCESS;
}
