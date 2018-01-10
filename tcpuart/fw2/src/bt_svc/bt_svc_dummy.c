/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 */

#include "bt_svc.h"

size_t tu_bt_get_tx_buf_len(void) {
  return 0;
}

void tu_bt_close(struct tu_bt_conn *tubc) {
  (void) tubc;
}

bool tu_bt_send(struct tu_bt_conn *tubc, struct mg_str value) {
  (void) tubc;
  (void) value;

  return false;
}

void tu_bt_set_rx_cb(bt_tcpuart_rx_cb_t cb) {
  (void) cb;
}

void tu_bt_set_conn_cb(bt_tcpuart_conn_cb_t cb) {
  (void) cb;
}

void tu_bt_service_init(void) {
}
