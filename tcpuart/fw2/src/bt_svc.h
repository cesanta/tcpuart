/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 */

#ifndef CS_PROJECTS_TCPUART_FW2_SRC_BT_SVC_H_
#define CS_PROJECTS_TCPUART_FW2_SRC_BT_SVC_H_

#include "common/mg_str.h"
#include "common/queue.h"

#include "mgos_app.h"
#include "mgos_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

struct tu_bt_conn {
  uint16_t gatt_if;
  uint16_t conn_id;
  uint16_t mtu;
  bool notify;
  SLIST_ENTRY(tu_bt_conn) next;
};

typedef bool (*bt_tcpuart_rx_cb_t)(struct mg_str value);
typedef void (*bt_tcpuart_conn_cb_t)(struct tu_bt_conn *tubc);

void tu_bt_service_init(void);
void tu_bt_close(struct tu_bt_conn *tubc);
bool tu_bt_send(struct tu_bt_conn *tubc, struct mg_str value);
size_t tu_bt_get_tx_buf_len(void);

void tu_bt_set_rx_cb(bt_tcpuart_rx_cb_t cb);
void tu_bt_set_conn_cb(bt_tcpuart_conn_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* CS_PROJECTS_TCPUART_FW2_SRC_BT_SVC_H_ */
