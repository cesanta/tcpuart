/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 */

#ifndef CS_PROJECTS_TCPUART_FW2_SRC_TCPUART_H_
#define CS_PROJECTS_TCPUART_FW2_SRC_TCPUART_H_

#include "mongoose/mongoose.h"

#include "mgos_app.h"
#include "mgos_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

enum tu_conn_type {
  TU_CONN_TYPE_NONE,
  TU_CONN_TYPE_TCP,
  TU_CONN_TYPE_BT_GATTS,
};

struct tu_conn {
  enum tu_conn_type type;
  struct mg_connection *conn;
  struct tu_bt_conn *tubc;
};

/*
 * A hook to pre-process data in the UART buffer.
 * An app based on TCPUART can perform local processing of some data this way.
 * Default implementation does nothing and just forwards everything.
 */
typedef void (*tu_uart_processor_fn)(int uart_no, struct tu_conn *tu_conn);
extern tu_uart_processor_fn tu_uart_processor;

enum mgos_app_init_result tu_processor_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CS_PROJECTS_TCPUART_FW2_SRC_TCPUART_H_ */
