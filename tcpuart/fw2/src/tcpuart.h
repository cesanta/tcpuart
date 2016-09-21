/*
 * Copyright (c) 2014-2016 Cesanta Software Limited
 * All rights reserved
 */

#ifndef CS_PROJECTS_TCPUART_FW2_SRC_TCPUART_H_
#define CS_PROJECTS_TCPUART_FW2_SRC_TCPUART_H_

#include "mongoose/mongoose.h"

#include "fw/src/mg_app.h"
#include "fw/src/mg_uart.h"

/*
 * A hook to pre-process data in the UART buffer.
 * An app based on TCPUART can perform local processing of some data this way.
 * Default implementation does nothing and just forwards everything.
 */
typedef void (*tu_uart_processor_fn)(struct mg_uart_state *us,
                                     struct mg_connection *nc);
extern tu_uart_processor_fn tu_uart_processor;

enum mg_app_init_result tu_processor_init(void);

#endif /* CS_PROJECTS_TCPUART_FW2_SRC_TCPUART_H_ */
