/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 */

#include <ctype.h>
#include <stdlib.h>

#include "common/cs_dbg.h"
#include "common/mg_str.h"
#include "common/queue.h"

#include "bt_svc.h"

#include "mgos_config_util.h"
#include "mgos_debug.h"
#include "mgos_event.h"
#include "mgos_hal.h"
#include "mgos_sys_config.h"
#include "mgos_utils.h"
#include "mgos_timers.h"

#include "esp32_bt_gatts.h"

static bt_tcpuart_rx_cb_t s_rx_cb;
static bt_tcpuart_conn_cb_t s_conn_cb;

static struct mbuf s_buf_tx;

/* Note: UUIDs below are in reverse, because that's how ESP wants them. */
static const esp_bt_uuid_t mos_tcpuart_svc_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 =
        {
         /* _mOS_TUA_SVC_ID_, 5f6d4f53-5f54-5541-5f53-56435f49445f */
         0x5f, 0x44, 0x49, 0x5f, 0x43, 0x56, 0x53, 0x5f, 0x41, 0x55, 0x54, 0x5f,
         0x53, 0x4f, 0x6d, 0x5f,
        },
};

static const esp_bt_uuid_t mos_tcpuart_data_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 =
        {
         /* 0mOS_TUA_data__0, 306d4f53-5f54-5541-5f64-6174615f5f30 */
         0x30, 0x5f, 0x5f, 0x61, 0x74, 0x61, 0x64, 0x5f, 0x41, 0x55, 0x54, 0x5f,
         0x53, 0x4f, 0x6d, 0x30,
        },
};
static uint16_t mos_tcpuart_data_ah, mos_tcpuart_data_cc_ah;

const esp_gatts_attr_db_t mos_tcpuart_gatt_db[4] = {
    {
     .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
     .att_desc =
         {
          .uuid_length = ESP_UUID_LEN_16,
          .uuid_p = (uint8_t *) &primary_service_uuid,
          .perm = ESP_GATT_PERM_READ,
          .max_length = ESP_UUID_LEN_128,
          .length = ESP_UUID_LEN_128,
          .value = (uint8_t *) mos_tcpuart_svc_uuid.uuid.uuid128,
         },
    },
    /* data */
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &char_decl_uuid, ESP_GATT_PERM_READ, 1, 1,
      (uint8_t *) &char_prop_read_notify}},
    {{ESP_GATT_RSP_BY_APP},
     {ESP_UUID_LEN_128, (uint8_t *) mos_tcpuart_data_uuid.uuid.uuid128,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 0, 0, NULL}},
    {{ESP_GATT_RSP_BY_APP},
     {ESP_UUID_LEN_16, (uint8_t *) &char_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 0, 0, NULL}},
};

static struct mg_str s_last_debug_entry = MG_NULL_STR;

static SLIST_HEAD(s_conns,
                  tu_bt_conn) s_conns = SLIST_HEAD_INITIALIZER(s_conns);

static void dispatch_to_bt(struct tu_bt_conn *tubc) {
  if (mgos_bt_gatts_is_send_queue_empty() && s_buf_tx.len > 0) {
    size_t len = MIN(s_buf_tx.len, tubc->mtu - 6);

    mgos_bt_gatts_send_indicate(tubc->gatt_if, tubc->conn_id,
                                mos_tcpuart_data_ah,
                                mg_mk_str_n(s_buf_tx.buf, len),
                                mgos_sys_config_get_tu_bt_need_confirm());

    mbuf_remove(&s_buf_tx, len);
  }
}

static bool mgos_bt_tcpuart_svc_ev(struct esp32_bt_session *bs,
                                   esp_gatts_cb_event_t ev,
                                   esp_ble_gatts_cb_param_t *ep) {
  char buf[BT_UUID_STR_LEN];
  bool ret = false;
  struct tu_bt_conn *tubc = NULL;
  struct esp32_bt_connection *bc = NULL;
  if (bs != NULL) { /* CREAT_ATTR_TAB is not associated with any session. */
    bc = bs->bc;
    tubc = (struct tu_bt_conn *) bs->user_data;
  }
  switch (ev) {
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
      const struct gatts_add_attr_tab_evt_param *p = &ep->add_attr_tab;
      uint16_t svch = p->handles[0];
      mos_tcpuart_data_ah = p->handles[2];
      mos_tcpuart_data_cc_ah = p->handles[3];
      LOG(LL_DEBUG, ("svch = %d data_ah = %d", svch, mos_tcpuart_data_ah));
      break;
    }
    case ESP_GATTS_CONNECT_EVT: {
      tubc = (struct tu_bt_conn *) calloc(1, sizeof(*tubc));
      if (tubc == NULL) break;
      tubc->gatt_if = bs->bc->gatt_if;
      tubc->conn_id = bs->bc->conn_id;
      tubc->mtu = bs->bc->mtu;
      LOG(LL_DEBUG, ("MTU %u", tubc->mtu));
      tubc->notify = false;
      bs->user_data = tubc;
      SLIST_INSERT_HEAD(&s_conns, tubc, next);
      if (s_conn_cb != NULL) {
        s_conn_cb(tubc);
      }
      break;
    }
    case ESP_GATTS_READ_EVT: {
      const struct gatts_read_evt_param *p = &ep->read;
      if (p->handle != mos_tcpuart_data_ah || tubc == NULL) break;
      size_t len = s_last_debug_entry.len;
      if (len < p->offset) {
        len = 0;
      } else {
        len -= p->offset;
      }
      if (len > bs->bc->mtu - 1) len = bs->bc->mtu - 1;
      esp_gatt_rsp_t rsp = {.attr_value = {.handle = mos_tcpuart_data_ah,
                                           .offset = p->offset,
                                           .len = len}};
      memcpy(rsp.attr_value.value, s_last_debug_entry.p + p->offset, len);
      esp_ble_gatts_send_response(bc->gatt_if, bc->conn_id, p->trans_id,
                                  ESP_GATT_OK, &rsp);
      ret = true;
      break;
    }
    case ESP_GATTS_WRITE_EVT: {
      if (tubc == NULL) break;

      const struct gatts_write_evt_param *p = &ep->write;
      if (p->handle == mos_tcpuart_data_cc_ah) {
        /* Client config control write - toggle notification. */
        if (p->len != 2) break;
        /* We interpret notify and indicate bits the same. */
        tubc->notify = (p->value[0] != 0);
        tubc->mtu = bs->bc->mtu;
        LOG(LL_DEBUG, ("%s: data notify %s", mgos_bt_addr_to_str(p->bda, buf),
                       (tubc->notify ? "on" : "off")));
        ret = true;
      } else if (p->handle == mos_tcpuart_data_ah) {
        ret = true;
        if (p->len != 0) {
          if (s_rx_cb != NULL) {
            ret = s_rx_cb(mg_mk_str_n((const char *) p->value, p->len));
          } else {
            ret = false;
          }
        }
      }
      break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
      if (tubc != NULL) {
        SLIST_REMOVE(&s_conns, tubc, tu_bt_conn, next);
        free(tubc);
      }
      break;
    }
    case ESP_GATTS_CONF_EVT: {
      ret = true;

      /*
       * TODO(dfrank): I don't quite understand why we can't just invoke
       * dispatch_to_bt(tubc) directly: current function also runs on the
       * main mgos task, so there shouldn't be any difference. However,
       * if we invoke it directly, weird things begin happening:
       * at least we're getting multiple CONF events (with non-zero status) on
       * a single indication, which breaks invariants.
       */
      mgos_invoke_cb((mgos_cb_t) dispatch_to_bt, tubc, false /* from_isr */);
      break;
    }
    default:
      break;
  }
  return ret;
}

size_t tu_bt_get_tx_buf_len(void) {
  return s_buf_tx.len;
}

void tu_bt_close(struct tu_bt_conn *tubc) {
  mgos_bt_gatts_close(tubc->gatt_if, tubc->conn_id);
}

bool tu_bt_send(struct tu_bt_conn *tubc, struct mg_str value) {
  mbuf_append(&s_buf_tx, value.p, value.len);
  dispatch_to_bt(tubc);
  return true;
}

void tu_bt_set_rx_cb(bt_tcpuart_rx_cb_t cb) {
  s_rx_cb = cb;
}

void tu_bt_set_conn_cb(bt_tcpuart_conn_cb_t cb) {
  s_conn_cb = cb;
}

void tu_bt_service_init(void) {
  if (mgos_sys_config_get_bt_enable()) {
    mbuf_init(&s_buf_tx, 0);

    mgos_bt_gatts_register_service(mos_tcpuart_gatt_db,
                                   ARRAY_SIZE(mos_tcpuart_gatt_db),
                                   mgos_bt_tcpuart_svc_ev);
  }
}
