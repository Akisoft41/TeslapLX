// Copyright 2020 Pascal Akermann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bt.h"

// #include "time.h"
// #include "sys/time.h"

#define SPP_TAG "bt"
#define SPP_SERVER_NAME "teslap-server"
#define SPP_DEVICE_NAME "TeslapLX"

#define BT_SPP_RINGBUF_RX_SIZE 100
#define BT_SPP_RINGBUF_TX_SIZE (10 * 1024)

#define BT_SPP_DATA_MAX_SIZE ESP_SPP_MAX_MTU // (3 * 330)

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static uint32_t spp_handle = 0;
static RingbufHandle_t spp_rx_buffer = NULL;
static RingbufHandle_t spp_tx_buffer = NULL;
static bt_cb_t spp_open_cb = NULL;
static bt_cb_t spp_close_cb = NULL;
static uint32_t spp_size;
static uint32_t spp_stat_us;
static uint32_t spp_last_us;
static int spp_rx_buffer_min_free;
static int spp_tx_buffer_min_free;
static TickType_t spp_tx_ticksToWait = 1;

static bool spp_writing = false;
static bool spp_second_connexion = false;

void _open(uint32_t handle)
{
    if (spp_handle != 0) {
        ESP_LOGE(SPP_TAG, "connexion already oppened (handle=%d)", spp_handle);
        spp_second_connexion = true;
        esp_spp_disconnect(handle);
        return;
    }
    ESP_LOGD(SPP_TAG, "_open handle=%u", handle);
    spp_rx_buffer = xRingbufferCreate(BT_SPP_RINGBUF_RX_SIZE, RINGBUF_TYPE_BYTEBUF);
    spp_tx_buffer = xRingbufferCreate(BT_SPP_RINGBUF_TX_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (spp_rx_buffer == NULL || spp_tx_buffer == NULL) {
        spp_handle = 0;
        if (spp_rx_buffer)
            vRingbufferDelete(spp_rx_buffer);
        spp_rx_buffer = NULL;
        if (spp_tx_buffer)
            vRingbufferDelete(spp_tx_buffer);
        spp_tx_buffer = NULL;
        return;
    }

    spp_handle = handle;
    spp_rx_buffer_min_free = BT_SPP_RINGBUF_RX_SIZE;
    spp_tx_buffer_min_free = BT_SPP_RINGBUF_TX_SIZE;
    // spp_buffer_init();
    spp_writing = false;
    spp_size = 0;
    spp_stat_us = esp_timer_get_time();

    if (spp_open_cb) spp_open_cb(handle);
}

void _close(uint32_t handle)
{
    if (handle != spp_handle) {
        if (spp_second_connexion) {
            ESP_LOGE(SPP_TAG, "second connexion closed (handle=%d)", handle);
            spp_second_connexion = false;
            return;
        }
        if (spp_handle != 0) ESP_LOGE(SPP_TAG, "_close handle error %u != %u", handle, spp_handle);
        errno = EBADF;
        return;
    }

    ESP_LOGD(SPP_TAG, "_close handle=%u", handle);
    if (spp_close_cb) spp_close_cb(handle);

    if (spp_rx_buffer)
        vRingbufferDelete(spp_rx_buffer);
    spp_rx_buffer = NULL;
    if (spp_tx_buffer)
        vRingbufferDelete(spp_tx_buffer);
    spp_tx_buffer = NULL;
    spp_handle = 0;
}

void _read(uint32_t handle, uint8_t* data, uint16_t len)
{
    if (handle != spp_handle) {
        if (spp_handle != 0) ESP_LOGE(SPP_TAG, "read handle error %u != %u", handle, spp_handle);
        return;
    }

    BaseType_t done = xRingbufferSend(spp_rx_buffer, data, len, 0);
    if (done == 0)
        ESP_LOGW(SPP_TAG, "rx buffer full handle=%u", handle);
    else {
        int free = xRingbufferGetCurFreeSize(spp_rx_buffer);
        if (spp_rx_buffer_min_free > free) spp_rx_buffer_min_free = free;
    }
}

void _write(uint32_t handle)
{
    if (handle != spp_handle) {
        if (spp_handle != 0) ESP_LOGE(SPP_TAG, "write handle error %u != %u", handle, spp_handle);
        return;
    }

    size_t len;
    uint8_t* data = xRingbufferReceiveUpTo(spp_tx_buffer, &len, spp_tx_ticksToWait, BT_SPP_DATA_MAX_SIZE);
    if (data) {
        ESP_LOGD(SPP_TAG, "write handle=%u len=%i", handle, len);
        // if (len != 21) ESP_LOGI(SPP_TAG, "write handle=%u len=%i", handle, len);

        esp_err_t err = esp_spp_write(handle, len, data);
        vRingbufferReturnItem(spp_tx_buffer, data);
        if (err != ESP_OK) {
            ESP_LOGE(SPP_TAG, "esp_spp_write error %x %s, handle=%u", err, esp_err_to_name(err), handle);
            return;
        }

        spp_last_us = esp_timer_get_time();

        // stat
        spp_size += len;
        uint32_t time_us = spp_last_us - spp_stat_us;
        if (time_us >= 10 * 1000000) {
            // ESP_LOGI(SPP_TAG, "stat: size=%uB %iB/s",
            //          spp_size,
            //          (int)((float)spp_size / ((float)time_us / 1000000)));
            ESP_LOGI(SPP_TAG, "stat: size=%uB %iB/s rxbuf=%0.1f%% txbuf=%0.1f%%",
                     spp_size,
                     (int)((float)spp_size / ((float)time_us / 1000000)),
                     (float)spp_rx_buffer_min_free / BT_SPP_RINGBUF_RX_SIZE * 100,
                     (float)spp_tx_buffer_min_free / BT_SPP_RINGBUF_TX_SIZE * 100);
            spp_size = 0;
            spp_stat_us = spp_last_us;
        }
    }
    else {
        ESP_LOGD(SPP_TAG, "spp_writing false");
        spp_writing = false;
    }
}

size_t bt_write(uint32_t handle, const void* buf, size_t count)
{
    if (buf == NULL || count == 0) return 0;
    if (handle != spp_handle) {
        if (spp_handle != 0) ESP_LOGE(SPP_TAG, "bt_write handle error %u != %u", handle, spp_handle);
        errno = EBADF;
        return -1;
    }

    BaseType_t done = xRingbufferSend(spp_tx_buffer, buf, count, 0);
    ESP_LOGV(SPP_TAG, "bt_write handle=%i count=%i done=%i spp_writing=%i", handle, count, done, spp_writing);
    // ESP_LOGI(SPP_TAG, "bt_write handle=%i count=%i done=%i spp_writing=%i", handle, count, done, spp_writing);
    if (!done) {
        ESP_LOGD(SPP_TAG, "tx buffer full handle=%u", handle);
        ESP_LOGW(SPP_TAG, "tx buffer full handle=%u", handle);

        if (spp_writing && (esp_timer_get_time() - spp_last_us) > 5 * 1000000) {
            // bug: spp_writing = true and no event
            ESP_LOGE(SPP_TAG, "tx buffer full and timeout, reinit write handle=%u", handle);
            _write(handle);
        }
        if (!spp_writing) {
            // bug: spp_writing = false and no event
            ESP_LOGE(SPP_TAG, "tx buffer full and not spp_writing, reinit write handle=%u", handle);
            spp_writing = true;
            _write(handle);
        }
        return 0;
    }
    else {
        int free = xRingbufferGetCurFreeSize(spp_tx_buffer);
        if (spp_tx_buffer_min_free > free) spp_tx_buffer_min_free = free;
    }
    if (!spp_writing) {
        spp_writing = true;
        _write(handle);
    }

    return count;
}

size_t bt_read(uint32_t handle, void* buf, size_t count, TickType_t ticksToWait)
{
    if (handle != spp_handle) {
        if (spp_handle != 0) ESP_LOGE(SPP_TAG, "bt_read handle error %u != %u", handle, spp_handle);
        errno = EBADF;
        return -1;
    }

    size_t size = 0;
    uint8_t* data = xRingbufferReceiveUpTo(spp_rx_buffer, &size, ticksToWait, count);
    if (data == NULL || size == 0) return 0;
    if (size > count) size = count;

    memcpy(buf, data, size);
    vRingbufferReturnItem(spp_rx_buffer, data);

    return size;
}

int bt_close(uint32_t handle)
{
    if (handle != spp_handle) {
        if (spp_handle != 0) ESP_LOGE(SPP_TAG, "bt_close handle error %u != %u", handle, spp_handle);
        errno = EBADF;
        return -1;
    }

    ESP_LOGI(SPP_TAG, "bt_close handle=%u", handle);
    esp_spp_disconnect(handle);
    // _close(handle);
    return 0;
}

size_t bt_get_tx_free(uint32_t handle)
{
    if (handle != spp_handle) {
        errno = EBADF;
        return -1;
    }
    return xRingbufferGetCurFreeSize(spp_tx_buffer);
}

size_t bt_get_rx_free(uint32_t handle)
{
    if (handle != spp_handle) {
        errno = EBADF;
        return -1;
    }
    return xRingbufferGetCurFreeSize(spp_rx_buffer);
}

int bt_discard_tx_buffer(uint32_t handle)
{
    if (handle != spp_handle) {
        errno = EBADF;
        return -1;
    }

    size_t size = 0;
    // while (true) {
    //     uint8_t* data = xRingbufferReceiveUpTo(spp_tx_buffer, &size, 0, BT_SPP_RINGBUF_TX_SIZE);
    //     if (data == NULL) break;
    //     vRingbufferReturnItem(spp_tx_buffer, data);
    // }
    uint8_t* data = xRingbufferReceiveUpTo(spp_tx_buffer, &size, 0, BT_SPP_RINGBUF_TX_SIZE);
    if (data == NULL) return 0;
    vRingbufferReturnItem(spp_tx_buffer, data);
    return size;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        ESP_LOGI(SPP_TAG, "set device name: %s", SPP_DEVICE_NAME);
        esp_bt_dev_set_device_name(SPP_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT handle=%d", param->open.handle);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT addr=" ESP_BD_ADDR_STR " handle=%d", ESP_BD_ADDR_HEX(param->srv_open.rem_bda), param->srv_open.handle);
        _open(param->srv_open.handle);
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT handle=%d", param->close.handle);
        _close(param->close.handle);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle=%d", param->start.handle);
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT handle=%d", param->cl_init.handle);
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGV(SPP_TAG, "ESP_SPP_WRITE_EVT handle=%d, len=%d, cong=%d", param->write.handle, param->write.len, param->write.cong);
        // ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT handle=%d, len=%d, cong=%d", param->write.handle, param->write.len, param->write.cong);
        if (param->write.cong == 0) {
            _write(param->write.handle);
        }
        // else {
        //     ESP_LOGW(SPP_TAG, "ESP_SPP_WRITE_EVT handle=%d, len=%d, cong=%d", param->write.handle, param->write.len, param->write.cong);
        // }
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGV(SPP_TAG, "ESP_SPP_DATA_IND_EVT handle=%d len=%d",
                 param->data_ind.handle, param->data_ind.len);
        // esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
        _read(param->data_ind.handle, param->data_ind.data, param->data_ind.len);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGD(SPP_TAG, "ESP_SPP_CONG_EVT handle=%d, cong=%d", param->cong.handle, param->cong.cong);
        // ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT handle=%d, cong=%d", param->cong.handle, param->cong.cong);
        if (param->cong.cong == 0) {
            // bt_discard_tx_buffer(param->cong.handle);
            _write(param->cong.handle);
        }
        else {
            ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT handle=%d, cong=%d", param->cong.handle, param->cong.cong);
        }
        break;
    default:
        ESP_LOGI(SPP_TAG, "ESP_SPP event: %d", event);
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CB event: %d", event);
        break;
    }
    return;
}

int bt_init(bt_cb_t bt_open_cb, bt_cb_t bt_close_cb)
{
    esp_err_t ret;

    ESP_LOGI(SPP_TAG, "Initialize bt");
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    spp_open_cb = bt_open_cb;
    spp_close_cb = bt_close_cb;
    return 0;
}

// -----------------------------  bt cookie io functions  -----------------------------

ssize_t _bt_read(void* cookie, char* buf, size_t size)
{
    // ESP_LOGW(TAG, "_bt_read %u", (uint32_t)cookie);
    return bt_read((uint32_t)cookie, buf, size, portMAX_DELAY);
}

ssize_t _bt_write(void* cookie, const char* buf, size_t size)
{
    return bt_write((uint32_t)cookie, buf, size);
}

int _bt_close(void *cookie)
{
    return bt_close((uint32_t)cookie);
}

static const cookie_io_functions_t bt_cookie_func = {
    .read = _bt_read,
    .write = _bt_write,
    .seek = NULL,
    .close = _bt_close};

FILE *bt_fopen(uint32_t handle, const char *mode)
{
    return fopencookie((void*)handle, mode, bt_cookie_func);
}
