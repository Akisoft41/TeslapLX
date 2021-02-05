// Copyright 2021 Pascal Akermann
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"

#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"

#include "cJSON.h"
#include "esp_http_server.h"

#include "sdkconfig.h"

#include "httpd.h"

static const char* TAG = "net-httpd";
static const char* TAG_WS = "net-httpd-ws";

#define HTTPD_WS_RINGBUF_RX_SIZE 256

static RingbufHandle_t httpd_ws_rx_buffer = NULL;
static int httpd_ws_fd = 0;
static httpd_handle_t httpd_ws_hd = NULL;
static net_httpd_cb_t httpd_ws_open_cb = NULL;
static net_httpd_cb_t httpd_ws_close_cb = NULL;

static httpd_handle_t server = NULL;

// -----------------------------  _httpd_handler_ws  -----------------------------

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
    httpd_ws_type_t type;
    char* data;
    int len;
};

void ws_async_send(void* arg)
{
    struct async_resp_arg* resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)resp_arg->data;
    ws_pkt.len = resp_arg->len;
    ws_pkt.type = resp_arg->type;

    esp_err_t err = httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WS, "httpd_ws_send_frame_async failed with %d", err);
    }

    free(resp_arg->data);
    free(resp_arg);
}

void _httpd_ws_open(httpd_req_t* req)
{
    httpd_ws_fd = httpd_req_to_sockfd(req);
    httpd_ws_hd = req->handle;
    if (httpd_ws_rx_buffer) vRingbufferDelete(httpd_ws_rx_buffer);
    httpd_ws_rx_buffer = xRingbufferCreate(HTTPD_WS_RINGBUF_RX_SIZE, RINGBUF_TYPE_BYTEBUF);
    httpd_ws_open_cb(httpd_ws_fd);
}

void _httpd_ws_close()
{
    if (httpd_ws_fd) httpd_ws_close_cb(httpd_ws_fd);
    httpd_ws_fd = 0;
    httpd_ws_hd = NULL;
    if (httpd_ws_rx_buffer) vRingbufferDelete(httpd_ws_rx_buffer);
    httpd_ws_rx_buffer = NULL;
}

esp_err_t _httpd_handler_ws(httpd_req_t* req)
{
    if (httpd_ws_fd == 0) {
        _httpd_ws_open(req);
    }

    uint8_t buf[128] = {0};
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 128);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_WS, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG_WS, "Got packet type %d with message: %s", ws_pkt.type, ws_pkt.payload);

    BaseType_t done;
    switch (ws_pkt.type) {

    case HTTPD_WS_TYPE_TEXT:
        done = xRingbufferSend(httpd_ws_rx_buffer, ws_pkt.payload, ws_pkt.len, 0);
        if (done == 0) ESP_LOGW(TAG_WS, "rx buffer full");
        done = xRingbufferSend(httpd_ws_rx_buffer, "\r", 1, 0);
        if (done == 0) ESP_LOGW(TAG_WS, "rx buffer full");
        return ESP_OK;

    case HTTPD_WS_TYPE_CLOSE:
        _httpd_ws_close();
        return ESP_OK;

    default:
        break;
    }
    return ESP_ERR_HTTPD_INVALID_REQ;
}

static const httpd_uri_t _httpd_uri_ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = _httpd_handler_ws,
    .user_ctx = NULL,
    .is_websocket = true};

// -----------------------------  ws cookie io functions  -----------------------------

ssize_t _ws_read(void* cookie, char* buf, size_t max_size)
{
    int fd = (int)cookie;
    if (fd != httpd_ws_fd) {
        if (httpd_ws_fd != 0) ESP_LOGE(TAG_WS, "_ws_read fd error %u != %u", fd, httpd_ws_fd);
        errno = EBADF;
        return -1;
    }

    size_t size = 0;
    uint8_t* data = xRingbufferReceiveUpTo(httpd_ws_rx_buffer, &size, portMAX_DELAY, max_size);
    if (data == NULL || size == 0) return 0;
    if (size > max_size) size = max_size;

    memcpy(buf, data, size);
    vRingbufferReturnItem(httpd_ws_rx_buffer, data);

    return size;
}

ssize_t _ws_write(void* cookie, const char* buf, size_t size)
{
    int fd = (int)cookie;
    if (fd != httpd_ws_fd) {
        if (httpd_ws_fd != 0) ESP_LOGE(TAG_WS, "_ws_write fd error %u != %u", fd, httpd_ws_fd);
        errno = EBADF;
        return -1;
    }

    struct async_resp_arg* resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = httpd_ws_hd;
    resp_arg->fd = httpd_ws_fd;
    resp_arg->type = HTTPD_WS_TYPE_TEXT;
    resp_arg->len = size;
    resp_arg->data = malloc(size);
    memcpy(resp_arg->data, buf, size);
    httpd_queue_work(server, ws_async_send, resp_arg);

    return size;
}

int _ws_close(void* cookie)
{
    int fd = (int)cookie;
    if (fd != httpd_ws_fd) {
        if (httpd_ws_fd != 0) ESP_LOGE(TAG_WS, "_ws_close fd error %u != %u", fd, httpd_ws_fd);
        errno = EBADF;
        return -1;
    }

    struct async_resp_arg* resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = httpd_ws_hd;
    resp_arg->fd = httpd_ws_fd;
    resp_arg->type = HTTPD_WS_TYPE_CLOSE;
    resp_arg->len = 0;
    resp_arg->data = NULL;
    httpd_queue_work(server, ws_async_send, resp_arg);

    _httpd_ws_close();
    return 0;
}

static const cookie_io_functions_t ws_cookie_func = {
    .read = _ws_read,
    .write = _ws_write,
    .seek = NULL,
    .close = _ws_close};

FILE* net_httpd_ws_fopen(int fd, const char* mode)
{
    return fopencookie((void*)fd, mode, ws_cookie_func);
}

// -----------------------------  _httpd_handler_get_system_info  -----------------------------

/* Simple handler for getting system info */
static esp_err_t _httpd_handler_get_system_info(httpd_req_t* req)
{
    httpd_resp_set_type(req, HTTPD_TYPE_JSON);
    cJSON* root = cJSON_CreateObject();
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    cJSON_AddStringToObject(root, "version", IDF_VER);
    cJSON_AddNumberToObject(root, "cores", chip_info.cores);
    cJSON_AddNumberToObject(root, "revision", chip_info.revision);
    const char* sys_info = cJSON_Print(root);
    httpd_resp_sendstr(req, sys_info);
    free((void*)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/* URI handler for fetching system info */
static const httpd_uri_t _httpd_uri_get_system_info = {
    .uri = "/api/system/info",
    .method = HTTP_GET,
    .handler = _httpd_handler_get_system_info,
    .user_ctx = NULL};

// -----------------------------  net_httpd_start/stop  -----------------------------

bool net_httpd_start()
{
    esp_err_t err;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error starting httpd server! %d", err);
        return false;
    }

    // Registering the ws handler
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &_httpd_uri_ws);
    httpd_register_uri_handler(server, &_httpd_uri_get_system_info);

    return true;
}

bool net_httpd_stop()
{
    if (server == NULL) return true;

    return httpd_stop(server) == ESP_OK;
}

int net_httpd_ws_init(net_httpd_cb_t ws_open_cb, net_httpd_cb_t ws_close_cb)
{
    httpd_ws_open_cb = ws_open_cb;
    httpd_ws_close_cb = ws_close_cb;
    return 0;
}
