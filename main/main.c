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

#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#include "elog.h"
#include "uart.h"
#include "bt.h"
#include "can.h"
#include "elm.h"
#include "wifi.h"
#include "httpd.h"

static const char* TAG = "TeslapLX";


// -----------------------------  bt  -----------------------------

void bt_task(void* param)
{
    uint32_t handle = (uint32_t)param;
    ESP_LOGI(TAG, "bt task started handle=%u", handle);

    stdin  = bt_fopen(handle, "r");
    stdout = bt_fopen(handle, "w");

    setvbuf(stdin, NULL, _IONBF, 0);

    elm_do("elm-bt");

    ESP_LOGI(TAG, "bt task ended handle=%u", handle);
    fclose(stdin);
    fclose(stdout);
    vTaskDelete(NULL);
}

void bt_open_cb(uint32_t handle)
{
    ESP_LOGI(TAG, "bt start handle=%u", handle);
    xTaskCreatePinnedToCore(bt_task, "elm-bt", 8 * 1024, (void*)handle, 5, NULL, 0);
}

void bt_close_cb(uint32_t handle)
{
    ESP_LOGI(TAG, "bt stop handle=%u", handle);
}

// -----------------------------  tcp  -----------------------------

void tcp_task(void* param)
{
    int soc = (int)param;
    ESP_LOGI(TAG, "tcp task started socket=%u", soc);


    stdin  = tcp_fopen(soc, "r");
    stdout = tcp_fopen(soc, "w");

    setvbuf(stdin, NULL, _IONBF, 0);

    elm_do("elm-tcp");

    ESP_LOGI(TAG, "tcp task ended socket=%u", soc);
    fclose(stdin);
    fclose(stdout);
}

void tcp_open_cb(int soc)
{
    ESP_LOGI(TAG, "tcp start socket=%u", soc);
    tcp_task((void*)soc);
    //CONFIG_ESP32_WIFI_TASK_PINNED_TO_CORE_0
    // xTaskCreatePinnedToCore(bt_task, "elm-bt", 8 * 1024, (void*)soc, 5, NULL, 0);
}


// -----------------------------  ws  -----------------------------

void ws_task(void* param)
{
    int fd = (int)param;
    ESP_LOGI(TAG, "ws task started fd=%u", fd);

    stdin  = net_httpd_ws_fopen(fd, "r");
    stdout = net_httpd_ws_fopen(fd, "w");

    setvbuf(stdin, NULL, _IONBF, 0);

    elm_do("elm-ws");

    ESP_LOGI(TAG, "ws task ended fd=%u", fd);
    fclose(stdin);
    fclose(stdout);
    vTaskDelete(NULL);
}

void ws_open_cb(int fd)
{
    ESP_LOGI(TAG, "ws start handle=%u", fd);
    xTaskCreatePinnedToCore(ws_task, "elm-ws", 8 * 1024, (void*)fd, 5, NULL, 0);
}

void ws_close_cb(int fd)
{
    ESP_LOGI(TAG, "ws stop handle=%u", fd);
}

// -----------------------------  uart  -----------------------------

void uart_task(void* param)
{
    uart_port_t port = (uart_port_t)param;
    ESP_LOGI(TAG, "uart task started port=%u", port);

    ESP_ERROR_CHECK(uart_driver_install(port, 256, 2048, 0, NULL, 0));

    stdin  = uart_fopen(port, "r");
    stdout = uart_fopen(port, "w");

    elog_out_set(stdout);
    elog_level_set("*", ESP_LOG_WARN);

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    elm_do("elm-uart");

    ESP_LOGI(TAG, "uart task ended port=%u", port);
    fclose(stdin);
    fclose(stdout);
}

void uart_start(uart_port_t port)
{
    ESP_LOGI(TAG, "uart start port=%u", port);
    uart_task((void*)port);
}


// -----------------------------  app_main  -----------------------------

void app_main()
{
    // init
    log_init();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    can_init();

    bt_init(bt_open_cb, bt_close_cb);

    wifi_init(tcp_open_cb);

    net_httpd_ws_init(ws_open_cb, ws_close_cb);

    // serial elm
    while (true) {
        uart_start(UART_NUM_0);
    }
}
