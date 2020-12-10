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

#pragma once

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

typedef void (*bt_cb_t)(uint32_t handle);

int bt_init(bt_cb_t bt_open_cb, bt_cb_t bt_close_cb);
size_t bt_write(uint32_t handle, const void *buf, size_t count);
size_t bt_read(uint32_t handle, void *buf, size_t count, TickType_t ticksToWait);
int bt_close(uint32_t handle);

// size_t bt_get_tx_free(uint32_t handle);
// size_t bt_get_rx_free(uint32_t handle);
int bt_discard_tx_buffer(uint32_t handle);

FILE *bt_fopen(uint32_t handle, const char *mode);
