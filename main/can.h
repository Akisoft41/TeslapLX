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

#include "hal/can_types.h"

typedef struct {
    can_message_t msg;
    uint64_t timestamp;
} can_message_timestamp_t;

typedef void (*can_rx_cb_t)(can_message_t* rx_msg, uint64_t timestamp, void* ctx);

bool can_init();

RingbufHandle_t can_ringbuf_new(size_t itemNum);
bool can_ringbuf_del(RingbufHandle_t ringbuf);

void can_simu_start();
void can_simu_stop();
