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

#define TESLAP_HOSTNAME "TeslapLX"

typedef void (*wifi_cb_t)(int sock);

void wifi_init(wifi_cb_t cb);

bool wifi_status();
bool wifi_sta(char* ssid, char* password);
bool wifi_ap(char* ssid, char* password);
bool wifi_stop();
bool wifi_scan();

FILE *tcp_fopen(int sock, const char *mode);
