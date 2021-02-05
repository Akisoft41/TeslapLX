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

#include "esp_log.h"
#include "esp_err.h"

#include "elog.h"

static const char* TAG = "elog";

static FILE* _elog_stdout = NULL;
// static esp_log_level_t _elog_level;

esp_err_t esp_wifi_internal_set_log_level(int level);
esp_err_t esp_wifi_internal_set_log_mod(int module, uint32_t submodule, bool enable);
// esp_err_t esp_wifi_internal_set_log_level(wifi_log_level_t level);
// esp_err_t esp_wifi_internal_set_log_mod(wifi_log_module_t module, uint32_t submodule, bool enable);

// -----------------------------  elog  -----------------------------

void elog_out_set(FILE* out)
{
    _elog_stdout = out;
    stderr = out;
}

void elog_level_set(const char* tag, esp_log_level_t level)
{
    if (tag == NULL || *tag == 0) tag = "*";
    ESP_LOGI(TAG, "set log level %i for tag '%s'", level, tag);
    esp_log_level_set(tag, level);
    if (strcmp(tag, "*") == 0 || strcmp(tag, "wifi") == 0) {
        // _elog_level = level;
        if (level > 0) level--;
        esp_wifi_internal_set_log_level(level);
    }
}

// -----------------------------  elog_init  -----------------------------

int _elog_vprintf(const char* format, va_list arg)
{
    FILE* out = _elog_stdout;
    if (out == NULL) out = stderr;
    int len = vfprintf(out, format, arg);
    int err = fflush(out);
    if (len < 0 || err < 0) {
        out = _GLOBAL_REENT->_stderr;
        len = vfprintf(out, format, arg);
        err = fflush(out);
        _elog_stdout = out;
    }
    return len;
}

void log_init()
{
    ESP_LOGI(TAG, "Initialize log");
    esp_log_set_vprintf(_elog_vprintf);
    elog_level_set("*", ESP_LOG_INFO);
}
