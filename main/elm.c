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
#include <sys/stat.h>

#include "esp32/himem.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "hal/can_types.h"
#include "sdkconfig.h"

// #define LOG_LOCAL_LEVEL 4 // ESP_LOG_DEBUG
#include "esp_log.h"

#include "can.h"
#include "elm.h"
#include "elog.h"
#include "ota.h"
#include "wifi.h"

static const char* ELM_TAG = "elm";

#define ELM_BUFFER_LEN 128
#define ELM_ST_FILTER_LEN 100

#define ELM_MONITOR_TASK_RUN_PRIO 8
#define ELM_MONITOR_TASK_RUN_CORE 0 // tskNO_AFFINITY

typedef struct {
    uint32_t pattern;
    uint32_t mask;
} elm_filter_t;

typedef struct elm_globals_s {
    const char* elm_tag;

    bool elm_echo;
    bool elm_linefeed;
    bool elm_headers;
    bool elm_spaces;
    bool elm_dlc;
    uint32_t elm_timeout;
    char* elm_device_identifier;
    char elm_previous_cmd[ELM_BUFFER_LEN];

    // futur use
    bool elm_memory;
    int elm_adaptive;
    bool elm_can_auto_format;
    bool elm_can_flow_control;
    bool elm_can_silent_mode;
    bool elm_long_message;
    char elm_protocol;
    bool elm_protocol_auto;

    // monitor
    bool elm_monitor;
    bool elm_monitor_task_run;
    FILE* elm_monitor_out;

    // filter
    elm_filter_t elm_filter;
    elm_filter_t pass_filter[ELM_ST_FILTER_LEN];
    elm_filter_t block_filter[ELM_ST_FILTER_LEN];

} elm_globals_t;

#define G (*g)
#define TAG G.elm_tag

// -----------------------------  elm327  -----------------------------

/// There are 11 or so protocols, but fewer types of header layout
enum elm_protocol_header_type_t {
    OBDHEADER_NULL, // Not recognised headers
    OBDHEADER_J1850PWM,
    OBDHEADER_J1850VPW,
    OBDHEADER_14230,
    OBDHEADER_CAN29,
    OBDHEADER_CAN11
};

/// Each OBDII Protocol has a number and description
typedef struct
{
    const char* num;
    const char* desc;
    const enum elm_protocol_header_type_t header_type;
} elm_protocol_t;

/// All the protocols I know or care about
/** Borrowed from the ELM327 datasheet */
static const elm_protocol_t elm_protocols[] = {
    {"0", "Automatic", OBDHEADER_NULL},
    {"1", "SAE J1850 PWM", OBDHEADER_J1850PWM},         // 41.6 Kbaud
    {"2", "SAE J1850 VPW", OBDHEADER_J1850VPW},         // 10.4 Kbaud
    {"3", "ISO 9141-2", OBDHEADER_J1850VPW},            // 5 baud init, 10.4 Kbaud
    {"4", "ISO 14230-4 (KWP 5BAUD)", OBDHEADER_14230},  // 5 baud init, 10.4 Kbaud
    {"5", "ISO 14230-4 (KWP FAST)", OBDHEADER_14230},   // fast init, 10.4 Kbaud
    {"6", "ISO 15765-4 (CAN 11/500)", OBDHEADER_CAN11}, // 11 bit ID, 500 Kbaud
    {"7", "ISO 15765-4 (CAN 29/500)", OBDHEADER_CAN29}, // 29 bit ID, 500 Kbaud
    {"8", "ISO 15765-4 (CAN 11/250)", OBDHEADER_CAN11}, // 11 bit ID, 250 Kbaud
    {"9", "ISO 15765-4 (CAN 29/250)", OBDHEADER_CAN29}, // 29 bit ID, 250 Kbaud
    {"A", "SAE J1939 (CAN 29/250)", OBDHEADER_CAN29},   // 29 bit ID, 250* Kbaud
    {"B", "USER1 CAN", OBDHEADER_CAN11},                // 11* bit ID, 125* Kbaud
    {"C", "USER2 CAN", OBDHEADER_CAN11},                // 11* bit ID, 50* Kbaud
};

/// Default protocol. Looked up early on - better match something from the above list
#define ELM_DEFAULT_PROTOCOL '0'

// "OBDSim default Version String (ATI, ATZ)")
#define ELM_VERSION_STRING "ELM327 v1.3a Teslap"
#define ST_VERSION_STRING "STN1110 r0.1 Teslap"

// "OBDSim default Device String (AT@1)")
#define ELM_DEVICE_STRING "Espnux"

/// This is the elm prompt
#define ELM_PROMPT ">"

/// Default hide headers
#define ELM_HEADERS true

/// Default show spaces
#define ELM_SPACES true

/// Default echo
#define ELM_ECHO true

/// Default linefeed
#define ELM_LINEFEED true

/// Default memory
#define ELM_MEMORY true

/// Default timeout, milliseconds
#define ELM_TIMEOUT 5000

/// Adaptive timing [0,1,2]
#define ELM_ADAPTIVETIMING 1

/// DLC Dispaly
#define ELM_DISPLAYDLC false

///
#define ELM_CAN_AUTO_FORMAT false

///
#define ELM_CAN_FLOW_CONTROL true

///
#define ELM_CAN_SILENT_MODE true

///
#define ELM_LONG_MESSAGE false

/// ELM "don't know" prompt
#define ELM_QUERY_PROMPT "?"

/// ELM "OK" prompt
#define ELM_OK_PROMPT "OK"

/// ELM "ERROR" prompt
#define ELM_ERROR_PROMPT "ERROR"

/// ELM "NO DATA" prompt
#define ELM_NODATA_PROMPT "NO DATA"

/// Print out benchmarks every this often [seconds]
#define OBDSIM_BENCHMARKTIME 10

/// Default car battery voltage
#define OBDSIM_BATTERYV 12.0

/// Hardcode maximum number of ECUs/generators
#define OBDSIM_MAXECUS 6

/// Max number of frames for freeze frame
#define OBDSIM_MAXFREEZEFRAMES 5

#define NEWLINE_CRLF "\r\n"
#define NEWLINE_CR "\r"
#define ELM_NEWLINE(g) (G.elm_linefeed ? NEWLINE_CRLF : NEWLINE_CR)
#define ELM_STR_NEWLINE "%s"
#define ELM_SPACE (G.elm_spaces ? " " : "")
#define ELM_STR_SPACE "%s"

#define TO_UPPER(c) ((c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c)

// -----------------------------  def  -----------------------------

void elm_monitor_start(elm_globals_t* g);
void elm_monitor_stop(elm_globals_t* g);

// -----------------------------  st_filter  -----------------------------

void elm_filter_clear(elm_globals_t* g, elm_filter_t* filter)
{
    for (int i = 0; i < ELM_ST_FILTER_LEN; i++) {
        if (filter[i].mask == 0) break;
        filter[i].pattern = 0;
        filter[i].mask = 0;
    }
}

bool elm_filter_add(elm_globals_t* g, elm_filter_t* filter, uint32_t pattern, uint32_t mask)
{
    for (int i = 0; i < ELM_ST_FILTER_LEN; i++) {
        if (filter[i].mask == 0) {
            filter[i].pattern = pattern;
            filter[i].mask = mask;
            return true;
        }
    }
    return false;
}

void elm_filter_log(elm_globals_t* g, elm_filter_t* filter)
{
    ESP_LOGI(TAG, "  pattern=%03X mask=%03X", filter->pattern, filter->mask);
}

void elm_filters_log(elm_globals_t* g, elm_filter_t* filter)
{
    for (int i = 0; i < ELM_ST_FILTER_LEN; i++) {
        if (filter[i].mask == 0) return;
        elm_filter_log(g, filter);
    }
    return;
}

bool elm_filter_test(elm_globals_t* g, uint32_t id, uint32_t us)
{
    if (id == 0) return false;

    // test AT filter
    if ((id & G.elm_filter.mask) != (G.elm_filter.pattern & G.elm_filter.mask)) return false;

    // test ST filters
    bool ok = true;

    if (G.pass_filter[0].mask != 0) {
        // test pass filter
        ok = false;
        for (int i = 0; i < ELM_ST_FILTER_LEN; i++) {
            if (G.pass_filter[i].mask == 0) break;
            if ((id & G.pass_filter[i].mask) == (G.pass_filter[i].pattern & G.pass_filter[i].mask)) {
                ok = true;
                break;
            }
        }
    }

    if (ok && G.block_filter[0].mask != 0) {
        // test block filter
        for (int i = 0; i < ELM_ST_FILTER_LEN; i++) {
            if (G.block_filter[i].mask == 0) break;
            if ((id & G.block_filter[i].mask) == (G.block_filter[i].pattern & G.block_filter[i].mask)) {
                ok = false;
                break;
            }
        }
    }

    return ok;
}

// -----------------------------  util  -----------------------------

void elm_reset(elm_globals_t* g)
{
    G.elm_echo = ELM_ECHO;
    G.elm_linefeed = ELM_LINEFEED;
    G.elm_headers = ELM_HEADERS;
    G.elm_spaces = ELM_SPACES;
    G.elm_dlc = ELM_DISPLAYDLC;
    G.elm_timeout = ELM_TIMEOUT;
    G.elm_previous_cmd[0] = 0;

    G.elm_memory = ELM_MEMORY;
    G.elm_adaptive = ELM_ADAPTIVETIMING;
    G.elm_can_auto_format = ELM_CAN_AUTO_FORMAT;
    G.elm_can_flow_control = ELM_CAN_FLOW_CONTROL;
    G.elm_can_silent_mode = ELM_CAN_SILENT_MODE;
    G.elm_long_message = ELM_LONG_MESSAGE;
    G.elm_protocol = ELM_DEFAULT_PROTOCOL;
    G.elm_protocol_auto = true;

    G.elm_monitor = false;

    G.elm_filter.pattern = 0;
    G.elm_filter.mask = 0;
    elm_filter_clear(g, G.pass_filter);
    elm_filter_clear(g, G.block_filter);
}

void elm_newline(elm_globals_t* g)
{
    printf(ELM_NEWLINE(g));
    fflush(stdout);
}

void elm_writeln(elm_globals_t* g, const char* str)
{
    if (str) printf(str);
    elm_newline(g);
}

void elm_write_prompt(elm_globals_t* g)
{
    printf(ELM_PROMPT);
    fflush(stdout);
}

void elm_write_ok(elm_globals_t* g)
{
    printf(ELM_OK_PROMPT);
    elm_newline(g);
}

void elm_write_error(elm_globals_t* g)
{
    printf(ELM_ERROR_PROMPT);
    elm_newline(g);
}

void elm_write_ok_error(elm_globals_t* g, bool ok)
{
    if (ok)
        elm_write_ok(g);
    else
        elm_write_error(g);
}

elm_protocol_t elm_get_protocol(char protocol_num)
{
    int p = 0;
    for (int i = 0; i < sizeof(elm_protocols) / sizeof(*elm_protocols); i++) {
        if (elm_protocols[i].num[0] == protocol_num) {
            p = i;
            break;
        }
    }
    return elm_protocols[p];
}

char* elm_read_hexa(char* c, uint32_t* h)
{
    *h = 0;
    while (*c) {
        if (*c >= '0' && *c <= '9')
            *h = (*h << 4) + (*c - '0');
        else if (*c >= 'A' && *c <= 'F')
            *h = (*h << 4) + (*c - 'A' + 10);
        else if (*c >= 'a' && *c <= 'f')
            *h = (*h << 4) + (*c - 'a' + 10);
        else
            break;
        c++;
    }
    return c;
}

char* elm_read_str(char* c, char** str)
{
    while (*c == ' ')
        c++;
    if (*c == '"') {
        c++;
        *str = c;
        while (*c != 0 && *c != '"')
            c++;
    }
    else if (*c == '\'') {
        c++;
        *str = c;
        while (*c != 0 && *c != '\'')
            c++;
    }
    else {
        *str = c;
        while (*c != 0 && *c != ' ')
            c++;
    }
    if (*c != 0) {
        *c++ = 0;
    }
    return c;
}

// -----------------------------  cmd_ps  -----------------------------

int _cmd_ps_cmp(const void* a, const void* b)
{
    const TaskStatus_t* pa = a;
    const TaskStatus_t* pb = b;
    if (pa->xTaskNumber > pb->xTaskNumber) return 1;
    if (pa->xTaskNumber < pb->xTaskNumber) return -1;
    return 0;
}

int cmd_ps()
{
    static uint32_t prev_totalRunTime;
    static TaskStatus_t* prev_stats = NULL;

    uint32_t n = uxTaskGetNumberOfTasks();
    TaskStatus_t* stats = malloc(n * sizeof(TaskStatus_t));
    if (stats == NULL) {
        fprintf(stderr, "ps: no mem\r\n");
        return 1;
    }

    uint32_t totalRunTime;
    uint32_t m = uxTaskGetSystemState(stats, n, &totalRunTime);
    if (m < n) {
        fprintf(stderr, "ps: error getting status\r\n");
        return 1;
    }

    qsort(stats, n, sizeof(TaskStatus_t), _cmd_ps_cmp);

    static const char* state_char = "*RBSD";
    printf("PID  STAT  PRIO    HWM  CORE  LAST  TOTAL  NAME\r\n");
    float t = (float)totalRunTime / 100.0f;
    float t_last = (float)(totalRunTime - prev_totalRunTime) / 100.0f;
    if (t == 0) t = 1;
    if (t_last == 0) t_last = 1;

    for (int i = 0; i < n; i++) {

        int i_last = 0;
        while (prev_stats != NULL && i_last >= 0 && stats[i].xTaskNumber != prev_stats[i_last].xTaskNumber) {
            i_last++;
            if (i_last >= n) i_last = -1;
        }

        printf("%3u    %c   %3u   %5u   %2hd %5.1f%% %5.1f%%  %s\r\n",
               stats[i].xTaskNumber,
               state_char[stats[i].eCurrentState],
               stats[i].uxCurrentPriority,
               stats[i].usStackHighWaterMark,
               (int)stats[i].xCoreID,
               (prev_stats != NULL && i_last >= 0) ? (float)(stats[i].ulRunTimeCounter - prev_stats[i_last].ulRunTimeCounter) / t_last : (float)stats[i].ulRunTimeCounter / t,
               (float)stats[i].ulRunTimeCounter / t,
               stats[i].pcTaskName);
    }

    free(prev_stats);
    prev_totalRunTime = totalRunTime;
    prev_stats = stats;

    return 0;
}

// -----------------------------  cmd_free  -----------------------------

void _cmd_free_print_info(const char* name, multi_heap_info_t* info)
{
    printf("%-10s %10u %10u %10u %10u %10u\r\n",
           name,
           info->total_allocated_bytes + info->total_free_bytes,
           info->total_allocated_bytes,
           info->total_free_bytes,
           info->largest_free_block,
           info->minimum_free_bytes);
}

// void himem_get_info(multi_heap_info_t* info)
// {
//     memset(info, 0, sizeof(*info));
//     info->total_free_bytes = esp_himem_get_free_size();
//     info->total_allocated_bytes = esp_himem_get_phys_size() - info->total_free_bytes;
// }

int cmd_free()
{
    multi_heap_info_t multi_heap_info;

    printf("                total       used       free    largest   life-min\r\n");
    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_DEFAULT);
    _cmd_free_print_info("DEFAULT:", &multi_heap_info);
    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_INTERNAL);
    _cmd_free_print_info("INTERNAL:", &multi_heap_info);
    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_SPIRAM);
    _cmd_free_print_info("SPIRAM:", &multi_heap_info);
    // himem_get_info(&multi_heap_info);
    // _cmd_free_print_info("HIMEM:", &multi_heap_info);

    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_EXEC);
    _cmd_free_print_info("EXEC:", &multi_heap_info);
    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_32BIT);
    _cmd_free_print_info("32BIT:", &multi_heap_info);
    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_8BIT);
    _cmd_free_print_info("8BIT:", &multi_heap_info);
    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_DMA);
    _cmd_free_print_info("DMA:", &multi_heap_info);
    heap_caps_get_info(&multi_heap_info, MALLOC_CAP_INVALID);
    _cmd_free_print_info("INVALID:", &multi_heap_info);

    return 0;
}

// -----------------------------  elm_do_cmd  -----------------------------

void elm_do_cmd(elm_globals_t* g, char* cmd)
{
    if (cmd == NULL) return;
    ESP_LOGD(TAG, "Do cmd: '%s'", cmd);

    // Previous command v1.0
    if (*cmd == 0) {
        cmd = G.elm_previous_cmd;
    }
    else {
        strcpy(G.elm_previous_cmd, cmd);
    }

    while (*cmd == ' ')
        cmd++;

    // do shell commands

    if (strcasecmp(cmd, "REBOOT") == 0 || strcasecmp(cmd, "RESTART") == 0) {
        esp_restart();
        return;
    }
    if (strcasecmp(cmd, "PS") == 0) {
        printf("\r\n");
        cmd_ps();
        return;
    }
    else if (strcasecmp(cmd, "FREE") == 0) {
        printf("\r\n");
        cmd_free();
        return;
    }
    else if (strncasecmp(cmd, "ELOG", 4) == 0) {
        cmd += 4;
        while (*cmd == ' ')
            cmd++;
        uint32_t level = ESP_LOG_INFO;
        char* tag = "*";
        if (*cmd != 0) cmd = elm_read_hexa(cmd, &level);
        while (*cmd == ' ')
            cmd++;
        if (*cmd != 0) tag = cmd;
        elog_out_set(stdout);
        elog_level_set(tag, level);
        return;
    }
    else if (strncasecmp(cmd, "SIMU", 4) == 0) {
        cmd += 4;
        while (*cmd == ' ')
            cmd++;
        if (strncasecmp(cmd, "START", 3) == 0) {
            can_simu_start();
            return;
        }
        else if (strncasecmp(cmd, "STOP", 3) == 0) {
            can_simu_stop();
            return;
        }
    }
    else if (strncasecmp(cmd, "WIFI", 4) == 0) {
        cmd += 4;
        while (*cmd == ' ')
            cmd++;

        if (*cmd == 0) {
            printf("\r\n");
            wifi_status();
            return;
        }
        else if (strncasecmp(cmd, "STA", 3) == 0) {
            cmd += 3;
            while (*cmd == ' ')
                cmd++;
            if (*cmd == 0) {
                // elm_write_ok_error(g, wifi_sta_disc(false));
                return;
            }

            char* ssid;
            char* pwd;
            cmd = elm_read_str(cmd, &ssid);
            cmd = elm_read_str(cmd, &pwd);
            elm_write_ok_error(g, wifi_sta(ssid, pwd));
            G.elm_previous_cmd[0] = 0;
            return;
        }
        else if (strncasecmp(cmd, "AP", 2) == 0) {
            cmd += 2;
            while (*cmd == ' ')
                cmd++;
            if (*cmd == 0) return;

            char* ssid;
            char* pwd;
            cmd = elm_read_str(cmd, &ssid);
            cmd = elm_read_str(cmd, &pwd);
            ESP_LOGI(TAG, "set wifi ap ssid='%s' pwd='%s'", ssid, pwd);
            elm_write_ok_error(g, wifi_ap(ssid, pwd));
            G.elm_previous_cmd[0] = 0;
            return;
        }
        else if (strncasecmp(cmd, "STOP", 4) == 0) {
            elm_write_ok_error(g, wifi_stop());
            G.elm_previous_cmd[0] = 0;
            return;
        }
        else if (strncasecmp(cmd, "SCAN", 4) == 0) {
            printf("\r\n");
            wifi_scan();
            return;
        }
    }
    else if (strncasecmp(cmd, "OTA", 3) == 0) {
        cmd += 3;
        while (*cmd == ' ')
            cmd++;
        if (*cmd == 0) {
            ota_info();
            return;
        }
        else {
            ota_update(cmd);
            return;
        }
    }

    // do ST commands

    else if (strncasecmp(cmd, "ST", 2) == 0) {

        char* c = cmd + 2;

        if (strcasecmp(c, "DI") == 0) {
            ESP_LOGI(TAG, "%s ->  %s", cmd, ST_VERSION_STRING);
            elm_writeln(g, ST_VERSION_STRING);
            return;
        }
        if (strcasecmp(c, "F") == 0) {
            ESP_LOGI(TAG, "elm filter:");
            elm_filters_log(g, &G.elm_filter);
            ESP_LOGI(TAG, "pass filters:");
            elm_filters_log(g, G.pass_filter);
            ESP_LOGI(TAG, "block filters:");
            elm_filters_log(g, G.block_filter);
            return;
        }
        // Clear all filters
        if (strcasecmp(c, "FAC") == 0 || strcasecmp(c, "FCA") == 0) {
            ESP_LOGI(TAG, "%s ->  Clear all filters", cmd);
            elm_filter_clear(g, G.pass_filter);
            elm_filter_clear(g, G.block_filter);
            elm_write_ok(g);
            return;
        }
        // Add a pass filter
        if (strncasecmp(c, "FPA", 3) == 0 || strncasecmp(c, "FAP", 3) == 0) {
            c += 3;
            while (*c == ' ')
                c++;
            if (*c == 0) goto _err;

            // ESP_LOGI(TAG, "  Add a pass filter %s", c);
            uint32_t pattern;
            uint32_t mask;
            c = elm_read_hexa(c, &pattern);
            while (*c == ' ')
                c++;
            if (*c != ',') goto _err;
            c++;
            while (*c == ' ')
                c++;
            c = elm_read_hexa(c, &mask);
            ESP_LOGI(TAG, "%s ->  Add pass filter pattern=0x%03X mask=0x%03X", cmd, pattern, mask);
            bool ok = elm_filter_add(g, G.pass_filter, pattern, mask);

            if (ok)
                elm_write_ok(g);
            else
                elm_writeln(g, "??");
            return;
        }
        // Clear all pass filters
        if (strcasecmp(c, "FPC") == 0 || strcasecmp(c, "FCP") == 0) {
            ESP_LOGI(TAG, "%s ->  Clear all pass filters", cmd);
            elm_filter_clear(g, G.pass_filter);
            elm_write_ok(g);
            return;
        }
        // Add block filter
        if (strncasecmp(c, "FBA", 3) == 0 || strncasecmp(c, "FAB", 3) == 0) {
            c += 3;
            while (*c == ' ')
                c++;
            if (*c == 0) goto _err;

            uint32_t pattern;
            uint32_t mask;
            c = elm_read_hexa(c, &pattern);
            while (*c == ' ')
                c++;
            if (*c != ',') goto _err;
            c++;
            while (*c == ' ')
                c++;
            c = elm_read_hexa(c, &mask);
            ESP_LOGI(TAG, "%s ->  Add block filter pattern=0x%03X mask=0x%03X", cmd, pattern, mask);
            bool ok = elm_filter_add(g, G.block_filter, pattern, mask);

            if (ok)
                elm_write_ok(g);
            else
                elm_writeln(g, "??");
            return;
        }
        // Clear all block filters
        if (strcasecmp(c, "FBC") == 0 || strcasecmp(c, "FCB") == 0) {
            ESP_LOGI(TAG, "%s ->  Clear all block filters", cmd);
            elm_filter_clear(g, G.block_filter);
            elm_write_ok(g);
            return;
        }
        // // Add flow control filter
        // if (strncasecmp(c, "FFCA", 3) == 0 || strncasecmp(c, "FAFC", 3) == 0) {
        //     elm_write_ok(g);
        //     return;
        // }
        // // Clear all flow control filters
        // if (strcasecmp(c, "FFCC") == 0 || strcasecmp(c, "FCFC") == 0) {
        //     elm_write_ok(g);
        //     return;
        // }
        if (strcasecmp(c, "M") == 0) {
            ESP_LOGI(TAG, "%s ->  Monitor bus using current filters", cmd);
            elm_monitor_start(g);
            return;
        }
        if (strcasecmp(c, "MA") == 0) {
            ESP_LOGI(TAG, "%s ->  Monitor all messages on bus", cmd);
            elm_monitor_start(g);
            return;
        }
    }

    // do AT commands

    else if (strncasecmp(cmd, "AT", 2) == 0) {

        char* c = cmd + 2;

        if (strcasecmp(c, "@1") == 0) { // General v1.0
            ESP_LOGI(TAG, "%s ->  %s", cmd, ELM_DEVICE_STRING);
            elm_writeln(g, ELM_DEVICE_STRING);
            return;
        }
        if (strcasecmp(c, "@2") == 0) { // General v1.0
            ESP_LOGI(TAG, "%s ->  %s", cmd, G.elm_device_identifier);
            elm_writeln(g, G.elm_device_identifier);
            return;
        }
        if (strcasecmp(c, "@3") == 0) { // General v1.0
            c += 2;
            while (*c == ' ')
                c++;
            free(G.elm_device_identifier);
            G.elm_device_identifier = strdup(c);
            ESP_LOGI(TAG, "%s ->  Set device identifier to \"%s\"", cmd, G.elm_device_identifier);
            elm_write_ok(g);
            return;
        }

        if (strcasecmp(c, "AL") == 0) { // OBD v1.0
            ESP_LOGI(TAG, "%s ->  Allow Long message", cmd);
            G.elm_long_message = true;
            elm_write_ok(g);
            return;
        }

        // if (strcasecmp(c, "AR") == 0) { // OBD v1.2
        //     ESP_LOGI(TAG, "%s ->  -Automatic receive", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "AT", 2) == 0 && (c[2] == '0' || c[2] == '1' || c[2] == '2')) { // OBD v1.2
            G.elm_adaptive = c[2] - '0';
            ESP_LOGI(TAG, "%s ->  Adaptive Timing %d", cmd, G.elm_adaptive);
            elm_write_ok(g);
            return;
        }

        // if (strcasecmp(c, "BD") == 0) { // OBD v1.0
        //     ESP_LOGI(TAG, "%s ->  -Buffer Dump", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "BI") == 0) { // OBD v1.0
        //     ESP_LOGI(TAG, "%s ->  -Bypass the Initialization sequence", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "BRD", 3) == 0) { // General v1.2
        //     c += 3;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Set Baud Rate Divisor %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "BRT", 3) == 0) { // General v1.2
        //     c += 3;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Set Baud Rate Timeout %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "CAF", 3) == 0 && (c[3] == '0' || c[3] == '1')) { // CAN v1.0
            bool en = c[3] != '0';
            ESP_LOGI(TAG, "%s ->  CAN Automatic Formating %s", cmd, en ? "enable" : "disable");
            G.elm_can_auto_format = en;
            elm_write_ok(g);
            return;
        }

        // if (strcasecmp(c, "CEA") == 0) { // CAN v1.4
        //     ESP_LOGI(TAG, "%s ->  -disable CAN Extended Addressing", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "CEA", 3) == 0) { // CAN v1.4
        //     c += 3;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Set CAN Extended Address %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "CFC", 3) == 0 && (c[3] == '0' || c[3] == '1')) { // CAN v1.0
            bool en = c[3] != '0';
            ESP_LOGI(TAG, "%s ->  CAN Flow Control %s", cmd, en ? "enable" : "disable");
            G.elm_can_flow_control = en;
            elm_write_ok(g);
            return;
        }

        if (strncasecmp(c, "CF", 2) == 0) { // CAN v1.0
            c += 2;
            uint32_t h;
            elm_read_hexa(c, &h);
            ESP_LOGI(TAG, "%s ->  CAN Filter 0x%X", cmd, h);
            G.elm_filter.pattern = h;
            elm_write_ok(g);
            return;
        }

        if (strncasecmp(c, "CM", 2) == 0) { // CAN v1.0
            c += 2;
            uint32_t h;
            elm_read_hexa(c, &h);
            ESP_LOGI(TAG, "%s ->  CAN Mask 0x%X", cmd, h);
            G.elm_filter.mask = h;
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "CP", 2) == 0) { // CAN v1.0
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -CAN Priotity %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strcasecmp(c, "CRA") == 0) { // CAN v1.4b
            c += 2;
            ESP_LOGI(TAG, "%s ->  CAN reset Receive Address filters", cmd);
            G.elm_filter.pattern = 0;
            G.elm_filter.mask = 0;
            elm_write_ok(g);
            return;
        }

        if (strncasecmp(c, "CRA", 3) == 0) { // CAN v1.3
            c += 3;
            uint32_t f = 0;
            uint32_t m = 0xffffffff;
            while (*c) {
                if (*c >= '0' && *c <= '9') {
                    f = (f << 4) + (*c - '0');
                    m = (m << 4) + 0xf;
                }
                else if (*c >= 'A' && *c <= 'F') {
                    f = (f << 4) + (*c - 'A' + 10);
                    m = (m << 4) + 0xf;
                }
                else if (*c >= 'a' && *c <= 'f') {
                    f = (f << 4) + (*c - 'a' + 10);
                    m = (m << 4) + 0xf;
                }
                else if (*c == 'X' || *c == 'x') {
                    f = (f << 4);
                    m = (m << 4);
                }
                c++;
            }
            ESP_LOGI(TAG, "%s ->  CAN set Receive Address filter=0x%X mask=0x%X", cmd, f, m);
            G.elm_filter.pattern = f;
            G.elm_filter.mask = m;
            elm_write_ok(g);
            return;
        }

        if (strcasecmp(c, "CS") == 0) { // CAN v1.0
            ESP_LOGI(TAG, "%s ->  -CAN Status", cmd);
            elm_writeln(g, "STARTED");
            return;
        }

        if (strncasecmp(c, "CSM", 3) == 0 && (c[3] == '0' || c[3] == '1')) { // CAN v1.4b
            bool en = c[3] != '0';
            ESP_LOGI(TAG, "%s ->  CAN Silent Mode %s", cmd, en ? "enable" : "disable");
            G.elm_can_silent_mode = en;
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "CV", 2) == 0) { // Volts v1.0
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Calibrate the voltage to %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strcasecmp(c, "D") == 0) { // General v1.0
            ESP_LOGI(TAG, "%s ->  Set all to Defaults", cmd);
            elm_reset(g);
            elm_write_ok(g);
            return;
        }

        if (strncasecmp(c, "D", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // CAN v1.3
            bool en = c[1] != '0';
            ESP_LOGI(TAG, "%s ->  DLC display %s", cmd, en ? "enable" : "disable");
            G.elm_dlc = en;
            elm_write_ok(g);
            return;
        }

        // if (strcasecmp(c, "DM1") == 0) { // J1939 v1.2
        //     ESP_LOGI(TAG, "%s ->  -(J1939) Monitor for DM1 Message", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strcasecmp(c, "DP") == 0) { // OBD v1.0
            ESP_LOGI(TAG, "%s ->  Describe the current Protocol", cmd);
            elm_protocol_t p = elm_get_protocol(G.elm_protocol);
            printf("%s%s%s", G.elm_protocol_auto ? "Auto, " : "", p.desc, ELM_NEWLINE(g));
            return;
        }

        if (strcasecmp(c, "DPN") == 0) { // OBD v1.0
            ESP_LOGI(TAG, "%s ->  Describe the current Protocol Num", cmd);
            elm_protocol_t p = elm_get_protocol(G.elm_protocol);
            printf("%s%s%s", G.elm_protocol_auto ? "A" : "", p.num, ELM_NEWLINE(g));
            return;
        }

        if (strncasecmp(c, "E", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // General v1.0
            bool en = c[1] != '0';
            ESP_LOGI(TAG, "%s ->  Echo %s", en ? "enable" : "disable", cmd);
            G.elm_echo = en;
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "FC", 2) == 0) { // CAN v1.1
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     if (strncasecmp(c, "SD", 2) == 0) {
        //         c += 2;
        //         while (*c == ' ')
        //             c++;
        //         ESP_LOGI(TAG, "%s ->  -Flow Control Set Data to %s", cmd, c);
        //         return;
        //     }
        //     if (strncasecmp(c, "SH", 2) == 0) {
        //         c += 2;
        //         while (*c == ' ')
        //             c++;
        //         ESP_LOGI(TAG, "%s ->  -Flow Control Set Header to %s", cmd, c);
        //         return;
        //     }
        //     if (strncasecmp(c, "SM", 2) == 0) {
        //         c += 2;
        //         while (*c == ' ')
        //             c++;
        //         ESP_LOGI(TAG, "%s ->  -Flow Control Set Mode to %s", cmd, c);
        //         return;
        //     }
        // }

        // if (strcasecmp(c, "FE") == 0) { // General v1.3a
        //     ESP_LOGI(TAG, "%s ->  -Forget Events", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "FI") == 0) { // ISO v1.4
        //     ESP_LOGI(TAG, "%s ->  -perform a Fast Initiation", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "H", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // OBD v1.0
            bool en = c[1] != '0';
            ESP_LOGI(TAG, "%s ->  Headers %s", cmd, en ? "enable" : "disable");
            G.elm_headers = en;
            elm_write_ok(g);
            return;
        }

        if (strcasecmp(c, "I") == 0) { // General v1.0
            ESP_LOGI(TAG, "%s ->  %s", cmd, ELM_VERSION_STRING);
            elm_writeln(g, ELM_VERSION_STRING);
            return;
        }

        // if (strncasecmp(c, "IB", 2) == 0) { // ISO v1.0
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -set ISO Baud rate to %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "IFR", 3) == 0) { // J1850 v1.2
        //     c += 3;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -IFR value %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "IGN") == 0) { // Other v1.4
        //     ESP_LOGI(TAG, "%s ->  -read the IgnMon input level", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "IIA", 3) == 0) { // ISO v1.2
        //     c += 3;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -set ISO (slow) Init Address to %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "JE") == 0) { // J1939 v1.3
        //     ESP_LOGI(TAG, "%s ->  -use J1939 Elm data format", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "JHF0") == 0) { // J1939 v1.4b
        //     ESP_LOGI(TAG, "%s ->  -J1939 Header Formatting off", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "JHF1") == 0) { // J1939 v1.4b
        //     ESP_LOGI(TAG, "%s ->  -J1939 Header Formatting on", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "JS") == 0) { // J1939 v1.3
        //     ESP_LOGI(TAG, "%s ->  -use J1939 SEA data format", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "JTM1") == 0) { // J1939 v1.4b
        //     ESP_LOGI(TAG, "%s ->  -set J1939 Timer Multiplier to 1x", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "JTM5") == 0) { // J1939 v1.4b
        //     ESP_LOGI(TAG, "%s ->  -set J1939 Timer Multiplier to 5x", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "KW") == 0) { // ISO v1.3
        //     ESP_LOGI(TAG, "%s ->  -display the Key Words", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "KW0") == 0) { // ISO v1.2
        //     ESP_LOGI(TAG, "%s ->  -Key Word checking off", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "KW1") == 0) { // ISO v1.2
        //     ESP_LOGI(TAG, "%s ->  -Key Word checking on", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "L", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // General v1.0
            bool en = c[1] != '0';
            ESP_LOGI(TAG, "%s ->  LineFeed %s", cmd, en ? "enable" : "disable");
            G.elm_linefeed = en;
            elm_write_ok(g);
            return;
        }

        // if (strcasecmp(c, "LP") == 0) { // General v1.4
        //     ESP_LOGI(TAG, "%s ->  -Go to Low Power mode", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "M", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // General v1.0
            bool en = c[1] != '0';
            ESP_LOGI(TAG, "%s ->  Memory %s", cmd, en ? "enable" : "disable");
            G.elm_memory = en;
            elm_write_ok(g);
            return;
        }

        if (strcasecmp(c, "MA") == 0) { // OBD v1.0
            ESP_LOGI(TAG, "%s ->  Monitor All", cmd);
            elm_monitor_start(g);
            return;
        }

        // if (strncasecmp(c, "MP", 2) == 0) { // J1939 v1.2
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -(J1939) Monitor for PGN %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "MR", 2) == 0) { // ODB v1.0
            c += 2;
            uint32_t h;
            elm_read_hexa(c, &h);
            uint32_t f = G.elm_filter.pattern;
            uint32_t m = G.elm_filter.mask;
            f = (f & 0xffffff00) + (h & 0xff);
            m = (m | 0xff);
            G.elm_filter.pattern = f;
            G.elm_filter.mask = m;
            ESP_LOGI(TAG, "%s ->  Monitor for Receiver filter=0x%X mask=0x%X", cmd, f, m);
            elm_monitor_start(g);
            return;
        }

        if (strncasecmp(c, "MT", 2) == 0) { // ODB v1.0
            c += 2;
            uint32_t h;
            elm_read_hexa(c, &h);
            uint32_t f = G.elm_filter.pattern;
            uint32_t m = G.elm_filter.mask;
            f = (f & 0xff) + (h & 0xffffff00);
            m = (m | 0xffffff00);
            G.elm_filter.pattern = f;
            G.elm_filter.mask = m;
            ESP_LOGI(TAG, "%s ->  Monitor for Transmitter filter=0x%X mask=0x%X", cmd, f, m);
            elm_monitor_start(g);
            return;
        }

        // if (strncasecmp(c, "NL", 2) == 0) { // ODB v1.0
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Normal Length (7 bytes) messages", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "PB", 2) == 0) { // CAN v1.4
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -set Protocol B options and baud rate %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "PC") == 0) { // OBD v1.0
        //     ESP_LOGI(TAG, "%s ->  -Protocol Close", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "PP", 2) == 0) { // PPs v1.1
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Protocol Parameter %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "R", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // OBD v1.0
            bool en = c[1] != '0';
            ESP_LOGI(TAG, "%s ->  Response %s", cmd, en ? "enable" : "disable");
            // G.elm_response = en;
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "RA", 2) == 0) { // OBD v1.3
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -set Receiver Address to %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "RD") == 0) { // General v1.4
        //     ESP_LOGI(TAG, "%s ->  -Read the stored Data", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "RTR") == 0) { // CAN v1.3
        //     ESP_LOGI(TAG, "%s ->  -send an RTR message", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "RV") == 0) { // Volts v1.0
        //     char v[8];
        //     snprintf(v, 7, "%.1fV", OBDSIM_BATTERYV);
        //     ESP_LOGI(TAG, "%s ->  -Read the Voltage %s", cmd, v);
        //     elm_writeln(g, v);
        //     return;
        // }

        if (strncasecmp(c, "S", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // OBD v1.0
            bool en = c[1] != '0';
            ESP_LOGI(TAG, "%s ->  Spaces %s", cmd, en ? "enable" : "disable");
            G.elm_spaces = en;
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "SD", 2) == 0) { // General v1.4
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Store Data byte %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "SH", 2) == 0) { // OBD v1.0
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Set Header %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "SI") == 0) { // ISO v1.4
        //     ESP_LOGI(TAG, "%s ->  -perform a Slow Initiation", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "SP", 2) == 0) { // OBD v1.0
            c += 2;
            while (*c == ' ')
                c++;
            if (*c == 'A') {
                G.elm_protocol_auto = true;
                c++;
            }
            else
                G.elm_protocol_auto = false;
            G.elm_protocol = *c; // todo: test protocol exist
            ESP_LOGI(TAG, "%s ->  Set Protocol %s%c", cmd, G.elm_protocol_auto ? "auto " : "", G.elm_protocol);
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "SR", 2) == 0) { // OBD v1.2
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Set Receiver address to %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strcasecmp(c, "SS") == 0) { // OBD v1.4
        //     ESP_LOGI(TAG, "%s ->  -set Standard Search order (J1978)", cmd);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "ST", 2) == 0) { // OBD v1.0
            c += 2;
            uint32_t t;
            elm_read_hexa(c, &t);
            G.elm_timeout = t;
            ESP_LOGI(TAG, "%s ->  Set Timeout to %d", cmd, t);
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "SW", 2) == 0) { // ISO v1.0
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Set Wakeup interval to %s x 20 msec", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "TA", 2) == 0) { // OBD v1.4
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -Set Tester Address to %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strncasecmp(c, "TP", 2) == 0) { // OBD v1.0
            c += 2;
            while (*c == ' ')
                c++;
            if (*c == 'A') {
                G.elm_protocol_auto = true;
                c++;
            }
            else
                G.elm_protocol_auto = false;
            G.elm_protocol = *c; // todo: test protocol exist
            if (G.elm_protocol == '0')
                G.elm_protocol_auto = true;
            ESP_LOGI(TAG, "%s ->  Try Protocol %s%c", cmd, G.elm_protocol_auto ? "auto " : "", G.elm_protocol);
            elm_write_ok(g);
            return;
        }

        // if (strncasecmp(c, "V", 1) == 0 && (c[1] == '0' || c[1] == '1')) { // CAN v1.3
        //     bool en = c[1] != '0';
        //     ESP_LOGI(TAG, "%s ->  -use of Variable DLC %s", cmd, en ? "enable" : "disable");
        //     // G.elm_variable_dlc = en;
        //     elm_write_ok(g);
        //     return;
        // }

        // if (strncasecmp(c, "WM", 2) == 0) { // ISO v1.0
        //     c += 2;
        //     while (*c == ' ')
        //         c++;
        //     ESP_LOGI(TAG, "%s ->  -set the Wakeup Message to %s", cmd, c);
        //     elm_write_ok(g);
        //     return;
        // }

        if (strcasecmp(c, "WS") == 0) { // General v1.0
            ESP_LOGI(TAG, "%s ->  Warm Start", cmd);
            elm_reset(g);
            vTaskDelay(100 / 2);
            elm_writeln(g, ELM_VERSION_STRING);
            return;
        }

        if (strcasecmp(c, "Z") == 0) { // General v1.0
            ESP_LOGI(TAG, "%s ->  Reset all", cmd);
            elm_reset(g);
            vTaskDelay(100);
            elm_writeln(g, ELM_VERSION_STRING);
            return;
        }
    }

_err:
    if (*cmd) ESP_LOGW(TAG, "Unreconized command '%s'", cmd);
    elm_writeln(g, ELM_QUERY_PROMPT);
}

// -----------------------------  elm_monitor  -----------------------------

bool _elm_write_can(elm_globals_t* g, can_message_t* msg)
{
    if (G.elm_headers) {
        fprintf(G.elm_monitor_out, "%03X", msg->identifier);
        if (G.elm_spaces) fprintf(G.elm_monitor_out, " ");
    }
    if (G.elm_dlc) {
        fprintf(G.elm_monitor_out, "%02X", msg->data_length_code);
        if (G.elm_spaces) fprintf(G.elm_monitor_out, " ");
    }
    for (int i = 0; i < msg->data_length_code; i++) {
        fprintf(G.elm_monitor_out, "%02X", msg->data[i]);
        if (G.elm_spaces) fprintf(G.elm_monitor_out, " ");
    }
    fprintf(G.elm_monitor_out, ELM_NEWLINE(g));
    return fflush(G.elm_monitor_out) >= 0;
}

void elm_monitor_task(void* param)
{
    elm_globals_t* g = (elm_globals_t*)param;
    ESP_LOGI(TAG, "Monitor task started");

    stdout = G.elm_monitor_out;
    G.elm_monitor_task_run = true;

    RingbufHandle_t buf = can_ringbuf_new(50);
    if (buf == NULL) {
        ESP_LOGE(TAG, "monitor error create buffer, nomem");
        goto _exit;
    }

    uint32_t stat_us = esp_timer_get_time();
    uint32_t last_us = stat_us;
    uint32_t count = 0;

    while (G.elm_monitor) {

        size_t size;
        can_message_timestamp_t* rx_msg = xRingbufferReceive(buf, &size, pdMS_TO_TICKS(100)); // min timeout: 100 ms
        uint32_t us = esp_timer_get_time();

        // filter
        if (rx_msg) {
            if (elm_filter_test(g, rx_msg->msg.identifier, rx_msg->timestamp)) {
                // write data
                last_us = rx_msg->timestamp;
                count++;
                if (!_elm_write_can(g, &rx_msg->msg)) {
                    // error
                    ESP_LOGE(TAG, "monitor write error %s", strerror(errno));
                    break;
                }
            }
            vRingbufferReturnItem(buf, rx_msg);
        }
        // test timeout
        if ((us - last_us) >= G.elm_timeout * 1000) {
            ESP_LOGW(TAG, "monitor timeout");
            printf(ELM_NODATA_PROMPT "%s", ELM_NEWLINE(g));
            fflush(stdout);
            break;
        }

        // stat
        uint32_t time_us = us - stat_us;
        if (time_us >= 10 * 1000000) {
            ESP_LOGI(TAG, "monitor stat: count=%u %i/s",
                     count,
                     (int)((float)count / ((float)time_us / 1000000)));
            count = 0;
            stat_us = us;
        }
    }

_exit:
    ESP_LOGI(TAG, "Monitor task ended");
    can_ringbuf_del(buf);
    G.elm_monitor = false;
    G.elm_monitor_task_run = false;
    vTaskDelete(NULL);
}

void elm_monitor_start(elm_globals_t* g)
{
    if (G.elm_monitor)
        return;
    ESP_LOGI(TAG, "Start monitor");

    G.elm_monitor = true;
    xTaskCreatePinnedToCore(elm_monitor_task, "elm-monitor", 4 * 1024, g, ELM_MONITOR_TASK_RUN_PRIO, NULL, ELM_MONITOR_TASK_RUN_CORE);
}

void elm_monitor_stop(elm_globals_t* g)
{
    if (!G.elm_monitor)
        return;
    ESP_LOGI(TAG, "Stop monitor");

    G.elm_monitor = false;

    while (G.elm_monitor_task_run)
        vTaskDelay(10);

    fprintf(G.elm_monitor_out, "%s" ELM_PROMPT, ELM_NEWLINE(g));
    fflush(G.elm_monitor_out);
}

// -----------------------------  elm_do  -----------------------------

void elm_do(const char* tag)
{
    // init
    elm_globals_t* g;
    g = malloc(sizeof(*g));
    if (g == NULL) {
        ESP_LOGE(ELM_TAG, "no mem for globals");
        return;
    }
    memset(g, 0, sizeof(*g));
    G.elm_tag = tag;
    G.elm_monitor_out = stdout;

    // loop
    elm_reset(g);
    elm_write_prompt(g);

    char line[ELM_BUFFER_LEN];
    int pos = 0;
    while (true) {
        int c = fgetc(stdin);
        if (c == 0) continue;
        if (c < 0) {
            ESP_LOGW(TAG, "stop on EOF");
            break;
        }
        if (c == 4) {
            ESP_LOGW(TAG, "stop on ctrl-D");
            break;
        }
        if (G.elm_monitor) {
            ESP_LOGW(TAG, "char %i receveid, stop monitor", c);
            elm_monitor_stop(g);
        }

        switch (c) {
        case 8:
        case 127:
            if (pos > 0) {
                pos--;
                printf("\b \b");
                fflush(stdout);
            }
            break;

        case '\r':
            line[pos] = 0;
            elm_newline(g);
            elm_newline(g);
            elm_do_cmd(g, line);
            // new cmd
            pos = 0;
            line[0] = 0;
            if (!G.elm_monitor) {
                elm_write_prompt(g);
                fflush(stdout);
            }
            break;

        case '\n':
            break;

        default:
            if (c < ' ') break;
            if (pos >= ELM_BUFFER_LEN - 1) break;
            line[pos++] = c;
            if (G.elm_echo) {
                putc(c, stdout);
                fflush(stdout);
            }
            break;
        }
    }

    // deinit
    elm_monitor_stop(g);
    free(G.elm_device_identifier);
    free(g);
}
