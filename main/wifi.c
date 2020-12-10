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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mdns.h"
#include "lwip/apps/netbiosns.h"


#include "sdkconfig.h"

#include "wifi.h"

static const char* TAG = "net";

static bool reconnect = false;
static esp_netif_t* netif_ap = NULL;
static esp_netif_t* netif_sta = NULL;

static EventGroupHandle_t wifi_event_group;
static const int WIFI_AP_STARTED_BIT = BIT0;
static const int WIFI_STA_CONNECTING_BIT = BIT1;
static const int WIFI_STA_CONNECTED_BIT = BIT2;

static const char* second_chan_str[] = {"NONE", "ABOVE", "BELOW"};
static const int second_chan_str_count = sizeof(second_chan_str) / sizeof(*second_chan_str);
static const char* auth_mode_str[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", "WPA2 ENTERPRISE", "WPA3 PSK", "WPA2 WPA3 PSK"};
static const int auth_mode_str_count = sizeof(auth_mode_str) / sizeof(*auth_mode_str);
static const char* cipher_type_str[] = {"NONE", "WEP40", "WEP104", "TKIP", "CCMP", "TKIP CCMP"};
static const int cipher_type_str_count = sizeof(cipher_type_str) / sizeof(*cipher_type_str);
static const char* wifi_event_str[] = {"WIFI_READY", "SCAN_DONE", "STA_START", "STA_STOP", "STA_CONNECTED", "STA_DISCONNECTED", "STA_AUTHMODE_CHANGE", "STA_WPS_ER_SUCCESS",
                                       "STA_WPS_ER_FAILED", "STA_WPS_ER_TIMEOUT", "STA_WPS_ER_PIN", "STA_WPS_ER_PBC_OVERLAP",
                                       "AP_START", "AP_STOP", "AP_STACONNECTED", "AP_STADISCONNECTED", "AP_PROBEREQRECVED", "UNKNOW"};
static const int wifi_event_str_count = sizeof(wifi_event_str) / sizeof(*wifi_event_str);

#define WIFI_TCP_PORT 35000
#define WIFI_TCP_TASK_PRIO 4
#define WIFI_TCP_TASK_CORE 1 // tskNO_AFFINITY

static wifi_cb_t wifi_cb = NULL;
static TaskHandle_t wifi_tcp_server_handle = NULL;
static int wifi_tcp_server_listen_sock = -1;

// tcp stat
static uint32_t tcp_size;
static uint32_t tcp_stat_us;
static uint32_t tcp_last_us;



// -----------------------------  tcp cookie io functions  -----------------------------

ssize_t _tcp_read(void *cookie, char *buf, size_t size)
{
    int soc = (int)cookie;
    return recv(soc, buf, size, 0);
}

ssize_t _tcp_write(void *cookie, const char *buf, size_t size)
{
    int soc = (int)cookie;
    size_t to_write = size;
    while (to_write > 0) {
        int written = send(soc, buf + (size - to_write), to_write, 0);
        if (written < 0) {
            // ESP_LOGE(TAG, "Error occurred during sending: socket=%d errno %d: %s", socket, errno, strerror(errno));
            return -1;
        }
        to_write -= written;
    }

    // tcp global stat
    tcp_last_us = esp_timer_get_time();
    tcp_size += size;
    uint32_t time_us = tcp_last_us - tcp_stat_us;
    if (time_us >= 10 * 1000000) {
        ESP_LOGI(TAG, "tcp stat: size=%uB %iB/s",
                tcp_size, (int)((float)tcp_size / ((float)time_us / 1000000)));
        tcp_size = 0;
        tcp_stat_us = tcp_last_us;
    }

    return size;
}

static const cookie_io_functions_t tcp_cookie_func = {
    .read  = _tcp_read,
    .write = _tcp_write,
    .seek  = NULL,
    .close = NULL
};

FILE *tcp_fopen(int sock, const char *mode)
{
    return fopencookie((void*)sock, mode, tcp_cookie_func);
}


// -----------------------------  wifi_handler  -----------------------------

static void wifi_tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    int sock = -1;

    ESP_LOGI(TAG, "tcp server started");

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(WIFI_TCP_PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    wifi_tcp_server_listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (wifi_tcp_server_listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d: %s", errno, strerror(errno));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(wifi_tcp_server_listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: %u errno %d: %s", WIFI_TCP_PORT, errno, strerror(errno));
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", WIFI_TCP_PORT);

    err = listen(wifi_tcp_server_listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: %s errno %d: %s", addr_str, errno, strerror(errno));
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening, port %d", WIFI_TCP_PORT);

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        sock = accept(wifi_tcp_server_listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d: %s", errno, strerror(errno));
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        } else if (source_addr.sin6_family == PF_INET6) {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s socket %d", addr_str, sock);

        if (wifi_cb) wifi_cb(sock);

        shutdown(sock, 0);
        close(sock);
        sock = -1;
    }

CLEAN_UP:
    ESP_LOGI(TAG, "tcp server stoped");
    if (sock >= 0) {
        shutdown(sock, 0);
        close(sock);
    }
    close(wifi_tcp_server_listen_sock);
    wifi_tcp_server_listen_sock = -1;
    wifi_tcp_server_handle = NULL;
    vTaskDelete(NULL);
}

// -----------------------------  wifi_servers_  -----------------------------

void _wifi_servers_start()
{
    if (wifi_tcp_server_handle) {
        ESP_LOGE(TAG, "servers already started");
        return;
    }

    xTaskCreatePinnedToCore(wifi_tcp_server_task, "tcp_server", 4096, NULL, WIFI_TCP_TASK_PRIO, &wifi_tcp_server_handle, WIFI_TCP_TASK_CORE);
}

void _wifi_servers_stop()
{
    if (wifi_tcp_server_listen_sock >= 0) {
        close(wifi_tcp_server_listen_sock);
        wifi_tcp_server_listen_sock = -1;
        vTaskDelay(100);
    }
    if (wifi_tcp_server_handle) {
        vTaskDelete(wifi_tcp_server_handle);
        wifi_tcp_server_handle = NULL;
    }
}

// -----------------------------  wifi_handler  -----------------------------

void wifi_handler(void* arg, esp_event_base_t event_base,
                  int32_t event_id, void* event_data)
{
    if (event_id < 0 || event_id >= wifi_event_str_count) event_id = wifi_event_str_count - 1;
    ESP_LOGI(TAG, "wifi event: %s", wifi_event_str[event_id]);

    switch (event_id) {

    case WIFI_EVENT_AP_START:
        xEventGroupSetBits(wifi_event_group, WIFI_AP_STARTED_BIT);
        _wifi_servers_start();
        break;

    case WIFI_EVENT_AP_STOP:
        xEventGroupClearBits(wifi_event_group, WIFI_AP_STARTED_BIT);
        _wifi_servers_stop();
        break;

    case WIFI_EVENT_AP_STACONNECTED:
        break;

    case WIFI_EVENT_AP_STADISCONNECTED:
        break;

    case WIFI_EVENT_STA_START:
        xEventGroupClearBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);
        xEventGroupSetBits(wifi_event_group, WIFI_STA_CONNECTING_BIT);
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_STOP:
        xEventGroupClearBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);
        xEventGroupClearBits(wifi_event_group, WIFI_STA_CONNECTING_BIT);
        _wifi_servers_stop();
        break;

    case WIFI_EVENT_STA_CONNECTED:
        // never ??
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        xEventGroupClearBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);
        _wifi_servers_stop();

        if (reconnect) {
            ESP_LOGI(TAG, "wifi event: STA disconnect, reconnect...");
            esp_wifi_connect();
        }
        else {
            // ESP_LOGI(TAG, "wifi event: STA disconnect");
            xEventGroupClearBits(wifi_event_group, WIFI_STA_CONNECTING_BIT);
        }
        break;

    default:
        break;
    }
}

static void got_ip_handler(void* arg, esp_event_base_t event_base,
                           int32_t event_id, void* event_data)
{
    // ip_event_got_ip_t* data = event_data;
    // ESP_LOGI(TAG, "ip event: got ip " IPSTR ", netmask " IPSTR ", gw " IPSTR, IP2STR(&data->ip_info.ip), IP2STR(&data->ip_info.netmask), IP2STR(&data->ip_info.gw));

    xEventGroupClearBits(wifi_event_group, WIFI_STA_CONNECTING_BIT);
    xEventGroupSetBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);

    // initialize_sntp();
    _wifi_servers_start();
}

// -----------------------------  wifi_get_local_ip  -----------------------------

uint32_t wifi_get_local_ip()
{
    esp_netif_t* netif = netif_ap;
    esp_netif_ip_info_t ip_info;
    wifi_mode_t mode;

    esp_wifi_get_mode(&mode);
    if (WIFI_MODE_STA == mode) {
        int bits = xEventGroupWaitBits(wifi_event_group, WIFI_STA_CONNECTED_BIT, 0, 1, 0);
        if (bits & WIFI_STA_CONNECTED_BIT) {
            netif = netif_sta;
        }
        else {
            // ESP_LOGE(TAG, "sta has no IP");
            return 0;
        }
    }

    esp_netif_get_ip_info(netif, &ip_info);
    return ip_info.ip.addr;
}

// -----------------------------  mdns  -----------------------------

void wifi_mdns_init()
{
    ESP_LOGI(TAG, "Initialize mdns: %s", TESLAP_HOSTNAME);

    mdns_init();
    mdns_hostname_set(TESLAP_HOSTNAME);
    mdns_instance_name_set(TESLAP_HOSTNAME " web server");

    mdns_txt_item_t serviceTxtData[] = {
        {"board", TESLAP_HOSTNAME},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

// -----------------------------  netbios  -----------------------------

void wifi_netbios_init()
{
    ESP_LOGI(TAG, "Initialize netbios: %s", TESLAP_HOSTNAME);
    netbiosns_init();
    netbiosns_set_name(TESLAP_HOSTNAME);
}

// -----------------------------  initialise_wifi  -----------------------------

#define IP4TOUINT32(a,b,c,d) (((uint32_t)((a) & 0xffU) << 24) | ((uint32_t)((b) & 0xffU) << 16) | ((uint32_t)((c) & 0xffU) << 8) | (uint32_t)((d) & 0xffU))
#define IP4TOADDR(a,b,c,d) esp_netif_htonl(IP4TOUINT32(a, b, c, d))

static const esp_netif_ip_info_t teslap_soft_ap_ip = {
        .ip = { .addr = IP4TOADDR( 192, 168, 0, 10) },
        .gw = { .addr = IP4TOADDR( 192, 168, 0, 10) },
        .netmask = { .addr = IP4TOADDR( 255, 255, 255, 0) },
};

const esp_netif_inherent_config_t teslap_g_esp_netif_inherent_ap_config = {
        .flags = ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP,
        .ip_info = (esp_netif_ip_info_t*)&teslap_soft_ap_ip,
        .if_key = "WIFI_AP_DEF",
        .if_desc = "ap",
        .route_prio = 10
};

/**
 * @brief User init default AP teslap
 */
esp_netif_t* teslap_esp_netif_create_default_wifi_ap(void)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_AP();
    cfg.base = &teslap_g_esp_netif_inherent_ap_config;
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif);
    esp_netif_attach_wifi_ap(netif);
    esp_wifi_set_default_wifi_ap_handlers();
    return netif;
}

void wifi_init(wifi_cb_t cb)
{
    static bool initialized = false;
    if (initialized) return;

    ESP_LOGI(TAG, "Initialize wifi");

    wifi_mdns_init();
    wifi_netbios_init();

    esp_log_level_set("wifi", ESP_LOG_WARN);
    ESP_LOGI(TAG, "Initialize LOG LEVEL \"wifi\" to %s", "W");

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();

    netif_ap = teslap_esp_netif_create_default_wifi_ap();
    // netif_ap = esp_netif_create_default_wifi_ap();
    assert(netif_ap);
    netif_sta = esp_netif_create_default_wifi_sta();
    assert(netif_sta);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    // ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );
    initialized = true;

    wifi_cb = cb;

    // auto connect
    reconnect = true;

    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);

    if (mode == WIFI_MODE_NULL) {
        // default mode AP
        wifi_ap(TESLAP_HOSTNAME, NULL);
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());
}

// -----------------------------  wifi_print_  -----------------------------

void wifi_print_ap_record_detail(wifi_ap_record_t* ap_record)
{
    printf("    BSSID                   " MACSTR "\n", MAC2STR(ap_record->bssid));
    printf("    SSID                    %s\n", ap_record->ssid);
    printf("    Primary channel         %u\n", ap_record->primary);
    printf("    Second channel          %s\n", ap_record->second < second_chan_str_count ? second_chan_str[ap_record->second] : "?");
    printf("    RSSI                    %i\n", ap_record->rssi);
    printf("    Authmode                %s\n", ap_record->authmode < auth_mode_str_count ? auth_mode_str[ap_record->authmode] : "?");
    printf("    Pairwise cipher         %s\n", ap_record->pairwise_cipher < cipher_type_str_count ? cipher_type_str[ap_record->pairwise_cipher] : "?");
    printf("    Group cipher            %s\n", ap_record->group_cipher < cipher_type_str_count ? cipher_type_str[ap_record->group_cipher] : "?");
    printf("    802.11                  %s%s%s%s%s\n", ap_record->phy_11b ? "b " : "", ap_record->phy_11g ? "g " : "", ap_record->phy_11n ? "n " : "", ap_record->phy_lr ? "LR " : "", ap_record->wps ? "WPS " : "");
    printf("    Country                 %c%c%c\n", ap_record->country.cc[0] != 0 ? ap_record->country.cc[0] : ' ', ap_record->country.cc[1] != 0 ? ap_record->country.cc[1] : ' ', ap_record->country.cc[2] != 0 ? ap_record->country.cc[2] : ' ');
}

void wifi_print_sta_list(wifi_sta_info_t* sta_info)
{
    printf("    " MACSTR "  %4i   %s%s%s%s\n",
           MAC2STR(sta_info->mac),
           sta_info->rssi,
           sta_info->phy_11b ? "b " : "", sta_info->phy_11g ? "g " : "", sta_info->phy_11n ? "n " : "", sta_info->phy_lr ? "LR " : "");
}

void wifi_print_ap_record_list(wifi_ap_record_t* ap_record)
{
    printf("%-30s %4d   %3d     %s\n",
           ap_record->ssid,
           ap_record->rssi,
           ap_record->primary,
           ap_record->authmode < auth_mode_str_count ? auth_mode_str[ap_record->authmode] : "?");
}

// -----------------------------  wifi_status  -----------------------------

bool wifi_status()
{
    int bits = xEventGroupWaitBits(wifi_event_group, WIFI_AP_STARTED_BIT | WIFI_STA_CONNECTING_BIT | WIFI_STA_CONNECTED_BIT, 0, 1, 0);
    if ((bits & (WIFI_AP_STARTED_BIT | WIFI_STA_CONNECTING_BIT | WIFI_STA_CONNECTED_BIT)) == 0) {
        printf("wifi stoped\n");
        return true;
    }

    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: get mode error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    if (mode == WIFI_MODE_NULL) {
        printf("wifi mode NULL\n");
        return true;
    }

    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        if (bits & WIFI_STA_CONNECTED_BIT) {
            wifi_ap_record_t ap = {0};
            esp_err_t err = esp_wifi_sta_get_ap_info(&ap);
            if (err != ESP_OK) {
                fprintf(stderr, "wifi: sta get ap info error 0x%x %s\n", err, esp_err_to_name(err));
                return false;
            }
            printf("wifi STA: Connected\n");
            wifi_print_ap_record_detail(&ap);
        }
        else if (bits & WIFI_STA_CONNECTING_BIT) {
            printf("wifi STA: Connecting\n");
        }
        else {
            printf("wifi STA: Disconnected\n");
        }
    }

    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        if (bits & WIFI_AP_STARTED_BIT) {
            wifi_config_t cfg;
            err = esp_wifi_get_config(WIFI_IF_AP, &cfg);
            if (err != ESP_OK) {
                fprintf(stderr, "wifi: esp_wifi_get_config error 0x%x %s\n", err, esp_err_to_name(err));
                return false;
            }
            printf("wifi AP '%s'\n", cfg.ap.ssid);

            wifi_sta_list_t sta_list = {0};
            err = esp_wifi_ap_get_sta_list(&sta_list);
            if (err != ESP_OK) {
                fprintf(stderr, "wifi: get sta list error 0x%x %s\n", err, esp_err_to_name(err));
                return false;
            }

            if (sta_list.num == 0) {
                printf("  No sta connected\n");
            }
            else {
                printf("  %i sta connected:\n", sta_list.num);
                for (int i = 0; i < sta_list.num; i++) {
                    wifi_print_sta_list(&sta_list.sta[i]);
                }
            }
        }
        else {
            printf("wifi AP: Disconnected\n");
        }
    }

    return true;
}

// -----------------------------  wifi_sta  -----------------------------

// wifi sta Aki PascalAkiWiFi7

bool wifi_sta(char* ssid, char* password)
{
    esp_err_t err;

    err = esp_wifi_stop();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: stop error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set sta mode error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    wifi_config_t cfg = {0};
    if (ssid != NULL) {
        strlcpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    }
    if (password != NULL && *password != 0) {
        strlcpy((char*)cfg.sta.password, password, sizeof(cfg.sta.password));
        // cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // force WPA2  WIFI_AUTH_WPA_WPA2_PSK
    }
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.pmf_cfg.required = false;

    ESP_LOGI(TAG, "wifi sta ssid='%s' pwd='%s'", cfg.sta.ssid, cfg.sta.password);
    err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set sta config error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    reconnect = true;
    err = esp_wifi_start();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: start error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }
    return true;
}

// -----------------------------  wifi_ap  -----------------------------

bool wifi_ap(char* ssid, char* password)
{
    esp_err_t err;

    err = esp_wifi_stop();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: stop error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set ap mode error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    wifi_config_t cfg = {0};
    cfg.ap.max_connection = 6;
    if (ssid != NULL) {
        strlcpy((char*)cfg.ap.ssid, ssid, sizeof(cfg.ap.ssid));
    }
    cfg.ap.authmode = WIFI_AUTH_OPEN;
    if (password != NULL && *password != 0) {
        strlcpy((char*)cfg.ap.password, password, sizeof(cfg.ap.password));
        cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &cfg);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set ap config error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: start error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }
    return true;
}

// -----------------------------  wifi_stop  -----------------------------

bool wifi_stop()
{
    esp_err_t err;

    reconnect = false;
    
    err = esp_wifi_stop();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: stop error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_set_mode(WIFI_MODE_NULL);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set null mode error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    return true;
}

// -----------------------------  wifi_scan  -----------------------------

bool wifi_scan()
{
    esp_err_t err = ESP_OK;

    err = esp_wifi_stop();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: stop error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set sta mode error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    wifi_config_t cfg = {0};
    err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set sta config error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: start error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    printf(" Scanning...\r");
    fflush(stdout);
    err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: scanning error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }
    printf("Scan completed :\n");

    uint16_t ap_count = 0;
    wifi_ap_record_t* ap_record = NULL;

    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0) {
        printf("AP not found\n");
        return false;
    }
    ap_record = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_record == NULL) {
        fprintf(stderr, "wifi: out of memory\n");
        return false;
    }

    err = esp_wifi_scan_get_ap_records(&ap_count, ap_record);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: get records error 0x%x %s\n", err, esp_err_to_name(err));
        // free ap records ?
        free(ap_record);
        return false;
    }

    for (int i = 0; i < ap_count; i++) {
        wifi_print_ap_record_list(&ap_record[i]);
    }

    free(ap_record);

    err = esp_wifi_stop();
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: stop error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_set_mode(WIFI_MODE_NULL);
    if (err != ESP_OK) {
        fprintf(stderr, "wifi: set sta mode error 0x%x %s\n", err, esp_err_to_name(err));
        return false;
    }

    return true;
}
