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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash_partitions.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

#include "ota.h"

// static const char* TAG = "ota";

#define OTA_BUFFSIZE 1024

bool ota_info()
{
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    printf("Running partition: label '%s', type %d, subtype %d, address 0x%08x, size 0x%08x\n",
           running->label, running->type, running->subtype, running->address, running->size);
    esp_app_desc_t app_desc;
    if (esp_ota_get_partition_description(running, &app_desc) == ESP_OK) {
        printf("    firmware: %s version %s\n", app_desc.project_name, app_desc.version);
    }
    printf("Next update partition: label '%s', type %d, subtype %d, address 0x%08x, size 0x%08x\n",
           next->label, next->type, next->subtype, next->address, next->size);
    return true;
}

void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

bool ota_update(const char* url)
{
    esp_err_t err;
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 5000,
    };

    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(NULL);
    printf("Update partition '%s' from '%s'\n", update_partition->label, url);

    char* ota_write_data = malloc(OTA_BUFFSIZE);
    if (ota_write_data == NULL) {
        printf("Error: no mem\n");
        return false;
    }

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        printf("Error: Failed to initialise HTTP connection\n");
        free(ota_write_data);
        return false;
    }

    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        printf("Error: Failed to open HTTP connection: %s\n", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(ota_write_data);
        return false;
    }

    esp_http_client_fetch_headers(client);

    esp_ota_handle_t update_handle = 0 ;
    int binary_file_length = 0;
    /*deal with all receive packet*/
    bool image_header_was_checked = false;
    while (1) {
        int data_read = esp_http_client_read(client, ota_write_data, OTA_BUFFSIZE);
        if (data_read < 0) {
            printf("Error: SSL data read error\n");
            http_cleanup(client);
            free(ota_write_data);
            return false;
        }
        else if (data_read > 0) {
            if (image_header_was_checked == false) {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                    // check current version with downloading
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    printf("New firmware: %s version %s\n", new_app_info.project_name, new_app_info.version);

                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                    if (err != ESP_OK) {
                        printf("Error: esp_ota_begin failed (%s)\n", esp_err_to_name(err));
                        http_cleanup(client);
                        free(ota_write_data);
                        return false;
                    }
                    printf("Begin update\n");
                    fflush(stdout);
                }
                else {
                    printf("Error: received package is not fit len\n");
                    http_cleanup(client);
                    free(ota_write_data);
                    return false;
                }
            }
            err = esp_ota_write(update_handle, (const void*)ota_write_data, data_read);
            if (err != ESP_OK) {
                printf("Error: esp_ota_write failed (%s)\n", esp_err_to_name(err));
                http_cleanup(client);
                free(ota_write_data);
                return false;
            }
            binary_file_length += data_read;
            // printf("Written image length %d", binary_file_length);
            if ((binary_file_length & 0x3fff) == 0) printf(" %08x\r", binary_file_length); 
            fflush(stdout);
        }
        else if (data_read == 0) {
            /*
            * As esp_http_client_read never returns negative error code, we rely on
            * `errno` to check for underlying transport connectivity closure if any
            */
            if (errno == ECONNRESET || errno == ENOTCONN) {
                printf("\nError: Connection closed, errno = %d\n", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client)) {
                break;
            }
        }
    }
    if (!esp_http_client_is_complete_data_received(client)) {
        printf("\nError in receiving complete file\n");
        http_cleanup(client);
        free(ota_write_data);
        return false;
    }

    printf("\nUpdate completed, binary data length: %d\n", binary_file_length);
    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            printf("Error: Image validation failed, image is corrupted\n");
        }
        printf("Error: esp_ota_end failed (%s)!\n", esp_err_to_name(err));
        http_cleanup(client);
        free(ota_write_data);
        return false;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        printf("Error: esp_ota_set_boot_partition failed (%s)!\n", esp_err_to_name(err));
        http_cleanup(client);
        free(ota_write_data);
        return false;
    }
    printf("Prepare to restart system\n");
    fflush(stdout);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("Restart!\n");
    fflush(stdout);
    esp_restart();
    return true;
}

